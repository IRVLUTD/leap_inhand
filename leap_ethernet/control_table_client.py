"""Send control-table commands to an OpenRB-150 over UDP.

Wire formats:
    READ_ALL <address> <size>
    WRITE_ALL <address> <size> <value_0> ... <value_15>
    WRITE_READ_ALL <write_address> <write_size> <read_address> <read_size> <value_0> ... <value_15>
    READ <address> <size> <motor_id> ...
    WRITE <address> <size> <motor_id> <value> ...
    WRITE_READ <write_address> <write_size> <read_address> <read_size> <motor_id> <value> ...

Read methods return dictionaries keyed by motor ID. Write methods wait for an
echoed binary acknowledgement. Requests and responses can optionally be logged
to CSV as hexadecimal packet strings.
"""

import csv
from enum import IntEnum
import socket
import struct
import time
from dataclasses import dataclass

DEFAULT_IP = "10.42.42.50"
DEFAULT_PORT = 8888
DEFAULT_TIMEOUT = 2.0
MOTOR_COUNT = 16
PACKET_FORMAT = "<BBBBBB" + "Bi" * MOTOR_COUNT
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)
READ_ALL = 0
WRITE_ALL = 1
WRITE_READ_ALL = 2
READ = 3
WRITE = 4
WRITE_READ = 5
ERROR = 255


class DataNames(IntEnum):
    """Control-table data names and their Dynamixel control-table addresses."""

    def __new__(cls, address: int, size: int):
        member = int.__new__(cls, address)
        member._value_ = address
        object.__setattr__(member, "_size", size)
        return member

    MODEL_NUMBER = (0, 2)
    MODEL_INFORMATION = (2, 4)
    FIRMWARE_VERSION = (6, 1)
    ID = (7, 1)
    BAUD_RATE = (8, 1)
    RETURN_DELAY_TIME = (9, 1)
    DRIVE_MODE = (10, 1)
    OPERATING_MODE = (11, 1)
    SECONDARY_ID = (12, 1)
    PROTOCOL_TYPE = (13, 1)
    HOMING_OFFSET = (20, 4)
    MOVING_THRESHOLD = (24, 4)
    TEMPERATURE_LIMIT = (31, 1)
    MIN_VOLTAGE_LIMIT = (32, 2)
    MAX_VOLTAGE_LIMIT = (34, 2)
    PWM_LIMIT = (36, 2)
    CURRENT_LIMIT = (38, 2)
    VELOCITY_LIMIT = (44, 4)
    MAX_POSITION_LIMIT = (48, 4)
    MIN_POSITION_LIMIT = (52, 4)
    STARTUP_CONFIGURATION = (60, 1)
    PWM_SLOPE = (62, 1)
    SHUTDOWN = (63, 1)
    TORQUE_ENABLE = (64, 1)
    LED = (65, 1)
    STATUS_RETURN_LEVEL = (68, 1)
    REGISTERED_INSTRUCTION = (69, 1)
    HARDWARE_ERROR_STATUS = (70, 1)
    VELOCITY_I_GAIN = (76, 2)
    VELOCITY_P_GAIN = (78, 2)
    POSITION_D_GAIN = (80, 2)
    POSITION_I_GAIN = (82, 2)
    POSITION_P_GAIN = (84, 2)
    FEEDFORWARD_2ND_GAIN = (88, 2)
    FEEDFORWARD_1ST_GAIN = (90, 2)
    BUS_WATCHDOG = (98, 1)
    GOAL_PWM = (100, 2)
    GOAL_CURRENT = (102, 2)
    GOAL_VELOCITY = (104, 4)
    PROFILE_ACCELERATION = (108, 4)
    PROFILE_VELOCITY = (112, 4)
    GOAL_POSITION = (116, 4)
    REALTIME_TICK = (120, 2)
    MOVING = (122, 1)
    MOVING_STATUS = (123, 1)
    PRESENT_PWM = (124, 2)
    PRESENT_CURRENT = (126, 2)
    PRESENT_VELOCITY = (128, 4)
    PRESENT_POSITION = (132, 4)
    VELOCITY_TRAJECTORY = (136, 4)
    POSITION_TRAJECTORY = (140, 4)
    PRESENT_INPUT_VOLTAGE = (144, 2)
    PRESENT_TEMPERATURE = (146, 1)
    BACKUP_READY = (147, 1)

    @property
    def size(self) -> int:
        return object.__getattribute__(self, "_size")


ControlTableItem = DataNames


@dataclass(frozen=True)
class BinaryPacket:
    """Decoded representation of one 86-byte protocol packet."""

    cmd: int
    address: int
    length: int
    count: int
    read_address: int
    read_length: int
    motors: tuple[tuple[int, int], ...]


class ControlTableError(Exception):
    """An ERR response returned by the OpenRB."""

    def __init__(self, reason: str, response: BinaryPacket) -> None:
        self.reason = reason
        self.response = response
        super().__init__(reason)


class ControlTableTimeout(TimeoutError):
    """The OpenRB did not respond before the configured timeout."""


class ControlTableProtocolError(ValueError):
    """The OpenRB returned a response that did not match the protocol."""


ERROR_REASONS = {
    -1: "NO_MOTOR_IDS",
    -2: "TORQUE_SAFETY",
    -3: "UNKNOWN_COMMAND",
}


def _item_metadata(item: ControlTableItem | str) -> tuple[DataNames, int]:
    """Return the control-table item and its byte size."""
    try:
        control_table_item = (
            item
            if isinstance(item, ControlTableItem)
            else ControlTableItem[item.upper()]
        )
    except (KeyError, AttributeError) as error:
        raise ValueError(f"unknown control-table item: {item}") from error
    try:
        return control_table_item, control_table_item.size
    except AttributeError as error:
        raise ValueError(
            f"no control-table size is defined for {control_table_item.name}"
        ) from error


def _validate_motor_id(motor_id: int) -> None:
    if not 0 <= motor_id <= 15:
        raise ValueError("motor_id must be between 0 and 15")


def _validate_value(value: int) -> None:
    if not isinstance(value, int) or not -(2**31) <= value <= 2**31 - 1:
        raise ValueError("value must be a signed 32-bit integer")


def _build_packet(
    command: int,
    item: ControlTableItem | str,
    count: int,
    motors: list[tuple[int, int]],
    read_item: ControlTableItem | str | None = None,
) -> bytes:
    control_table_item, size = _item_metadata(item)
    if read_item is not None:
        read_control_table_item, read_size = _item_metadata(read_item)
    else:
        read_control_table_item, read_size = control_table_item, size
    if not 0 <= count <= MOTOR_COUNT:
        raise ValueError(f"count must be between 0 and {MOTOR_COUNT}")
    if len(motors) > MOTOR_COUNT:
        raise ValueError(f"a packet can contain at most {MOTOR_COUNT} motors")

    padded_motors = motors + [(0, 0)] * (MOTOR_COUNT - len(motors))
    for motor_id, value in padded_motors:
        _validate_motor_id(motor_id)
        _validate_value(value)
    packet = struct.pack(
        PACKET_FORMAT,
        command,
        control_table_item.value,
        size,
        count,
        read_control_table_item.value,
        read_size,
        *[part for motor in padded_motors for part in motor],
    )
    if len(packet) != PACKET_SIZE:
        raise AssertionError(f"binary packet has unexpected size: {len(packet)}")
    return packet


def unpack_packet(packet: bytes) -> BinaryPacket:
    """Decode and validate one complete binary protocol packet."""
    if len(packet) != PACKET_SIZE:
        raise ControlTableProtocolError(
            f"Expected {PACKET_SIZE}-byte packet, got {len(packet)} bytes"
        )
    unpacked = struct.unpack(PACKET_FORMAT, packet)
    motors = tuple(
        (unpacked[index], unpacked[index + 1])
        for index in range(6, len(unpacked), 2)
    )
    return BinaryPacket(
        unpacked[0], unpacked[1], unpacked[2], unpacked[3], unpacked[4], unpacked[5], motors
    )


def build_read_all_command(item: ControlTableItem | str) -> bytes:
    return _build_packet(READ_ALL, item, MOTOR_COUNT, list(enumerate([0] * MOTOR_COUNT)))


def build_write_all_command(
    item: ControlTableItem | str, values: list[int] | tuple[int, ...]
) -> bytes:
    if len(values) != 16:
        raise ValueError("WRITE_ALL requires exactly 16 values")
    for value in values:
        _validate_value(value)
    return _build_packet(WRITE_ALL, item, MOTOR_COUNT, list(enumerate(values)))


def build_read_command(
    item: ControlTableItem | str, motor_ids: list[int] | tuple[int, ...]
) -> bytes:
    if not motor_ids:
        raise ValueError("READ requires at least one motor ID")
    for motor_id in motor_ids:
        _validate_motor_id(motor_id)
    return _build_packet(READ, item, len(motor_ids), [(motor_id, 0) for motor_id in motor_ids])


def build_write_command(
    item: ControlTableItem | str, values_by_motor: dict[int, int]
) -> bytes:
    if not values_by_motor:
        raise ValueError("WRITE requires at least one motor ID and value")
    parts = []
    for motor_id, value in values_by_motor.items():
        _validate_motor_id(motor_id)
        _validate_value(value)
        parts.append((motor_id, value))
    return _build_packet(WRITE, item, len(parts), parts)


def build_write_read_all_command(
    write_item: ControlTableItem | str,
    values: list[int] | tuple[int, ...],
    read_item: ControlTableItem | str | None = None,
) -> bytes:
    if len(values) != 16:
        raise ValueError("WRITE_READ_ALL requires exactly 16 values")
    for value in values:
        _validate_value(value)
    return _build_packet(
        WRITE_READ_ALL,
        write_item,
        MOTOR_COUNT,
        list(enumerate(values)),
        read_item=read_item,
    )


def build_write_read_command(
    write_item: ControlTableItem | str,
    values_by_motor: dict[int, int],
    read_item: ControlTableItem | str | None = None,
) -> bytes:
    if not values_by_motor:
        raise ValueError("WRITE_READ requires at least one motor ID and value")
    parts = []
    for motor_id, value in values_by_motor.items():
        _validate_motor_id(motor_id)
        _validate_value(value)
        parts.append((motor_id, value))
    return _build_packet(
        WRITE_READ,
        write_item,
        len(parts),
        parts,
        read_item=read_item,
    )


class ControlTableClient:
    """UDP client for control-table packet types."""

    def __init__(
        self,
        ip: str = DEFAULT_IP,
        port: int = DEFAULT_PORT,
        timeout: float = DEFAULT_TIMEOUT,
        log_path: str | None = None,
    ) -> None:
        self.address = (ip, port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.settimeout(timeout)
        self.log_path = log_path
        self.start_time = time.monotonic_ns()

    def close(self) -> None:
        self.socket.close()

    def print_command(self, command: bytes) -> str:
        """Print and return a binary command as hexadecimal without sending it."""
        command_text = command.hex(" ")
        print(command_text)
        return command_text

    def _log(self, command: str, response: str) -> None:
        if self.log_path is None:
            return
        with open(self.log_path, "a", newline="") as log_file:
            writer = csv.writer(log_file)
            writer.writerow(
                ((time.monotonic_ns() - self.start_time) // 1_000, command, response)
            )

    def _request(self, command: bytes) -> bytes:
        command_text = command.hex(" ")
        self.socket.sendto(command, self.address)
        try:
            response_bytes, _ = self.socket.recvfrom(1024)
        except (socket.timeout, ConnectionResetError) as error:
            self._log(command_text, "TIMEOUT")
            raise ControlTableTimeout(
                f"OpenRB did not respond within the configured timeout: {command_text}"
            ) from error

        self._log(command_text, response_bytes.hex(" "))
        return response_bytes

    def _response_error(self, response: BinaryPacket) -> None:
        if response.cmd != ERROR:
            return
        error_code = response.motors[0][1]
        reason = ERROR_REASONS.get(error_code, f"DYNAMIXEL_ERROR_{error_code}")
        raise ControlTableError(reason, response)

    def _validate_response_header(
        self, request: BinaryPacket, response: BinaryPacket
    ) -> None:
        self._response_error(response)
        if response.cmd != request.cmd:
            raise ControlTableProtocolError(
                f"Expected response command {request.cmd}, got {response.cmd}"
            )
        if (response.address, response.length) != (request.address, request.length):
            raise ControlTableProtocolError("Response address or length did not match request")
        if (response.read_address, response.read_length) != (request.read_address, request.read_length):
            raise ControlTableProtocolError("Response read address or length did not match request")

    def _read_values(self, command: bytes, motor_ids: list[int]) -> dict[int, int]:
        request = unpack_packet(command)
        response = unpack_packet(self._request(command))
        self._validate_response_header(request, response)
        if response.count != len(motor_ids):
            raise ControlTableProtocolError(
                f"Expected {len(motor_ids)} read values, got {response.count}"
            )
        returned_motors = response.motors[: response.count]
        returned_ids = [motor_id for motor_id, _ in returned_motors]
        if returned_ids != motor_ids:
            raise ControlTableProtocolError(
                f"Expected motor IDs {motor_ids}, got {returned_ids}"
            )
        return dict(returned_motors)

    def _write(self, command: bytes) -> None:
        request = unpack_packet(command)
        response = unpack_packet(self._request(command))
        self._validate_response_header(request, response)
        if response.count != request.count:
            raise ControlTableProtocolError(
                f"Expected {request.count} acknowledged writes, got {response.count}"
            )
        if response.motors[: response.count] != request.motors[: request.count]:
            raise ControlTableProtocolError("Write acknowledgement did not echo the request")

    def read_all(self, item: ControlTableItem | str) -> dict[int, int]:
        """
        Read the value of a control table item for all motors.

        Args:
            item: The control table item to read.

        Returns:
            A dictionary mapping motor IDs to their corresponding read values.
            E.g., {0: 123, 1: 456, 2: 789}.
        """
        return self._read_values(build_read_all_command(item), list(range(16)))

    def write_all(
        self, item: ControlTableItem | str, values: list[int] | tuple[int, ...]
    ) -> None:
        """
        Write a control-table value for all 16 motors.

        Args:
            item: The control-table item to write.
            values: Exactly 16 values, ordered by motor ID 0 through 15.

        Raises:
            ValueError: If values does not contain exactly 16 entries.
            ControlTableError: If the OpenRB returns an ERR response.
        """
        self._write(build_write_all_command(item, values))


    def read(
        self, item: ControlTableItem | str, motor_ids: list[int] | tuple[int, ...]
    ) -> dict[int, int]:
        """
        Read the value of a control table item for multiple motors.

        Args:
            item: The control table item to read.
            motor_ids: A list of motor IDs to read from, in the range 0-15.

        Returns:
            A dictionary mapping motor IDs to their corresponding read values.
            E.g., {0: 123, 1: 456, 2: 789}.
        """
        return self._read_values(build_read_command(item, motor_ids), list(motor_ids))

    def write(
        self, item: ControlTableItem | str, values_by_motor: dict[int, int]
    ) -> None:
        """
        Write the value of a control table item for multiple motors.

        Args:
            item: The control table item to write.
            values_by_motor: A dictionary mapping motor IDs to their values.

        Raises:
            ControlTableError: If the OpenRB returns an ERR response.
        """
        self._write(build_write_command(item, values_by_motor))

    def write_read_all(
        self,
        write_item: ControlTableItem | str,
        values: list[int] | tuple[int, ...],
        read_item: ControlTableItem | str | None = None,
    ) -> dict[int, int]:
        """
        Write a control-table value for all 16 motors and return their read values.

        Args:
            write_item: The control-table item to write.
            values: Exactly 16 values, ordered by motor ID 0 through 15.
            read_item: The control-table item to read. Defaults to write_item if None.

        Returns:
            A dictionary mapping motor IDs to their corresponding read values.
            E.g., {0: 123, 1: 456, ...}.

        Raises:
            ValueError: If values does not contain exactly 16 entries.
            ControlTableError: If the OpenRB returns an ERR response.
        """
        return self._read_values(
            build_write_read_all_command(write_item, values, read_item=read_item),
            list(range(16)),
        )

    def write_read(
        self,
        write_item: ControlTableItem | str,
        values_by_motor: dict[int, int],
        read_item: ControlTableItem | str | None = None,
    ) -> dict[int, int]:
        """
        Write the value of a control table item for multiple motors and return their read values.

        Args:
            write_item: The control table item to write.
            values_by_motor: A dictionary mapping motor IDs to their values.
            read_item: The control-table item to read. Defaults to write_item if None.

        Returns:
            A dictionary mapping motor IDs to their corresponding read values.
            E.g., {0: 123, 1: 456}.

        Raises:
            ValueError: If values_by_motor is empty.
            ControlTableError: If the OpenRB returns an ERR response.
        """
        return self._read_values(
            build_write_read_command(write_item, values_by_motor, read_item=read_item),
            list(values_by_motor.keys()),
        )

