"""Send textual control-table commands to an OpenRB-150 over UDP.

Wire formats:
    READ_ALL <address> <size>
    WRITE_ALL <address> <size> <value_0> ... <value_15>
    READ <address> <size> <motor_id> ...
    WRITE <address> <size> <motor_id> <value> ...

Read methods return dictionaries keyed by motor ID. Requests and responses can
optionally be logged to CSV as their textual packet strings.
"""

import csv
from enum import IntEnum
import socket
import time

DEFAULT_IP = "10.42.42.50"
DEFAULT_PORT = 8888
DEFAULT_TIMEOUT = 2.0
MOTOR_COUNT = 16
READ_ALL = "READ_ALL"
WRITE_ALL = "WRITE_ALL"
READ = "READ"
WRITE = "WRITE"


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


class ControlTableError(Exception):
    """An ERR response returned by the OpenRB."""

    def __init__(self, reason: str, response: str | None = None) -> None:
        self.reason = reason
        self.response = response
        super().__init__(reason)


class ControlTableTimeout(TimeoutError):
    """The OpenRB did not respond before the configured timeout."""


class ControlTableProtocolError(ValueError):
    """The OpenRB returned a response that did not match the protocol."""


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
    return control_table_item, control_table_item.size


def _validate_motor_id(motor_id: int) -> None:
    if not isinstance(motor_id, int) or not 0 <= motor_id <= 15:
        raise ValueError("motor_id must be between 0 and 15")


def _validate_value(value: int) -> None:
    if not isinstance(value, int) or not -(2**31) <= value <= 2**31 - 1:
        raise ValueError("value must be a signed 32-bit integer")


def _build_command(
    command: str,
    item: ControlTableItem | str,
    arguments: list[int],
) -> str:
    control_table_item, size = _item_metadata(item)
    return " ".join(
        [
            command,
            str(control_table_item.value),
            str(size),
            *(str(value) for value in arguments),
        ]
    )


def build_read_all_command(item: ControlTableItem | str) -> str:
    return _build_command(READ_ALL, item, [])


def build_write_all_command(
    item: ControlTableItem | str, values: list[int] | tuple[int, ...]
) -> str:
    if len(values) != MOTOR_COUNT:
        raise ValueError("WRITE_ALL requires exactly 16 values")
    for value in values:
        _validate_value(value)
    return _build_command(WRITE_ALL, item, list(values))


def build_read_command(
    item: ControlTableItem | str, motor_ids: list[int] | tuple[int, ...]
) -> str:
    if not motor_ids:
        raise ValueError("READ requires at least one motor ID")
    for motor_id in motor_ids:
        _validate_motor_id(motor_id)
    return _build_command(READ, item, list(motor_ids))


def build_write_command(
    item: ControlTableItem | str, values_by_motor: dict[int, int]
) -> str:
    if not values_by_motor:
        raise ValueError("WRITE requires at least one motor ID and value")
    arguments = []
    for motor_id, value in values_by_motor.items():
        _validate_motor_id(motor_id)
        _validate_value(value)
        arguments.extend((motor_id, value))
    return _build_command(WRITE, item, arguments)


class ControlTableClient:
    """UDP client for the four textual control-table command types."""

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

    def print_command(self, command: str) -> str:
        """Print and return a textual command without sending it."""
        print(command)
        return command

    def _log(self, command: str, response: str) -> None:
        if self.log_path is None:
            return
        with open(self.log_path, "a", newline="") as log_file:
            writer = csv.writer(log_file)
            writer.writerow(
                ((time.monotonic_ns() - self.start_time) // 1_000, command, response)
            )

    def _request(self, command: str) -> str:
        self.socket.sendto(command.encode("ascii"), self.address)
        try:
            response_bytes, _ = self.socket.recvfrom(1024)
        except (socket.timeout, ConnectionResetError) as error:
            self._log(command, "TIMEOUT")
            raise ControlTableTimeout(
                f"OpenRB did not respond within the configured timeout: {command}"
            ) from error

        response = response_bytes.decode("ascii", errors="replace").strip()
        self._log(command, response)
        return response

    @staticmethod
    def _parse_response(response: str) -> list[int]:
        fields = response.split()
        if not fields:
            raise ControlTableProtocolError("OpenRB returned an empty response")
        if fields[0] == "ERR":
            reason = " ".join(fields[1:]) or "OpenRB returned ERR"
            raise ControlTableError(reason, response)
        if fields[0] != "OK":
            raise ControlTableProtocolError(
                f"Expected OK or ERR response, got {response!r}"
            )
        try:
            return [int(value) for value in fields[1:]]
        except ValueError as error:
            raise ControlTableProtocolError(
                f"Response contained a non-integer value: {response!r}"
            ) from error

    def _read_values(self, command: str, motor_ids: list[int]) -> dict[int, int]:
        values = self._parse_response(self._request(command))
        if len(values) != len(motor_ids):
            raise ControlTableProtocolError(
                f"Expected {len(motor_ids)} read values, got {len(values)}"
            )
        return dict(zip(motor_ids, values))

    def _write(self, command: str) -> None:
        values = self._parse_response(self._request(command))
        if values:
            raise ControlTableProtocolError(
                f"Expected an empty OK response for write, got {values}"
            )

    def read_all(self, item: ControlTableItem | str) -> dict[int, int]:
        """Read the value of a control table item for all motors."""
        return self._read_values(build_read_all_command(item), list(range(MOTOR_COUNT)))

    def write_all(
        self, item: ControlTableItem | str, values: list[int] | tuple[int, ...]
    ) -> None:
        """Write a control-table value for all 16 motors."""
        self._write(build_write_all_command(item, values))

    def read(
        self, item: ControlTableItem | str, motor_ids: list[int] | tuple[int, ...]
    ) -> dict[int, int]:
        """Read a control-table value for multiple motors."""
        return self._read_values(build_read_command(item, motor_ids), list(motor_ids))

    def write(
        self, item: ControlTableItem | str, values_by_motor: dict[int, int]
    ) -> None:
        """Write a control-table value for multiple motors."""
        self._write(build_write_command(item, values_by_motor))