"""Send control-table commands to an OpenRB-150 over UDP.

The board accepts only one packet layout (little-endian, 183 bytes), matching
``UdpPacket`` in leap_hand_bridge.ino. It is not a text protocol.

    struct UdpPacket {
        uint8_t cmd;          // 0 READ_ALL, 1 WRITE_ALL, 2 WRITE_READ_ALL,
                              // 3 READ, 4 WRITE, 5 WRITE_READ, 254 REBOOT, 255 ERR
        uint8_t flags;        // 0 NONE, 1 IGNORE_ERRORS
        uint8_t addr;         // Dynamixel write address (read address for READ*)
        uint8_t length;       // write/read size in bytes: 1, 2, 4, or 10
        uint8_t count;        // motors used in this packet (0-16)
        uint8_t read_addr;    // Dynamixel read address for WRITE_READ*
        uint8_t read_length;  // read size in bytes: 1, 2, 4, or 10
        struct {
            uint8_t id;       // motor ID 0-15
            uint8_t val[10];  // 80-bit raw value buffer (unused slots zeroed)
        } motors[16];
    };

WRITE_ALL assumes motors[0..15] are IDs 0-15 in order. Partial READ/WRITE
use count and motors[0..count-1]. WRITE_READ* writes using addr/length, then
reads using read_addr/read_length. The OpenRB echoes the same 183-byte struct
(or cmd=255 with motors[0].val set to an error code).

Read methods return dictionaries keyed by motor ID. Write methods wait for
that echoed acknowledgement. Requests and responses can optionally be logged
to CSV as hexadecimal packet strings.

Position, velocity, and acceleration registers are converted at this API:
SI value = (raw register - offset) * scale. Absolute positions use offset 2048
so 0 rad is the motor center. The UDP payload is still Dynamixel integers.
See DataNames members marked ``SI:`` for which fields are converted.
"""

import csv
from enum import Enum
import math
import socket
import struct
import time
from dataclasses import dataclass
from pathlib import Path

DEFAULT_IP = "10.42.42.50"
DEFAULT_PORT = 8888
DEFAULT_TIMEOUT = 2.0
MOTOR_COUNT = 16
PROJECT_ROOT = Path(__file__).resolve().parent.parent
LOG_DIR = PROJECT_ROOT / "logs"
PLOT_DIR = PROJECT_ROOT / "plots"
PACKET_FORMAT = "<BBBBBBB" + "B10s" * MOTOR_COUNT
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)
READ_ALL = 0
WRITE_ALL = 1
WRITE_READ_ALL = 2
READ = 3
WRITE = 4
WRITE_READ = 5
REBOOT = 254
REBOOT_BOARD = 0
REBOOT_MOTORS = 1
REBOOT_ALL = 2
ERROR = 255
FLAG_NONE = 0x00
FLAG_IGNORE_ERRORS = 0x01

# XC330-M288 register units → SI. Applied only on DataNames members that pass a
# scale argument. Unscaled members stay raw integers.
# Absolute positions also subtract POSITION_CENTER so 0 rad is 2048 ticks.
# https://emanual.robotis.com/docs/en/dxl/x/xc330-m288/
POSITION_SCALE = 2.0 * math.pi / 4096  # 1 tick = 0.088 deg → rad
POSITION_CENTER = 2048  # ticks corresponding to 0 rad
VELOCITY_SCALE = 0.229 * 2.0 * math.pi / 60.0  # 1 unit = 0.229 rpm → rad/s
ACCELERATION_SCALE = 214.577 * 2.0 * math.pi / 3600.0  # 1 unit = 214.577 rev/min² → rad/s²


class DataNames(Enum):
    """Control-table data names and their Dynamixel control-table addresses.

    Members defined with a ``scale`` argument are converted on read/write:
    SI = (raw - offset) * scale. Absolute position members also pass
    ``POSITION_CENTER`` so 0 rad is 2048 ticks. Those members are marked
    ``SI:`` below. Everything else is still a raw register integer (modes,
    gains, current, PWM, voltage, …).
    """

    def __new__(
        cls,
        address: int,
        size: int,
        scale: float | None = None,
        offset: int = 0,
        unique_id: object | None = None,
    ):
        val = unique_id if unique_id is not None else address
        member = object.__new__(cls)
        member._value_ = val
        member._address = address
        member._size = size
        member._scale = scale
        member._offset = offset
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
    HOMING_OFFSET = (20, 4, POSITION_SCALE)  # SI: rad (relative, not centered)
    MOVING_THRESHOLD = (24, 4, VELOCITY_SCALE)  # SI: rad/s
    TEMPERATURE_LIMIT = (31, 1)
    MIN_VOLTAGE_LIMIT = (32, 2)
    MAX_VOLTAGE_LIMIT = (34, 2)
    PWM_LIMIT = (36, 2)
    CURRENT_LIMIT = (38, 2)
    VELOCITY_LIMIT = (44, 4, VELOCITY_SCALE)  # SI: rad/s
    MAX_POSITION_LIMIT = (48, 4, POSITION_SCALE, POSITION_CENTER)  # SI: rad (0 = 2048 ticks)
    MIN_POSITION_LIMIT = (52, 4, POSITION_SCALE, POSITION_CENTER)  # SI: rad (0 = 2048 ticks)
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
    GOAL_VELOCITY = (104, 4, VELOCITY_SCALE)  # SI: rad/s
    PROFILE_ACCELERATION = (108, 4, ACCELERATION_SCALE)  # SI: rad/s²
    PROFILE_VELOCITY = (112, 4, VELOCITY_SCALE)  # SI: rad/s
    GOAL_POSITION = (116, 4, POSITION_SCALE, POSITION_CENTER)  # SI: rad (0 = 2048 ticks)
    REALTIME_TICK = (120, 2)
    MOVING = (122, 1)
    MOVING_STATUS = (123, 1)
    PRESENT_PWM = (124, 2)
    PRESENT_CURRENT = (126, 2)
    PRESENT_VELOCITY = (128, 4, VELOCITY_SCALE)  # SI: rad/s
    PRESENT_POSITION = (132, 4, POSITION_SCALE, POSITION_CENTER)  # SI: rad (0 = 2048 ticks)
    VELOCITY_TRAJECTORY = (136, 4, VELOCITY_SCALE)  # SI: rad/s
    POSITION_TRAJECTORY = (140, 4, POSITION_SCALE, POSITION_CENTER)  # SI: rad (0 = 2048 ticks)
    PRESENT_INPUT_VOLTAGE = (144, 2)
    PRESENT_TEMPERATURE = (146, 1)
    BACKUP_READY = (147, 1)
    FULL_STATE = (126, 10, None, 0, "FULL_STATE")

    @property
    def value(self) -> int:
        return object.__getattribute__(self, "_address")

    @property
    def address(self) -> int:
        return object.__getattribute__(self, "_address")

    @property
    def size(self) -> int:
        return object.__getattribute__(self, "_size")

    @property
    def scale(self) -> float | None:
        """Raw-to-SI multiplier, or None if this register is left as an integer."""
        return object.__getattribute__(self, "_scale")

    @property
    def offset(self) -> int:
        """Raw ticks subtracted before scaling. 2048 for absolute positions."""
        return object.__getattribute__(self, "_offset")

    def __int__(self) -> int:
        return object.__getattribute__(self, "_address")


ControlTableItem = DataNames


@dataclass(frozen=True)
class FullMotorState:
    """Complete motor state decoded from the 80-bit (10-byte) register block (126-135)."""

    position: float  # SI: rad (0 rad = 2048 ticks)
    velocity: float  # SI: rad/s
    current: float   # Milliamperes (mA)


@dataclass(frozen=True)
class BinaryPacket:
    """Decoded representation of one 183-byte protocol packet."""

    cmd: int
    flags: int
    address: int
    length: int
    count: int
    read_address: int
    read_length: int
    motors: tuple[tuple[int, bytes], ...]


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


HW_ERROR_NAMES = {
    0x01: "INPUT_VOLTAGE_ERROR",
    0x04: "OVERHEATING_ERROR",
    0x08: "MOTOR_ENCODER_ERROR",
    0x10: "ELECTRICAL_SHOCK_ERROR",
    0x20: "OVERLOAD_ERROR",
}


def decode_hardware_error(hw_err: int) -> list[str]:
    """Decode Dynamixel Hardware Error Status register bits."""
    errors = [name for bit, name in HW_ERROR_NAMES.items() if (hw_err & bit)]
    return errors or ["HARDWARE_ALERT"]


ERROR_REASONS = {
    -1: "NO_MOTOR_IDS",
    -2: "TORQUE_SAFETY",
    -3: "UNKNOWN_COMMAND",
    -4: "SYNC_READ_TIMEOUT",
    -5: "MOTOR_HARDWARE_ERROR",
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


def _to_raw(item: ControlTableItem | str, value: int | float | bytes) -> int | bytes:
    """Convert an API value to a Dynamixel register integer or bytes."""
    if isinstance(value, bytes):
        return value
    control_table_item, _ = _item_metadata(item)
    scale = control_table_item.scale
    if scale is None:
        if not isinstance(value, bytes):
            _validate_value(value)
        return value
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(
            f"{control_table_item.name} value must be a number in SI units"
        )
    raw_value = int(round(value / scale)) + control_table_item.offset
    _validate_value(raw_value)
    return raw_value


def _from_raw(item: DataNames, value: int) -> int | float:
    """Convert a Dynamixel register integer to an API value."""
    scale = item.scale
    if scale is None:
        return value
    return (value - item.offset) * scale


def _pack_motor_val(size: int, value: int | float | bytes) -> bytes:
    """Pack an integer or byte value into a fixed 10-byte (80-bit) buffer."""
    if isinstance(value, bytes):
        return value.ljust(10, b"\x00")[:10]
    int_val = int(value)
    if size == 1:
        b = struct.pack("<b", int_val)
    elif size == 2:
        b = struct.pack("<h", int_val)
    else:
        b = struct.pack("<i", int_val)
    return b.ljust(10, b"\x00")


def _unpack_value(length: int, raw_bytes: bytes) -> int:
    """Extract a 1, 2, or 4-byte signed integer from raw motor bytes."""
    if length == 1:
        return struct.unpack_from("<b", raw_bytes)[0]
    elif length == 2:
        return struct.unpack_from("<h", raw_bytes)[0]
    else:
        return struct.unpack_from("<i", raw_bytes)[0]


def _build_packet(
    command: int,
    item: ControlTableItem | str,
    count: int,
    motors: list[tuple[int, int | bytes]],
    read_item: ControlTableItem | str | None = None,
    flags: int = FLAG_NONE,
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
    motor_parts = []
    for motor_id, value in padded_motors:
        _validate_motor_id(motor_id)
        if isinstance(value, int):
            _validate_value(value)
        val_bytes = _pack_motor_val(size, value)
        motor_parts.append(motor_id)
        motor_parts.append(val_bytes)

    packet = struct.pack(
        PACKET_FORMAT,
        command,
        flags,
        control_table_item.address,
        size,
        count,
        read_control_table_item.address,
        read_size,
        *motor_parts,
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
        for index in range(7, len(unpacked), 2)
    )
    return BinaryPacket(
        unpacked[0], unpacked[1], unpacked[2], unpacked[3], unpacked[4], unpacked[5], unpacked[6], motors
    )


def build_read_all_command(item: ControlTableItem | str, ignore_errors: bool = False) -> bytes:
    flags = FLAG_IGNORE_ERRORS if ignore_errors else FLAG_NONE
    return _build_packet(READ_ALL, item, MOTOR_COUNT, list(enumerate([0] * MOTOR_COUNT)), flags=flags)


def build_write_all_command(
    item: ControlTableItem | str,
    values: list[int | float] | tuple[int | float, ...],
    ignore_errors: bool = False,
) -> bytes:
    if len(values) != 16:
        raise ValueError("WRITE_ALL requires exactly 16 values")
    raw_values = [_to_raw(item, value) for value in values]
    flags = FLAG_IGNORE_ERRORS if ignore_errors else FLAG_NONE
    return _build_packet(WRITE_ALL, item, MOTOR_COUNT, list(enumerate(raw_values)), flags=flags)


def build_read_command(
    item: ControlTableItem | str,
    motor_ids: list[int] | tuple[int, ...],
    ignore_errors: bool = False,
) -> bytes:
    if not motor_ids:
        raise ValueError("READ requires at least one motor ID")
    for motor_id in motor_ids:
        _validate_motor_id(motor_id)
    flags = FLAG_IGNORE_ERRORS if ignore_errors else FLAG_NONE
    return _build_packet(READ, item, len(motor_ids), [(motor_id, 0) for motor_id in motor_ids], flags=flags)


def build_write_command(
    item: ControlTableItem | str,
    values_by_motor: dict[int, int | float],
    ignore_errors: bool = False,
) -> bytes:
    if not values_by_motor:
        raise ValueError("WRITE requires at least one motor ID and value")
    parts = []
    for motor_id, value in values_by_motor.items():
        _validate_motor_id(motor_id)
        parts.append((motor_id, _to_raw(item, value)))
    flags = FLAG_IGNORE_ERRORS if ignore_errors else FLAG_NONE
    return _build_packet(WRITE, item, len(parts), parts, flags=flags)


def build_write_read_all_command(
    write_item: ControlTableItem | str,
    values: list[int | float] | tuple[int | float, ...],
    read_item: ControlTableItem | str | None = None,
    ignore_errors: bool = False,
) -> bytes:
    if len(values) != 16:
        raise ValueError("WRITE_READ_ALL requires exactly 16 values")
    raw_values = [_to_raw(write_item, value) for value in values]
    flags = FLAG_IGNORE_ERRORS if ignore_errors else FLAG_NONE
    return _build_packet(
        WRITE_READ_ALL,
        write_item,
        MOTOR_COUNT,
        list(enumerate(raw_values)),
        read_item=read_item,
        flags=flags,
    )


def build_write_read_command(
    write_item: ControlTableItem | str,
    values_by_motor: dict[int, int | float],
    read_item: ControlTableItem | str | None = None,
    ignore_errors: bool = False,
) -> bytes:
    if not values_by_motor:
        raise ValueError("WRITE_READ requires at least one motor ID and value")
    parts = []
    for motor_id, value in values_by_motor.items():
        _validate_motor_id(motor_id)
        parts.append((motor_id, _to_raw(write_item, value)))
    flags = FLAG_IGNORE_ERRORS if ignore_errors else FLAG_NONE
    return _build_packet(
        WRITE_READ,
        write_item,
        len(parts),
        parts,
        read_item=read_item,
        flags=flags,
    )

def build_reboot_command(
    target: int = REBOOT_BOARD,
    motor_ids: list[int] | tuple[int, ...] | None = None,
) -> bytes:
    """Build a 183-byte REBOOT command packet.

    Args:
        target: Target to reboot: REBOOT_BOARD (0) for OpenRB MCU reset,
            REBOOT_MOTORS (1) for Dynamixel motors, or REBOOT_ALL (2) for both.
        motor_ids: For REBOOT_MOTORS, optional list of motor IDs (0-15) to reboot.
            If None or empty, all motors are rebooted (broadcast).
    """
    if target not in (REBOOT_BOARD, REBOOT_MOTORS, REBOOT_ALL):
        raise ValueError(
            f"target must be REBOOT_BOARD ({REBOOT_BOARD}), "
            f"REBOOT_MOTORS ({REBOOT_MOTORS}), or REBOOT_ALL ({REBOOT_ALL})"
        )
    parts = []
    if motor_ids:
        for motor_id in motor_ids:
            _validate_motor_id(motor_id)
            parts.append((motor_id, 0))
    count = len(parts)
    padded_motors = parts + [(0, 0)] * (MOTOR_COUNT - count)
    motor_parts = []
    for motor_id, value in padded_motors:
        val_bytes = _pack_motor_val(4, value)
        motor_parts.append(motor_id)
        motor_parts.append(val_bytes)
    packet = struct.pack(
        PACKET_FORMAT,
        REBOOT,
        FLAG_NONE,
        target,
        0,
        count,
        0,
        0,
        *motor_parts,
    )
    if len(packet) != PACKET_SIZE:
        raise AssertionError(f"binary packet has unexpected size: {len(packet)}")
    return packet

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
        motor_id = response.motors[0][0]
        raw_val = response.motors[0][1]
        error_code = struct.unpack("<i", raw_val[:4])[0] if isinstance(raw_val, bytes) else raw_val

        if error_code <= -200:
            dxl_err = -error_code - 200
            reason = f"MOTOR_PROTOCOL_ERROR (error byte 0x{dxl_err:02X})"
        elif error_code <= -100:
            hw_err = -error_code - 100
            hw_names = decode_hardware_error(hw_err)
            reason = f"MOTOR_HARDWARE_ERROR: {', '.join(hw_names)} (register 70 = 0x{hw_err:02X})"
        else:
            reason = ERROR_REASONS.get(error_code, f"DYNAMIXEL_ERROR_{error_code}")

        if motor_id != 255:
            reason = f"{reason} on motor {motor_id}"

        raise ControlTableError(reason, response)

    def _validate_response_header(
        self, request: BinaryPacket, response: BinaryPacket, ignore_errors: bool = False
    ) -> None:
        if not ignore_errors:
            self._response_error(response)
        elif response.cmd == ERROR:
            return
        if response.cmd != request.cmd:
            raise ControlTableProtocolError(
                f"Expected response command {request.cmd}, got {response.cmd}"
            )
        if (response.address, response.length) != (request.address, request.length):
            raise ControlTableProtocolError("Response address or length did not match request")
        if (response.read_address, response.read_length) != (request.read_address, request.read_length):
            raise ControlTableProtocolError("Response read address or length did not match request")

    def _read_values(
        self, command: bytes, motor_ids: list[int], ignore_errors: bool = False
    ) -> dict[int, int | float]:
        request = unpack_packet(command)
        response = unpack_packet(self._request(command))
        self._validate_response_header(request, response, ignore_errors=ignore_errors)
        if response.cmd == ERROR:
            return {}
        if response.count != len(motor_ids) and not ignore_errors:
            raise ControlTableProtocolError(
                f"Expected {len(motor_ids)} read values, got {response.count}"
            )
        returned_motors = response.motors[: response.count]
        returned_ids = [motor_id for motor_id, _ in returned_motors]
        if returned_ids != motor_ids and not ignore_errors:
            raise ControlTableProtocolError(
                f"Expected motor IDs {motor_ids}, got {returned_ids}"
            )
        if request.read_length == 10:
            result = {}
            for motor_id, raw_bytes in returned_motors:
                raw_curr, raw_vel, raw_pos = struct.unpack("<hii", raw_bytes)
                result[motor_id] = FullMotorState(
                    position=(raw_pos - POSITION_CENTER) * POSITION_SCALE,
                    velocity=raw_vel * VELOCITY_SCALE,
                    current=float(raw_curr),
                )
            return result

        read_item = DataNames(request.read_address)
        return {
            motor_id: _from_raw(read_item, _unpack_value(request.read_length, raw_bytes))
            for motor_id, raw_bytes in returned_motors
        }

    def _write(self, command: bytes, ignore_errors: bool = False) -> None:
        request = unpack_packet(command)
        response = unpack_packet(self._request(command))
        self._validate_response_header(request, response, ignore_errors=ignore_errors)
        if response.cmd == ERROR:
            return
        if response.count != request.count and not ignore_errors:
            raise ControlTableProtocolError(
                f"Expected {request.count} acknowledged writes, got {response.count}"
            )
        if response.motors[: response.count] != request.motors[: request.count] and not ignore_errors:
            raise ControlTableProtocolError("Write acknowledgement did not echo the request")

    def read_all(
        self, item: ControlTableItem | str, ignore_errors: bool = False
    ) -> dict[int, int | float]:
        """
        Read the value of a control table item for all motors.

        Args:
            item: The control table item to read.
            ignore_errors: If True, do not raise errors on motor faults or timeouts.

        Returns:
            Motor ID to value. Position/velocity/acceleration items are SI
            (rad, rad/s, rad/s²); other items are raw integers.
        """
        return self._read_values(
            build_read_all_command(item, ignore_errors=ignore_errors),
            list(range(16)),
            ignore_errors=ignore_errors,
        )

    def write_all(
        self,
        item: ControlTableItem | str,
        values: list[int | float] | tuple[int | float, ...],
        ignore_errors: bool = False,
    ) -> None:
        """
        Write a control-table value for all 16 motors.

        Args:
            item: The control-table item to write.
            values: Exactly 16 values, ordered by motor ID 0 through 15.
                Use SI units for scaled items (see DataNames ``SI:`` comments).
            ignore_errors: If True, do not block on torque safety or raise on errors.

        Raises:
            ValueError: If values does not contain exactly 16 entries.
            ControlTableError: If the OpenRB returns an ERR response (when ignore_errors=False).
        """
        self._write(
            build_write_all_command(item, values, ignore_errors=ignore_errors),
            ignore_errors=ignore_errors,
        )

    def read(
        self,
        item: ControlTableItem | str,
        motor_ids: list[int] | tuple[int, ...],
        ignore_errors: bool = False,
    ) -> dict[int, int | float]:
        """
        Read the value of a control table item for multiple motors.

        Args:
            item: The control table item to read.
            motor_ids: A list of motor IDs to read from, in the range 0-15.
            ignore_errors: If True, do not raise errors on motor faults or timeouts.

        Returns:
            Motor ID to value. Position/velocity/acceleration items are SI
            (rad, rad/s, rad/s²); other items are raw integers.
        """
        return self._read_values(
            build_read_command(item, motor_ids, ignore_errors=ignore_errors),
            list(motor_ids),
            ignore_errors=ignore_errors,
        )

    def write(
        self,
        item: ControlTableItem | str,
        values_by_motor: dict[int, int | float],
        ignore_errors: bool = False,
    ) -> None:
        """
        Write the value of a control table item for multiple motors.

        Args:
            item: The control table item to write.
            values_by_motor: Motor ID to value. Use SI units for scaled items.
            ignore_errors: If True, do not block on torque safety or raise on errors.

        Raises:
            ControlTableError: If the OpenRB returns an ERR response (when ignore_errors=False).
        """
        self._write(
            build_write_command(item, values_by_motor, ignore_errors=ignore_errors),
            ignore_errors=ignore_errors,
        )

    def write_read_all(
        self,
        write_item: ControlTableItem | str,
        values: list[int | float] | tuple[int | float, ...],
        read_item: ControlTableItem | str | None = None,
        ignore_errors: bool = False,
    ) -> dict[int, int | float]:
        """
        Write a control-table value for all 16 motors and return their read values.

        Args:
            write_item: The control-table item to write.
            values: Exactly 16 values, ordered by motor ID 0 through 15.
                Use SI units for scaled items.
            read_item: The control-table item to read. Defaults to write_item if None.
            ignore_errors: If True, do not block on torque safety or raise on errors.

        Returns:
            Motor ID to value. Scaled items are returned in SI units.

        Raises:
            ValueError: If values does not contain exactly 16 entries.
            ControlTableError: If the OpenRB returns an ERR response (when ignore_errors=False).
        """
        return self._read_values(
            build_write_read_all_command(
                write_item, values, read_item=read_item, ignore_errors=ignore_errors
            ),
            list(range(16)),
            ignore_errors=ignore_errors,
        )

    def write_read(
        self,
        write_item: ControlTableItem | str,
        values_by_motor: dict[int, int | float],
        read_item: ControlTableItem | str | None = None,
        ignore_errors: bool = False,
    ) -> dict[int, int | float]:
        """
        Write the value of a control table item for multiple motors and return their read values.

        Args:
            write_item: The control table item to write.
            values_by_motor: Motor ID to value. Use SI units for scaled items.
            read_item: The control-table item to read. Defaults to write_item if None.
            ignore_errors: If True, do not block on torque safety or raise on errors.

        Returns:
            Motor ID to value. Scaled items are returned in SI units.

        Raises:
            ValueError: If values_by_motor is empty.
            ControlTableError: If the OpenRB returns an ERR response (when ignore_errors=False).
        """
        return self._read_values(
            build_write_read_command(
                write_item, values_by_motor, read_item=read_item, ignore_errors=ignore_errors
            ),
            list(values_by_motor.keys()),
            ignore_errors=ignore_errors,
        )

    def reboot(
        self,
        target: int = REBOOT_BOARD,
        motor_ids: list[int] | tuple[int, ...] | None = None,
        wait_for_reconnect: bool = True,
        timeout: float = 10.0,
    ) -> bool:
        """
        Send a reboot command to the OpenRB-150 board and/or Dynamixel motors.

        Args:
            target: REBOOT_BOARD (0) for OpenRB MCU reset,
                    REBOOT_MOTORS (1) for Dynamixel motors reboot,
                    REBOOT_ALL (2) for motors reboot + OpenRB MCU reset.
            motor_ids: Specific motor IDs to reboot (only used when target is REBOOT_MOTORS).
                       If None or empty, reboots all motors.
            wait_for_reconnect: If True and target reboots OpenRB (REBOOT_BOARD or REBOOT_ALL),
                                waits for the board to finish rebooting and respond to ping.
            timeout: Maximum seconds to wait for reconnection.

        Returns:
            True if reboot (and reconnection, if enabled) succeeded.

        Raises:
            ControlTableTimeout: If wait_for_reconnect is True and OpenRB does not come
                                 back online within the timeout.
            ControlTableError: If the OpenRB returns an error packet.
        """
        command = build_reboot_command(target=target, motor_ids=motor_ids)
        command_text = command.hex(" ")
        self.socket.sendto(command, self.address)

        # Attempt to read ACK response from OpenRB before it resets
        orig_timeout = self.socket.gettimeout()
        try:
            self.socket.settimeout(1.0)
            response_bytes, _ = self.socket.recvfrom(1024)
            self._log(command_text, response_bytes.hex(" "))
            response = unpack_packet(response_bytes)
            if response.cmd == ERROR:
                self._response_error(response)
        except (socket.timeout, ConnectionResetError):
            # If the board reset immediately without an ACK response, that's acceptable
            self._log(command_text, "NO_ACK_OR_RESET")
        finally:
            self.socket.settimeout(orig_timeout)

        # If only rebooting motors, the board does not reset
        if target == REBOOT_MOTORS:
            return True

        if not wait_for_reconnect:
            return True

        # Wait for OpenRB to reset and come back online
        # OpenRB setup() takes ~2.0s (delay(2000) + Ethernet initialization)
        time.sleep(2.0)
        start_wait = time.monotonic()
        poll_interval = 0.3

        while time.monotonic() - start_wait < timeout:
            try:
                res = self.read(DataNames.MODEL_NUMBER, [0], ignore_errors=True)
                if res is not None:
                    return True
            except (ControlTableTimeout, socket.timeout, ConnectionResetError, OSError):
                pass
            time.sleep(poll_interval)

        raise ControlTableTimeout(
            f"OpenRB at {self.address[0]}:{self.address[1]} did not come back online within {timeout}s after reboot"
        )

