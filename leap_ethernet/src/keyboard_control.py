"""Shared keyboard control for the 16-motor LEAP hand.

Joint order is Index, Middle, Ring, Thumb; each finger is
MCP side, MCP forward, PIP, DIP (IDs 0-15).
"""

from collections.abc import Callable
from math import pi

from client import ControlTableClient, DataNames, LOG_DIR, MOTOR_COUNT

CURRENT_LIMIT_MA = 150
POSITION_MODE = 3
POSITION_P_GAIN = 400
POSITION_I_GAIN = 0
POSITION_D_GAIN = 0
FINGERS = ("Index", "Middle", "Ring", "Thumb")
AXES = ("MCP-S", "MCP-F", "PIP", "DIP")
JOINT_LABELS = [
    f"{finger} {axis}" for finger in FINGERS for axis in AXES
]

# Open: fingers at encoder center; thumb MCP-forward matches leaphand_node home.
OPEN_POSITION = [
    0.0, 0.0, 0.0, 0.0,  # Index
    0.0, 0.0, 0.0, 0.0,  # Middle
    0.0, 0.0, 0.0, 0.0,  # Ring
    0.0, pi / 2, 0.0, 0.0,  # Thumb
]

# Closed: MCP-side stays near 0; remaining joints curl. Thumb opposes slightly.
CLOSED_POSITION = [
    0.0, 0.5, 0.5, 0.5,  # Index
    0.0, 0.5, 0.5, 0.5,  # Middle
    0.0, 0.5, 0.5, 0.5,  # Ring
    0.5, pi / 2, 0.5, 0.5,  # Thumb
]


def print_help(torque_enabled: bool = False) -> None:
    print("Hand keyboard control")
    print("  T: toggle torque")
    print("  I: move to open position")
    print("  C: move to closed position")
    print("  R: read present positions")
    print("  H: reprint this help")
    print("  Q: quit (torque off)")
    print(f"Torque is currently {'on' if torque_enabled else 'off'}.")


def print_positions(positions: dict[int, float]) -> None:
    print("Present positions (rad)")
    print(f"  {'ID':>2}  {'Joint':<14}  {'rad':>8}")
    for motor_id in range(MOTOR_COUNT):
        print(
            f"  {motor_id:2d}  {JOINT_LABELS[motor_id]:<14}  "
            f"{float(positions[motor_id]):8.4f}"
        )


def configure_motors(client: ControlTableClient) -> None:
    print(f"Setting current limit of all motors to {CURRENT_LIMIT_MA} mA")
    client.write_all(DataNames.CURRENT_LIMIT, [CURRENT_LIMIT_MA] * MOTOR_COUNT)

    print("Setting return delay time of all motors to 0")
    client.write_all(DataNames.RETURN_DELAY_TIME, [0] * MOTOR_COUNT)

    print("Setting motors to position mode")
    client.write_all(DataNames.OPERATING_MODE, [POSITION_MODE] * MOTOR_COUNT)

    print(
        "Setting position PID "
        f"P={POSITION_P_GAIN} I={POSITION_I_GAIN} D={POSITION_D_GAIN}"
    )
    client.write_all(DataNames.POSITION_P_GAIN, [POSITION_P_GAIN] * MOTOR_COUNT)
    client.write_all(DataNames.POSITION_I_GAIN, [POSITION_I_GAIN] * MOTOR_COUNT)
    client.write_all(DataNames.POSITION_D_GAIN, [POSITION_D_GAIN] * MOTOR_COUNT)


def disable_torque(client: ControlTableClient) -> None:
    print("Disabling torque on all motors")
    client.write_all(DataNames.TORQUE_ENABLE, [0] * MOTOR_COUNT)


def set_position(
    client: ControlTableClient,
    position: list[float],
    torque_enabled: bool,
    name: str,
) -> None:
    if not torque_enabled:
        print("Torque isn't on; enable torque to run the hand.")
        return
    if len(position) != MOTOR_COUNT:
        print(f"{name} must contain exactly {MOTOR_COUNT} values")
        return
    client.write_all(DataNames.GOAL_POSITION, position)
    print(f"Set {name.lower()} position.")


def run(read_key: Callable[[], str | None]) -> None:
    """Run the control loop. ``read_key`` returns a lowercase letter or None."""
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    client = ControlTableClient(log_path=str(LOG_DIR / "hand_control_log.csv"))
    torque_enabled = False
    try:
        configure_motors(client)
        print_help(torque_enabled)
        while True:
            key = read_key()
            if key is None:
                continue
            if key == "t":
                torque_enabled = not torque_enabled
                client.write_all(
                    DataNames.TORQUE_ENABLE,
                    [int(torque_enabled)] * MOTOR_COUNT,
                )
                print(f"Torque {'enabled' if torque_enabled else 'disabled'}.")
            elif key == "i":
                set_position(client, OPEN_POSITION, torque_enabled, "open")
            elif key == "c":
                set_position(
                    client, CLOSED_POSITION, torque_enabled, "closed"
                )
            elif key == "r":
                print_positions(client.read_all(DataNames.PRESENT_POSITION))
            elif key == "h":
                print_help(torque_enabled)
            elif key == "q":
                print("Exiting hand control.")
                break
            else:
                print(f"Unknown key: {key!r}")
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        try:
            disable_torque(client)
        except Exception as error:
            print(f"Failed to disable torque: {error}")
        client.close()
