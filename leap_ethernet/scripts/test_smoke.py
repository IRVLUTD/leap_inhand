"""Hardware smoke test for the OpenRB-150 control-table client.

Sets current limit, position mode, and torque, commands a 0 rad (center)
goal, waits, then prints present positions. Torque is turned off on exit.

Run from leap_ethernet/:

    uv run python scripts/test_smoke.py
    uv run python scripts/test_smoke.py --motor 15

Arguments:
    --motor ID    Test a single motor (0-15) instead of all 16.
    -h, --help    Show argparse help.
"""

import argparse
import time

from client import ControlTableClient, DataNames, LOG_DIR, MOTOR_COUNT

GOAL_POSITION = 0.0  # rad; Dynamixel center (2048 ticks)
CURRENT_LIMIT_MA = 150
MOVE_WAIT_S = 2.0
POSITION_MODE = 3


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Smoke-test the control-table client."
    )
    parser.add_argument(
        "--motor",
        type=int,
        metavar="ID",
        help="Test a single motor ID (0-15). Default: all 16 motors.",
    )
    return parser.parse_args()


def run_all_motors(client: ControlTableClient) -> None:
    print(f"Setting current limit of all motors to {CURRENT_LIMIT_MA} mA")
    client.write_all(DataNames.CURRENT_LIMIT, [CURRENT_LIMIT_MA] * MOTOR_COUNT)

    print("Setting motors to position mode")
    client.write_all(DataNames.OPERATING_MODE, [POSITION_MODE] * MOTOR_COUNT)

    print("Enabling torque on all motors")
    client.write_all(DataNames.TORQUE_ENABLE, [1] * MOTOR_COUNT)

    print(f"Setting all goal positions to {GOAL_POSITION:.4f} rad")
    client.write_all(DataNames.GOAL_POSITION, [GOAL_POSITION] * MOTOR_COUNT)

    print(f"Waiting {MOVE_WAIT_S:.1f} seconds for the motors to move")
    time.sleep(MOVE_WAIT_S)

    print("Reading all motors' present positions")
    positions = client.read_all(DataNames.PRESENT_POSITION)
    for motor_id in range(MOTOR_COUNT):
        print(f"Motor {motor_id} present position: {positions[motor_id]:.4f} rad")


def run_one_motor(client: ControlTableClient, motor_id: int) -> None:
    print(f"Setting motor {motor_id} to position mode")
    client.write(DataNames.OPERATING_MODE, {motor_id: POSITION_MODE})

    print(f"Enabling torque on motor {motor_id}")
    client.write(DataNames.TORQUE_ENABLE, {motor_id: 1})

    print(f"Setting goal position to {GOAL_POSITION:.4f} rad")
    client.write(DataNames.GOAL_POSITION, {motor_id: GOAL_POSITION})

    print(f"Waiting {MOVE_WAIT_S:.1f} seconds for the motor to move")
    time.sleep(MOVE_WAIT_S)

    print("Reading motor present position")
    positions = client.read(DataNames.PRESENT_POSITION, [motor_id])
    print(f"Motor {motor_id} present position: {positions[motor_id]:.4f} rad")


def disable_torque(
    client: ControlTableClient, motor_id: int | None
) -> None:
    if motor_id is None:
        print("Disabling torque on all motors")
        client.write_all(DataNames.TORQUE_ENABLE, [0] * MOTOR_COUNT)
    else:
        print(f"Disabling torque on motor {motor_id}")
        client.write(DataNames.TORQUE_ENABLE, {motor_id: 0})


def main() -> None:
    args = parse_args()
    motor_id: int | None = args.motor
    if motor_id is not None and not 0 <= motor_id < MOTOR_COUNT:
        raise SystemExit(f"--motor must be between 0 and {MOTOR_COUNT - 1}")

    LOG_DIR.mkdir(parents=True, exist_ok=True)
    client = ControlTableClient(log_path=str(LOG_DIR / "control_table_log.csv"))
    try:
        if motor_id is None:
            run_all_motors(client)
        else:
            run_one_motor(client, motor_id)
    finally:
        try:
            disable_torque(client, motor_id)
        except Exception as error:
            print(f"Failed to disable torque: {error}")
        client.close()


if __name__ == "__main__":
    main()
