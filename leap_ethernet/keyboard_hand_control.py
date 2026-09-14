"""Keyboard control for the 16-motor hand."""

import msvcrt
import time

from control_table_client import ControlTableClient, DataNames

MOTOR_COUNT = 16
INITIAL_POSITION = [2048] * MOTOR_COUNT
CLOSED_POSITION: list[int] | None = None


def validate_position(name: str, position: list[int] | None) -> list[int]:
    if position is None:
        raise ValueError(
            f"{name} is not configured yet. Add exactly {MOTOR_COUNT} values."
        )
    if len(position) != MOTOR_COUNT:
        raise ValueError(f"{name} must contain exactly {MOTOR_COUNT} values")
    return position


def set_position(
    client: ControlTableClient,
    position: list[int] | None,
    torque_enabled: bool,
    name: str,
) -> None:
    if not torque_enabled:
        print("Torque isn't on; enable torque to run the hand.")
        return
    try:
        validated_position = validate_position(name, position)
    except ValueError as error:
        print(error)
        return

    client.write_all(DataNames.GOAL_POSITION, validated_position)
    print(f"Set {name.lower()} position.")


def main() -> None:
    torque_enabled = False
    client = ControlTableClient(log_path="hand_control_log.csv")

    print("Hand keyboard control")
    print("  T: toggle torque")
    print("  I: move to initial position")
    print("  C: move to closed position")
    print("  R: read present positions")
    print("  Q: quit")
    print("Torque is currently off.")

    try:
        while True:
            key = msvcrt.getwch().lower()

            if key == "t":
                torque_enabled = not torque_enabled
                client.write_all(
                    DataNames.TORQUE_ENABLE,
                    [int(torque_enabled)] * MOTOR_COUNT,
                )
                print(f"Torque {'enabled' if torque_enabled else 'disabled'}.")
            elif key == "i":
                set_position(
                    client, INITIAL_POSITION, torque_enabled, "initial position"
                )
            elif key == "c":
                set_position(
                    client, CLOSED_POSITION, torque_enabled, "closed position"
                )
            elif key == "r":
                positions = client.read_all(DataNames.PRESENT_POSITION)
                print("Present positions:", positions)
            elif key == "q":
                print("Exiting hand control.")
                break
            else:
                print(f"Unknown key: {key!r}")
    finally:
        client.close()


if __name__ == "__main__":
    main()
