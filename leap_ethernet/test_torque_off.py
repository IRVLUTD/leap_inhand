"""Exercise the control-table client with motor 15."""

import time

from control_table_client import ControlTableClient, DataNames

def main() -> None:
    client = ControlTableClient(log_path="control_table_log.csv")
    try:
        print(f"Setting motor to position mode")
        client.write(DataNames.OPERATING_MODE, {15: 3})

        print(f"Enabling torque on motor")
        client.write(DataNames.TORQUE_ENABLE, {15: 1})

        print(f"Setting goal position to {2048}")
        client.write(DataNames.GOAL_POSITION, {15: 2048})

        print(f"Waiting {2.0:.1f} seconds for the motors to move")
        time.sleep(2.0)

        print(f"Reading motor present position")
        positions = client.read(DataNames.PRESENT_POSITION, [15])
        print(f"Motor present position: {positions[15]}")

    finally:
        client.close()


if __name__ == "__main__":
    main()
