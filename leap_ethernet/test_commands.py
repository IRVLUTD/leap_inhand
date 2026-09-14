"""Exercise the control-table client with motor 15."""

import time

from control_table_client import ControlTableClient, DataNames

def main() -> None:
    client = ControlTableClient(log_path="control_table_log.csv")
    try:
        print(f"Setting current limit of all motors to {150} mA")
        client.write_all(DataNames.CURRENT_LIMIT, [150] * 16)

        print(f"Setting motors to position mode")
        client.write_all(DataNames.OPERATING_MODE, [3] * 16)

        print(f"Enabling torque on all motors")
        client.write_all(DataNames.TORQUE_ENABLE, [1] * 16)

        print(f"Setting all goal positions to {2048}")
        client.write_all(DataNames.GOAL_POSITION, [2048] * 16)

        print(f"Waiting {2.0:.1f} seconds for the motors to move")
        time.sleep(2.0)

        print(f"Reading all motors' present positions")
        positions = client.read_all(DataNames.PRESENT_POSITION)
        for i in range(16):
            print(f"Motor {i} present position: {positions[i]}")

        print(f"Disabling torque on all motors")
        client.write_all(DataNames.TORQUE_ENABLE, [0] * 16)
    finally:
        client.close()


if __name__ == "__main__":
    main()
