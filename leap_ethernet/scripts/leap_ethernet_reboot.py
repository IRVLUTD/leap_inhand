#!/usr/bin/env python3
"""Reboot utility for LEAP Hand OpenRB-150 board and Dynamixel motors over Ethernet.

Can reboot:
  1. The OpenRB-150 board (hardware MCU reset via NVIC_SystemReset)
  2. Dynamixel motors (clearing hardware faults, red LED, and overload errors)
  3. Both motors and the OpenRB-150 board

Usage:
    # Reboot OpenRB-150 board and wait for reconnection:
    python3 scripts/leap_ethernet_reboot.py

    # Reboot all Dynamixel motors only (board stays running):
    python3 scripts/leap_ethernet_reboot.py --target motors

    # Reboot specific motors (e.g., motor 0 and 1):
    python3 scripts/leap_ethernet_reboot.py --target motors --motors 0 1

    # Reboot motors and OpenRB-150 board:
    python3 scripts/leap_ethernet_reboot.py --target all

    # Reboot without waiting for reconnection:
    python3 scripts/leap_ethernet_reboot.py --no-wait
"""

import argparse
from pathlib import Path
import sys
import time

# Ensure leap_ethernet/src is accessible
_SRC_DIR = Path(__file__).resolve().parent.parent / "src"
if str(_SRC_DIR) not in sys.path:
    sys.path.insert(0, str(_SRC_DIR))

from client import (
    ControlTableClient,
    ControlTableError,
    ControlTableTimeout,
    DEFAULT_IP,
    DEFAULT_PORT,
    MOTOR_COUNT,
    REBOOT_ALL,
    REBOOT_BOARD,
    REBOOT_MOTORS,
)

TARGET_MAP = {
    "board": REBOOT_BOARD,
    "motors": REBOOT_MOTORS,
    "all": REBOOT_ALL,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Reboot the LEAP Hand OpenRB-150 board and/or Dynamixel motors over UDP."
    )
    parser.add_argument(
        "--ip",
        type=str,
        default=DEFAULT_IP,
        help=f"Target IP address of the OpenRB-150 (default: {DEFAULT_IP}).",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=DEFAULT_PORT,
        help=f"Target UDP port of the OpenRB-150 (default: {DEFAULT_PORT}).",
    )
    parser.add_argument(
        "--target",
        type=str,
        choices=["board", "motors", "all"],
        default="all",
        help="Reboot target: 'board' (OpenRB MCU reset), 'motors' (reboot Dynamixels), or 'all' (both). Default: 'all'.",
    )
    parser.add_argument(
        "--motors",
        type=int,
        nargs="+",
        metavar="ID",
        help="Specific motor IDs to reboot (0-15). Only applies when --target is 'motors'. Defaults to all motors.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="Timeout in seconds to wait for reconnection after reboot (default: 10.0s).",
    )
    parser.add_argument(
        "--no-wait",
        action="store_true",
        help="Send the reboot command without waiting for OpenRB reconnection.",
    )
    parser.add_argument(
        "--raw-udp",
        action="store_true",
        help="Bypass the ROS /leap_hand/reboot service and send raw UDP command directly to OpenRB.",
    )
    return parser.parse_args()


def try_call_ros_service() -> bool:
    """Attempt to call /leap_hand/reboot if ROS is running and the service is available."""
    try:
        import rosgraph
        if not rosgraph.is_master_online():
            return False

        import rospy
        from std_srvs.srv import Trigger

        if not rospy.core.is_initialized():
            rospy.init_node("leap_ethernet_reboot_cli", anonymous=True, disable_signals=True)
        service_name = "/leap_hand/reboot"
        rospy.wait_for_service(service_name, timeout=0.5)
        reboot_srv = rospy.ServiceProxy(service_name, Trigger)
        print(f"Calling ROS service {service_name} on running leap_ethernet_node...")
        resp = reboot_srv()
        if resp.success:
            print(f"[SUCCESS] {resp.message}")
            return True
        else:
            print(f"[FAILED] ROS Service failed: {resp.message}")
            return False
    except Exception:
        return False


def main() -> None:
    args = parse_args()

    # Validate motor IDs if specified
    if args.motors:
        for mid in args.motors:
            if not 0 <= mid < MOTOR_COUNT:
                print(
                    f"Error: Invalid motor ID {mid}. Must be between 0 and {MOTOR_COUNT - 1}.",
                    file=sys.stderr,
                )
                sys.exit(1)

    target_code = TARGET_MAP[args.target]

    # If leap_ethernet_node is running, prefer calling its reboot service to reset state & reconfigure
    if not args.raw_udp and args.target in ("all", "board") and not args.motors:
        if try_call_ros_service():
            sys.exit(0)

    print(
        f"Connecting to OpenRB-150 at {args.ip}:{args.port} (target: {args.target})..."
    )
    client = ControlTableClient(ip=args.ip, port=args.port, timeout=2.0)

    try:
        start_time = time.monotonic()
        if args.target == "motors":
            if args.motors:
                print(f"Rebooting motors {args.motors}...")
            else:
                print("Broadcasting reboot instruction to all 16 motors...")
        elif args.target == "board":
            print("Sending system reset command to OpenRB-150...")
        else:
            print("Rebooting all motors and resetting OpenRB-150...")

        client.reboot(
            target=target_code,
            motor_ids=args.motors,
            wait_for_reconnect=not args.no_wait,
            timeout=args.timeout,
        )

        elapsed = time.monotonic() - start_time
        if args.no_wait or args.target == "motors":
            print(f"[SUCCESS] Reboot command sent successfully ({elapsed:.2f}s).")
        else:
            print(
                f"[SUCCESS] OpenRB-150 rebooted and reconnected successfully in {elapsed:.2f}s!"
            )

    except ControlTableTimeout as e:
        print(f"[ERROR] Timeout during reboot: {e}", file=sys.stderr)
        sys.exit(1)
    except ControlTableError as e:
        print(f"[ERROR] Control table error from OpenRB: {e}", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"[ERROR] Unexpected error: {e}", file=sys.stderr)
        sys.exit(1)
    finally:
        client.close()


if __name__ == "__main__":
    main()
