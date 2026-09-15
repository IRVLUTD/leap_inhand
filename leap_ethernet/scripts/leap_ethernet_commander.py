#!/usr/bin/env python3
"""Example publisher node for LEAP hand joint commands.

Publishes sensor_msgs/JointState messages to /leap_hand/cmd with smooth
sinusoidal open/close trajectories to test the hand's motion.
"""

import math
from pathlib import Path
import sys

_SRC_DIR = Path(__file__).resolve().parent.parent / "src"
if str(_SRC_DIR) not in sys.path:
    sys.path.insert(0, str(_SRC_DIR))

from client import MOTOR_COUNT
import rospy
from sensor_msgs.msg import JointState

# Open: fingers centered at 0 rad; thumb MCP-F at pi/2 rad
OPEN_POSITION = [
    0.0, 0.0, 0.0, 0.0,        # Index: MCP-S, MCP-F, PIP, DIP
    0.0, 0.0, 0.0, 0.0,        # Middle: MCP-S, MCP-F, PIP, DIP
    0.0, 0.0, 0.0, 0.0,        # Ring: MCP-S, MCP-F, PIP, DIP
    0.0, math.pi / 2, 0.0, 0.0  # Thumb: MCP-S, MCP-F, PIP, DIP
]

# Closed: finger MCP-F, PIP, DIP curled to 0.6 rad; thumb opposes
CLOSED_POSITION = [
    0.0, 0.6, 0.6, 0.6,        # Index
    0.0, 0.6, 0.6, 0.6,        # Middle
    0.0, 0.6, 0.6, 0.6,        # Ring
    0.4, math.pi / 2, 0.5, 0.5  # Thumb
]


class LeapCommanderNode:
    """Publishes periodic joint trajectory commands to the LEAP hand."""

    def __init__(self) -> None:
        rospy.init_node("leap_ethernet_commander")

        self.cmd_topic = rospy.get_param("~cmd_topic", "/leap_hand/cmd")
        self.state_topic = rospy.get_param("~state_topic", "/leap_hand/state")
        self.frequency = float(rospy.get_param("~frequency", 30.0))
        self.cycle_time = float(rospy.get_param("~cycle_time", 4.0))  # seconds per open/close cycle
        self.mode = rospy.get_param("~mode", "sine")  # "sine", "open", "close"

        self.joint_names = [f"joint_{i}" for i in range(MOTOR_COUNT)]

        self.cmd_pub = rospy.Publisher(
            self.cmd_topic, JointState, queue_size=1
        )
        self.state_sub = rospy.Subscriber(
            self.state_topic, JointState, self._state_callback, queue_size=1
        )

        self.latest_state: list[float] | None = None

        rospy.loginfo(
            f"LeapCommanderNode started. Publishing to {self.cmd_topic} at "
            f"{self.frequency:.1f} Hz (mode: '{self.mode}', cycle: {self.cycle_time:.1f}s)."
        )

    def _state_callback(self, msg: JointState) -> None:
        if len(msg.position) == MOTOR_COUNT:
            self.latest_state = list(msg.position)

    def compute_positions(self, t: float) -> list[float]:
        """Compute target positions for timestamp t based on selected mode."""
        if self.mode == "open":
            return list(OPEN_POSITION)
        elif self.mode == "close":
            return list(CLOSED_POSITION)
        else:
            # Default "sine": smoothly oscillates between open (alpha=0) and closed (alpha=1)
            # alpha in [0, 1] using cosine: 0 at t=0, 1 at t=cycle/2, 0 at t=cycle
            alpha = 0.5 * (1.0 - math.cos(2.0 * math.pi * t / self.cycle_time))
            return [
                OPEN_POSITION[i] + alpha * (CLOSED_POSITION[i] - OPEN_POSITION[i])
                for i in range(MOTOR_COUNT)
            ]

    def spin(self) -> None:
        rate = rospy.Rate(self.frequency)
        start_time = rospy.Time.now()

        # Wait a moment for connections to establish
        rospy.sleep(0.5)

        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            positions = self.compute_positions(elapsed)

            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            msg.name = self.joint_names
            msg.position = positions
            self.cmd_pub.publish(msg)

            rate.sleep()


def main() -> None:
    node = LeapCommanderNode()
    node.spin()


if __name__ == "__main__":
    main()
