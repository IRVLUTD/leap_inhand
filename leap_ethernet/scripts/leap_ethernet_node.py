#!/usr/bin/env python3
"""ROS node for UDP Ethernet control of the LEAP Hand via OpenRB-150.

Subscribes to joint position commands (sensor_msgs/JointState) in radians,
executes a dual-register write_read_all UDP command to command GOAL_POSITION
and read back PRESENT_POSITION in a single packet round-trip, and publishes
the current joint positions (sensor_msgs/JointState).
"""

from pathlib import Path
import sys
import threading

# Ensure leap_ethernet/src is accessible for client import
_SRC_DIR = Path(__file__).resolve().parent.parent / "src"
if str(_SRC_DIR) not in sys.path:
    sys.path.insert(0, str(_SRC_DIR))

from client import (
    ControlTableClient,
    ControlTableError,
    ControlTableTimeout,
    DataNames,
    MOTOR_COUNT,
)
import rospy
from sensor_msgs.msg import JointState


class LeapEthernetNode:
    """ROS node that bridges ROS joint topics to OpenRB-150 over UDP."""

    def __init__(self) -> None:
        rospy.init_node("leap_ethernet_node")

        # Load parameters
        self.ip = rospy.get_param("~ip", "10.42.42.50")
        self.port = int(rospy.get_param("~port", 8888))
        self.frequency = float(rospy.get_param("~frequency", 60.0))
        self.timeout = float(rospy.get_param("~timeout", 0.05))
        self.cmd_topic = rospy.get_param("~cmd_topic", "/leap_hand/cmd")
        self.state_topic = rospy.get_param("~state_topic", "/leap_hand/state")
        self.current_limit = int(rospy.get_param("~current_limit", 550))
        self.kP = int(rospy.get_param("~kP", 400))
        self.kI = int(rospy.get_param("~kI", 0))
        self.kD = int(rospy.get_param("~kD", 0))
        self.auto_configure = bool(rospy.get_param("~auto_configure", True))

        self.joint_names = [f"joint_{i}" for i in range(MOTOR_COUNT)]
        self.lock = threading.Lock()
        self.target_positions: list[float] = [0.0] * MOTOR_COUNT
        self.latest_positions: list[float] | None = None
        self.has_received_cmd = False

        rospy.loginfo(
            f"Connecting to OpenRB-150 at {self.ip}:{self.port} "
            f"(target rate: {self.frequency:.1f} Hz, timeout: {self.timeout:.3f} s)..."
        )
        self.client = ControlTableClient(
            ip=self.ip,
            port=self.port,
            timeout=self.timeout,
        )

        rospy.on_shutdown(self._shutdown)

        if self.auto_configure:
            self._configure_hand()

        # Setup Publisher & Subscriber
        self.state_pub = rospy.Publisher(
            self.state_topic, JointState, queue_size=1
        )
        self.cmd_sub = rospy.Subscriber(
            self.cmd_topic, JointState, self._cmd_callback, queue_size=1
        )

        rospy.loginfo(
            f"LeapEthernetNode ready. Subscribed to {self.cmd_topic}, "
            f"publishing state to {self.state_topic} at {self.frequency:.1f} Hz."
        )

    def _configure_hand(self) -> None:
        """Initialize motor settings and read initial positions before enabling torque."""
        rospy.loginfo("Configuring LEAP hand motors over UDP...")
        try:
            # Set current limit
            rospy.loginfo(f"Setting current limit to {self.current_limit} mA...")
            self.client.write_all(
                DataNames.CURRENT_LIMIT, [self.current_limit] * MOTOR_COUNT
            )

            # Set return delay time to 0
            self.client.write_all(DataNames.RETURN_DELAY_TIME, [0] * MOTOR_COUNT)

            # Set operating mode to position control (3)
            self.client.write_all(DataNames.OPERATING_MODE, [3] * MOTOR_COUNT)

            # Set PID gains
            self.client.write_all(DataNames.POSITION_P_GAIN, [self.kP] * MOTOR_COUNT)
            self.client.write_all(DataNames.POSITION_I_GAIN, [self.kI] * MOTOR_COUNT)
            self.client.write_all(DataNames.POSITION_D_GAIN, [self.kD] * MOTOR_COUNT)

            # Read initial positions before torque is enabled
            rospy.loginfo("Reading initial hand positions...")
            initial_positions = self.client.read_all(DataNames.PRESENT_POSITION)
            with self.lock:
                self.target_positions = [
                    float(initial_positions[i]) for i in range(MOTOR_COUNT)
                ]
                self.latest_positions = list(self.target_positions)

            rospy.loginfo(
                "Holding initial positions and enabling torque on all motors..."
            )
            # Command initial position and enable torque
            self.client.write_all(DataNames.TORQUE_ENABLE, [1] * MOTOR_COUNT)
            self.client.write_all(DataNames.GOAL_POSITION, self.target_positions)
            rospy.loginfo("Hand configuration complete, torque enabled.")

        except Exception as e:
            rospy.logerr(f"Failed to configure LEAP hand on startup: {e}")
            raise

    def _cmd_callback(self, msg: JointState) -> None:
        """Process incoming joint commands."""
        if len(msg.position) != MOTOR_COUNT:
            rospy.logwarn_throttle(
                2.0,
                f"Received JointState with {len(msg.position)} positions, "
                f"expected {MOTOR_COUNT}. Ignoring.",
            )
            return

        with self.lock:
            self.target_positions = list(msg.position)
            self.has_received_cmd = True

    def _shutdown(self) -> None:
        """Hold current positions safely on shutdown without dropping held objects."""
        rospy.loginfo("Shutting down LeapEthernetNode, holding positions...")
        try:
            # Read current positions directly with ignore_errors=True in packet
            current_positions = None
            try:
                current_positions = self.client.read_all(
                    DataNames.PRESENT_POSITION, ignore_errors=True
                )
            except Exception as e:
                rospy.logwarn(f"Could not read positions during shutdown: {e}")

            with self.lock:
                if current_positions and len(current_positions) == MOTOR_COUNT:
                    hold_positions = [
                        float(current_positions[i]) for i in range(MOTOR_COUNT)
                    ]
                elif self.latest_positions is not None:
                    hold_positions = list(self.latest_positions)
                else:
                    hold_positions = None

            if hold_positions is not None:
                rospy.loginfo(
                    "Holding current positions on motors (ignore_errors=True)..."
                )
                try:
                    self.client.write_all(
                        DataNames.GOAL_POSITION, hold_positions, ignore_errors=True
                    )
                    rospy.loginfo("Hold positions commanded successfully.")
                except Exception as e:
                    rospy.logwarn(
                        f"Could not command hold positions during shutdown: {e}"
                    )
            else:
                rospy.logwarn("No positions available to hold on shutdown.")

            self.client.close()
            rospy.loginfo("Shutdown completed.")
        except Exception as e:
            rospy.logwarn(f"Error during shutdown: {e}")

    def spin(self) -> None:
        """Main control loop running at configured frequency."""
        rate = rospy.Rate(self.frequency)

        while not rospy.is_shutdown():
            with self.lock:
                targets = list(self.target_positions)

            try:
                # Dual-register write_read_all: commands GOAL_POSITION and returns PRESENT_POSITION
                read_positions = self.client.write_read_all(
                    write_item=DataNames.GOAL_POSITION,
                    values=targets,
                    read_item=DataNames.PRESENT_POSITION,
                )

                with self.lock:
                    self.latest_positions = [
                        float(read_positions[i]) for i in range(MOTOR_COUNT)
                    ]

                # Publish current joint state
                state_msg = JointState()
                state_msg.header.stamp = rospy.Time.now()
                state_msg.name = self.joint_names
                state_msg.position = list(self.latest_positions)
                self.state_pub.publish(state_msg)

            except ControlTableTimeout:
                rospy.logwarn_throttle(
                    1.0, "Timeout communicating with OpenRB-150 over UDP"
                )
            except ControlTableError as e:
                rospy.logerr(f"Fatal motor/control table error: {e}")
                raise
            except Exception as e:
                rospy.logwarn_throttle(
                    1.0, f"Unexpected communication error: {e}"
                )

            rate.sleep()


def main() -> None:
    node = LeapEthernetNode()
    node.spin()


if __name__ == "__main__":
    main()
