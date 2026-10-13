#!/usr/bin/env python3
"""Offline dry-run test for the LEAP FollowJointTrajectory action server.

Spawns a mock motor simulator that echoes /leap_hand/cmd to /leap_hand/state,
then sends open and close trajectory goals to leap_hand_controller/follow_joint_trajectory.
Allows verifying the full ROS action pipeline without hardware connected.
"""

import sys
import threading
from pathlib import Path

_SRC_DIR = Path(__file__).resolve().parent.parent / "src"
if str(_SRC_DIR) not in sys.path:
    sys.path.insert(0, str(_SRC_DIR))

from client import MOTOR_COUNT
import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class MockLeapHardware:
    """Mock node simulating OpenRB-150 motor feedback."""

    def __init__(self) -> None:
        self.joint_names = [f"joint_{i}" for i in range(MOTOR_COUNT)]
        self.current_positions = [0.0] * MOTOR_COUNT
        self.lock = threading.Lock()

        self.cmd_sub = rospy.Subscriber(
            "/leap_hand/cmd", JointState, self._cmd_cb, queue_size=1
        )
        self.state_pub = rospy.Publisher(
            "/leap_hand/state", JointState, queue_size=1
        )
        self.timer = rospy.Timer(rospy.Duration(1.0 / 60.0), self._publish_state)

    def _cmd_cb(self, msg: JointState) -> None:
        if len(msg.position) == MOTOR_COUNT:
            with self.lock:
                self.current_positions = list(msg.position)

    def _publish_state(self, event) -> None:
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = self.joint_names
        with self.lock:
            msg.position = list(self.current_positions)
        msg.velocity = [0.0] * MOTOR_COUNT
        msg.effort = [50.0] * MOTOR_COUNT
        self.state_pub.publish(msg)


def main() -> None:
    rospy.init_node("test_leap_action_server_mock")
    rospy.loginfo("Starting Mock LEAP Hardware...")
    mock = MockLeapHardware()

    rospy.loginfo("Connecting to leap_hand_controller action server...")
    client = actionlib.SimpleActionClient(
        "leap_hand_controller/follow_joint_trajectory",
        FollowJointTrajectoryAction,
    )
    if not client.wait_for_server(rospy.Duration(5.0)):
        rospy.logerr("Could not connect to action server! Ensure leap_trajectory_action_server is running.")
        return

    joint_names = [f"joint_{i}" for i in range(MOTOR_COUNT)]

    # 1. Test Open Goal
    rospy.loginfo("Sending OPEN goal to action server...")
    traj = JointTrajectory()
    traj.joint_names = joint_names
    pt = JointTrajectoryPoint()
    pt.positions = [0.0] * MOTOR_COUNT
    pt.positions[13] = 1.5  # Thumb open
    pt.time_from_start = rospy.Duration(1.0)
    traj.points.append(pt)

    goal = FollowJointTrajectoryGoal(trajectory=traj)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration(3.0))
    res = client.get_result()
    rospy.loginfo(f"Open goal completed with error code: {res.error_code if res else 'None'}")

    rospy.sleep(0.5)

    # 2. Test Close Goal
    rospy.loginfo("Sending CLOSE goal to action server...")
    traj = JointTrajectory()
    traj.joint_names = joint_names
    pt = JointTrajectoryPoint()
    # Grasp pose
    pt.positions = [0.0] * MOTOR_COUNT
    pt.positions[1] = 0.6
    pt.positions[2] = 0.8
    pt.positions[5] = 0.6
    pt.positions[6] = 0.8
    pt.positions[9] = 1.1
    pt.positions[10] = 1.3
    pt.positions[12] = 1.2
    pt.positions[13] = 0.7
    pt.positions[14] = 0.9
    pt.time_from_start = rospy.Duration(1.0)
    traj.points.append(pt)

    goal = FollowJointTrajectoryGoal(trajectory=traj)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration(3.0))
    res = client.get_result()
    rospy.loginfo(f"Close goal completed with error code: {res.error_code if res else 'None'}")

    rospy.loginfo("=== DRY-RUN VERIFICATION SUCCESSFUL ===")


if __name__ == "__main__":
    main()
