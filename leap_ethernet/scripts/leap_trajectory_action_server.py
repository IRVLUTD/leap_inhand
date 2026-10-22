#!/usr/bin/env python3
"""FollowJointTrajectory action server bridge for the LEAP Hand.

Bridges MoveIt / standard ROS trajectory controllers to the leap_ethernet driver.
Receives FollowJointTrajectory goals (for leap_hand_controller), smoothly interpolates
joint positions over time, streams them to /leap_hand/cmd, and optionally forwards
hand state feedback to /joint_states for TF and MoveIt visualization.
"""

import copy
import math
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
    FollowJointTrajectoryFeedback,
    FollowJointTrajectoryGoal,
    FollowJointTrajectoryResult,
)
import rospy
from sensor_msgs.msg import JointState


class LeapTrajectoryActionServer:
    """Action server providing follow_joint_trajectory interface for LEAP Hand."""

    def __init__(self) -> None:
        rospy.init_node("leap_trajectory_action_server")

        self.action_name = rospy.get_param(
            "~action_name", "leap_hand_controller/follow_joint_trajectory"
        )
        self.cmd_topic = rospy.get_param("~cmd_topic", "/leap_hand/cmd")
        self.state_topic = rospy.get_param("~state_topic", "/leap_hand/state")
        self.publish_to_joint_states = bool(
            rospy.get_param("~publish_to_joint_states", True)
        )
        self.rate_hz = float(rospy.get_param("~rate", 60.0))
        self.goal_tolerance = float(rospy.get_param("~goal_tolerance", 0.15))  # rad

        self.joint_names = [f"joint_{i}" for i in range(MOTOR_COUNT)]

        self.lock = threading.Lock()
        self.current_positions: list[float] = [0.0] * MOTOR_COUNT
        self.has_state = False

        # Publisher to leap_ethernet_node command topic
        self.cmd_pub = rospy.Publisher(self.cmd_topic, JointState, queue_size=1)

        # Optional forwarder to global /joint_states so MoveIt & RViz show real fingers
        if self.publish_to_joint_states and self.state_topic != "/joint_states":
            self.joint_states_pub = rospy.Publisher(
                "/joint_states", JointState, queue_size=5
            )
        else:
            self.joint_states_pub = None

        # Feedback subscriber from leap_ethernet_node
        self.state_sub = rospy.Subscriber(
            self.state_topic, JointState, self._state_callback, queue_size=1
        )

        # Action server
        self.server = actionlib.SimpleActionServer(
            self.action_name,
            FollowJointTrajectoryAction,
            execute_cb=self._execute_callback,
            auto_start=False,
        )
        self.server.start()

        rospy.loginfo(
            f"LeapTrajectoryActionServer ready on '{self.action_name}'. "
            f"Forwarding commands to '{self.cmd_topic}' at {self.rate_hz} Hz."
        )

    def _state_callback(self, msg: JointState) -> None:
        """Cache current positions and optionally forward to /joint_states."""
        if len(msg.position) == MOTOR_COUNT:
            with self.lock:
                self.current_positions = list(msg.position)
                self.has_state = True

            # Forward to /joint_states if enabled
            if self.joint_states_pub is not None:
                forward_msg = JointState()
                forward_msg.header.stamp = rospy.Time.now()
                forward_msg.name = msg.name if msg.name else self.joint_names
                forward_msg.position = msg.position
                forward_msg.velocity = msg.velocity
                forward_msg.effort = msg.effort
                self.joint_states_pub.publish(forward_msg)

    def _execute_callback(self, goal: FollowJointTrajectoryGoal) -> None:
        """Execute incoming trajectory goal."""
        rospy.loginfo(
            f"Received trajectory with {len(goal.trajectory.points)} waypoints "
            f"for joints: {goal.trajectory.joint_names}"
        )

        # Map trajectory joint order to internal joint_0..15 order
        joint_indices = []
        for name in goal.trajectory.joint_names:
            if name in self.joint_names:
                joint_indices.append(self.joint_names.index(name))
            else:
                try:
                    idx = int(name.replace("joint_", ""))
                    if 0 <= idx < MOTOR_COUNT:
                        joint_indices.append(idx)
                    else:
                        joint_indices.append(-1)
                except ValueError:
                    joint_indices.append(-1)

        points = goal.trajectory.points
        if not points:
            rospy.logwarn("Received empty trajectory goal.")
            self.server.set_succeeded(FollowJointTrajectoryResult())
            return

        with self.lock:
            start_positions = list(self.current_positions)

        rate = rospy.Rate(self.rate_hz)
        start_time = rospy.Time.now()
        last_point_time = 0.0

        # Immediate 1-point goal (direct position command / fast response)
        if len(points) == 1 and points[0].time_from_start.to_sec() <= 0.05:
            target_positions = list(start_positions)
            for i, pos in enumerate(points[0].positions):
                motor_id = joint_indices[i]
                if 0 <= motor_id < MOTOR_COUNT:
                    target_positions[motor_id] = pos

            cmd_msg = JointState()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.name = self.joint_names
            cmd_msg.position = target_positions
            self.cmd_pub.publish(cmd_msg)

            with self.lock:
                self.current_positions = list(target_positions)

            rospy.loginfo("Immediate goal dispatched directly to /leap_hand/cmd.")
            result = FollowJointTrajectoryResult()
            result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
            self.server.set_succeeded(result)
            return

        # Timed trajectory (smooth teleop or MoveIt path)
        for pt_idx, point in enumerate(points):
            pt_target_time = point.time_from_start.to_sec()

            target_positions = list(start_positions)
            for i, pos in enumerate(point.positions):
                motor_id = joint_indices[i]
                if 0 <= motor_id < MOTOR_COUNT:
                    target_positions[motor_id] = pos

            # If this is the start waypoint at t=0, advance without delay
            if pt_idx == 0 and pt_target_time <= 0.0 and len(points) > 1:
                start_positions = list(target_positions)
                continue

            pt_duration = pt_target_time - last_point_time
            if pt_duration <= 0.0:
                cmd_msg = JointState()
                cmd_msg.header.stamp = rospy.Time.now()
                cmd_msg.name = self.joint_names
                cmd_msg.position = target_positions
                self.cmd_pub.publish(cmd_msg)
            else:
                pt_start_time = rospy.Time.now()
                while not rospy.is_shutdown():
                    if self.server.is_preempt_requested():
                        rospy.loginfo("Trajectory execution preempted.")
                        self.server.set_preempted()
                        return

                    elapsed = (rospy.Time.now() - pt_start_time).to_sec()
                    t_ratio = min(1.0, elapsed / pt_duration)

                    interp_pos = [
                        start_positions[k] + t_ratio * (target_positions[k] - start_positions[k])
                        for k in range(MOTOR_COUNT)
                    ]

                    cmd_msg = JointState()
                    cmd_msg.header.stamp = rospy.Time.now()
                    cmd_msg.name = self.joint_names
                    cmd_msg.position = interp_pos
                    self.cmd_pub.publish(cmd_msg)

                    feedback = FollowJointTrajectoryFeedback()
                    feedback.header.stamp = rospy.Time.now()
                    feedback.joint_names = self.joint_names
                    with self.lock:
                        feedback.actual.positions = list(self.current_positions)
                    feedback.desired.positions = interp_pos
                    self.server.publish_feedback(feedback)

                    if t_ratio >= 1.0:
                        break

                    rate.sleep()

            start_positions = list(target_positions)
            last_point_time = pt_target_time

        rospy.loginfo("Trajectory execution completed successfully.")
        result = FollowJointTrajectoryResult()
        result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
        self.server.set_succeeded(result)


def main() -> None:
    server = LeapTrajectoryActionServer()
    rospy.spin()


if __name__ == "__main__":
    main()
