#!/usr/bin/env python3
"""Translate streamed UHAS actions into LEAP joint targets and Fetch wrist motion.

Subscribe to a 26-value Float64MultiArray: linear XYZ velocities (m/s), angular
XYZ velocities (rad/s), 15 encompassing values, then five lateral values. UHAS
converts the finger values into absolute joint targets. Wrist velocities use
~twist_frame (default uhas_sphere) and are limited by vector norm. The sphere
pose comes from UHAS and is published as a static TF relative to the LEAP palm.

An independent watchdog stops expired wrist commands and holds recent measured
finger positions. Run in the Python 3 environment containing UHAS and ROS.
Arm and torso positioning remain manual.
"""

import math
import threading
from time import monotonic

import numpy as np
import rospy
from geometry_msgs.msg import TransformStamped, TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Header
from tf2_msgs.msg import TFMessage
from uhas import UHAS


JOINT_NAMES = ["joint_%d" % index for index in range(16)]


def finite_vector(values, size):
    """Return a finite numeric list of the requested size, or raise ValueError."""
    result = [float(value) for value in values]
    if len(result) != size or not all(math.isfinite(value) for value in result):
        raise ValueError("expected %d finite values" % size)
    return result


def quaternion_from_matrix(pose):
    """Return an XYZW unit quaternion from a pose using only NumPy.

    The symmetric eigenproblem also handles rotations near 180 degrees.
    Avoid importing tf: its package imports compiled ROS Python bindings.
    """
    r = np.asarray(pose, dtype=float)[:3, :3]
    matrix = np.array([
        [r[0, 0] - r[1, 1] - r[2, 2], r[0, 1] + r[1, 0], r[0, 2] + r[2, 0], r[2, 1] - r[1, 2]],
        [r[0, 1] + r[1, 0], r[1, 1] - r[0, 0] - r[2, 2], r[1, 2] + r[2, 1], r[0, 2] - r[2, 0]],
        [r[0, 2] + r[2, 0], r[1, 2] + r[2, 1], r[2, 2] - r[0, 0] - r[1, 1], r[1, 0] - r[0, 1]],
        [r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1], np.trace(r)],
    ])
    _, eigenvectors = np.linalg.eigh(matrix)
    quaternion = eigenvectors[:, -1]
    return quaternion if quaternion[3] >= 0.0 else -quaternion


class UHASFetchLeapController:
    """Convert the latest action while an independent watchdog stops stale motion."""

    def __init__(self):
        """Initialize the hand and start listening without commanding a pose."""
        rospy.init_node("uhas_fetch_leap_controller")
        self.input_topic = rospy.get_param("~input_topic", "/uhas/hand_joints")
        self.twist_topic = rospy.get_param("~twist_topic", "/arm_controller/cartesian_twist/command")
        self.leap_cmd_topic = rospy.get_param("~leap_cmd_topic", "/leap_hand/cmd")
        self.leap_state_topic = rospy.get_param("~leap_state_topic", "/leap_hand/state")
        self.sphere_frame = rospy.get_param("~sphere_frame", "uhas_sphere")
        self.twist_frame = rospy.get_param("~twist_frame", self.sphere_frame)
        self.input_timeout = self._positive_param("~input_timeout", 0.25)
        self.max_linear_speed = self._positive_param("~max_linear_speed", 0.4)
        self.max_angular_speed = self._positive_param("~max_angular_speed", 1.5)
        self.hand = UHAS("leap", "right")
        self.hand.set_joint_order(JOINT_NAMES)
        self._publish_sphere_frame()

        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._work = threading.Event()
        self._pending = None
        # Timestamp of the input behind the last published command, not solve completion.
        self._last_command_time = None
        self._stale = True
        self._hand_state = None
        self.twist_pub = rospy.Publisher(self.twist_topic, TwistStamped, queue_size=1)
        self.leap_pub = rospy.Publisher(self.leap_cmd_topic, JointState, queue_size=1)
        rospy.on_shutdown(self._shutdown)
        self.state_sub = rospy.Subscriber(
            rospy.get_param("~leap_state_topic", "/leap_hand/state"),
            JointState, self._state_callback, queue_size=1,
        )
        self.input_sub = rospy.Subscriber(self.input_topic, Float64MultiArray, self._input_callback, queue_size=1)
        self._processor = threading.Thread(target=self._process_loop)
        self._processor.daemon = True
        self._processor.start()
        self._watchdog = threading.Thread(target=self._watchdog_loop)
        self._watchdog.daemon = True
        self._watchdog.start()
        rospy.loginfo("UHAS Fetch/LEAP ready; waiting for input. Arm and torso positioning are manual.")

    def _publish_sphere_frame(self):
        """Expose UHAS's fixed sphere pose in the robot's TF tree."""
        pose = np.asarray(self.hand.sphere_frame(), dtype=float)
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        # UHAS base_link names the hand's palm, not Fetch's mobile base_link.
        transform.header.frame_id = self.hand.base_link
        transform.child_frame_id = self.sphere_frame
        translation = transform.transform.translation
        translation.x, translation.y, translation.z = pose[:3, 3]
        rotation = transform.transform.rotation
        rotation.x, rotation.y, rotation.z, rotation.w = quaternion_from_matrix(pose)
        # Match StaticTransformBroadcaster without loading tf2's compiled extension.
        self._sphere_tf_pub = rospy.Publisher("/tf_static", TFMessage, queue_size=1, latch=True)
        self._sphere_tf_pub.publish(TFMessage(transforms=[transform]))

    @staticmethod
    def _positive_param(name, default):
        """Read a finite positive scalar ROS parameter."""
        value = finite_vector([rospy.get_param(name, default)], 1)[0]
        if value <= 0.0:
            raise rospy.ROSException("%s must be greater than zero" % name)
        return value

    def _state_callback(self, message):
        """Cache measured motor-order positions for holding the hand when input ends."""
        try:
            positions = finite_vector(message.position, 16)
            if len(message.name) != 16 or set(message.name) != set(JOINT_NAMES):
                return
            by_name = dict(zip(message.name, positions))
            positions = [by_name[name] for name in JOINT_NAMES]
        except (ValueError, TypeError, OverflowError):
            return
        with self._lock:
            self._hand_state = (positions, monotonic())

    @staticmethod
    def _clamp(vector, maximum):
        """Limit a velocity vector by its Euclidean norm, avoiding overflow."""
        vector = np.asarray(vector, dtype=float)
        scale = float(np.max(np.abs(vector)))
        if scale == 0.0:
            return vector
        unit = vector / scale
        norm = float(np.linalg.norm(unit))
        return vector if scale <= maximum / norm else unit * (maximum / norm)

    def _publish_zero_twist(self):
        """Publish a wrist stop without changing the arm configuration."""
        twist = TwistStamped()
        twist.header.stamp = rospy.Time.now()
        twist.header.frame_id = self.twist_frame
        self.twist_pub.publish(twist)

    def _hold_hand(self):
        """Hold recent measured positions, or leave the last target if feedback is stale."""
        if self._hand_state is not None:
            positions, received = self._hand_state
            if monotonic() - received <= self.input_timeout:
                self.leap_pub.publish(JointState(
                    header=Header(stamp=rospy.Time.now()), name=JOINT_NAMES, position=positions,
                ))

    def _watchdog_loop(self):
        """Stop expired commands even while a solve blocks or ROS time pauses."""
        while not self._stop.wait(min(0.05, self.input_timeout / 2.0)):
            with self._lock:
                if (self._last_command_time is not None
                        and monotonic() - self._last_command_time > self.input_timeout):
                    self._publish_zero_twist()
                    if not self._stale:
                        self._hold_hand()
                        self._stale = True

    def _input_callback(self, message):
        """Replace one pending action without blocking the ROS subscriber."""
        received = monotonic()
        try:
            values = finite_vector(message.data, 26)
        except (ValueError, TypeError, OverflowError):
            rospy.logwarn_throttle(2.0, "UHAS input must contain exactly 26 finite values")
            return
        with self._lock:
            if not self._stop.is_set():
                self._pending = (values, received)
                self._work.set()

    def _process_loop(self):
        """Solve the newest pending action and discard results that arrive too late."""
        while not self._stop.is_set():
            self._work.wait(0.1)
            with self._lock:
                frame = self._pending
                self._pending = None
                self._work.clear()
            if frame is None:
                continue
            values, received = frame
            if monotonic() - received > self.input_timeout:
                continue
            # Solve outside the lock so the watchdog can stop motion during a slow solve.
            try:
                bending = np.asarray(values[6:21], dtype=float).reshape(5, 3)
                target = finite_vector(self.hand.get_cmd(bending, laterals=np.asarray(values[21:26])), 16)
            except Exception as error:
                with self._lock:
                    if not self._stop.is_set():
                        self._publish_zero_twist()
                        self._hold_hand()
                        self._stale = True
                rospy.logerr_throttle(2.0, "Could not process UHAS frame: %s" % error)
                continue
            with self._lock:
                if self._stop.is_set() or monotonic() - received > self.input_timeout:
                    continue
                now = rospy.Time.now()
                self.leap_pub.publish(JointState(header=Header(stamp=now), name=JOINT_NAMES, position=target))
                twist = TwistStamped()
                twist.header.stamp = now
                twist.header.frame_id = self.twist_frame
                twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z = self._clamp(values[:3], self.max_linear_speed)
                twist.twist.angular.x, twist.twist.angular.y, twist.twist.angular.z = self._clamp(values[3:6], self.max_angular_speed)
                self.twist_pub.publish(twist)
                self._last_command_time = received
                self._stale = False

    def _shutdown(self):
        """Stop streaming and hold position without initiating any recovery motion."""
        with self._lock:
            if self._stop.is_set():
                return
            self._stop.set()
            self._pending = None
            self._work.set()
            self._publish_zero_twist()
            self._hold_hand()


def main():
    """Listen until manually stopped; startup and shutdown never tuck the arm."""
    UHASFetchLeapController()
    rospy.spin()


if __name__ == "__main__":
    main()
