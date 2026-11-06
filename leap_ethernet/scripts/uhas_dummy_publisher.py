#!/usr/bin/env python3
"""Publish one editable finger/wrist test routine, then hold its final hand target.

Open and grasp targets come from UHAS's LEAP right-hand reference poses. Edit
_routine_steps() to skip movements or change individual durations and peak wrist
speeds; ~durations/<step_name> overrides timing at runtime (zero skips a step).

Publish 26-value Float64MultiArray actions for uhas_fetch_leap_controller.py:
linear XYZ (m/s), angular XYZ (rad/s), 15 encompassing values, five laterals.
Wrist directions use the controller's configured command frame. Run in the
Python 3 environment containing UHAS and ROS; position the arm/torso manually.
"""

import math
from collections import namedtuple
from time import monotonic

import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray
from uhas import UHAS


JOINT_NAMES = ["joint_%d" % index for index in range(16)]


def finite_vector(values, size):
    """Return a finite numeric list of the requested size, or raise ValueError."""
    result = [float(value) for value in values]
    if len(result) != size or not all(math.isfinite(value) for value in result):
        raise ValueError("expected %d finite values" % size)
    return result


# A missing target holds the previous hand pose; twist is (axis, peak velocity).
RoutineStep = namedtuple("RoutineStep", "name label seconds target twist", defaults=(None, None))


class UHASDummyPublisher:
    """Interpolate UHAS reference actions and publish timed wrist velocity pulses."""

    def __init__(self):
        """Read settings and obtain open/grasp reference actions directly from UHAS."""
        rospy.init_node("uhas_dummy_publisher")

        self._last_action = None
        self.topic = rospy.get_param("~topic", "/uhas/hand_joints")
        self.rate_hz = float(rospy.get_param("~rate", 30.0))
        self.transition_duration = float(
            rospy.get_param("~transition_duration", 2.0)
        )
        self.hold_duration = float(rospy.get_param("~hold_duration", 1.0))
        self.grasp_scale = float(rospy.get_param("~grasp_scale", 1.0))
        finite_vector([self.rate_hz, self.transition_duration, self.hold_duration, self.grasp_scale], 4)
        if (
            self.rate_hz <= 0.0
            or self.transition_duration <= 0.0
            or self.hold_duration < 0.0
            or not 0.0 <= self.grasp_scale <= 1.0
        ):
            raise rospy.ROSException(
                "~rate and ~transition_duration must be positive, "
                "~hold_duration must be non-negative, and ~grasp_scale must be in [0, 1]"
            )

        self.publisher = rospy.Publisher(
            self.topic, Float64MultiArray, queue_size=1
        )
        hand = UHAS("leap", "right")
        hand.set_joint_order(JOINT_NAMES)
        actions = []
        for pose in (hand.q_open(), hand.q_sphere()):
            encompassing, laterals = hand.get_uhas(pose)
            actions.append(np.concatenate((
                np.zeros(6),
                np.asarray(encompassing).reshape(-1),
                np.asarray(laterals).reshape(-1),
            )))
        self.open_action, full_grasp_action = actions
        self.grasp_action = self.open_action + self.grasp_scale * (
            full_grasp_action - self.open_action
        )

    def _finger_pose(self, planes):
        """Return an open-hand action with only the supplied canonical planes curled."""
        target = self.open_action.copy()
        # Canonical planes: pinky, ring, middle, index, thumb. LEAP's ring
        # owns both pinky/ring planes, so change both to retain its full motion.
        for plane in planes:
            target[6 + 3 * plane:9 + 3 * plane] = self.grasp_action[6 + 3 * plane:9 + 3 * plane]
            target[21 + plane] = self.grasp_action[21 + plane]
        return target

    def _routine_steps(self):
        """Return the editable routine in execution order, with durations in seconds.

        Comment out any row or section to skip it. A target of None holds the
        previous hand pose. ROS ~durations/<name> overrides a row's duration;
        zero skips it. Wrist twist tuples contain (axis, peak velocity), where
        axes 0..2 are linear XYZ (m/s) and 3..5 are angular XYZ (rad/s).
        """
        opened = self.open_action
        grasp = self.grasp_action
        transition = self.transition_duration
        hold = self.hold_duration
        index = self._finger_pose((3,))
        middle = self._finger_pose((2,))
        ring = self._finger_pose((0, 1))
        thumb = self._finger_pose((4,))

        # Change any seconds value below independently (e.g. transition -> 4.0).
        # Wrist directions use the controller's configured frame, which defaults
        # to wrist_roll_link. Positive Z is not necessarily ground-relative up.
        return [
            RoutineStep("initial_open", "Hand opening; wrist stationary", 2.0, opened),

            # Index finger.
            RoutineStep("index_curl", "Index finger curling", transition, index),
            RoutineStep("index_hold", "Holding index finger curled", hold),
            RoutineStep("index_open", "Index finger opening", transition, opened),

            # Middle finger.
            RoutineStep("middle_curl", "Middle finger curling", transition, middle),
            RoutineStep("middle_hold", "Holding middle finger curled", hold),
            RoutineStep("middle_open", "Middle finger opening", transition, opened),

            # Ring finger.
            RoutineStep("ring_curl", "Ring finger curling", transition, ring),
            RoutineStep("ring_hold", "Holding ring finger curled", hold),
            RoutineStep("ring_open", "Ring finger opening", transition, opened),

            # Thumb.
            RoutineStep("thumb_curl", "Thumb finger curling", transition, thumb),
            RoutineStep("thumb_hold", "Holding thumb finger curled", hold),
            RoutineStep("thumb_open", "Thumb finger opening", transition, opened),

            # Whole-hand grasp.
            RoutineStep("grasp_close", "All fingers closing into grasp", transition, grasp),
            RoutineStep("grasp_hold", "Holding grasp; wrist stationary", hold),
            RoutineStep("grasp_open", "All fingers opening", transition, opened),

            # Wrist roll: positive pulse, pause, negative pulse, pause.
            RoutineStep("roll_positive", "Wrist rolling positive (+X)", 1.0, twist=(3, 0.3)),
            RoutineStep("roll_positive_pause", "Wrist stopped; holding hand steady", 1.0),
            RoutineStep("roll_negative", "Wrist rolling back (-X)", 1.0, twist=(3, -0.3)),
            RoutineStep("roll_negative_pause", "Wrist stopped; holding hand steady", 1.0),

            # Wrist pitch.
            RoutineStep("pitch_positive", "Wrist pitching positive (+Y)", 1.0, twist=(4, 0.3)),
            RoutineStep("pitch_positive_pause", "Wrist stopped; holding hand steady", 1.0),
            RoutineStep("pitch_negative", "Wrist pitching back (-Y)", 1.0, twist=(4, -0.3)),
            RoutineStep("pitch_negative_pause", "Wrist stopped; holding hand steady", 1.0),

            # Wrist yaw.
            RoutineStep("yaw_positive", "Wrist yawing positive (+Z)", 1.0, twist=(5, 0.3)),
            RoutineStep("yaw_positive_pause", "Wrist stopped; holding hand steady", 1.0),
            RoutineStep("yaw_negative", "Wrist yawing back (-Z)", 1.0, twist=(5, -0.3)),
            RoutineStep("yaw_negative_pause", "Wrist stopped; holding hand steady", 1.0),

            # Wrist translation along X.
            RoutineStep("x_positive", "Wrist moving forward (+X)", 1.0, twist=(0, 0.15)),
            RoutineStep("x_positive_pause", "Wrist stopped; holding hand steady", 1.0),
            RoutineStep("x_negative", "Wrist moving backward (-X)", 1.0, twist=(0, -0.15)),
            RoutineStep("x_negative_pause", "Wrist stopped; holding hand steady", 1.0),

            # Wrist translation along Y.
            RoutineStep("y_positive", "Wrist moving left (+Y)", 1.0, twist=(1, 0.15)),
            RoutineStep("y_positive_pause", "Wrist stopped; holding hand steady", 1.0),
            RoutineStep("y_negative", "Wrist moving right (-Y)", 1.0, twist=(1, -0.15)),
            RoutineStep("y_negative_pause", "Wrist stopped; holding hand steady", 1.0),

            # Wrist translation along Z.
            RoutineStep("z_positive", "Wrist moving up (+Z)", 1.0, twist=(2, 0.15)),
            RoutineStep("z_positive_pause", "Wrist stopped; holding hand steady", 1.0),
            RoutineStep("z_negative", "Wrist moving down (-Z)", 1.0, twist=(2, -0.15)),
            RoutineStep("z_negative_pause", "Wrist stopped; holding hand steady", 1.0),
        ]

    def _build_routine(self):
        """Resolve timing overrides and connect enabled steps, returning phases and hold."""
        phases = []
        current = self.open_action.copy()
        for step in self._routine_steps():
            name = "~durations/" + step.name
            try:
                duration = finite_vector([rospy.get_param(name, step.seconds)], 1)[0]
            except (ValueError, TypeError, OverflowError) as error:
                raise rospy.ROSException("%s must be finite seconds: %s" % (name, error))
            if duration < 0.0:
                raise rospy.ROSException("%s must be non-negative seconds" % name)
            if duration == 0.0:
                continue
            target = current.copy() if step.target is None else step.target.copy()
            twist = None
            label = step.label
            if step.twist is not None:
                axis, speed = step.twist
                twist = np.zeros(6)
                twist[axis] = speed
                label += " in configured command frame"
            phases.append((label, current.copy(), target, duration, twist))
            current = target.copy()
        # Hold the last enabled target, even when sections were removed.
        return phases, current

    def _publish_action(self, action):
        """Publish and remember one complete finger-action and wrist-velocity frame."""
        message = Float64MultiArray()
        message.data = action.tolist()
        self.publisher.publish(message)
        self._last_action = action.copy()

    def _run_phase(self, label, start, end, duration, twist, rate):
        """Narrate and publish a smooth timed phase, returning false on shutdown."""
        rospy.loginfo(label)
        started = monotonic()
        while not rospy.is_shutdown():
            ratio = (monotonic() - started) / duration
            if ratio >= 1.0:
                # Finish with an exact zero twist before the next phase.
                stopped = end.copy()
                stopped[:6] = 0.0
                self._publish_action(stopped)
                return True
            action = self._interpolate(start, end, ratio)
            action[:6] = 0.0
            if twist is not None:
                # Sine pulse travel is 2 * peak_speed * duration / pi.
                action[:6] = twist * np.sin(np.pi * max(0.0, ratio))
            self._publish_action(action)
            rate.sleep()
        return False

    def spin(self):
        """Run once and then hold; never repeat wrist movements automatically."""
        rate = rospy.Rate(self.rate_hz)
        rospy.loginfo("Waiting for the UHAS controller to subscribe...")
        try:
            while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
                rate.sleep()
            if rospy.is_shutdown():
                return
            rospy.loginfo("Starting configured routine. Wrist directions use the controller's command frame.")
            phases, resting = self._build_routine()
            for label, start, end, duration, twist in phases:
                if not self._run_phase(label, start, end, duration, twist, rate):
                    return
            rospy.loginfo("Routine complete. Holding last hand target with zero wrist velocity; Ctrl+C to stop publishing.")
            while not rospy.is_shutdown():
                self._publish_action(resting)
                rate.sleep()
        finally:
            if self._last_action is not None:
                # Preserve the last finger action; never jump to another hand pose on exit.
                stopped = self._last_action.copy()
                stopped[:6] = 0.0
                try:
                    self._publish_action(stopped)
                except rospy.ROSException:
                    pass  # The controller's watchdog also stops stale input.

    @staticmethod
    def _interpolate(start, end, ratio):
        """Interpolate hand actions with zero slope at the start and end."""
        ratio = min(1.0, max(0.0, ratio))
        eased = ratio * ratio * (3.0 - 2.0 * ratio)
        return start + eased * (end - start)


def main():
    """Run the narrated streaming test; no tuck or torso pose is ever requested."""
    UHASDummyPublisher().spin()


if __name__ == "__main__":
    main()
