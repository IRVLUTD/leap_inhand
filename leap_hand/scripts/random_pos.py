#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState
import leap_hand_utils.leap_hand_utils as lhu

def main():
    rospy.init_node('leap_hand_random_position', anonymous=True)
    pub = rospy.Publisher('/leaphand_node/cmd_leap', JointState, queue_size=10)
    frequency = rospy.get_param('~frequency', 1.0)  # Hz
    rate = rospy.Rate(frequency)
    
    # Get joint limits
    min_limits, max_limits = lhu.LEAP_limits()
    
    # Ensure limits are numpy arrays
    min_limits = np.array(min_limits)
    max_limits = np.array(max_limits)
    
    # Fixed joints with clipping to limits
    fixed_joints = {
        0: np.clip(0.0, min_limits[0], max_limits[0]),
        4: np.clip(0.0, min_limits[4], max_limits[4]),
        8: np.clip(0.0, min_limits[8], max_limits[8]),
        13: np.clip(np.pi/2, min_limits[12], max_limits[12])
    }
    
    # Joint names
    joint_names = [f"joint_{i}" for i in range(16)]
    
    while not rospy.is_shutdown():
        try:
            input("Press Enter to send a new random position...")
        except KeyboardInterrupt:
            break

        # Generate random positions for non-fixed joints
        positions = np.random.uniform(min_limits, max_limits, size=16)
        positions = positions * 0.3
        
        # Set fixed joints
        for idx, value in fixed_joints.items():
            positions[idx] = value

        
        # Create JointState message
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = joint_names
        msg.position = positions.tolist()
        
        # Publish
        pub.publish(msg)
        
        # Sleep Use instead of input if you like
        # rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass