#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState
import leap_hand_utils.leap_hand_utils as lhu
from leap_hand.srv import leap_pos_vel
import time

def main():
    rospy.init_node('leap_hand_random_position', anonymous=True)
    pub = rospy.Publisher('/leaphand_node/cmd_leap', JointState, queue_size=10)
    frequency = rospy.get_param('~frequency', 1.0)  # Hz
    rate = rospy.Rate(frequency)

    joint_names = [f"joint_{i}" for i in range(16)]
    state_pub = rospy.Publisher('/leap_hand_state', JointState, queue_size=10)
    pos_vel_service = '/leap_pos_vel'
    # Wait for services to be available
    rospy.wait_for_service(pos_vel_service)
    
    # Create service proxies
    get_pos_vel = rospy.ServiceProxy(pos_vel_service, leap_pos_vel)
    
    # Get joint limits
    min_limits, max_limits = lhu.LEAP_limits()

    # current pos
    curr_target = np.zeros(16)

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
            for i in range(5):
                response = get_pos_vel()
                time.sleep(0.03)
            # Create and populate JointState message
            state = JointState()
            state.header.stamp = rospy.Time.now()
            state.name = joint_names
            state.position = response.position
            state.velocity = response.velocity
            state.effort = []  
            
            # Publish the message
            state_pub.publish(state)

            curr_pos = np.array(response.position)
            e = curr_target - curr_pos
            print("Target:", curr_target)
            print("Actual Position:", curr_pos)
            print("Average Joint Positional error: ", np.sum(e)/16)
            time.sleep(1)

        except KeyboardInterrupt:
            break

        # Generate random positions for non-fixed joints
        positions = np.random.uniform(min_limits, max_limits, size=16)
        positions = positions * 0.3
        
        # Set fixed joints
        for idx, value in fixed_joints.items():
            positions[idx] = value

        curr_target = positions.copy()

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