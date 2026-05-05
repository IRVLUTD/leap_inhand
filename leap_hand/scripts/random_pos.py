#!/usr/bin/env python3
import os

import rospy
import numpy as np
from sensor_msgs.msg import JointState
import leap_hand_utils.leap_hand_utils as lhu
from leap_hand.srv import leap_pos_vel_eff_tar
import time
import csv

def main():
    rospy.init_node('leap_hand_random_position', anonymous=True)
    rate = 20
    write_rate = rospy.Rate(rate)
    read_rate = rospy.Rate(60)
    pub = rospy.Publisher('/leaphand_node/cmd_leap', JointState, queue_size=1)
    test_time = 30
    # frequency = rospy.get_param('~frequency', 60.0)  # Hz
    # rate = rospy.Rate(frequency)

    joint_names = [f"joint_{i}" for i in range(16)]
    # state_pub = rospy.Publisher('/leap_hand_state', JointState, queue_size=10)]
    print("Waiting for LEAP Service")
    pos_vel_service = '/leap_pos_vel_eff_tar'

    # Wait for services to be available
    rospy.wait_for_service(pos_vel_service)
    print("Service Connected")
    
    # Create service proxies
    get_pos_vel_eff_tar = rospy.ServiceProxy(pos_vel_service, leap_pos_vel_eff_tar)
    
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
        12: np.clip(0.0, min_limits[12], max_limits[12]),
        13: np.clip(np.pi/2, min_limits[13], max_limits[13]),
        14: np.clip(0.0, min_limits[14], max_limits[14]),
        15: np.clip(0.0, min_limits[15], max_limits[15])
    }
    
    
    # Joint names
    joint_names = [f"joint_{i}" for i in range(16)]
    init_pose = np.zeros(16)
    for idx, value in fixed_joints.items():
        init_pose[idx] = value

    msg = JointState()
    msg.header.stamp = rospy.Time.now()
    msg.name = joint_names
    msg.position = init_pose.tolist()
    pub.publish(msg)
    write_rate.sleep()  # Sleep to ensure the message is sent before proceeding
    
    kP = float(rospy.get_param('/leaphand_node/kP'))
    # kI = float(rospy.get_param('/leaphand_node/kI'))
    kD = float(rospy.get_param('/leaphand_node/kD'))
    curr_lim = float(rospy.get_param('/leaphand_node/curr_lim'))
    vel_lim = float(rospy.get_param('/leaphand_node/vel_lim'))

    csv_path = os.path.expanduser(
        f"~/Projects/UGAS_rw/src/UGAS_rw/leap_rl_control/data/data_pos_vel_eff_{kP}_{kD}_{curr_lim}_{vel_lim}.csv"
    )
    csv_file = open(csv_path, mode="w", newline="")
    csv_writer = csv.writer(csv_file)

    header = (
        [
            "time",
            "target_position",
            "internal_goal",
            "cur_position",
            "velocity",
            "current",
        ]
    )
    csv_writer.writerow(header)

    start = time.time_ns() / 1000000.0  # Convert to milliseconds


    z = 1
    n = 1
    c = 0
    # input()
    time.sleep(1) # Give some time to start the node and service
    while not rospy.is_shutdown():
        try:
            # input("Press Enter to read from a new random position...")
            response = get_pos_vel_eff_tar()
            # Create and populate JointState message
            state = JointState()
            state.header.stamp = rospy.Time.now()
            state.name = joint_names
            state.position = response.position
            state.velocity = response.velocity
            state.effort = response.effort  
            goal_pos = response.target  
            # Publish the message
            # state_pub.publish(state)

            curr_pos = np.array(response.position)
            # e = curr_target - curr_pos
            # print("Target:", curr_target)
            # print("Actual Position:", curr_pos)
            # print("Actual Velocity:", response.velocity)
            # print("Actual Effort:", response.effort)
            # print("Average Joint Positional error: ", np.sum(e)/16)

            csv_writer.writerow((time.time_ns() / 1000000.0 - start, curr_target, response.target, response.position, response.velocity, response.effort ))

            if z % n == 0:
                # Generate random positions for non-fixed joints
                positions = np.random.uniform(min_limits, max_limits, size=16)
                # positions = positions * 0.5
                
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
                read_rate.sleep()  # Sleep to maintain the read rate
                pub.publish(msg)

            print(f"time: {c/(rate)}")

            z += 1

            c+=1
            
            # Sleep Use instead of input if you like
            write_rate.sleep()
        
            if c >= test_time*(rate):
                csv_file.close() 
                break
        except:
            write_rate.sleep()
            csv_file.close()   
            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            msg.name = joint_names
            msg.position = init_pose.tolist()
            pub.publish(msg)
            break
    msg = JointState()
    msg.header.stamp = rospy.Time.now()
    msg.name = joint_names
    msg.position = init_pose.tolist()
    pub.publish(msg)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass