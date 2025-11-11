#!/usr/bin/env python3
import numpy as np
import rospy
import rospkg
import os
import sys
import time

import threading
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Header

from leap_hand_utils.dynamixel_client import *
import leap_hand_utils.leap_hand_utils as lhu
from leap_hand.srv import *


#LEAP hand conventions:

#The joint numbering goes from Index (0-3), Middle(4-7), Ring(8-11) to Thumb(12-15) and from MCP Side, MCP Forward, PIP, DIP for each finger.
#For instance, the MCP Side of Index is ID 0, the MCP Forward of Ring is 9, the DIP of Ring is 11

# Author recommends only to query when necessary and below 90 samples a second.  
# Use the combined commands if you can to save time.  Also don't forget about the USB latency settings in the readme.


class LeapNode:
    def __init__(self, frequency ):
        self.timingpub = rospy.Publisher("/timing", Header, queue_size=1)
        ####Some parameters to control the hand #! Reduce PD values for less jittery control, Increase for more strength
        self.kP = float(rospy.get_param('/leaphand_node/kP', 0.0)) 
        self.kI = float(rospy.get_param('/leaphand_node/kI', 0.0))
        self.kD = float(rospy.get_param('/leaphand_node/kD', 0.0))
        self.curr_lim = float(rospy.get_param('/leaphand_node/curr_lim', 0.0)) #don't go past 600ma on this, or it'll overcurrent sometimes for regular, 350ma for lite.
        self.prev_pos = self.pos = self.curr_pos = np.zeros(16)
        self.frequency = frequency
        self.lock = threading.Lock()

        # Internal state variables
        self.latest_pos = np.zeros(16)
        self.latest_vel = np.zeros(16)
        self.latest_eff = np.zeros(16)        

        #You can put the correct port here or have the node auto-search for a hand at the first 3 ports.
        # For example ls /dev/serial/by-id/* to find your LEAP Hand. Then use the result.  
        # For example: /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT7W91VW-if00-port0
        self.motors = motors = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
        self.joint_names = [f"joint_{i}" for i in self.motors]

        ports = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyUSB2']
        for port in ports:
            try:
                self.dxl_client = DynamixelClient(motors, port, 4000000)
                self.dxl_client.connect()
                print(f"Connected successfully to {port}")
                break  # Exit loop if connection succeeds
            except Exception as e:
                print(f"Failed to connect to {port}: {e}")
        
        # Enables position-current control mode, it commands a position and then caps the current so the motors don't overload
        self.dxl_client.sync_write(motors, np.ones(len(motors))*5, 11, 1)
        self.dxl_client.sync_write(motors, np.zeros(len(motors)), 9, 1) # Set return time delay to 0
        self.dxl_client.sync_write(motors, np.ones(len(motors))*125, 112, 4) # Velocity 
        self.dxl_client.sync_write(motors, np.ones(len(motors))*0, 108, 4) # Acceleration
        self.dxl_client.set_torque_enabled(motors, True)

        # Set parameters for PID control
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kP, 84, 2) # Pgain stiffness     
        self.dxl_client.sync_write(motors, np.zeros(len(motors)), 88, 2) # FF Gain     
        self.dxl_client.sync_write(motors, np.zeros(len(motors)), 90, 2) # FF2 Gain     
        # self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kP * 0.75), 84, 2) # Pgain stiffness for side to side should be a bit less
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kI, 82, 2) # Igain
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kD, 80, 2) # Dgain damping
        # self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kD * 0.75), 80, 2) # Dgain damping for side to side should be a bit less
        #Max at current (in unit 1mA) so don't overheat and grip too hard #500 normal or #350 for lite
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.curr_lim, 102, 2)

        # Get Min and max
        self.min, self.max = lhu.LEAP_limits()
        try: 
            #Move motors to 0 position
            self.set_initial_position(self.curr_pos)
            time.sleep(1.0) # Wait before reading positions

            # Read initial state from hardware after delay
            output = self.dxl_client.read_pos_vel()
            self.latest_pos = output[0] - np.pi
            self.latest_vel = output[1]

            # Setup timer for periodic publishing (60 Hz default)
            self.publish_rate = float(rospy.get_param('/leaphand_node/publish_rate', self.frequency))
            self.publish_timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.publish_state)

            # Initialize services after everything is ready
            rospy.Service('leap_position', leap_position, self.pos_srv)
            rospy.Service('leap_velocity', leap_velocity, self.vel_srv)
            # rospy.Service('leap_effort', leap_effort, self.eff_srv)
            rospy.Service('leap_pos_vel', leap_pos_vel, self.pos_vel_srv)
            # rospy.Service('leap_pos_vel_eff', leap_pos_vel_eff, self.pos_vel_eff_srv)

            # Publish state of hand every time you fullfill a service
            self.state_pub = rospy.Publisher('/leap_hand_state', JointState, queue_size=1)
            self.rate = rospy.Rate(self.frequency)

            self.read_thread = threading.Thread(target=self.read_process)
            self.read_thread.daemon = True  
            self.read_thread.start()

            self.write_thread = threading.Thread(target=self.write_process)
            self.write_thread.daemon = True  
            self.write_thread.start()
            
            while not rospy.is_shutdown():
                rospy.spin()
        finally:
            self.dxl_client.set_torque_enabled(motors, False)

    def read_process(self):
        """Runs at 60Hz to read hardware and send commands"""
        while not rospy.is_shutdown():
            try:
                with self.lock:
                    # Read hardware
                    pos, vel = self.dxl_client.read_pos_vel() #! R event
                    self.latest_pos = pos - np.pi
                    self.latest_vel = vel
                    self.rate.sleep()
                    
            except Exception as e:
                rospy.logerr(f"Hardware communication error: {e}")
    
    def write_process(self):
        # Subscribes to a variety of sources that can command the hand, and creates services that can give information about the hand out
        rospy.Subscriber("/leaphand_node/cmd_leap", JointState, self._receive_pose, queue_size=1)
        """Waits for a write command and sends it to hardware"""
        while not rospy.is_shutdown():
            rospy.spin()

    # Receive LEAP pose and directly control the robot.  Fully open here is 180 and increases in this value closes the hand.
    def _receive_pose(self, pose):
        # Clip pose with limits (Current control does not enforce them)
        pose = np.array(pose.position)
        pose = np.clip(pose, self.min, self.max)
        self.prev_pos = self.curr_pos
        
        # Add offset so it is alligned with the simulation LeapHand
        self.curr_pos = pose + np.pi
        with self.lock:
            self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
            # print timestamp that the servos received a command
            msg = Header()
            msg.stamp = rospy.Time.now()
            self.timingpub.publish(msg)
            self.rate.sleep()

    def set_initial_position(self, pose):
        # Clip pose with limits (Current control does not enforce them)
        pose = np.clip(pose, self.min, self.max)
        self.prev_pos = self.curr_pos
        
        # Add offset so it is alligned with the simulation LeapHand
        self.curr_pos = pose + np.pi
        with self.lock:
            self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
            time.sleep(1/self.frequency)



    #Service that reads and returns the pos of the robot in regular LEAP Embodiment scaling.
    def pos_srv(self, req):
        return {"position": self.latest_pos}

    #Service that reads and returns the vel of the robot in LEAP Embodiment
    def vel_srv(self, req):
        return {"velocity": self.latest_vel}

    # #Service that reads and returns the effort/current of the robot in LEAP Embodiment
    # def eff_srv(self, req):
    #     eff = self.dxl_client.read_cur()
    #     self.latest_eff = eff
    #     return {"effort": self.latest_eff}

    #Use these combined services to save a lot of latency if you need multiple datapoints
    def pos_vel_srv(self, req):
        # pos, vel = self.dxl_client.read_pos_vel() #! R event
        # self.latest_pos = pos - np.pi
        # self.latest_vel = vel
        return {"position": self.latest_pos, "velocity": self.latest_vel}

    # #Use these combined services to save a lot of latency if you need multiple datapoints
    # def pos_vel_eff_srv(self, req):
    #     pos, vel, eff = self.dxl_client.read_pos_vel_cur()
    #     self.latest_pos = pos - np.pi
    #     self.latest_vel = vel
    #     self.latest_eff = eff
    #     return {"position": self.latest_pos, "velocity": self.latest_vel, "effort": self.latest_eff}

    def publish_state(self, event=None):
        """Publish the current internal state without reading from hardware."""
        state = JointState()
        state.header.stamp = rospy.Time.now()
        state.name = self.joint_names
        state.position = self.latest_pos
        state.velocity = self.latest_vel
        state.effort = self.latest_eff
        self.state_pub.publish(state)

import argparse
def make_args():
    parser = argparse.ArgumentParser(
        description="Process the args"
    )

    parser.add_argument(
        "--frequency",
        type=float,
        default=60.0, 
        help="Frequency for Reading the Motor position and velocity",
    )

    args = parser.parse_args()
    return args

if __name__ == "__main__":
    # Filter Ros Args
    ros_args = [arg for arg in sys.argv if arg.startswith('__')]
    clean_argv = [arg for arg in sys.argv if not arg.startswith('__')]

    # Temporarily replace sys.argv to exclude ROS arguments
    sys.argv = clean_argv
    args = make_args()

    rospy.init_node("leaphand_node")
    leaphand_node = LeapNode(args.frequency)
