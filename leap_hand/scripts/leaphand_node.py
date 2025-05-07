#!/usr/bin/env python3
import numpy as np
import rospy
import rospkg
import os
import sys


from sensor_msgs.msg import JointState
from std_msgs.msg import String

from leap_hand_utils.dynamixel_client import *
import leap_hand_utils.leap_hand_utils as lhu
from leap_hand.srv import *


#LEAP hand conventions:

#The joint numbering goes from Index (0-3), Middle(4-7), Ring(8-11) to Thumb(12-15) and from MCP Side, MCP Forward, PIP, DIP for each finger.
#For instance, the MCP Side of Index is ID 0, the MCP Forward of Ring is 9, the DIP of Ring is 11

# Author recommends only to query when necessary and below 90 samples a second.  
# Use the combined commands if you can to save time.  Also don't forget about the USB latency settings in the readme.


class LeapNode:
    def __init__(self):
        ####Some parameters to control the hand #! Reduce PD values for less jittery control, Increase for more strength
        self.kP = float(rospy.get_param('/leaphand_node/kP', 800.0)) 
        self.kI = float(rospy.get_param('/leaphand_node/kI', 80.0))
        self.kD = float(rospy.get_param('/leaphand_node/kD', 200.0))
        self.curr_lim = float(rospy.get_param('/leaphand_node/curr_lim', 550.0)) #don't go past 600ma on this, or it'll overcurrent sometimes for regular, 350ma for lite.
        self.prev_pos = self.pos = self.curr_pos = np.zeros(16)
        
        #subscribes to a variety of sources that can command the hand, and creates services that can give information about the hand out
        rospy.Subscriber("/leaphand_node/cmd_leap", JointState, self._receive_pose)

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
        self.dxl_client.sync_write(motors, np.ones(len(motors))*100, 112, 4) # Velocity 
        self.dxl_client.sync_write(motors, np.ones(len(motors))*50, 108, 4) # Acceleration
        self.dxl_client.set_torque_enabled(motors, True)

        # Set parameters for PID control
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kP, 84, 2) # Pgain stiffness     
        self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kP * 0.75), 84, 2) # Pgain stiffness for side to side should be a bit less
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kI, 82, 2) # Igain
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kD, 80, 2) # Dgain damping
        self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kD * 0.75), 80, 2) # Dgain damping for side to side should be a bit less

        #Max at current (in unit 1mA) so don't overheat and grip too hard #500 normal or #350 for lite
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.curr_lim, 102, 2)

        # Get Min and max
        self.min, self.max = lhu.LEAP_limits()

        #Move motors to 0 position
        self.set_initial_position(self.curr_pos)

        # Initialize services after everything is ready
        rospy.Service('leap_position', leap_position, self.pos_srv)
        rospy.Service('leap_velocity', leap_velocity, self.vel_srv)
        rospy.Service('leap_effort', leap_effort, self.eff_srv)
        rospy.Service('leap_pos_vel', leap_pos_vel, self.pos_vel_srv)
        rospy.Service('leap_pos_vel_eff', leap_pos_vel_eff, self.pos_vel_eff_srv)

        # Publish state of hand every time you fullfill a service
        self.state_pub = rospy.Publisher('/leap_hand_state', JointState, queue_size=10)
        
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
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)

    def set_initial_position(self, pose):
        # Clip pose with limits (Current control does not enforce them)
        pose = np.clip(pose, self.min, self.max)
        self.prev_pos = self.curr_pos
        
        # Add offset so it is alligned with the simulation LeapHand
        self.curr_pos = pose + np.pi
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)

    #Service that reads and returns the pos of the robot in regular LEAP Embodiment scaling.
    def pos_srv(self, req):
        response = {"position": self.dxl_client.read_pos() - np.pi}
        self.publish_state(pos = response["position"])
        return response
    #Service that reads and returns the vel of the robot in LEAP Embodiment
    def vel_srv(self, req):
        response = {"velocity": self.dxl_client.read_vel()}
        self.publish_state(vel = response["velocity"])
        return response
    #Service that reads and returns the effort/current of the robot in LEAP Embodiment
    def eff_srv(self, req):
        response = {"effort": self.dxl_client.read_cur()}
        self.publish_state(eff = response["effort"])
        return response
    #Use these combined services to save a lot of latency if you need multiple datapoints
    def pos_vel_srv(self, req):
        output = self.dxl_client.read_pos_vel()
        response = {"position": output[0] - np.pi, "velocity": output[1]}
        self.publish_state(pos = response["position"], vel = response["velocity"])
        return response
    #Use these combined services to save a lot of latency if you need multiple datapoints
    def pos_vel_eff_srv(self, req):
        output = self.dxl_client.read_pos_vel_cur()
        response = {"position": output[0] - np.pi, "velocity": output[1], "effort": output[2]}
        self.publish_state(pos = response["position"], vel = response["velocity"], eff=response["effort"])
        return response
    
    def publish_state(self, pos= [], vel = [], eff = []):
        state = JointState()
        state.header.stamp = rospy.Time.now()
        state.name = self.joint_names
        state.position = pos
        state.velocity = vel
        state.effort = eff 
        self.state_pub.publish(state)

#init the arm node
def main(**kwargs):
    rospy.init_node("leaphand_node")
    leaphand_node = LeapNode()
'''
Init node
'''

if __name__ == "__main__":
    main()