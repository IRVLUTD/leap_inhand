#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from leap_hand.srv import leap_pos_vel

class LeapHandStatePublisher:
    def __init__(self, frequency):
        # Define joint names: joint_0 to joint_15
        self.joint_names = [f"joint_{i}" for i in range(16)]
        # Set up publisher for /leap_hand_state topic
        self.pub = rospy.Publisher('/leap_hand_state', JointState, queue_size=10)
        # Set publishing rate based on input frequency
        self.rate = rospy.Rate(frequency)
        # Service names
        self.pos_vel_service = '/leap_pos_vel'
        
        # Wait for services to be available
        rospy.wait_for_service(self.pos_vel_service)
        
        # Create service proxies
        self.get_pos_vel = rospy.ServiceProxy(self.pos_vel_service, leap_pos_vel)
    
    def publish_state(self):
        # Main publishing loop
        while not rospy.is_shutdown():
            try:
                # Call services to get position and velocity
                response = self.get_pos_vel()
                
                # Create and populate JointState message
                state = JointState()
                state.header.stamp = rospy.Time.now()
                state.name = self.joint_names
                state.position = response.position
                state.velocity = response.velocity
                state.effort = []  
                
                # Publish the message
                self.pub.publish(state)
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
            
            # Sleep to maintain the specified frequency
            self.rate.sleep()

if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('leap_hand_state_publisher', anonymous=True)
    # Get frequency parameter, default to 60 Hz if not set
    frequency = rospy.get_param('~reading_frequency', 30.0)
    # Create publisher instance and start publishing
    publisher = LeapHandStatePublisher(frequency)
    publisher.publish_state()