#!/usr/bin/env python
import rospy
import smach
from std_msgs.msg import String
from geometry_msgs.msg import Twist


#Class Drop zone
class DropDuploState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])
        self.container_pub = rospy.Publisher('/set_dynamixel_pos', String, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo("Executing DropDuplo State.")
        # Open the container
        rospy.loginfo("Opening container")
        self.container_pub.publish('CONTAINER_OPEN')

        # Wait for a specified duration
        rospy.sleep(5.0)

        self.publish_cmd_vel(0.1)
        rospy.sleep(3)
        self.publish_cmd_vel(0.0)

        rospy.sleep(2)


        # Close the container
        rospy.loginfo("Closing container")
        self.container_pub.publish('CONTAINER_CLOSE')

        return 'success'
    
    def publish_cmd_vel(self, linear_x):
        twist_cmd = Twist()
        twist_cmd.linear.x = linear_x
        self.cmd_vel_pub.publish(twist_cmd)