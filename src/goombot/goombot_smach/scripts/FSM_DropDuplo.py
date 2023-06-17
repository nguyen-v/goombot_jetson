#!/usr/bin/env python
import rospy
import smach
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty


#Class Drop zone
class DropDuploState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'pause'])
        self.container_pub = rospy.Publisher('/servo_command', String, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pause_subscriber = rospy.Subscriber('pause_robot', Empty, self.pause_callback)

        self.pause_robot = False

        self.is_in_state = False


    def pause_callback(self, msg):
        if self.is_in_state:
            rospy.loginfo("Pause signal received, transitioning to Pause state")
            self.pause_robot = True

    def execute(self, userdata):
        self.pause_robot = False
        self.is_in_state = True
        
        rospy.loginfo("Executing DropDuplo State.")
        # Go back a bit
        self.publish_cmd_vel(-0.1)
        rospy.sleep(2)
        self.publish_cmd_vel(0.0)
        rospy.sleep(1)
        # Open the container
        rospy.loginfo("Opening container")
        self.container_pub.publish('CONTAINER_OPEN')

        # Wait for a specified duration
        rospy.sleep(5.0)

        self.publish_cmd_vel(0.1)
        rospy.sleep(5)
        self.publish_cmd_vel(0.0)

        rospy.sleep(2)


        # Close the container
        rospy.loginfo("Closing container")
        self.container_pub.publish('CONTAINER_CLOSE')
        self.is_in_state = False
        # if self.pause_robot:
        return 'pause'
        # return 'success'
    
    def publish_cmd_vel(self, linear_x):
        twist_cmd = Twist()
        twist_cmd.linear.x = linear_x
        self.cmd_vel_pub.publish(twist_cmd)