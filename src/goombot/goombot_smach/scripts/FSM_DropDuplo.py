#!/usr/bin/env python
import rospy
import smach
from std_msgs.msg import String


#Class Drop zone
class DropDuploState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])
        self.container_pub = rospy.Publisher('/set_dynamixel_pos', String, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo("Executing DropDuplo State.")
        # Open the container
        rospy.loginfo("Opening container")
        self.container_pub.publish('OPEN_CONTAINER')

        # Wait for a specified duration
        rospy.sleep(10.0)

        # Close the container
        rospy.loginfo("Closing container")
        self.container_pub.publish('CLOSE_CONTAINER')

        return 'success'