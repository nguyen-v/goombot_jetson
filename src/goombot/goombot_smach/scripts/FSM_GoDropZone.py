#!/usr/bin/env python
import rospy
import smach
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray

# define state GO_TO_DROP_ZONE
class GoDropZoneState(smach.State):
    def __init__(self):
        self.drop_zone_reached = False
        smach.State.__init__(self, outcomes=['success'])
        self.status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.status_callback)
        self.goal_pub = rospy.Publisher('/goombot/goal', PoseStamped, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo("Going to the drop zone")
        # Get the drop zone coordinate (e.g., from a list or service)
        goal_msg = PoseStamped()
        goal_msg.header.stamp = rospy.Time.now()
        
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = 2.5 
        goal_msg.pose.position.y = 3.0 
        goal_msg.pose.position.z = 0.0 

        goal_msg.pose.orientation.w = 0.383
        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = -0.924

        self.goal_pub.publish(goal_msg)
        # Wait for the navigation stack to reach the drop zone
        # (Implement appropriate wait logic based on feedback or timeout)
        while not rospy.is_shutdown():
            if self.drop_zone_reached:
                rospy.loginfo("Arrived to the drop zone")
                self.drop_zone_reached = False
                return 'success'
            rospy.sleep(0.1)
        
    def status_callback(self, msg):
        for status in msg.status_list:
            if status.status == 3:  # Status 3 indicates a successful goal reach
                self.drop_zone_reached = True