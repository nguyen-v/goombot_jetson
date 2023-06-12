#!/usr/bin/env python
import rospy
import smach
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from std_msgs.msg import Empty

# define state GO_TO_DROP_ZONE
class GoDropZoneState(smach.State):
    def __init__(self):
        self.drop_zone_reached = False
        smach.State.__init__(self, outcomes=['success', 'pause'])
        # self.status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.status_callback)
        self.goal_pub = rospy.Publisher('/goombot/goal', PoseStamped, queue_size=1)
        self.pause_subscriber = rospy.Subscriber('pause_robot', Empty, self.pause_callback)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.is_in_state=False
        self.client.wait_for_server()

        self.pause_robot = False


    def pause_callback(self, msg):
        if self.is_in_state:
            rospy.loginfo("Pause signal received, transitioning to Pause state")
            self.pause_robot = True

    def execute(self, userdata):
        rospy.loginfo("Going to the drop zone")
        self.pause_robot = False
        self.is_in_state = True
        self.drop_zone_reached = False
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

        self.publish_goal(goal_msg)
        # Wait for the navigation stack to reach the drop zone
        # (Implement appropriate wait logic based on feedback or timeout)
        while not rospy.is_shutdown():
            if self.drop_zone_reached:
                rospy.loginfo("Arrived to the drop zone")
                self.is_in_state = False
                return 'success'
            elif self.pause_robot:
                return 'pause'
            rospy.sleep(0.1)
        

    def publish_goal(self, goal_pose):
        if self.is_in_state:
            goal_msg = MoveBaseGoal()
            goal_msg.target_pose.header.stamp = rospy.Time.now()
            goal_msg.target_pose.pose = goal_pose.pose
            goal_msg.target_pose.header.frame_id = 'map'

            # Publish the goal
            # self.goal_publisher.publish(goal_msg)
            self.client.send_goal(goal_msg, done_cb=self.goal_reached_callback)

    def goal_reached_callback(self, state, result):
        if self.is_in_state:
            if state == actionlib.GoalStatus.SUCCEEDED:
                self.drop_zone_reached = True
                rospy.loginfo("Goal reached successfully!")
            else:
                rospy.loginfo("Goal was not reached.")
        
    def status_callback(self, msg):
        for status in msg.status_list:
            if status.status == 3:  # Status 3 indicates a successful goal reach
                self.drop_zone_reached = True