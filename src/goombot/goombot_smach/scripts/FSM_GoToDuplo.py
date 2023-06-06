#!/usr/bin/env python
import rospy
import smach
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import numpy as np

# Define state GO_TO_DUPLO
class GoToDuploState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["success", "failure", "low_time"], input_keys=['init_time'])

        # Initialize the last_detection_time
        self.last_detection_time = rospy.Time.now()

        # Subscribe to closest_duplo_filtered topic
        self.init_task_time = rospy.Time.now()
        self.goal = None
        self.current_pose = PoseStamped()

        self.duplo_reached = False

        # Subscribe to the odometry topic
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)

        self.goal_pub = rospy.Publisher('/goombot/goal', PoseStamped, queue_size=1)

        self.republish_goal_pub = rospy.Publisher('/republish_goal', Bool, queue_size=1)

        self.closest_duplo_sub = rospy.Subscriber("/closest_duplo_goal_filtered", PoseStamped, self.closest_duplo_callback)
        self.status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.status_callback)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        rospy.loginfo("Executing GoToDuploState")
        self.goal = None

        self.last_detection_time = rospy.Time.now()
        self.init_task_time = rospy.Time.now()

        msg = Bool()
        msg.data = True
        self.republish_goal_pub.publish(msg)

        init_time = userdata.init_time

        


        while self.goal is None:
            if rospy.Time.now() - self.last_detection_time > rospy.Duration(5):
                return "failure"
            rospy.sleep(1)

        # Loop until a condition is met
        while not rospy.is_shutdown():
            # Publish goal to /goal_loop
            
        
            # Check if distance between robot and goal is less than 20 cm
            # robot_to_goal_distance = np.linalg.norm([self.goal.pose.position.x - self.current_pose.pose.position.x,
            #                                          self.goal.pose.position.y - self.current_pose.pose.position.y])
            # if robot_to_goal_distance < 0.5:
            #     rospy.loginfo ("close to duplo")
            #     return "success" 
            
            if self.duplo_reached:
                self.client.cancel_goal()
                self.duplo_reached = False
                return "success"

            # Check if no detection received for 5 seconds
            if rospy.Time.now() - self.last_detection_time > rospy.Duration(5):
                rospy.loginfo("Go to duplo hasn't seen a duplo in 5s.")
                self.client.cancel_goal()
                return "failure"

            # Check if task initialization time exceeds 15 seconds
            if rospy.Time.now() - self.init_task_time > rospy.Duration(30):
                rospy.loginfo("Go to duplo took too long.")
                self.client.cancel_goal()
                return "failure"

            # Check if total task execution time exceeds 9 minutes
            if rospy.Time.now() - init_time > rospy.Duration(540):
                rospy.loginfo("Low on time.")
                self.client.cancel_goal()
                return "low_time"

        #     rospy.sleep(0.1)  # Control the loop rate
        self.client.wait_for_result()
        return "success"

    def publish_goal(self, goal_pose):
        goal_msg = MoveBaseGoal()
        goal_msg.target_pose.header.stamp = rospy.Time.now()
        goal_msg.target_pose.pose = goal_pose.pose
        goal_msg.target_pose.header.frame_id = 'odom'

        # Publish the goal
        # self.goal_publisher.publish(goal_msg)
        self.client.send_goal(goal_msg, done_cb=self.goal_reached_callback)

    def goal_reached_callback(self, state, result):
        if state == actionlib.GoalStatus.SUCCEEDED:
            self.duplo_reached = True
            rospy.loginfo("Duplo reached successfully!")
        else:
            rospy.loginfo("Goal was not reached.")

    def closest_duplo_callback(self, msg):
        # Update last_detection_time when new detection received
        self.goal = msg
        # self.goal_pub.publish(self.goal)
        # self.client.send_goal(self.goal)
        self.publish_goal(self.goal)
        self.last_detection_time = rospy.Time.now()

    def odom_callback(self, msg):
        # Update current_pose using odometry data
        self.current_pose = PoseStamped()
        self.current_pose.pose = msg.pose.pose

    def status_callback(self, msg):
        if self.goal is not None:
            for status in msg.status_list:
                if status.status == 3:  # Status 3 indicates a successful goal reach
                    # rospy.loginfo("Goal reached in GoToDuplo")
                    pass
                    # self.duplo_reached = True

