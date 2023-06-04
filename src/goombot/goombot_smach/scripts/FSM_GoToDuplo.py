#!/usr/bin/env python
import rospy
import smach
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
from nav_msgs.msg import Odometry
import numpy as np

# Define state GO_TO_DUPLO
class GoToDuploState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["success", "failure", "low_time"], input_keys=['init_time'])

        # Initialize the last_detection_time
        self.last_detection_time = rospy.Time.now()

        # Subscribe to closest_duplo_filtered topic
        self.closest_duplo_sub = rospy.Subscriber("/closest_duplo_goal_filtered", PoseStamped, self.closest_duplo_callback)
        self.status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.status_callback)
        self.init_task_time = rospy.Time.now()
        self.goal = PoseStamped()
        self.current_pose = PoseStamped()

        # Subscribe to the odometry topic
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)

        self.goal_pub = rospy.Publisher('/goombot/goal', PoseStamped, queue_size=1)

        self.duplo_reached = False

    def execute(self, userdata):
        rospy.loginfo("Executing GoToDuploState")

        self.last_detection_time = rospy.Time.now()
        self.init_task_time = rospy.Time.now()

        init_time = userdata.init_time

        # Loop until a condition is met
        while not rospy.is_shutdown():
            # Publish goal to /goal_loop
            

            # Check if distance between robot and goal is less than 20 cm
            robot_to_goal_distance = np.linalg.norm([self.goal.pose.position.x - self.current_pose.pose.position.x,
                                                     self.goal.pose.position.y - self.current_pose.pose.position.y])
            if robot_to_goal_distance < 0.2:
                return "success"
            
            if self.duplo_reached:
                self.duplo_reached = False
                return "success"

            # Check if no detection received for 5 seconds
            if rospy.Time.now() - self.last_detection_time > rospy.Duration(5):
                return "failure"

            # Check if task initialization time exceeds 15 seconds
            if rospy.Time.now() - self.init_task_time > rospy.Duration(15):
                return "failure"

            # Check if total task execution time exceeds 9 minutes
            if rospy.Time.now() - init_time > rospy.Duration(540):
                return "low_time"

            rospy.sleep(0.1)  # Control the loop rate

    def closest_duplo_callback(self, msg):
        # Update last_detection_time when new detection received
        self.goal = msg
        self.goal_pub.publish(self.goal)
        self.last_detection_time = rospy.Time.now()

    def odom_callback(self, msg):
        # Update current_pose using odometry data
        self.current_pose = PoseStamped()
        self.current_pose.pose = msg.pose.pose

    def status_callback(self, msg):
        for status in msg.status_list:
            if status.status == 3:  # Status 3 indicates a successful goal reach
                self.duplo_reached = True

