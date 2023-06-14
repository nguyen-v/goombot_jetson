#!/usr/bin/env python
import rospy
import smach
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
# from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import numpy as np
from std_msgs.msg import Empty

from geometry_msgs.msg import PoseWithCovarianceStamped

# Define state GO_TO_DUPLO
class GoToDuploState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["success", "failure", "low_time", "pause"], input_keys=['init_time'])
        self.is_in_state = False
        # Initialize the last_detection_time
        self.last_detection_time = rospy.Time.now()

        # Subscribe to closest_duplo_filtered topic
        self.init_task_time = rospy.Time.now()
        self.goal = None
        self.current_pose = PoseStamped()

        self.duplo_reached = False
        self.manual_control = False

        self.duplo_pose_rel = None

        # Subscribe to the odometry topic
        # self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.amcl_callback)


        self.goal_pub = rospy.Publisher('/goombot/goal', PoseStamped, queue_size=1)

        self.republish_goal_pub = rospy.Publisher('/republish_goal', Bool, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.closest_duplo_sub = rospy.Subscriber("/closest_duplo_goal_filtered", PoseStamped, self.closest_duplo_callback)
        self.closest_duplo_camera_sub = rospy.Subscriber("/closest_duplo_camera", PoseStamped, self.closest_duplo_camera_callback)
        self.status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.status_callback)
        self.pause_subscriber = rospy.Subscriber('pause_robot', Empty, self.pause_callback)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        
        self.last_publish_time = rospy.Time.now()

        self.pause_robot = False

    def publish_cmd_vel(self, lin_vel, ang_vel):
            twist_cmd = Twist()
            twist_cmd.linear.x = lin_vel
            twist_cmd.angular.z = ang_vel
            self.cmd_vel_pub.publish(twist_cmd)

    def pause_callback(self, msg):
        if self.is_in_state:
            rospy.loginfo("Pause signal received, transitioning to Pause state")
            self.pause_robot = True

    def execute(self, userdata):
        rospy.loginfo("Executing GoToDuploState")

        self.manual_control = False

        self.pause_robot = False
        self.is_in_state = True
        self.duplo_reached = False

        self.closest_duplo_sub = rospy.Subscriber("/closest_duplo_goal_filtered", PoseStamped, self.closest_duplo_callback)
        self.closest_duplo_camera_sub = rospy.Subscriber("/closest_duplo_camera", PoseStamped, self.closest_duplo_camera_callback)

        self.goal = None

        self.last_detection_time = rospy.Time.now()
        self.init_task_time = rospy.Time.now()

        msg = Bool()
        msg.data = True
        self.republish_goal_pub.publish(msg)

        init_time = userdata.init_time

        


        while self.goal is None:
            if rospy.Time.now() - self.last_detection_time > rospy.Duration(5):
                self.closest_duplo_sub.unregister()
                self.is_in_state = False
                return "failure"
            rospy.sleep(1)

        # Loop until a condition is met
        while not rospy.is_shutdown():
            # Publish goal to /goal_loop
            
        
            # Check if distance between robot and goal is less than 20 cm
            robot_to_goal_distance = np.linalg.norm([self.goal.pose.position.x - self.current_pose.pose.position.x,
                                                     self.goal.pose.position.y - self.current_pose.pose.position.y])
            rospy.loginfo("Distance to duplo: %f", robot_to_goal_distance)
            # if robot_to_goal_distance < 0.15:
            #     rospy.loginfo ("close to duplo")
            #     self.is_in_state = False
            #     return "success" 
            if robot_to_goal_distance < 0.5:
                rospy.loginfo ("transitioning to manual alignment")
                self.manual_control = True
                self.client.cancel_goal()

            if self.manual_control:
                if self.duplo_pose_rel is not None:
                    self.publish_cmd_vel(0.5*self.duplo_pose_rel.pose.position.x, 0.5*self.duplo_pose_rel.pose.position.y)
                    rospy.loginfo("Manual control: x = %f, y = %f", self.duplo_pose_rel.pose.position.x, self.duplo_pose_rel.pose.position.y)
                    if self.duplo_pose_rel.pose.position.x < 0.1:
                        self.duplo_reached = True

            
            if self.duplo_reached:
                self.client.cancel_goal()
                self.closest_duplo_sub.unregister()
                self.closest_duplo_camera_sub.unregister()
                self.duplo_reached = False
                self.is_in_state = False
                return "success"

            # Check if no detection received for 5 seconds
            if rospy.Time.now() - self.last_detection_time > rospy.Duration(5):
                rospy.loginfo("Go to duplo hasn't seen a duplo in 5s.")
                self.closest_duplo_sub.unregister()
                self.closest_duplo_camera_sub.unregister()
                self.client.cancel_goal()
                self.is_in_state = False
                return "failure"

            # Check if task initialization time exceeds 15 seconds
            if rospy.Time.now() - self.init_task_time > rospy.Duration(30):
                rospy.loginfo("Go to duplo took too long.")
                self.closest_duplo_sub.unregister()
                self.closest_duplo_camera_sub.unregister()
                self.client.cancel_goal()
                self.is_in_state = False
                return "failure"

            # Check if total task execution time exceeds 9 minutes
            if rospy.Time.now() - init_time > rospy.Duration(540):
                rospy.loginfo("Low on time.")
                self.closest_duplo_sub.unregister()
                self.closest_duplo_camera_sub.unregister()
                self.client.cancel_goal()
                self.is_in_state = False
                return "low_time"
            
            if self.pause_robot:
                self.is_in_state = False
                return 'pause'

        #     rospy.sleep(0.1)  # Control the loop rate
        # self.client.wait_for_result()
        # return "success"

    def publish_goal(self, goal_pose):
        if self.is_in_state:
            goal_msg = MoveBaseGoal()
            goal_msg.target_pose.header.stamp = rospy.Time.now()
            goal_msg.target_pose.pose = goal_pose.pose
            goal_msg.target_pose.header.frame_id = 'map'

            # Publish the goal
            # self.goal_publisher.publish(goal_msg)
            self.client.send_goal(goal_msg, done_cb=self.goal_reached_callback)
            self.last_publish_time = rospy.Time.now()

    def goal_reached_callback(self, state, result):
        if state == actionlib.GoalStatus.SUCCEEDED:
            self.duplo_reached = True
            rospy.loginfo("Duplo reached successfully!")
        else:
            self.duplo_reached = False
            rospy.loginfo("Goal was not reached.")

    # def closest_duplo_callback(self, msg):
    #     if self.is_in_state:
    #         # Update last_detection_time when new detection received
    #         self.goal = msg
    #         # self.goal_pub.publish(self.goal)
    #         # self.client.send_goal(self.goal)
    #         rospy.loginfo("publishing goal gotoduplo")
    #         self.publish_goal(self.goal)

    #         self.last_detection_time = rospy.Time.now()
    def closest_duplo_callback(self, msg):
        if self.is_in_state:
            if msg.pose.position.x >= 1.5 and msg.pose.position.y >= 2:
                pass
            else:
                current_time = rospy.Time.now()
                time_since_last_publish = current_time - self.last_publish_time
                if self.manual_control == False:
                    if time_since_last_publish >= rospy.Duration(0.5):  # Limit the frequency to 1 publish per second
                        self.goal = msg
                        self.publish_goal(self.goal)

                        self.last_detection_time = current_time
    def closest_duplo_camera_callback(self, msg):
        if self.is_in_state:
            self.duplo_pose_rel = msg

    def odom_callback(self, msg):
        # Update current_pose using odometry data
        self.current_pose = PoseStamped()
        self.current_pose.pose = msg.pose.pose

    def amcl_callback(self, msg):
     # Access the pose information from the amcl_pose message
        self.current_pose = PoseStamped()
        self.current_pose.pose = msg.pose.pose

    def status_callback(self, msg):
        if self.goal is not None:
            for status in msg.status_list:
                if status.status == 3:  # Status 3 indicates a successful goal reach
                    # rospy.loginfo("Goal reached in GoToDuplo")
                    pass
                    # self.duplo_reached = True

