import rospy
import random
import smach
import actionlib
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatusArray
import math
import actionlib
from tf.transformations import quaternion_from_euler

class ExploreMapState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['duplo_detected','success','low_time', 'failure'], input_keys=['init_time'])
        self.points_list = [(1.0, 0.5, 0, 0), (0, 1.5, 0, math.pi/2), (-2, 1, 0, math.pi), (-1, -1, 0, -math.pi/2)]
        self.goal_reached = False
        self.duplo_detected = False
        self.goal_not_reached = False

        self.loop_rate = rospy.Rate(1)

        # Create a subscriber for move_base/status topic
        # self.status_subscriber = rospy.Subscriber('move_base/status', GoalStatusArray, self.status_callback)

        # Create a subscriber for the object PoseStamped topic
        self.pose_subscriber = rospy.Subscriber('/closest_duplo_goal_filtered', PoseStamped, self.pose_callback)

        # Create a publisher for the robot goal
        self.goal_publisher = rospy.Publisher('/goombot/goal', PoseStamped, queue_size=1)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.is_in_state=False
        self.point_index = 0
        self.client.wait_for_server()
        self.init_task_time = rospy.Time.now()

    def status_callback(self, data):
        if self.is_in_state:
            # Check if there is any goal status in the status array
            if len(data.status_list) > 0:
                # Get the status of the latest goal
                latest_status = data.status_list[-1].status
                # Get the status of the latest goal
                self.goal_status = latest_status
                # Check if the latest goal status is 'SUCCEEDED' (reached goal)
                if latest_status == 3:
                    # rospy.loginfo("Goal reached!")
                    self.goal_reached = True
                
    def pose_callback(self, data):
        # Check if the PoseStamped is not null
        if self.is_in_state:
            if data is not None:
                self.duplo_detected = True

    def get_random_point(self):
        # Get a random point from the points list
        point = random.choice(self.points_list)


        
        # Create a quaternion from the Euler angles (roll=0, pitch=0, yaw=orientation)
        quaternion = quaternion_from_euler(0, 0, point[3])
        
        # Create a PoseStamped message with the selected point coordinates and orientation
        goal_pose = PoseStamped()
        goal_pose.pose.position.x = point[0]
        goal_pose.pose.position.y = point[1]
        goal_pose.pose.position.z = point[2]
        goal_pose.pose.orientation.x = quaternion[0]
        goal_pose.pose.orientation.y = quaternion[1]
        goal_pose.pose.orientation.z = quaternion[2]
        goal_pose.pose.orientation.w = quaternion[3]
        
        return goal_pose

    def get_point(self, index):
        # Get a random point from the points list
        point = self.points_list[index]


        
        # Create a quaternion from the Euler angles (roll=0, pitch=0, yaw=orientation)
        quaternion = quaternion_from_euler(0, 0, point[3])
        
        # Create a PoseStamped message with the selected point coordinates and orientation
        goal_pose = PoseStamped()
        goal_pose.pose.position.x = point[0]
        goal_pose.pose.position.y = point[1]
        goal_pose.pose.position.z = point[2]
        goal_pose.pose.orientation.x = quaternion[0]
        goal_pose.pose.orientation.y = quaternion[1]
        goal_pose.pose.orientation.z = quaternion[2]
        goal_pose.pose.orientation.w = quaternion[3]
        
        return goal_pose

    def publish_goal(self, goal_pose):
        if self.is_in_state:
            goal_msg = MoveBaseGoal()
            goal_msg.target_pose.header.stamp = rospy.Time.now()
            goal_msg.target_pose.pose = goal_pose.pose
            goal_msg.target_pose.header.frame_id = 'odom'

            # Publish the goal
            # self.goal_publisher.publish(goal_msg)
            self.client.send_goal(goal_msg, done_cb=self.goal_reached_callback)

    def goal_reached_callback(self, state, result):
        if self.is_in_state:
            if state == actionlib.GoalStatus.SUCCEEDED:
                self.goal_reached = True
                rospy.loginfo("Goal reached successfully!")
            else:
                self.goal_not_reached = True
                rospy.loginfo("Goal was not reached.")

    def execute(self, userdata):
        rospy.loginfo("Executing ExploreMap State")
        self.init_task_time = rospy.Time.now()
        self.is_in_state=True
        self.goal_reached = False
        self.goal_not_reached = False
        self.duplo_detected = False
        
        # Get a random point from the list
        # point = self.get_random_point()
        if self.point_index >= len(self.points_list):
            self.point_index = 0
        point = self.get_point(self.point_index)
        
        init_time = userdata.init_time
        # Publish the goal
        self.publish_goal(point)
        
        while not rospy.is_shutdown():
            if self.goal_reached:
                self.is_in_state=False
                if self.point_index < len(self.points_list):
                    self.point_index += 1
                else:
                    self.point_index = 0
                self.client.cancel_all_goals()
                return 'success'
            
            elif self.goal_not_reached:
                self.is_in_state=False
                # self.points_list.pop(self.point_index)
                if self.point_index < len(self.points_list):
                    self.point_index += 1
                else:
                    self.point_index = 0
                self.client.cancel_all_goals()
                return 'failure'
            
            elif self.duplo_detected:
                self.is_in_state=False
                # self.client.cancel_all_goals()
                self.client.cancel_all_goals()
                return 'duplo_detected'
            
            # Check if task initialization time exceeds 15 seconds
            elif rospy.Time.now() - self.init_task_time > rospy.Duration(30):
                rospy.loginfo("Go to map waypoint took too long.")
                if self.point_index < len(self.points_list):
                    self.point_index += 1
                else:
                    self.point_index = 0
                self.client.cancel_all_goals()
                self.client.cancel_all_goals()
                self.is_in_state = False
                return "failure"
            
            elif rospy.Time.now()-init_time>rospy.Duration.from_sec(9*60):
                rospy.loginfo("Low on time. Returning to drop zone.")
                if self.point_index < len(self.points_list):
                    self.point_index += 1
                else:
                    self.point_index = 0
                self.client.cancel_all_goals()
                self.is_in_state=False
                self.client.cancel_all_goals()
                return 'low_time'
            else:
                pass
            
            self.publish_goal(point)
            self.loop_rate.sleep()