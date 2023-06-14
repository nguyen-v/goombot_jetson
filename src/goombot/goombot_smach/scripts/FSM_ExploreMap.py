import rospy
import random
import smach
import actionlib
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import Empty
import math
import actionlib
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np

class ExploreMapState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['duplo_detected','success','low_time', 'failure', 'pause', 'actuate_button'], input_keys=['init_time', 'explore_state_in'], output_keys=['explore_state_out'])
        #self.points_list = [(1.0, 0.5, 0, 0), (0, 1.5, 0, math.pi/2), (-2, 1, 0, math.pi), (-1, -1, 0, -math.pi/2)]
        self.zone_3_points = [(2, -3, 0, math.pi/4), (-0.5, -3, 0, math.pi)]
        # self.points_list = [(0, 0, 0, 0), (-2.7, 0.46, 0,-3/4*math.pi), (-3.5, 0.9, 0, math.pi/2), (-3.35, 2.5, 0, 3/4*math.pi), (2, 0, 0, -math.pi/4), (1.68, 1.23, 0, 3*math.pi/4), (-0.86, 2.6, 0, math.pi),  (-0.3, 2.73, 0, 0) ]
        self.points_list = [(0, 0, 0, 0), (-2.7, 0.46, 0,-3/4*math.pi), (-3.5, 0.9, 0, math.pi/2), (2, 0, 0, -math.pi/4), (1.68, 1.23, 0, 3*math.pi/4), (-0.86, 2.6, 0, math.pi),  (-0.3, 2.73, 0, 0),(-3.35, 2.5, 0, 3/4*math.pi) ]
        self.button_location = [(-0.704604974204, -3.50959064946, 0, -math.pi/2)]
        self.goal_reached = False
        self.duplo_detected = False
        self.goal_not_reached = False
        self.point = None


        self.loop_rate = rospy.Rate(1)

        self.current_pose = PoseStamped()
        rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.amcl_callback)
        # Create a subscriber for move_base/status topic
        # self.status_subscriber = rospy.Subscriber('move_base/status', GoalStatusArray, self.status_callback)

        # Create a subscriber for the object PoseStamped topic
        self.pose_subscriber = rospy.Subscriber('/closest_duplo_goal_filtered', PoseStamped, self.pose_callback)

        # Create a publisher for the robot goal
        self.goal_publisher = rospy.Publisher('/goombot/goal', PoseStamped, queue_size=1)

        self.pause_subscriber = rospy.Subscriber('pause_robot', Empty, self.pause_callback)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.is_in_state=False
        self.point_index = 0
        self.client.wait_for_server()
        self.init_task_time = rospy.Time.now()
        self.pause_robot = False


    def pause_callback(self, msg):
        if self.is_in_state:
            rospy.loginfo("Pause signal received, transitioning to Pause state")
            self.pause_robot = True
        

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
        # if self.current_pose.pose.position.x >= 1.5 and 2 <= self.current_pose.pose.position.y:
        #     pass
        # else:
        # if self.is_in_state:
        if data is not None:
            if data.pose.position.x >= 1.5 and data.pose.position.y >= 2:
                pass
            else:
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

    def get_point(self, point_list, index):
        # Get a random point from the points list
        point = point_list[index]


        
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
            goal_msg.target_pose.header.frame_id = 'map'

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
        self.pause_robot = False

        init_time = userdata.init_time
        
        # Get a random point from the list
        # point = self.get_random_point()
        
        length = 0
        if userdata.explore_state_in == "EXPLORE_ZONE_3":
            length = len(self.zone_3_points)
        elif userdata.explore_state_in == "EXPLORE_MAP":
            length = len(self.points_list)
        if self.point_index >= length:
            self.point_index = 0

        if userdata.explore_state_in == "GO_TO_BUTTON":
            self.point = self.get_point(self.button_location, 0)
        elif userdata.explore_state_in == "EXPLORE_ZONE_3":
            self.point = self.get_point(self.zone_3_points, self.point_index)
        else:
            self.point = self.get_point(self.points_list, self.point_index)
        
        
        # Publish the goal
        self.publish_goal(self.point)
        
        while not rospy.is_shutdown():

            robot_to_goal_distance = np.linalg.norm([self.point.pose.position.x - self.current_pose.pose.position.x,
                                                     self.point.pose.position.y - self.current_pose.pose.position.y])
            if robot_to_goal_distance < 0.30 and userdata.explore_state_in != "GO_TO_BUTTON":
                rospy.loginfo ("close to explore goal")
                self.is_in_state = False
                self.point_index += 1
                return "success" 
            
            if self.pause_robot:
                return 'pause'
            
            elif self.goal_reached:
                self.is_in_state=False
                if self.point_index < length:
                    self.point_index += 1
                else:
                    rospy.loginfo("Reached last index")
                    if userdata.explore_state_in == "EXPLORE_ZONE_3":
                        userdata.explore_state_out == "EXPLORE_MAP"
                    self.point_index = 0
                self.client.cancel_all_goals()
                if userdata.explore_state_in == "GO_TO_BUTTON":
                    userdata.explore_state_out = "EXPLORE_ZONE_3"
                    rospy.loginfo("Arrived to button")
                return 'actuate_button'
            
            elif self.goal_not_reached:
                self.is_in_state=False
                # self.points_list.pop(self.point_index)
                if self.point_index < length:
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
            
            elif rospy.Time.now() - self.init_task_time > rospy.Duration(15) and userdata.explore_state_in != "GO_TO_BUTTON":
                rospy.loginfo("Go to map waypoint took too long.")
                if self.point_index < length:
                    self.point_index += 1
                else:
                    self.point_index = 0
                self.client.cancel_all_goals()
                self.client.cancel_all_goals()
                self.is_in_state = False
                userdata.explore_state_out = "EXPLORE_MAP"
                return "failure"
            
            elif rospy.Time.now()-init_time>rospy.Duration.from_sec(9*60):
                rospy.loginfo("Low on time. Returning to drop zone.")
                if self.point_index < length:
                    self.point_index += 1
                else:
                    self.point_index = 0
                self.client.cancel_all_goals()
                self.is_in_state=False
                self.client.cancel_all_goals()
                return 'low_time'
            else:
                pass
            
            self.publish_goal(self.point)
            self.loop_rate.sleep()


    def amcl_callback(self, msg):
    # Access the pose information from the amcl_pose message
        self.current_pose = PoseStamped()
        self.current_pose.pose = msg.pose.pose