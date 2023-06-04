import rospy
import random
import smach
import actionlib
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatusArray
import math
from tf.transformations import quaternion_from_euler

class ExploreMapState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['duplo_detected','success','low_time'], input_keys=['init_time'])
        self.points_list = [(1.0, 0, 0,math.pi/2), (0, 1, 0, 0), (-1, 0, 0, -math.pi/2), (0, -1, 0, -math.pi)]
        self.goal_reached = False
        self.duplo_detected = False

        # Create a subscriber for move_base/status topic
        self.status_subscriber = rospy.Subscriber('move_base/status', GoalStatusArray, self.status_callback)

        # Create a subscriber for the object PoseStamped topic
        self.pose_subscriber = rospy.Subscriber('/closest_duplo_goal_filtered', PoseStamped, self.pose_callback)

        # Create a publisher for the robot goal
        self.goal_publisher = rospy.Publisher('/goombot/goal', PoseStamped, queue_size=1)

    def status_callback(self, data):
        # Check if there is any goal status in the status array
        if len(data.status_list) > 0:
            # Get the status of the latest goal
            latest_status = data.status_list[-1].status
            # Get the status of the latest goal
            self.goal_status = latest_status
            # Check if the latest goal status is 'SUCCEEDED' (reached goal)
            if latest_status == 3:
                rospy.loginfo("Goal reached!")
                self.goal_reached = True
            
    def pose_callback(self, data):
        # Check if the PoseStamped is not null
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

    def publish_goal(self, goal_pose):
        goal_msg = PoseStamped()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.pose = goal_pose.pose

        # Publish the goal
        self.goal_publisher.publish(goal_msg)

    def execute(self, userdata):
        rospy.loginfo("Executing GoToPointState")
        
        # Get a random point from the list
        point = self.get_random_point()
        
        init_time = userdata.init_time
        # Publish the goal
        self.publish_goal(point)
        
        while not rospy.is_shutdown():
            if self.goal_reached:
                return 'success'
            
            if self.duplo_detected:
                return 'duplo_detected'

            if rospy.Time.now()-init_time>rospy.Duration.from_sec(9*60):
                return 'low_time'