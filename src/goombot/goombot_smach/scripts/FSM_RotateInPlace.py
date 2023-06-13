#!/usr/bin/env python

import rospy
import smach
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import tf
import math
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseWithCovarianceStamped

class RotateInPlaceState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['no_duplo', 'duplo_detected', 'low_time', 'pause'], input_keys=['init_time', 'explore_state'])

        self.initial_yaw = None
        self.angular_speed = 0.3  # Adjust the angular speed as per your requirements
        self.goal_reached = False
        self.duplo_detected = False
        self.out_of_time = False
        self.current_yaw = None
        self.timer = None

        self.current_pose = PoseStamped()
        self.republisher_pub = rospy.Publisher('/republish_goal', Bool, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.closest_duplo_sub = rospy.Subscriber('/closest_duplo_goal_filtered', PoseStamped, self.duplo_callback)
        self.is_in_state=False
        self.pause_subscriber = rospy.Subscriber('pause_robot', Empty, self.pause_callback)

        self.pause_robot = False


    def pause_callback(self, msg):
        if self.is_in_state:
            rospy.loginfo("Pause signal received, transitioning to Pause state")
            self.pause_robot = True

    def odom_callback(self, msg):
        if self.initial_yaw is None:
            # Retrieve robot orientation from the /odom message
            quaternion = (
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            )
            # Convert the quaternion to roll, pitch, and yaw angles
            _, _, self.initial_yaw = tf.transformations.euler_from_quaternion(quaternion)
        
        # Calculate the current yaw angle
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        _, _, self.current_yaw = tf.transformations.euler_from_quaternion(quaternion)

    def duplo_callback(self, msg):
        if self.is_in_state:
            if msg is not None:
                if msg.pose.position.x >= 1.5 and msg.pose.position.y >= 2:
                    pass
                else:
                    self.duplo_detected = True

    def normalize_angle(self, angle):
        angle = math.fmod(angle, 2 * math.pi)
        if angle < -math.pi:
            angle += 2 * math.pi
        elif angle > math.pi:
            angle -= 2 * math.pi
        return angle
    def calculate_angle_difference(self, A, B):
        # Convert A and B to the range 0 to 2pi
        # A_normalized = (A + 2 * math.pi) % (2 * math.pi)
        diff = A-B
        if diff < 0:
            diff += 2*math.pi
        diff = abs(diff % (2*math.pi))

        # B_normalized = (A-B + 2 * math.pi) % (2 * math.pi)

        # # Calculate the absolute difference between A and B
        # difference = abs(B_normalized)
        # rospy.loginfo("current %f, init %f, diff %f", A_normalized, B_normalized, difference)
        rospy.loginfo("diff %f", diff)

        return diff
    
    def timer_callback(self, event):
        rospy.loginfo("timer callback")

        # Stop the timer
        self.timer.shutdown()
        self.goal_reached = True
        # pass
    

    def execute(self, userdata):
        rospy.loginfo('Executing RotateInPlace state')
        self.pause_robot = False
        self.is_in_state=True

        # Publish False on /republish_goal
        self.republisher_pub.publish(Bool(False))

        init_time = userdata.init_time
        explore_state = userdata.explore_state

        # Rotate until a full 360-degree rotation is completed or duplo is detected
        self.timer = rospy.Timer(rospy.Duration(2*math.pi/self.angular_speed), self.timer_callback)

        self.goal_reached = False
        
        while not rospy.is_shutdown() and not self.goal_reached and not self.duplo_detected and not self.out_of_time:
            if explore_state == "GO_TO_BUTTON":
                self.goal_reached = False
                rospy.loginfo('Transitioning to explore map with button goal')
                self.is_in_state=False
                return "no_duplo"
            # if self.current_yaw is not None:
                # Calculate the difference between the current yaw and initial yaw
                # yaw_diff = self.calculate_angle_difference(self.current_yaw, self.initial_yaw)

                # # Check if the goal (270-degree rotation) is reached
                # # rospy.loginfo("yaw diff %f", yaw_diff)
                # if yaw_diff >= 1.5 * math.pi:
                    
                #     self.goal_reached = True

            # Publish a twist command for rotation
            twist_cmd = Twist()
            twist_cmd.angular.z = self.angular_speed
            self.cmd_vel_pub.publish(twist_cmd)

            if rospy.Time.now() - init_time > rospy.Duration(540):
                self.out_of_time = True

            rospy.sleep(0.1)  # Sleep for a short duration

        stop_cmd = Twist()
        stop_cmd.angular.z = 0
        self.cmd_vel_pub.publish(stop_cmd)
        # Publish False on /republish_goal
        self.republisher_pub.publish(Bool(False))
        self.initial_yaw = None
        if self.pause_robot:
            self.is_in_state = False
            return 'pause'
        if self.duplo_detected:
            self.duplo_detected = False
            rospy.loginfo('Duplo detected')
            self.is_in_state=False
            return 'duplo_detected'
        elif self.out_of_time:
            self.out_of_time = False
            rospy.loginfo('Out of time, returning to drop zone')
            self.is_in_state=False
            return 'low_time'
        else:
            self.goal_reached = False
            rospy.loginfo('No duplo detected')
            self.is_in_state=False
            return 'no_duplo'
        