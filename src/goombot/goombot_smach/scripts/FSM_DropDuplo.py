#!/usr/bin/env python
import rospy
import smach
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import math
import numpy as np
import tf

from geometry_msgs.msg import PoseWithCovarianceStamped


#Class Drop zone
class DropDuploState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'pause'], input_keys=['init_time', 'explore_state_in'])
        self.container_pub = rospy.Publisher('/servo_command', String, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pause_subscriber = rospy.Subscriber('pause_robot', Empty, self.pause_callback)

        self.pause_robot = False

        self.is_in_state = False

        self.angle_reached = False

        self.desired_angle = -3/4*math.pi

        self.rotation_speed = 0.3

        self.P_gain = 0.1

        rospy.Subscriber("/robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, self.odom_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)


    def pause_callback(self, msg):
        if self.is_in_state:
            rospy.loginfo("Pause signal received, transitioning to Pause state")
            self.pause_robot = True

    def execute(self, userdata):
        self.pause_robot = False
        self.is_in_state = True
        self.angle_reached = False
        init_time = userdata.init_time
        while not rospy.is_shutdown() and self.angle_reached == False:
            if self.current_yaw is not None and self.angle_reached is not True:
                    angle_error = self.angle_difference(self.desired_angle, self.current_yaw)
                    self.publish_cmd_vel(0, -self.rotation_speed*np.sign(angle_error)-self.P_gain*angle_error)
                    rospy.loginfo("Current angle error %f", angle_error)
                    if np.abs(angle_error) < 0.05:
                        self.publish_cmd_vel(0, 0)
                        self.angle_reached = True
                        rospy.loginfo("Angle reached")
        
        rospy.loginfo("Executing DropDuplo State.")
        # Rotate to correct orientation

        # Open the container
        rospy.loginfo("Opening container")
        self.container_pub.publish('CONTAINER_OPEN')

        # Wait for a specified duration
        rospy.sleep(5.0)

        self.publish_cmd_vel(0.1, 0)
        rospy.sleep(3)
        self.publish_cmd_vel(0.0, 0)

        rospy.sleep(2)


        # Close the container
        rospy.loginfo("Closing container")
        self.container_pub.publish('CONTAINER_CLOSE')
        self.is_in_state = False
        if self.pause_robot or rospy.Time.now()-init_time>rospy.Duration.from_sec(9*60):
            return 'pause'
        return 'success'
    
    def publish_cmd_vel(self, linear_x, angular_z):
        twist_cmd = Twist()
        twist_cmd.linear.x = linear_x
        twist_cmd.angular.z = angular_z
        self.cmd_vel_pub.publish(twist_cmd)

    def angle_difference(self, angle1, angle2):
        # Normalize the angles to be between -pi and pi
        angle1 = math.atan2(math.sin(angle1), math.cos(angle1))
        angle2 = math.atan2(math.sin(angle2), math.cos(angle2))

        # Calculate the difference between the normalized angles
        difference = angle2 - angle1

        # Normalize the difference to be between -pi and pi
        difference = math.atan2(math.sin(difference), math.cos(difference))

        return difference
    
    def odom_callback(self, msg):
     # Access the pose information from the amcl_pose message
        if self.is_in_state and msg is not None:
        
            # Calculate the current yaw angle
            quaternion = (
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            )
            _, _, self.current_yaw = tf.transformations.euler_from_quaternion(quaternion)