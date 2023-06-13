#!/usr/bin/env python
import rospy
import smach
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import numpy as np
import tf
import math

from geometry_msgs.msg import PoseWithCovarianceStamped

# Define state GO_TO_DUPLO
class ActuateButtonState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["success"])
        self.is_in_state = False
        self.current_yaw = None
        self.desired_angle = -math.pi/2
        self.angle_reached = False
        self.button_pressed = False
        self.clear_door = False
        self.returned = False
        self.approached = False
        self.P_gain = 0.1
        self.rotation_speed = 0.2
        
        rospy.Subscriber("/robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, self.odom_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)


    def execute(self, userdata):
        rospy.loginfo("Executing Actuate button")

        self.is_in_state = True
        self.current_yaw = None
        self.angle_reached = False
        self.button_pressed = False
        self.clear_door = False
        self.returned = False
        self.approached = False

        while not rospy.is_shutdown() and self.clear_door == False:
            if self.current_yaw is not None and self.angle_reached is not True:
                angle_error = self.angle_difference(self.desired_angle, self.current_yaw)
                self.publish_vel(0, -self.rotation_speed*np.sign(angle_error)-self.P_gain*angle_error)
                rospy.loginfo("Current angle error %f", angle_error)
                if np.abs(angle_error) < 0.05:
                    self.publish_vel(0, 0)
                    self.angle_reached = True
                    rospy.loginfo("Angle reached")

            elif self.angle_reached == True and self.approached is not True:
                rospy.loginfo("Approaching button")
                self.publish_vel(0.1, 0)
                rospy.sleep(4)
                self.publish_vel(0, 0)
                self.approached = True
            
            elif self.approached == True and self.returned is not True:
                angle_error = self.angle_difference(math.pi/2, self.current_yaw)
                self.publish_vel(0, -self.rotation_speed*np.sign(angle_error)-self.P_gain*angle_error)
                rospy.loginfo("Current angle error %f", angle_error)
                if np.abs(angle_error) < 0.05:
                    self.publish_vel(0, 0)
                    self.returned = True
                    rospy.loginfo("Angle reached")

            elif self.returned == True and self.button_pressed is not True:
                rospy.loginfo("Pressing button")
                self.publish_vel(-0.2, 0)
                rospy.sleep(0.8)
                self.publish_vel(-0, 0)
                rospy.sleep(1)
                self.publish_vel(0.1, 0)
                rospy.sleep(3)
                self.publish_vel(-0, 0)
                self.button_pressed = True
                rospy.sleep(2)
            elif self.button_pressed == True and self.clear_door is not True:
                angle_error = self.angle_difference(0, self.current_yaw)
                self.publish_vel(0, -self.rotation_speed*np.sign(angle_error)-self.P_gain*angle_error)
                rospy.loginfo("Current angle error %f", angle_error)
                if np.abs(angle_error) < 0.05:
                    self.publish_vel(0, 0)
                    self.clear_door = True
                    rospy.loginfo("Facing door")
            else:
                rospy.loginfo("wtf")
            
            
            rospy.sleep(0.1)
        self.is_in_state = False
        return "success"

    def angle_difference(self, angle1, angle2):
        # Normalize the angles to be between -pi and pi
        angle1 = math.atan2(math.sin(angle1), math.cos(angle1))
        angle2 = math.atan2(math.sin(angle2), math.cos(angle2))

        # Calculate the difference between the normalized angles
        difference = angle2 - angle1

        # Normalize the difference to be between -pi and pi
        difference = math.atan2(math.sin(difference), math.cos(difference))

        return difference

    def publish_vel(self, lin_vel, ang_vel):
        twist_cmd = Twist()
        twist_cmd.angular.z = ang_vel
        twist_cmd.linear.x = lin_vel
        self.cmd_vel_pub.publish(twist_cmd)

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
        # else:
        #     rospy.loginfo("pose is none")
            

