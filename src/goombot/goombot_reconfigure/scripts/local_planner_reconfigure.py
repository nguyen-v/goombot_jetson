#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from dynamic_reconfigure.client import Client

def position_callback(msg):
    if msg.pose.pose.position.x < -1.5 and msg.pose.pose.position.y > 2:
        # Modify trajectory planner parameters
        client = Client("/move_base/TrajectoryPlannerROS")
        params = {'min_vel_x': 0.15}  # Set min_vel_x to 0.15
        client.update_configuration(params)
    else:
        # Modify trajectory planner parameters
        client = Client("/move_base/TrajectoryPlannerROS")
        params = {'min_vel_x': 0.05}  # Set min_vel_x to 0.05
        client.update_configuration(params)

def main():
    rospy.init_node('trajectory_planner_controller')

    # Subscribe to the robot's pose topic
    rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, position_callback)

    rospy.spin()

if __name__ == '__main__':
    main()
