#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from math import sqrt
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class DuploGoalFilter:
    def __init__(self):
        rospy.init_node('duplo_goal_filter')
        
        # Parameters
        self.alpha = rospy.get_param('~filter_alpha', 0.5)
        self.dist_threshold = rospy.get_param('~distance_threshold', 1.0)
        self.inflation_radius = rospy.get_param('~inflation_radius', 0.2)  # Robot's footprint
        
        # Initialize variables
        self.previous_pose = None
        self.filtered_pose = None
        self.costmap_data = None
        
        # Subscribers
        rospy.Subscriber('/closest_duplo_goal', PoseStamped, self.pose_callback)
        rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.costmap_callback)
        
        # Publisher
        self.filtered_pose_pub = rospy.Publisher('/closest_duplo_goal_filtered', PoseStamped, queue_size=10)
        
    def pose_callback(self, pose):
        if self.previous_pose is None:
            self.previous_pose = pose
            self.filtered_pose = pose
            return
        
        # Check if new pose is further away than dist_threshold
        dx = self.previous_pose.pose.position.x - pose.pose.position.x
        dy = self.previous_pose.pose.position.y - pose.pose.position.y
        distance = sqrt(dx*dx + dy*dy)
        
        if distance > self.dist_threshold:
            self.filtered_pose = pose
        else:
            # Apply low-pass filter
            self.filtered_pose.pose.position.x = self.alpha * self.filtered_pose.pose.position.x + \
                                                  (1 - self.alpha) * pose.pose.position.x
            self.filtered_pose.pose.position.y = self.alpha * self.filtered_pose.pose.position.y + \
                                                  (1 - self.alpha) * pose.pose.position.y
            self.filtered_pose.pose.position.z = 0
            self.filtered_pose.pose.orientation.x = self.alpha * self.filtered_pose.pose.orientation.x + \
                                                 (1 - self.alpha) * pose.pose.orientation.x
            self.filtered_pose.pose.orientation.y = self.alpha * self.filtered_pose.pose.orientation.y + \
                                                    (1 - self.alpha) * pose.pose.orientation.y
            self.filtered_pose.pose.orientation.z = self.alpha * self.filtered_pose.pose.orientation.z + \
                                                    (1 - self.alpha) * pose.pose.orientation.z
            self.filtered_pose.pose.orientation.w = self.alpha * self.filtered_pose.pose.orientation.w + \
                                                    (1 - self.alpha) * pose.pose.orientation.w

            _, _, yaw = euler_from_quaternion([
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w
            ])

            # Create a new orientation quaternion with only the yaw component
            orientation = quaternion_from_euler(0, 0, yaw)

            # Assign the new orientation to the filtered pose
            self.filtered_pose.pose.orientation.x = orientation[0]
            self.filtered_pose.pose.orientation.y = orientation[1]
            self.filtered_pose.pose.orientation.z = orientation[2]
            self.filtered_pose.pose.orientation.w = orientation[3]
        
        # Check if filtered pose is valid
        if self.is_pose_valid(self.filtered_pose):
            self.filtered_pose_pub.publish(self.filtered_pose)
    
    def costmap_callback(self, costmap_data):
        self.costmap_data = costmap_data
    
    def is_pose_valid(self, pose):
        if self.costmap_data is None:
            return False
        
        costmap_width = self.costmap_data.info.width
        costmap_height = self.costmap_data.info.height
        resolution = self.costmap_data.info.resolution
        
        pose_x = pose.pose.position.x
        pose_y = pose.pose.position.y
        
        # Convert pose to costmap grid coordinates
        grid_x = int((pose_x - self.costmap_data.info.origin.position.x) / resolution)
        grid_y = int((pose_y - self.costmap_data.info.origin.position.y) / resolution)
        
        # Check if pose is within the costmap bounds
        if grid_x < 0 or grid_x >= costmap_width or grid_y < 0 or grid_y >= costmap_height:
            return False
        
        # Check if pose is in an obstacle or inflation area
        index = grid_y * costmap_width + grid_x
        cost = self.costmap_data.data[index]

        # rospy.logerr("x: %f y: %f cost: %f width: %f height %f", pose_x, pose_y, cost, costmap_width, costmap_height)
        
        # Check if pose is in an inflated obstacle or near the robot's footprint
        if cost >= 80:
            return False
        
        return True

if __name__ == '__main__':
    try:
        node = DuploGoalFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
