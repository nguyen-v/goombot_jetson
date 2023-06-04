#!/usr/bin/env python

import rospy
import tf2_ros
import math
import tf
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from tf2_geometry_msgs import do_transform_pose

class DuploPoseRepublisher:
    def __init__(self):
        rospy.init_node('duplo_pose_republisher', anonymous=True)
        
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.pose_sub = rospy.Subscriber('/oakd/yolo/rel_duplo_pose', PoseArray, self.pose_callback)
        self.pose_pub = rospy.Publisher('/duplo_poses', PoseArray, queue_size=10)
        self.pose_goal_pub = rospy.Publisher('/closest_duplo_goal', PoseStamped, queue_size=10)

        self.distance_goal_to_duplo = rospy.get_param('~distance_goal_to_duplo', 0.35)  # Default: 0.35 meter
        
    def pose_callback(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform('map', msg.header.frame_id, rospy.Time())
            pose_array = PoseArray()
            closest_pose = None
            closest_distance = float('inf')
            for pose in msg.poses:
                # Calculate orientation with vector pointing outwards w.r.t. camera origin
                dx = pose.position.x
                dy = pose.position.y
                dz = pose.position.z
                yaw = math.atan2(dy, dx)
                quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
                # qx, qy, qz, qw = tf2_ros.quaternion_from_euler(0, 0, yaw)  # Calculate the orientation based on yaw angle
                

                # Create a PoseStamped with the calculated orientation and the original position
                pose_stamped = PoseStamped()
                pose_stamped.pose.orientation.x = quat[0]
                pose_stamped.pose.orientation.y = quat[1]
                pose_stamped.pose.orientation.z = quat[2]
                pose_stamped.pose.orientation.w = quat[3]
                pose_stamped.pose.position = pose.position
                pose_stamped.header.frame_id = msg.header.frame_id
                pose_stamped.header.stamp = rospy.Time.now()
                pose_array.poses.append(pose_stamped.pose)

                # Calculate the distance to the current pose
                distance = math.sqrt(pose.position.x ** 2 + pose.position.y ** 2)
                if (distance < closest_distance) and (distance != 0):
                    closest_pose = pose_stamped
                    closest_distance = distance


            transformed_msg = self.transform_pose_array(pose_array, transform)
            self.pose_pub.publish(transformed_msg)

            if closest_pose is not None:
                if closest_pose.pose.position.x > 0.4:
                    closest_pose.pose.position.y -= math.copysign(self.distance_goal_to_duplo * math.sqrt(closest_pose.pose.position.y**2/(closest_pose.pose.position.y**2+closest_pose.pose.position.x**2)), closest_pose.pose.position.y)
                closest_pose.pose.position.x -= self.distance_goal_to_duplo * math.sqrt(closest_pose.pose.position.x**2/(closest_pose.pose.position.y**2+closest_pose.pose.position.x**2))
                transformed_goal = do_transform_pose(closest_pose, transform)
                self.pose_goal_pub.publish(transformed_goal)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Transform lookup failed: %s", str(e))
        
    def transform_pose_array(self, pose_array, transform):
        transformed_pose_array = PoseArray()
        transformed_pose_array.header.frame_id = 'map'
        transformed_pose_array.header.stamp = rospy.Time.now()

        for pose in pose_array.poses:
            pose_stamped = PoseStamped()
            pose_stamped.pose = pose
            pose_stamped.header.frame_id = pose_array.header.frame_id
            pose_stamped.header.stamp = rospy.Time.now()

            transformed_pose = do_transform_pose(pose_stamped, transform)
            transformed_pose_array.poses.append(transformed_pose.pose)

        return transformed_pose_array

if __name__ == '__main__':
    try:
        duplo_pose_republisher = DuploPoseRepublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
