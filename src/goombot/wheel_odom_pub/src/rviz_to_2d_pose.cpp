#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_broadcaster.h>
#include <iostream>

using namespace std;

// Publishers
ros::Publisher goal_2d_pub;
ros::Publisher init_pose_2d_pub;


// Handles ----------------------------------------------------

// Converts move_base_simple/goal from rviz to 2d pose
void handle_goal(const geometry_msgs::PoseStamped &rviz_goal) {
    geometry_msgs::Pose2D goal_2d;
    goal_2d.x = rviz_goal.pose.position.x;
    goal_2d.y = rviz_goal.pose.position.y;
    // Compute yaw angle (Euler) from quaternion
    tf::Quaternion quat(rviz_goal.pose.orientation.x, 
                        rviz_goal.pose.orientation.y, 
                        rviz_goal.pose.orientation.z, 
                        rviz_goal.pose.orientation.w);
    tf::Matrix3x3 mat(quat);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    goal_2d.theta = yaw;
    goal_2d_pub.publish(goal_2d);
}

// Converts /initialpose from rviz to 2d pose
void handle_init_pose(const geometry_msgs::PoseWithCovarianceStamped &rviz_init_pose) {
    geometry_msgs::Pose2D init_pose_2d;
    init_pose_2d.x = rviz_init_pose.pose.pose.position.x;
    init_pose_2d.y = rviz_init_pose.pose.pose.position.y;
    // Compute yaw angle (Euler) from quaternion
    tf::Quaternion quat(rviz_init_pose.pose.pose.orientation.x, 
                        rviz_init_pose.pose.pose.orientation.y, 
                        rviz_init_pose.pose.pose.orientation.z, 
                        rviz_init_pose.pose.pose.orientation.w);
    tf::Matrix3x3 mat(quat);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    init_pose_2d.theta = yaw;
    init_pose_2d_pub.publish(init_pose_2d);
}

 
int main(int argc, char **argv) {
  ros::init(argc, argv, "rviz_to_2d_pose");
  ros::NodeHandle nh;
  goal_2d_pub = nh.advertise<geometry_msgs::Pose2D>("goal_2d", 0);
  init_pose_2d_pub = nh.advertise<geometry_msgs::Pose2D>("init_pose_2d", 0);

  ros::Subscriber rviz_goal_sub = nh.subscribe("move_base_simple/goal", 0, handle_goal);
  ros::Subscriber rviz_init_pose_sub = nh.subscribe("initialpose", 0, handle_init_pose);

  // Used once at the start, doesn't need high refresh rate
  ros::Rate loop_rate(10);
  while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
  }
  return 0;
}