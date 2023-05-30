#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>

#include <tf2/LinearMath/Quaternion.h>

#include <cmath>

using namespace std;

// Robot parameters

#define NUMBER_OF_POLES       (4.f)
#define REDUCTION_FACTOR      (60.f)
#define TICKS_PER_REVOLUTION  (NUMBER_OF_POLES*REDUCTION_FACTOR)
#define WHEEL_DIAMETER        (0.12f)
const double WHEEL_SPACING =        (0.324f);
// #define METERS_PER_TICKS      (M_PI*WHEEL_DIAMETER/TICKS_PER_REVOLUTION)
const double METERS_PER_TICKS =       (1/640.f);
const double TICKS_PER_METERS =     (640.f);

// Publishers

ros::Publisher odom_pub; // quaternions
ros::Publisher odom_euler_pub; // euler angles (for visualisation)
ros::Publisher calibrate_speed_pub; // speed calibration publisher

// Odometry messages (x, y, yaw)

nav_msgs::Odometry odom_prev;
nav_msgs::Odometry odom;

// Wheel distance

double distance_left = 0;
double distance_right = 0;

double velocity_left = 0;
double velocity_right = 0;

int dleft_ticks = 0;
int dright_ticks = 0;

int left_ticks = 0;
int right_ticks = 0;

int left_ticks_prev = 0;
int right_ticks_prev = 0;

// Only start publishing odometry when initial pose is received

bool received_init_pose = false;

// Handles

void handle_init_pose(const geometry_msgs::Pose2D &init_pose) {
  // Send calibration message to Arduino
  std_msgs::String calibrate_speed_msg;
  calibrate_speed_msg.data = "speed";
  calibrate_speed_pub.publish(calibrate_speed_msg);
  
  odom_prev.pose.pose.position.x = init_pose.x;
  odom_prev.pose.pose.position.y = init_pose.y;
  odom_prev.pose.pose.orientation.z = init_pose.theta;

  received_init_pose = true;
}

// Arduino sends a 16-bit integer which takes a value between [-32768, 32767]
// We assume that for both wheels, an increasing tick count means the robot is
// going forward

void compute_left_ticks() {
  dleft_ticks = left_ticks - left_ticks_prev;
  
  if (dleft_ticks > 32767) { // "underflow"
    dleft_ticks -= 65535;
  }
  else if (dleft_ticks < -32768) { // "overflow"
    dleft_ticks += 65535;
    // dleft_ticks = 65535 - dleft_ticks;
  }
  // distance_left = dleft_ticks*METERS_PER_TICKS;
  distance_left = dleft_ticks/TICKS_PER_METERS;
  left_ticks_prev = left_ticks;
}

void compute_right_ticks() {
  dright_ticks = right_ticks - right_ticks_prev;
  
  if (dright_ticks > 32767) { // "underflow"
    dright_ticks -= 65535;
  }
  else if (dright_ticks < -32768) { // "overflow"
    dright_ticks += 65535;
    // dright_ticks = 65535 - dright_ticks;
  }
  // distance_right = dright_ticks*METERS_PER_TICKS;
  distance_right = dright_ticks/TICKS_PER_METERS;
  right_ticks_prev = right_ticks;
}

 void update_left_ticks(const std_msgs::Int16 &left_ticks_received) {
  if (!received_init_pose) {
    left_ticks_prev = left_ticks;
  }
  left_ticks = left_ticks_received.data;
 }

 void update_right_ticks(const std_msgs::Int16 &right_ticks_received) {
  if (!received_init_pose) {
    right_ticks_prev = right_ticks;
  }
  right_ticks = right_ticks_received.data;
 }

// void compute_left_ticks(const std_msgs::Int16 &left_ticks) {
//   static int left_ticks_prev = 0;
//   dleft_ticks = left_ticks.data - left_ticks_prev;
  
//   if (dleft_ticks > 32767) { // "underflow"
//     dleft_ticks -= 65535;
//   }
//   else if (dleft_ticks < -32768) { // "overflow"
//     dleft_ticks += 65535;
//     // dleft_ticks = 65535 - dleft_ticks;
//   }
//   // distance_left = dleft_ticks*METERS_PER_TICKS;
//   distance_left = dleft_ticks/TICKS_PER_METERS;
//   left_ticks_prev = left_ticks.data;
// }

// void compute_right_ticks(const std_msgs::Int16 &right_ticks) {
//   static int right_ticks_prev = 0;
//   dright_ticks = right_ticks.data - right_ticks_prev;
  
//   if (dright_ticks > 32767) { // "underflow"
//     dright_ticks -= 65535;
//   }
//   else if (dleft_ticks < -32768) { // "overflow"
//     dright_ticks += 65535;
//     // dright_ticks = 65535 - dright_ticks;
//   }
//   // distance_right = dright_ticks*METERS_PER_TICKS;
//   distance_right = dright_ticks/TICKS_PER_METERS;
//   right_ticks_prev = right_ticks.data;
// }

// we use the ticks to determine if velocity is zero

void compute_left_speed(const std_msgs::Float32 &left_vel) {
  velocity_left = (dleft_ticks == 0 ? 0.0 : left_vel.data);
  // velocity_left = left_vel.data;
}

void compute_right_speed(const std_msgs::Float32 &right_vel) {
  velocity_right = (dright_ticks == 0 ? 0.0 : right_vel.data);
  // velocity_right = right_vel.data;
}

// Wrap angle around [-PI, PI]
void wrap_angle(double &angle) {
  if (angle > M_PI)
    angle -= 2*M_PI;
  else if (angle < -M_PI)
    angle += 2*M_PI;
}

void odom_update() {
  compute_left_ticks();
  compute_right_ticks();
  // https://medium.com/@nahmed3536/wheel-odometry-model-for-differential-drive-robotics-91b85a012299
  double avg_distance_travelled = (distance_left + distance_right)/2;
  double angle_turned = (distance_right - distance_left)/WHEEL_SPACING;
  // angle between horizontal and segment between previous and current CoM
  double angle_com = angle_turned/2 + odom_prev.pose.pose.orientation.z; 

  wrap_angle(angle_com);

  double x = odom_prev.pose.pose.position.x 
                            + avg_distance_travelled*cos(angle_com);
  double y = odom_prev.pose.pose.position.y
                            + avg_distance_travelled*sin(angle_com); 

  double yaw = odom_prev.pose.pose.orientation.z + angle_turned;

  // Make sure that the values are not NaN (necessary?)
  if (isnan(x) || isnan(y) || isnan(yaw)) {
    odom.pose.pose.position.x = odom_prev.pose.pose.position.x;
    odom.pose.pose.position.y = odom_prev.pose.pose.position.y;
    odom.pose.pose.orientation.z = odom_prev.pose.pose.orientation.z;
  } else {
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    wrap_angle(yaw);
    odom.pose.pose.orientation.z = yaw;
  }

  // Compute velocities
  odom.twist.twist.linear.x = (velocity_left + velocity_right)/2;
  odom.twist.twist.angular.z = (velocity_right - velocity_left)/WHEEL_SPACING; // negative clockwise

  // Save pose for next iteration
  odom_prev.pose.pose.position.x = odom.pose.pose.position.x;
  odom_prev.pose.pose.position.y = odom.pose.pose.position.y;
  odom_prev.pose.pose.orientation.z = odom.pose.pose.orientation.z;
}

void odom_publish() {

  nav_msgs::Odometry odom_quat;

  odom.header.stamp = ros::Time::now();
  // we track the base_link in the odom frame
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";

  odom_quat.header.stamp = odom.header.stamp;
  odom_quat.header.frame_id = odom.header.frame_id;
  odom_quat.child_frame_id = odom.child_frame_id;

  odom_quat.pose.pose.position.x = odom.pose.pose.position.x;
  odom_quat.pose.pose.position.y = odom.pose.pose.position.y;
  odom_quat.pose.pose.position.z = odom.pose.pose.position.z;

  odom_quat.twist.twist.linear.x = odom.twist.twist.linear.x;
  odom_quat.twist.twist.linear.y = odom.twist.twist.linear.y;
  odom_quat.twist.twist.linear.z = odom.twist.twist.linear.z;

  odom_quat.twist.twist.angular.x = odom.twist.twist.angular.x;
  odom_quat.twist.twist.angular.y = odom.twist.twist.angular.y;
  odom_quat.twist.twist.angular.z = odom.twist.twist.angular.z;

  // get quaternion from euler angles
  tf2::Quaternion q;
  q.setRPY(odom.pose.pose.orientation.x,
           odom.pose.pose.orientation.y,
           odom.pose.pose.orientation.z);

  odom_quat.pose.pose.orientation.x = q.x();
  odom_quat.pose.pose.orientation.y = q.y();
  odom_quat.pose.pose.orientation.z = q.z();
  odom_quat.pose.pose.orientation.w = q.w();

  // Compute 6x6 covariance matrix
  // x, y, z
  odom_quat.pose.covariance[0] = 0.01;
  odom_quat.pose.covariance[7] = 0.01;
  odom_quat.pose.covariance[14] = 0.01;
  // r, p, y
  odom_quat.pose.covariance[21] = 0.1;
  odom_quat.pose.covariance[28] = 0.1;
  odom_quat.pose.covariance[35] = 0.1;

  // Publish odometry message
  odom_euler_pub.publish(odom);
  odom_pub.publish(odom_quat);

}

int main(int argc, char **argv) {

  ros::init(argc, argv, "odom_pub");
  ros::NodeHandle nh;

  // Setup subscribers
  ros::Subscriber left_ticks_sub = nh.subscribe("left_ticks", 100, update_left_ticks, 
                                                ros::TransportHints().tcpNoDelay());
  ros::Subscriber right_ticks_sub = nh.subscribe("right_ticks", 100, update_right_ticks, 
                                                ros::TransportHints().tcpNoDelay());  

  ros::Subscriber left_vel_sub = nh.subscribe("left_wheel_vel", 100, compute_left_speed, 
                                                ros::TransportHints().tcpNoDelay());      
  ros::Subscriber right_vel_sub = nh.subscribe("right_wheel_vel", 100, compute_right_speed, 
                                                ros::TransportHints().tcpNoDelay()); 
                                                                                            
  ros::Subscriber init_pose_2d_sub = nh.subscribe("init_pose_2d", 1, handle_init_pose);

  // Setup publishers
  odom_pub = nh.advertise<nav_msgs::Odometry>("odom_quat", 100);
  odom_euler_pub = nh.advertise<nav_msgs::Odometry>("odom_euler", 100);
  calibrate_speed_pub = nh.advertise<std_msgs::String>("reset_controller", 10);

  // We assume the robot is at standstill intially
  // with no roll or pitch
  odom.pose.pose.position.x = 0;
  odom.pose.pose.position.y = 0;
  odom.pose.pose.position.z = 0;
  odom.twist.twist.linear.x = 0;
  odom.twist.twist.linear.y = 0;
  odom.twist.twist.linear.z = 0;
  odom.twist.twist.angular.x = 0;
  odom.twist.twist.angular.y = 0;
  odom.twist.twist.angular.z = 0;

  odom_prev.pose.pose.position.x = 0;
  odom_prev.pose.pose.position.y = 0;
  odom_prev.pose.pose.orientation.z = 0;


  // Max speed of robot: 50cm/s. If we want ~2cm of precision,
  // We need to publish at at least 25 Hz
  ros::Rate rate(20);

  while(ros::ok()) {

    if (received_init_pose) {
      odom_update();
      odom_publish();
    }

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}

// /*
//  * Automatic Addison
//  * Date: May 20, 2021
//  * ROS Version: ROS 1 - Melodic
//  * Website: https://automaticaddison.com
//  * Publishes odometry information for use with robot_pose_ekf package.
//  *   This odometry information is based on wheel encoder tick counts.
//  * Subscribe: ROS node that subscribes to the following topics:
//  *  right_ticks : Tick counts from the right motor encoder (std_msgs/Int16)
//  * 
//  *  left_ticks : Tick counts from the left motor encoder  (std_msgs/Int16)
//  * 
//  *  initial_2d : The initial position and orientation of the robot.
//  *               (geometry_msgs/PoseStamped)
//  *
//  * Publish: This node will publish to the following topics:
//  *  odom_data_euler : Position and velocity estimate. The orientation.z 
//  *                    variable is an Euler angle representing the yaw angle.
//  *                    (nav_msgs/Odometry)
//  *  odom_data_quat : Position and velocity estimate. The orientation is 
//  *                   in quaternion format.
//  *                   (nav_msgs/Odometry)
//  * Modified from Practical Robotics in C++ book (ISBN-10 : 9389423465)
//  *   by Lloyd Brombach
//  */
 
// // Include various libraries
// #include "ros/ros.h"
// #include "std_msgs/Int16.h"
// #include <nav_msgs/Odometry.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/Pose2D.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <cmath>
 
// // Create odometry data publishers
// ros::Publisher odom_data_pub;
// ros::Publisher odom_data_pub_quat;
// nav_msgs::Odometry odomNew;
// nav_msgs::Odometry odomOld;
 
// // Initial pose
// const double initialX = 0.0;
// const double initialY = 0.0;
// const double initialTheta = 0.00000000001;
// const double PI = 3.141592;
 
// // Robot physical constants
// const double TICKS_PER_REVOLUTION = 240; // For reference purposes.
// const double WHEEL_RADIUS = 0.033; // Wheel radius in meters
// const double WHEEL_DIAMETER = 0.12;
// const double WHEEL_BASE = 0.332; // Center of left tire to center of right tire
// // const double TICKS_PER_METER = 3100; // Original was 2800
// // const double TICKS_PER_METER  = (1/(M_PI*WHEEL_DIAMETER/TICKS_PER_REVOLUTION));
// const double TICKS_PER_METER = (TICKS_PER_REVOLUTION/(WHEEL_DIAMETER*PI));

// // #define NUMBER_OF_POLES       (4.f)
// // #define REDUCTION_FACTOR      (60.f)
// // #define TICKS_PER_REVOLUTION  (NUMBER_OF_POLES*REDUCTION_FACTOR)
// // #define WHEEL_DIAMETER        (0.12f)
// // #define WHEEL_SPACING         (0.332f)
// // #define METERS_PER_TICKS      (M_PI*WHEEL_DIAMETER/TICKS_PER_REVOLUTION)
 
// // Distance both wheels have traveled
// double distanceLeft = 0;
// double distanceRight = 0;
 
// // Flag to see if initial pose has been received
// bool initialPoseRecieved = false;
 
// using namespace std;
 
// // Get initial_2d message from either Rviz clicks or a manual pose publisher
// void set_initial_2d(const geometry_msgs::Pose2D &init_pose) {
 
//   odomOld.pose.pose.position.x = init_pose.x;
//   odomOld.pose.pose.position.y = init_pose.y;
//   odomOld.pose.pose.orientation.z = init_pose.theta;
//   initialPoseRecieved = true;
// }

// // Calculate the distance the left wheel has traveled since the last cycle
// void Calc_Left(const std_msgs::Int16& leftCount) {
 
//   static int lastCountL = 0;
//   if(leftCount.data != 0 && lastCountL != 0) {
         
//     int leftTicks = (leftCount.data - lastCountL);
 
//     if (leftTicks > 10000) {
//       leftTicks = 0 - (65535 - leftTicks);
//     }
//     else if (leftTicks < -10000) {
//       leftTicks = 65535-leftTicks;
//     }
//     else{}
//     distanceLeft = leftTicks/TICKS_PER_METER;
//   }
//   lastCountL = leftCount.data;
// }
 
// // Calculate the distance the right wheel has traveled since the last cycle
// void Calc_Right(const std_msgs::Int16& rightCount) {
   
//   static int lastCountR = 0;
//   if(rightCount.data != 0 && lastCountR != 0) {
 
//     int rightTicks = rightCount.data - lastCountR;
     
//     if (rightTicks > 10000) {
//       distanceRight = (0 - (65535 - distanceRight))/TICKS_PER_METER;
//     }
//     else if (rightTicks < -10000) {
//       rightTicks = 65535 - rightTicks;
//     }
//     else{}
//     distanceRight = rightTicks/TICKS_PER_METER;
//   }
//   lastCountR = rightCount.data;
// }
 
// // Publish a nav_msgs::Odometry message in quaternion format
// void publish_quat() {
 
//   tf2::Quaternion q;
         
//   q.setRPY(0, 0, odomNew.pose.pose.orientation.z);
 
//   nav_msgs::Odometry quatOdom;
//   quatOdom.header.stamp = odomNew.header.stamp;
//   quatOdom.header.frame_id = "odom";
//   quatOdom.child_frame_id = "base_link";
//   quatOdom.pose.pose.position.x = odomNew.pose.pose.position.x;
//   quatOdom.pose.pose.position.y = odomNew.pose.pose.position.y;
//   quatOdom.pose.pose.position.z = odomNew.pose.pose.position.z;
//   quatOdom.pose.pose.orientation.x = q.x();
//   quatOdom.pose.pose.orientation.y = q.y();
//   quatOdom.pose.pose.orientation.z = q.z();
//   quatOdom.pose.pose.orientation.w = q.w();
//   quatOdom.twist.twist.linear.x = odomNew.twist.twist.linear.x;
//   quatOdom.twist.twist.linear.y = odomNew.twist.twist.linear.y;
//   quatOdom.twist.twist.linear.z = odomNew.twist.twist.linear.z;
//   quatOdom.twist.twist.angular.x = odomNew.twist.twist.angular.x;
//   quatOdom.twist.twist.angular.y = odomNew.twist.twist.angular.y;
//   quatOdom.twist.twist.angular.z = odomNew.twist.twist.angular.z;
 
//   for(int i = 0; i<36; i++) {
//     if(i == 0 || i == 7 || i == 14) {
//       quatOdom.pose.covariance[i] = .01;
//      }
//      else if (i == 21 || i == 28 || i== 35) {
//        quatOdom.pose.covariance[i] += 0.1;
//      }
//      else {
//        quatOdom.pose.covariance[i] = 0;
//      }
//   }
 
//   odom_data_pub_quat.publish(quatOdom);
// }
 
// // Update odometry information
// void update_odom() {
 
//   // Calculate the average distance
//   double cycleDistance = (distanceRight + distanceLeft) / 2;
   
//   // Calculate the number of radians the robot has turned since the last cycle
//   double cycleAngle = asin((distanceRight-distanceLeft)/WHEEL_BASE);
 
//   // Average angle during the last cycle
//   double avgAngle = cycleAngle/2 + odomOld.pose.pose.orientation.z;
     
//   if (avgAngle > PI) {
//     avgAngle -= 2*PI;
//   }
//   else if (avgAngle < -PI) {
//     avgAngle += 2*PI;
//   }
//   else{}
 
//   // Calculate the new pose (x, y, and theta)
//   odomNew.pose.pose.position.x = odomOld.pose.pose.position.x + cos(avgAngle)*cycleDistance;
//   odomNew.pose.pose.position.y = odomOld.pose.pose.position.y + sin(avgAngle)*cycleDistance;
//   odomNew.pose.pose.orientation.z = cycleAngle + odomOld.pose.pose.orientation.z;
 
//   // Prevent lockup from a single bad cycle
//   if (isnan(odomNew.pose.pose.position.x) || isnan(odomNew.pose.pose.position.y)
//      || isnan(odomNew.pose.pose.position.z)) {
//     odomNew.pose.pose.position.x = odomOld.pose.pose.position.x;
//     odomNew.pose.pose.position.y = odomOld.pose.pose.position.y;
//     odomNew.pose.pose.orientation.z = odomOld.pose.pose.orientation.z;
//   }
 
//   // Make sure theta stays in the correct range
//   if (odomNew.pose.pose.orientation.z > PI) {
//     odomNew.pose.pose.orientation.z -= 2 * PI;
//   }
//   else if (odomNew.pose.pose.orientation.z < -PI) {
//     odomNew.pose.pose.orientation.z += 2 * PI;
//   }
//   else{}
 
//   // Compute the velocity
//   odomNew.header.stamp = ros::Time::now();
//   odomNew.twist.twist.linear.x = cycleDistance/(odomNew.header.stamp.toSec() - odomOld.header.stamp.toSec());
//   odomNew.twist.twist.angular.z = cycleAngle/(odomNew.header.stamp.toSec() - odomOld.header.stamp.toSec());
 
//   // Save the pose data for the next cycle
//   odomOld.pose.pose.position.x = odomNew.pose.pose.position.x;
//   odomOld.pose.pose.position.y = odomNew.pose.pose.position.y;
//   odomOld.pose.pose.orientation.z = odomNew.pose.pose.orientation.z;
//   odomOld.header.stamp = odomNew.header.stamp;
 
//   // Publish the odometry message
//   odom_data_pub.publish(odomNew);
// }
 
// int main(int argc, char **argv) {
   
//   // Set the data fields of the odometry message
//   odomNew.header.frame_id = "odom";
//   odomNew.pose.pose.position.z = 0;
//   odomNew.pose.pose.orientation.x = 0;
//   odomNew.pose.pose.orientation.y = 0;
//   odomNew.twist.twist.linear.x = 0;
//   odomNew.twist.twist.linear.y = 0;
//   odomNew.twist.twist.linear.z = 0;
//   odomNew.twist.twist.angular.x = 0;
//   odomNew.twist.twist.angular.y = 0;
//   odomNew.twist.twist.angular.z = 0;
//   odomOld.pose.pose.position.x = initialX;
//   odomOld.pose.pose.position.y = initialY;
//   odomOld.pose.pose.orientation.z = initialTheta;
 
//   // Launch ROS and create a node
//   ros::init(argc, argv, "ekf_odom_pub");
//   ros::NodeHandle node;
 
//   // Subscribe to ROS topics
//   ros::Subscriber subForRightCounts = node.subscribe("right_ticks", 100, Calc_Right, ros::TransportHints().tcpNoDelay());
//   ros::Subscriber subForLeftCounts = node.subscribe("left_ticks", 100, Calc_Left, ros::TransportHints().tcpNoDelay());
//   ros::Subscriber subInitialPose = node.subscribe("init", 1, set_initial_2d);
 
//   // Publisher of simple odom message where orientation.z is an euler angle
//   odom_data_pub = node.advertise<nav_msgs::Odometry>("odom_euler", 100);
 
//   // Publisher of full odom message where orientation is quaternion
//   odom_data_pub_quat = node.advertise<nav_msgs::Odometry>("odom_quat", 100);
 
//   ros::Rate loop_rate(30); 
     
//   while(ros::ok()) {
     
//     if(initialPoseRecieved) {
//       update_odom();
//       publish_quat();
//     }
//     ros::spinOnce();
//     loop_rate.sleep();
//   }
 
//   return 0;
// }