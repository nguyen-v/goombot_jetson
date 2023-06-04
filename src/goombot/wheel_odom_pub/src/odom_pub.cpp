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
const double TICKS_PER_METERS =     (583.f);
const double ROTATION_SLIP_FACTOR = (0.897);

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
  }
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
  }
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

// we use the ticks to determine if velocity is zero

void compute_left_speed(const std_msgs::Float32 &left_vel) {
  velocity_left = (dleft_ticks == 0 ? 0.0 : left_vel.data);
}

void compute_right_speed(const std_msgs::Float32 &right_vel) {
  velocity_right = (dright_ticks == 0 ? 0.0 : right_vel.data);
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

  double yaw = odom_prev.pose.pose.orientation.z + angle_turned*ROTATION_SLIP_FACTOR;

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
