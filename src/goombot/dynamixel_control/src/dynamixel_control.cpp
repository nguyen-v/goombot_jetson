#include <ros/ros.h>
#include <dynamixel_workbench_msgs/DynamixelCommand.h>
#include <std_msgs/Int16.h>

// Subscribers
ros::Subscriber dynamixel_sub;
ros::ServiceClient client;

void handle_dynamixel_pos(const std_msgs::Int16 &dyn_pos) {
    ROS_INFO("Called set pos dynamixel");
    dynamixel_workbench_msgs::DynamixelCommand srv_speed;
    srv_speed.request.command = "";
    srv_speed.request.id = 9;
    srv_speed.request.addr_name = "Moving_Speed";
    srv_speed.request.value = 100;
    if (client.call(srv_speed))
    {
        ROS_INFO("Service call for servo 1 speed successful");
    }
    else
    {
        ROS_ERROR("Failed to call service for servo 1 position");
    }
    dynamixel_workbench_msgs::DynamixelCommand srv1_pos;
    srv1_pos.request.command = "";
    srv1_pos.request.id = 9;
    srv1_pos.request.addr_name = "Goal_Position";
    srv1_pos.request.value = dyn_pos.data;
    if (client.call(srv1_pos))
    {
        ROS_INFO("Service call for servo 1 position successful");
    }
    else
    {
        ROS_ERROR("Failed to call service for servo 1 position");
    }

    srv_speed.request.command = "";
    srv_speed.request.id = 15;
    srv_speed.request.addr_name = "Moving_Speed";
    srv_speed.request.value = 100;
    if (client.call(srv_speed))
    {
        ROS_INFO("Service call for servo 1 speed successful");
    }
    else
    {
        ROS_ERROR("Failed to call service for servo 1 position");
    }
    srv1_pos.request.command = "";
    srv1_pos.request.id = 15;
    srv1_pos.request.addr_name = "Goal_Position";
    srv1_pos.request.value = dyn_pos.data;
    if (client.call(srv1_pos))
    {
        ROS_INFO("Service call for servo 1 position successful");
    }
    else
    {
        ROS_ERROR("Failed to call service for servo 1 position");
    }

    // srv_speed.request.command = "";
    // srv_speed.request.id = 14;
    // srv_speed.request.addr_name = "Moving_Speed";
    // srv_speed.request.value = 100;
    // if (client.call(srv_speed))
    // {
    //     ROS_INFO("Service call for servo 1 speed successful");
    // }
    // else
    // {
    //     ROS_ERROR("Failed to call service for servo 1 position");
    // }
    // srv1_pos.request.command = "";
    // srv1_pos.request.id = 14;
    // srv1_pos.request.addr_name = "Goal_Position";
    // srv1_pos.request.value = dyn_pos.data;
    // if (client.call(srv1_pos))
    // {
    //     ROS_INFO("Service call for servo 1 position successful");
    // }
    // else
    // {
    //     ROS_ERROR("Failed to call service for servo 1 position");
    // }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "dynamixel_control");
    ros::NodeHandle nh;

    ROS_INFO("Starting dynamixel node");
    // Create a ServiceClient for the dynamixel_command service
    client = nh.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");


    dynamixel_sub = nh.subscribe("set_dynamixel_pos", 0, handle_dynamixel_pos);

    ros::Rate loop_rate(100);

    while (ros::ok()) {
            ros::spinOnce();
            loop_rate.sleep();
    }

    return 0;
}



// #include <ros/ros.h>
// #include <dynamixel_workbench_msgs/DynamixelCommand.h>
// #include <thread>
// #include <mutex>

// Define a mutex to synchronize access to the service client
// std::mutex mutex;

// void control_servo(int id, int pos) {
//     Create a ServiceClient for the dynamixel_command service
//     ros::NodeHandle nh;
//     ros::ServiceClient client = nh.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");

//     Create a DynamixelCommand service message for the servo
//     dynamixel_workbench_msgs::DynamixelCommand srv;

//     Set the Moving_Speed for the servo
//     srv.request.command = "";
//     srv.request.id = id;
//     srv.request.addr_name = "Moving_Speed";
//     srv.request.value = 100;

//     Acquire the mutex lock to prevent multiple threads from accessing the service client at the same time
//     mutex.lock();

//     Call the service to set the Moving_Speed for the servo
//     if (client.call(srv))
//     {
//         ROS_INFO_STREAM("Service call for servo " << id << " speed successful");
//     }
//     else
//     {
//         ROS_ERROR_STREAM("Failed to call service for servo " << id << " speed");
//     }

//     Set the Goal_Position for the servo
//     srv.request.command = "";
//     srv.request.id = id;
//     srv.request.addr_name = "Goal_Position";
//     srv.request.value = pos;

//     Call the service to set the Goal_Position for the servo
//     if (client.call(srv))
//     {
//         ROS_INFO_STREAM("Service call for servo " << id << " position successful");
//     }
//     else
//     {
//         ROS_ERROR_STREAM("Failed to call service for servo " << id << " position");
//     }

//     Release the mutex lock
//     mutex.unlock();
// }

// void handle_dynamixel_pos(const std_msgs::Int16 &dyn_pos) {
//     ROS_INFO("Called set pos dynamixel");

//     Create three threads to control three servos at the same time
//     std::thread t1(control_servo, 9, dyn_pos.data);
//     std::thread t2(control_servo, 14, dyn_pos.data);
//     std::thread t3(control_servo, 15, dyn_pos.data);
//     std::thread t4(control_servo, 6, dyn_pos.data);
//      Wait for all threads to finish
//     t1.join();
//     t2.join();
//     t3.join();
//     t4.join();
// }

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "dynamixel_control");
//     ros::NodeHandle nh;

//     Create a Subscriber for the set_dynamixel_pos topic
//     ros::Subscriber sub = nh.subscribe("set_dynamixel_pos", 10, handle_dynamixel_pos);

//     Spin the node to handle incoming messages
//     ros::spin();

//     return 0;
// }
// #include <ros/ros.h>
// #include <dynamixel_workbench_msgs/DynamixelCommand.h>

// void control_servo(int id, int pos) {
//     Create a ServiceClient for the dynamixel_command service
//     ros::NodeHandle nh;
//     ros::ServiceClient client = nh.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");

//     Create a DynamixelCommand service message for the servo
//     dynamixel_workbench_msgs::DynamixelCommand srv;

//     Set the Moving_Speed for the servo
//     srv.request.command = "";
//     srv.request.id = id;
//     srv.request.addr_name = "Moving_Speed";
//     srv.request.value = 100;

//     Call the service to set the Moving_Speed for the servo
//     if (client.call(srv))
//     {
//         ROS_INFO_STREAM("Service call for servo " << id << " speed successful");
//     }
//     else
//     {
//         ROS_ERROR_STREAM("Failed to call service for servo " << id << " speed");
//     }

//     Set the Goal_Position for the servo
//     srv.request.command = "";
//     srv.request.id = id;
//     srv.request.addr_name = "Goal_Position";
//     srv.request.value = pos;

//     Call the service to set the Goal_Position for the servo
//     if (client.call(srv))
//     {
//         ROS_INFO_STREAM("Service call for servo " << id << " position successful");
//     }
//     else
//     {
//         ROS_ERROR_STREAM("Failed to call service for servo " << id << " position");
//     }
// }

// void handle_dynamixel_pos(const std_msgs::Int16 &dyn_pos) {
//     ROS_INFO("Called set pos dynamixel");

//     Control all servos at the same time
//     control_servo(9, dyn_pos.data);
//     control_servo(14, dyn_pos.data);
//     control_servo(15, dyn_pos.data);
//     control_servo(6, dyn_pos.data);
// }

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "dynamixel_control");
//     ros::NodeHandle nh;

//     Create a Subscriber for the set_dynamixel_pos topic
//     ros::Subscriber sub = nh.subscribe("set_dynamixel_pos", 10, handle_dynamixel_pos);

//     Spin the node to handle incoming messages
//     ros::spin();

//     return 0;
// }

