// #include <ros/ros.h>
// #include <dynamixel_workbench_msgs/DynamixelCommand.h>
// #include <std_msgs/Int16.h>

// // Subscribers
// ros::Subscriber dynamixel_sub;
// ros::ServiceClient client;

// void handle_dynamixel_pos(const std_msgs::Int16 &dyn_pos) {
//     ROS_INFO("Called set pos dynamixel");
//     dynamixel_workbench_msgs::DynamixelCommand srv_speed;
//     srv_speed.request.command = "";
//     srv_speed.request.id = 9;
//     srv_speed.request.addr_name = "Moving_Speed";
//     srv_speed.request.value = 100;
//     if (client.call(srv_speed))
//     {
//         ROS_INFO("Service call for servo 1 speed successful");
//     }
//     else
//     {
//         ROS_ERROR("Failed to call service for servo 1 position");
//     }
//     dynamixel_workbench_msgs::DynamixelCommand srv1_pos;
//     srv1_pos.request.command = "";
//     srv1_pos.request.id = 9;
//     srv1_pos.request.addr_name = "Goal_Position";
//     srv1_pos.request.value = dyn_pos.data;
//     if (client.call(srv1_pos))
//     {
//         ROS_INFO("Service call for servo 1 position successful");
//     }
//     else
//     {
//         ROS_ERROR("Failed to call service for servo 1 position");
//     }

//     srv_speed.request.command = "";
//     srv_speed.request.id = 6;
//     srv_speed.request.addr_name = "Moving_Speed";
//     srv_speed.request.value = 100;
//     if (client.call(srv_speed))
//     {
//         ROS_INFO("Service call for servo 1 speed successful");
//     }
//     else
//     {
//         ROS_ERROR("Failed to call service for servo 1 position");
//     }
//     srv1_pos.request.command = "";
//     srv1_pos.request.id = 6;
//     srv1_pos.request.addr_name = "Goal_Position";
//     srv1_pos.request.value = dyn_pos.data;
//     if (client.call(srv1_pos))
//     {
//         ROS_INFO("Service call for servo 1 position successful");
//     }
//     else
//     {
//         ROS_ERROR("Failed to call service for servo 1 position");
//     }

//     srv_speed.request.command = "";
//     srv_speed.request.id = 14;
//     srv_speed.request.addr_name = "Moving_Speed";
//     srv_speed.request.value = 100;
//     if (client.call(srv_speed))
//     {
//         ROS_INFO("Service call for servo 1 speed successful");
//     }
//     else
//     {
//         ROS_ERROR("Failed to call service for servo 1 position");
//     }
//     srv1_pos.request.command = "";
//     srv1_pos.request.id = 14;
//     srv1_pos.request.addr_name = "Goal_Position";
//     srv1_pos.request.value = dyn_pos.data;
//     if (client.call(srv1_pos))
//     {
//         ROS_INFO("Service call for servo 1 position successful");
//     }
//     else
//     {
//         ROS_ERROR("Failed to call service for servo 1 position");
//     }
//     srv_speed.request.command = "";
//     srv_speed.request.id = 18;
//     srv_speed.request.addr_name = "Moving_Speed";
//     srv_speed.request.value = 100;
//     if (client.call(srv_speed))
//     {
//         ROS_INFO("Service call for servo 1 speed successful");
//     }
//     else
//     {
//         ROS_ERROR("Failed to call service for servo 1 position");
//     }
//     srv1_pos.request.command = "";
//     srv1_pos.request.id = 18;
//     srv1_pos.request.addr_name = "Goal_Position";
//     srv1_pos.request.value = dyn_pos.data;
//     if (client.call(srv1_pos))
//     {
//         ROS_INFO("Service call for servo 1 position successful");
//     }
//     else
//     {
//         ROS_ERROR("Failed to call service for servo 1 position");
//     }
    
// }


// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "dynamixel_control");
//     ros::NodeHandle nh;

//     ROS_INFO("Starting dynamixel node");
//     // Create a ServiceClient for the dynamixel_command service
//     client = nh.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");


//     dynamixel_sub = nh.subscribe("set_dynamixel_pos", 0, handle_dynamixel_pos);

//     ros::Rate loop_rate(100);

//     while (ros::ok()) {
//             ros::spinOnce();
//             loop_rate.sleep();
//     }

//     return 0;
// }

#include <ros/ros.h>
#include <dynamixel_workbench_msgs/DynamixelCommand.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>

// Subscribers
ros::Subscriber dynamixel_sub;
ros::ServiceClient client;

void set_gripper_position(int id, int pos) {
    dynamixel_workbench_msgs::DynamixelCommand srv_speed;
    srv_speed.request.command = "";
    srv_speed.request.id = id;
    srv_speed.request.addr_name = "Moving_Speed";
    srv_speed.request.value = 500;
    if (client.call(srv_speed))
    {
        ROS_INFO_STREAM("Service call for servo " << id << " speed successful");
    }
    else
    {
        ROS_ERROR_STREAM("Failed to call service for servo " << id << " speed");
    }

    dynamixel_workbench_msgs::DynamixelCommand srv_pos;
    srv_pos.request.command = "";
    srv_pos.request.id = id;
    srv_pos.request.addr_name = "Goal_Position";
    srv_pos.request.value = pos;
    if (client.call(srv_pos))
    {
        ROS_INFO_STREAM("Service call for servo " << id << " position successful");
    }
    else
    {
        ROS_ERROR_STREAM("Failed to call service for servo " << id << " position");
    }

    dynamixel_workbench_msgs::DynamixelCommand srv_torque15;
    srv_torque15.request.command = "";
    srv_torque15.request.id = 15;
    srv_torque15.request.addr_name = "Torque_Limit";
    srv_torque15.request.value = 1023;
    if (client.call(srv_torque15))
    {
        ROS_INFO_STREAM("Service call for servo " << 15 << " torque successful");
    }
    else
    {
        ROS_ERROR_STREAM("Failed to call service for servo " << 15 << " torque");
    } 

    dynamixel_workbench_msgs::DynamixelCommand srv_torque14;
    srv_torque14.request.command = "";
    srv_torque14.request.id = 14;
    srv_torque14.request.addr_name = "Torque_Limit";
    srv_torque14.request.value = 1023;
    if (client.call(srv_torque14))
    {
        ROS_INFO_STREAM("Service call for servo " << 14 << " torque successful");
    }
    else
    {
        ROS_ERROR_STREAM("Failed to call service for servo " << 14 << " torque");
    } 
    
    // dynamixel_workbench_msgs::DynamixelCommand srv_torque9;
    // srv_torque9.request.command = "";
    // srv_torque9.request.id = 18;
    // srv_torque9.request.addr_name = "Torque_Limit";
    // srv_torque9.request.value = 1023;
    // if (client.call(srv_torque9))
    // {
    //     ROS_INFO_STREAM("Service call for servo " << 18 << " torque successful");
    // }
    // else
    // {
    //     ROS_ERROR_STREAM("Failed to call service for servo " << 18 << " torque");
    // }

    


    dynamixel_workbench_msgs::DynamixelCommand srv_torque7;
    srv_speed.request.command = "";
    srv_speed.request.id = 7;
    srv_speed.request.addr_name = "Torque_Limit";
    srv_speed.request.value = 1023;
    if (client.call(srv_speed))
    {
        ROS_INFO_STREAM("Service call for servo " << 7 << " torque successful");
    }
    else
    {
        ROS_ERROR_STREAM("Failed to call service for servo " << 7 << " torque");
    }
}

void handle_dynamixel_pos(const std_msgs::String& state_str)
{
    ROS_INFO("Received set pos dynamixel command");

    if (state_str.data == "GRIP_OPEN") {
        set_gripper_position(15, 170);
        ros::Duration(0.1).sleep();
        set_gripper_position(14, 655);
    }
    else if (state_str.data == "GRIP_CLOSE") {
        set_gripper_position(14, 0);
        ros::Duration(0.5).sleep();
        set_gripper_position(14, 500);
        ros::Duration(0.1).sleep();
        set_gripper_position(15, 825);
        ros::Duration(0.5).sleep();
        set_gripper_position(15, 300);
        ros::Duration(0.1).sleep();
        set_gripper_position(14, 0);
        set_gripper_position(15, 825);
    } 
    // else if (state_str == "GRIP_BACK") {
    //     set_gripper_position(14, 0)
    //     set_gripper_position(9, 0)
    // } 
    else if (state_str.data == "CONTAINER_OPEN") {
        set_gripper_position(7, 800);
    } 
    else if (state_str.data == "CONTAINER_CLOSE") {
        set_gripper_position(7, 470);
    } 
    else if (state_str.data == "CONTAINER_MID") {
        set_gripper_position(7, 700);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dynamixel_control");
    ros::NodeHandle nh;

    ROS_INFO("Starting dynamixel node");
    // Create a ServiceClient for the dynamixel_command service
    client = nh.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");


    dynamixel_sub = nh.subscribe("servo_command", 0, handle_dynamixel_pos);

    ros::Rate loop_rate(100);

    while (ros::ok()) {
        // state_machine();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}