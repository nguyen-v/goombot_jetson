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

enum class GripperState {
  OPEN,       // Gripper is fully open
  MOVE_DOWN,  // Gripper is moving down
  CLOSE,      // Gripper is fully closed
  MOVE_UP,    // Gripper is moving up
};

GripperState state = GripperState::OPEN;  // Start with the gripper open

void set_gripper_position(int id, int pos) {
    dynamixel_workbench_msgs::DynamixelCommand srv_speed;
    srv_speed.request.command = "";
    srv_speed.request.id = id;
    srv_speed.request.addr_name = "Moving_Speed";
    srv_speed.request.value = 40;
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
    dynamixel_workbench_msgs::DynamixelCommand srv_torque;
    srv_speed.request.command = "";
    srv_speed.request.id = id;
    srv_speed.request.addr_name = "Torque_Limit";
    srv_speed.request.value = 1023;
    if (client.call(srv_speed))
    {
        ROS_INFO_STREAM("Service call for servo " << id << " speed successful");
    }
    else
    {
        ROS_ERROR_STREAM("Failed to call service for servo " << id << " speed");
    }
}

void handle_dynamixel_pos(const std_msgs::String& state_str)
{
    ROS_INFO("Received set pos dynamixel command");

if (state_str.data == "OPEN") {
    state = GripperState::OPEN;
} else if (state_str.data == "MOVE_DOWN") {
    state = GripperState::MOVE_DOWN;
} else if (state_str.data == "CLOSE") {
    state = GripperState::CLOSE;
} else if (state_str.data == "MOVE_UP") {
    state = GripperState::MOVE_UP;
} else {
    ROS_ERROR("Invalid state string");
    return;
}

switch (state) {
    case GripperState::OPEN:
        set_gripper_position(6, 300);
        set_gripper_position(15, 100);
        state = GripperState::MOVE_DOWN;
        break;

    case GripperState::MOVE_DOWN:
        set_gripper_position(14, 810);
        set_gripper_position(9, 230);
        state = GripperState::CLOSE;
        break;

    case GripperState::CLOSE:
        set_gripper_position(6, 160);
        set_gripper_position(15, 240);
        state = GripperState::MOVE_UP;
        break;

    case GripperState::MOVE_UP:
        // set_gripper_position(14, 740);
        // set_gripper_position(9, 270);
        set_gripper_position(14, 150);
        set_gripper_position(9, 870);

        state = GripperState::OPEN;
        break;
}

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
        // state_machine();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}