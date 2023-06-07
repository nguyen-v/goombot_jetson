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
  LIFT_DOWN,  // Gripper is moving down
  LIFT_UP,    // Gripper is moving up
  LIFT_MID,
  GRIP_CLOSE,      // Gripper is fully closed
  GRIP_OPEN,       // Gripper is fully open
  GRIP_RELEASE,
  CONTAINER_OPEN,
  CONTAINER_CLOSE,
  CONTAINER_MID
};

GripperState state = GripperState::GRIP_OPEN;  // Start with the gripper open

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
    dynamixel_workbench_msgs::DynamixelCommand srv_torque6;
    srv_speed.request.command = "";
    srv_speed.request.id = 6;
    srv_speed.request.addr_name = "Torque_Limit";
    srv_speed.request.value = 1023;
    if (client.call(srv_speed))
    {
        ROS_INFO_STREAM("Service call for servo " << 6 << " torque successful");
    }
    else
    {
        ROS_ERROR_STREAM("Failed to call service for servo " << 6 << " speed");
    }
    dynamixel_workbench_msgs::DynamixelCommand srv_torque9;
    srv_speed.request.command = "";
    srv_speed.request.id = 9;
    srv_speed.request.addr_name = "Torque_Limit";
    srv_speed.request.value = 1000;
    if (client.call(srv_speed))
    {
        ROS_INFO_STREAM("Service call for servo " << 9 << " torque successful");
    }
    else
    {
        ROS_ERROR_STREAM("Failed to call service for servo " << 9 << " speed");
    }
    dynamixel_workbench_msgs::DynamixelCommand srv_torque18;
    srv_speed.request.command = "";
    srv_speed.request.id = 18;
    srv_speed.request.addr_name = "Torque_Limit";
    srv_speed.request.value = 1023;
    if (client.call(srv_speed))
    {
        ROS_INFO_STREAM("Service call for servo " << 18 << " torque successful");
    }
    else
    {
        ROS_ERROR_STREAM("Failed to call service for servo " << 18 << " speed");
    }
    dynamixel_workbench_msgs::DynamixelCommand srv_torque14;
    srv_speed.request.command = "";
    srv_speed.request.id = 14;
    srv_speed.request.addr_name = "Torque_Limit";
    srv_speed.request.value = 1023;
    if (client.call(srv_speed))
    {
        ROS_INFO_STREAM("Service call for servo " << 14 << " torque successful");
    }
    else
    {
        ROS_ERROR_STREAM("Failed to call service for servo " << 14 << " speed");
    }

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
        ROS_ERROR_STREAM("Failed to call service for servo " << 7 << " speed");
    }
}

void handle_dynamixel_pos(const std_msgs::String& state_str)
{
    ROS_INFO("Received set pos dynamixel command");

if (state_str.data == "GRIP_OPEN") {
    state = GripperState::GRIP_OPEN;
} else if (state_str.data == "GRIP_CLOSE") {
    state = GripperState::GRIP_CLOSE;
} else if (state_str.data == "GRIP_RELEASE") {
    state = GripperState::GRIP_RELEASE;
} else if (state_str.data == "LIFT_DOWN") {
    state = GripperState::LIFT_DOWN;
} else if (state_str.data == "LIFT_UP") {
    state = GripperState::LIFT_UP;
} else if (state_str.data == "LIFT_MID") {
    state = GripperState::LIFT_MID;
} else if (state_str.data == "CONTAINER_OPEN") {
    state = GripperState::CONTAINER_OPEN;
} else if (state_str.data == "CONTAINER_CLOSE") {
    state = GripperState::CONTAINER_CLOSE;
} else if (state_str.data == "CONTAINER_MID") {
    state = GripperState::CONTAINER_MID;
} else {
    ROS_ERROR("Invalid state string");
    return;
}


switch (state) {
    case GripperState::LIFT_DOWN:
        set_gripper_position(18, 770);
        set_gripper_position(6, 250);
        break;

    case GripperState::LIFT_UP:
        // set_gripper_position(14, 740);
        // set_gripper_position(9, 270);
        // set_gripper_position(15, 200);
        set_gripper_position(18, 200);
        set_gripper_position(6, 820);
        
        break;

    case GripperState::LIFT_MID:
        // set_gripper_position(14, 740);
        // set_gripper_position(9, 270);
        set_gripper_position(18, 500);
        set_gripper_position(6, 500);
        break;

    case GripperState::GRIP_OPEN:
        set_gripper_position(9, 700);
        set_gripper_position(14, 640);
        break;

    case GripperState::GRIP_CLOSE:
        //set_gripper_position(9, 490);
        //set_gripper_position(14, 820);
        set_gripper_position(9, 500);
        set_gripper_position(14, 820);
        break;

    case GripperState::GRIP_RELEASE:
        set_gripper_position(9, 400);
        set_gripper_position(14, 920);
        break;

    case GripperState::CONTAINER_OPEN:
        set_gripper_position(7, 800);
        break;

    case GripperState::CONTAINER_CLOSE:
        set_gripper_position(7, 470);
        break;
    
    case GripperState::CONTAINER_MID:
        set_gripper_position(7, 700);
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