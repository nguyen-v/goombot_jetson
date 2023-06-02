#!/usr/bin/env python
import rospy
import actionlib
from geometry_msgs.msg import PoseWithCovarianceStamped
from smach import State, StateMachine
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import tf
import math

roll = 0
pitch = 0
yaw = -math.pi/2



waypoints = [
    ['one', (-0.459, -3.343), (0.0, 0.0, -0.707, 0.707)],
    # ['one', (-0.459, -3.343), (0.0, 0.0, 0, 1)]
    ['two', (0.868, -2.528), (0, 0, 0.1, 1)]
]

class Waypoint(State):
    def __init__(self, position, orientation):
        State.__init__(self, outcomes=['success'])

        # Get an action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        # Define the goal
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.pose.position.x = position[0]
        self.goal.target_pose.pose.position.y = position[1]
        self.goal.target_pose.pose.position.z = 0.0
        self.goal.target_pose.pose.orientation.x = orientation[0]
        self.goal.target_pose.pose.orientation.y = orientation[1]
        self.goal.target_pose.pose.orientation.z = orientation[2]
        self.goal.target_pose.pose.orientation.w = orientation[3]
        self.client.send_goal(self.goal)

    def execute(self, userdata):

        self.client.wait_for_result()
        return 'success'


if __name__ == '__main__':
    rospy.init_node('fsm_goombot')

    # Set the initial pose
    initial_pose_publisher = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
    initial_pose_msg = PoseWithCovarianceStamped()
    initial_pose_msg.header.frame_id = 'map'
    initial_pose_msg.pose.pose.position.x = -0.603
    initial_pose_msg.pose.pose.position.y = -1.361
    initial_pose_msg.pose.pose.position.z = 0.0
    initial_pose_msg.pose.pose.orientation.x = 0.0
    initial_pose_msg.pose.pose.orientation.y = 0.0
    initial_pose_msg.pose.pose.orientation.z = -0.707
    initial_pose_msg.pose.pose.orientation.w = 0.707

    # Wait for the publisher to connect
    while initial_pose_publisher.get_num_connections() < 1:
        rospy.sleep(0.1)

    rospy.sleep(3)
    # Publish the initial pose
    initial_pose_publisher.publish(initial_pose_msg)

    rospy.sleep(5)

    quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

    rospy.loginfo("First converted waypoint coordinates: x %f y %f z %f w %f", quat[0], quat[1], quat[2], quat[3])

    patrol = StateMachine('success')
    with patrol:
        for i, w in enumerate(waypoints):
            StateMachine.add(w[0],
                             Waypoint(w[1], w[2]),
                             transitions={'success': waypoints[(i + 1) % len(waypoints)][0]})
                
    patrol.execute()

