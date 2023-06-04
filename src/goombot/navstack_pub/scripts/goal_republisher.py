#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from move_base_msgs.msg import MoveBaseActionResult
import threading

class GoalRepublisher:
    def __init__(self):
        self.goal = None
        self.mutex = threading.Lock()
        self.publish_frequency = rospy.get_param('~publish_frequency', 2.0)  # Default: 1.0 second

        # ROS publishers and subscribers
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.goal_sub = rospy.Subscriber('/goombot/goal', PoseStamped, self.goal_callback, queue_size=10)
        self.republish_goal_sub = rospy.Subscriber('/republish_goal', Bool, self.republish_goal_callback, queue_size=10)

        # self.goal_valid_pub = rospy.Publisher('/goal_is_valid', Bool, queue_size=10)
        # self.goal_valid_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.result_callback, queue_size=10)

        self.should_republish = True  # Default: True

    def result_callback(self, msg):
        is_valid = msg.status.status == 3  # Check if the status is 3 (SUCCEEDED) indicating a valid goal

        # Publish the result on the /goal_is_valid topic
        self.goal_valid_pub.publish(Bool(is_valid))

    def goal_callback(self, msg):
        with self.mutex:
            self.goal = msg

    def republish_goal_callback(self, msg):
        self.should_republish = msg.data

    def republish_goal(self):
        rate = rospy.Rate(self.publish_frequency)  # Convert publish_frequency to rate

        while not rospy.is_shutdown():
            with self.mutex:
                if self.goal is not None and self.should_republish:
                    self.goal.header.frame_id = 'odom'
                    self.goal_pub.publish(self.goal)
            rate.sleep()

def main():
    rospy.init_node('goal_republisher')
    republisher = GoalRepublisher()
    republisher.republish_goal()

if __name__ == '__main__':
    main()
