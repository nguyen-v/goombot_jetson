import rospy
import smach
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist


# Pause state
class PauseRobotState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue'])
        self.continue_subscriber = rospy.Subscriber('continue_robot', Empty, self.continue_callback)
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.continue_robot = False

    def continue_callback(self, msg):
        rospy.loginfo("Continue signal received, transitioning back to Move state")
        self.continue_robot = True

    def execute(self, userdata):
        rospy.loginfo("Inside PAUSE state")
        rospy.loginfo("Pausing the robot")

        self.continue_robot = False

        # Set all fields of cmd_vel to zero velocity
        twist_cmd = Twist()
        twist_cmd.linear.x = 0.0
        twist_cmd.linear.y = 0.0
        twist_cmd.linear.z = 0.0
        twist_cmd.angular.x = 0.0
        twist_cmd.angular.y = 0.0
        twist_cmd.angular.z = 0.0

        # Publish the zero velocity command
        self.cmd_vel_publisher.publish(twist_cmd)

        while not rospy.is_shutdown():
            if self.continue_robot:
                return 'continue'
            rospy.sleep(0.1)

        return 'continue'
