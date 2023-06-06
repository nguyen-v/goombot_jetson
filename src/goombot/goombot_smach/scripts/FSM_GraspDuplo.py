import rospy
import smach
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

class GraspDuploState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'success_full', 'retry', 'failure', 'low_time'], input_keys=['init_time'])

        self.timer = None
        self.n_retry = 0
        self.n_duplo = 0
        self.duplo_detected = False

        self.set_dynamixel_pos_pub = rospy.Publisher('/set_dynamixel_pos', String, queue_size=1)
        self.republish_goal_pub = rospy.Publisher('/republish_goal', Bool, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.closest_duplo_goal_filtered_sub = rospy.Subscriber('/closest_duplo_goal_filtered', PoseStamped, self.duplo_callback)

    def execute(self, userdata):
        rospy.loginfo('Executing GraspAndMove state')

        init_time = userdata.init_time

        # Grasp object
        self.publish_set_dynamixel_pos('LIFT_MID')
        rospy.sleep(10)
        self.publish_set_dynamixel_pos('GRIP_OPEN')
        rospy.sleep(10)
        self.publish_set_dynamixel_pos('LIFT_DOWN')
        rospy.sleep(10)
        self.publish_set_dynamixel_pos('GRIP_CLOSE')
        rospy.sleep(10)
        self.publish_set_dynamixel_pos('LIFT_UP')
        rospy.sleep(10)
        self.publish_set_dynamixel_pos('GRIP_RELEASE')
        rospy.sleep(10)
        self.publish_set_dynamixel_pos('GRIP_CLOSE')

        # if rospy.Time.now() - init_time > rospy.Duration(540):
        #     return "low_time"
        
        return "success"

        # # Publish False to /republish_goal
        # self.publish_republish_goal(False)

        # # Publish negative x velocity (0.1) to /cmd_vel
        # self.publish_cmd_vel(-0.1)
        # rospy.sleep(3)
        # self.publish_cmd_vel(0.0)
        # rospy.sleep(3)

        # # Launch timer for 5 seconds
        # self.timer = rospy.Timer(rospy.Duration(5), self.timer_callback)

        # # Wait for the timer to complete or duplo to be detected
        # while self.timer.is_alive() and not rospy.is_shutdown() and not self.duplo_detected:
        #     rospy.sleep(0.1)

        # # Stop the timer
        # self.timer.shutdown()

        # # Check if closest_duplo_goal_filtered callback was called during the timer
        # if self.duplo_detected:
        #     self.duplo_detected = False
        #     if self.n_retry < 3:
        #         self.publish_republish_goal(True)
        #         self.n_retry += 1
        #         return 'retry'
        #     else:
        #         self.publish_republish_goal(True)
        #         self.n_retry = 0
        #         return 'failure'

        # # Check if n_duplo > 4
        # if self.n_duplo > 4:
        #     self.publish_republish_goal(True)
        #     self.n_duplo = 0
        #     self.n_retry = 0
        #     return 'success_full'
        # else:
        #     self.publish_republish_goal(True)
        #     self.n_duplo += 1
        #     self.n_retry = 0
        #     return 'success'

    def publish_set_dynamixel_pos(self, position):
        msg = String()
        msg.data = position
        self.set_dynamixel_pos_pub.publish(msg)

    def publish_republish_goal(self, value):
        msg = Bool()
        msg.data = value
        self.republish_goal_pub.publish(msg)

    def publish_cmd_vel(self, linear_x):
        twist_cmd = Twist()
        twist_cmd.linear.x = linear_x
        self.cmd_vel_pub.publish(twist_cmd)

    def duplo_callback(self, msg):
        self.duplo_detected = True

    def timer_callback(self, event):
        pass