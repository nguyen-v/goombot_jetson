import rospy
import smach
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

class GraspDuploState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'success_full', 'retry', 'failure', 'low_time', 'pause'], input_keys=['init_time'])

        self.timer = None
        self.n_retry = 0
        self.n_duplo = 0
        self.duplo_detected = False
        self.is_in_state = False
        self.current_state = None
        self.servo_state = None

        self.set_dynamixel_pos_pub = rospy.Publisher('/servo_command', String, queue_size=1)
        self.republish_goal_pub = rospy.Publisher('/republish_goal', Bool, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.robot_state_pub = rospy.Publisher('/robot_state_command', String, queue_size=1)
        
        self.pause_subscriber = rospy.Subscriber('pause_robot', Empty, self.pause_callback)
        self.robot_state_sub = rospy.Subscriber('robot_state', String, self.robot_state_callback)
        self.servo_state_sub = rospy.Subscriber('servo_state', String, self.servo_state_callback)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        self.pause_robot = False


    def pause_callback(self, msg):
        if self.is_in_state:
            rospy.loginfo("Pause signal received, transitioning to Pause state")
            self.pause_robot = True

    def robot_state_callback(self, msg):
        if self.is_in_state:
            self.current_state = msg.data

    def servo_state_callback(self, msg):
        rospy.loginfo(msg.data)
        if self.is_in_state:
            self.servo_state = msg.data
    
    def execute(self, userdata):
        rospy.loginfo('Executing GraspAndMove state')

        self.servo_state = None
        self.current_state = None

        self.client.cancel_all_goals()
        # while self.current_state != "STOP":
        #     rospy.loginfo("stopping robot")
        #     self.client.cancel_goal()
        stop_command = String()
        stop_command.data = "STOP"
        self.robot_state_pub.publish(stop_command)
        for i in range(5):
            self.publish_cmd_vel(0.0)


        rospy.loginfo("Robot stopped")


        self.pause_robot = False
        self.is_in_state = True

        self.closest_duplo_goal_filtered_sub = rospy.Subscriber('/closest_duplo_goal_filtered', PoseStamped, self.duplo_callback)

        init_time = userdata.init_time


        # Grasp object
        for i in range(5):
            self.publish_set_dynamixel_pos('LIFT_DOWN')
            rospy.sleep(0.1)
        rospy.sleep(0.5)
        self.publish_set_dynamixel_pos('GRIP_CLOSE')
        rospy.sleep(4)
        self.publish_set_dynamixel_pos('LIFT_UP')
        rospy.sleep(2)
        self.publish_set_dynamixel_pos('GRIP_OPEN')
        rospy.sleep(0.5)
        self.publish_set_dynamixel_pos('LIFT_IDLE')
        # while self.servo_state != "LIFT_DOWN":
        #     self.publish_set_dynamixel_pos('LIFT_DOWN')
        #     rospy.sleep(0.1)
        #     rospy.loginfo("Lift down command")
        # rospy.sleep(2)

        # while self.servo_state != "GRIP_CLOSE":
        #     self.publish_set_dynamixel_pos('GRIP_CLOSE')
        #     rospy.loginfo("Grip close command")
        #     rospy.sleep(0.1)
        # rospy.sleep(5)

        # while self.servo_state != "LIFT_UP":
        #     self.publish_set_dynamixel_pos('LIFT_UP')
        #     rospy.sleep(0.1)
        # rospy.sleep(3)

        # while self.servo_state != "GRIP_OPEN":
        #     self.publish_set_dynamixel_pos('GRIP_OPEN')
        #     rospy.sleep(0.1)
        # rospy.sleep(1)

        # while self.servo_state != "GRIP_IDLE":
        #     self.publish_set_dynamixel_pos('GRIP_IDLE')
        #     rospy.sleep(0.1)
        # rospy.sleep(2)

        # while self.current_state != "MOVE":
        # move_command = String()
        # move_command.data = "MOVE"
        # self.robot_state_pub.publish(move_command)

        if rospy.Time.now() - init_time > rospy.Duration(540):
            move_command = String()
            move_command.data = "MOVE"
            self.robot_state_pub.publish(move_command)
            self.is_in_state = False
            return "low_time"
            
        
        return "success"

        # # Publish False to /republish_goal
        # self.publish_republish_goal(False)

        # Publish negative x velocity (0.1) to /cmd_vel
        self.duplo_detected = False
        # self.publish_cmd_vel(-0.1)
        # rospy.sleep(2)
        # self.publish_cmd_vel(0.0)
        # rospy.sleep(1)

        # Launch timer for 5 seconds
        rospy.loginfo("launching timer")
        self.timer = rospy.Timer(rospy.Duration(1), self.timer_callback)

        # Wait for the timer to complete or duplo to be detected
        while self.timer.is_alive() and not rospy.is_shutdown() and not self.duplo_detected:
            rospy.sleep(0.1)

            # Check if closest_duplo_goal_filtered callback was called during the timer
            if self.duplo_detected:
                self.duplo_detected = False
                if self.n_retry < 3:
                    rospy.loginfo("n_retry %f", self.n_retry)
                    self.publish_republish_goal(True)
                    self.n_retry += 1
                    self.is_in_state = False
                    self.closest_duplo_goal_filtered_sub.unregister()
                    move_command = String()
                    move_command.data = "MOVE"
                    self.robot_state_pub.publish(move_command)
                    return 'retry'
                else:
                    self.publish_republish_goal(True)
                    self.n_retry = 0
                    self.closest_duplo_goal_filtered_sub.unregister()
                    move_command = String()
                    move_command.data = "MOVE"
                    self.robot_state_pub.publish(move_command)
                    return 'failure'
            if self.pause_robot:
                return 'pause'

            # Check if n_duplo > 4
            if self.n_duplo > 7:
                self.publish_republish_goal(True)
                self.n_duplo = 0
                self.n_retry = 0
                self.is_in_state = False
                move_command = String()
                move_command.data = "MOVE"
                self.robot_state_pub.publish(move_command)
                return 'success_full'
            else:
                self.publish_republish_goal(True)
                rospy.loginfo("n_duplo %f", self.n_duplo)
                self.n_duplo += 1
                self.n_retry = 0
                self.is_in_state = False
                self.closest_duplo_goal_filtered_sub.unregister()
                move_command = String()
                move_command.data = "MOVE"
                self.robot_state_pub.publish(move_command)
                return 'success'
            
        
        

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
        twist_cmd.angular.z = 0
        self.cmd_vel_pub.publish(twist_cmd)

    def duplo_callback(self, msg):
        if self.is_in_state:
            self.duplo_detected = True

    def timer_callback(self, event):
        rospy.loginfo("timer callback")
        # Stop the timer
        self.timer.shutdown()
        # pass
