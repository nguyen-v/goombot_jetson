#!/usr/bin/env python
import rospy
import smach
import smach_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from FSM_GoToDuplo import GoToDuploState
from FSM_GraspDuplo import GraspDuploState
from FSM_DropDuplo import DropDuploState
from FSM_RotateInPlace import RotateInPlaceState
from FSM_ExploreMap import ExploreMapState
from FSM_GoDropZone import GoDropZoneState
from FSM_PauseRobot import PauseRobotState
from FSM_ActuateButton import ActuateButtonState

from std_msgs.msg import Int16


def main():
    rospy.init_node('state_machine_node')

    left_ticks = rospy.wait_for_message('/left_ticks', Int16)
    rospy.sleep(5)

    # Create a publisher for the /reset_controller topic
    reset_controller_pub = rospy.Publisher('/reset_controller', String, queue_size=10)

    # Create and publish the message with "speed" as data
    for i in range(5):
        reset_msg = String()
        reset_msg.data = "speed"
        reset_controller_pub.publish(reset_msg)
        rospy.sleep(1)

    # Set the initial pose
    initial_pose_publisher = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
    set_dynamixel_pos_pub = rospy.Publisher('/servo_command', String, queue_size=10)
    initial_pose_msg = PoseWithCovarianceStamped()
    initial_pose_msg.header.frame_id = 'map'
    initial_pose_msg.pose.pose.position.x = 2.35
    initial_pose_msg.pose.pose.position.y = 2.45
    initial_pose_msg.pose.pose.position.z = 0.0
    initial_pose_msg.pose.pose.orientation.x = 0.0
    initial_pose_msg.pose.pose.orientation.y = 0.0
    initial_pose_msg.pose.pose.orientation.z = -0.707
    initial_pose_msg.pose.pose.orientation.w = 0.707
    msg = String()
    msg.data = 'LIFT_IDLE'
    set_dynamixel_pos_pub.publish(msg)
    msg = String()
    msg.data = 'GRIP_OPEN'
    set_dynamixel_pos_pub.publish(msg)
    msg = String()
    msg.data = 'CONTAINER_CLOSE'
    set_dynamixel_pos_pub.publish(msg)


    # Wait for the publisher to connect
    while initial_pose_publisher.get_num_connections() < 1:
        rospy.sleep(0.1)

    rospy.sleep(3)
    # Publish the initial pose
    rospy.loginfo("Publishing initial pose")
    initial_pose_publisher.publish(initial_pose_msg)

    rospy.sleep(5)

    sm = smach.StateMachine(outcomes=['completed'])
    sm.userdata.init_time = rospy.Time.now()
    sm.userdata.explore_state = "GO_TO_BUTTON"

    with sm:
        smach.StateMachine.add('EXPLORE_MAP', ExploreMapState(),
                               transitions={'duplo_detected': 'GO_TO_DUPLO',
                                            'low_time': 'GO_TO_DROP_ZONE',
                                            'success' : 'ROTATE_IN_PLACE',
                                            'failure' : 'EXPLORE_MAP',
                                            'pause'   : 'PAUSE_ROBOT',
                                            'actuate_button' : 'ACTUATE_BUTTON'},
                               remapping={'init_time': 'init_time',
                                          'explore_state_in' : 'explore_state',
                                          'explore_state_out' : 'explore_state'})
        
        smach.StateMachine.add('ACTUATE_BUTTON', ActuateButtonState(),
                                transitions = {'success': 'EXPLORE_MAP'})
        
        
        smach.StateMachine.add('GO_TO_DUPLO', GoToDuploState(),
                               transitions={'success': 'GRASP_DUPLO',
                                            'low_time': 'GO_TO_DROP_ZONE',
                                            'failure': 'ROTATE_IN_PLACE',
                                            'pause'   : 'PAUSE_ROBOT'},
                                remapping={'init_time': 'init_time'})

        smach.StateMachine.add('GRASP_DUPLO', GraspDuploState(),
                               transitions={'retry': 'GO_TO_DUPLO',
                                            'success': 'ROTATE_IN_PLACE',
                                            'failure': 'ROTATE_IN_PLACE',
                                            'success_full': 'GO_TO_DROP_ZONE',
                                            'low_time': 'GO_TO_DROP_ZONE',
                                            'pause'   : 'PAUSE_ROBOT'},
                                remapping={'init_time': 'init_time'})

        smach.StateMachine.add('GO_TO_DROP_ZONE', GoDropZoneState(),
                               transitions={'success': 'DROP_DUPLO',
                                            'pause'   : 'PAUSE_ROBOT'})

        smach.StateMachine.add('DROP_DUPLO', DropDuploState(),
                               transitions={'success': 'EXPLORE_MAP',
                                            'pause'   : 'PAUSE_ROBOT'},
                                remapping = {'init_time' : 'init_time'})

        smach.StateMachine.add('ROTATE_IN_PLACE', RotateInPlaceState(),
                               transitions={'low_time': 'GO_TO_DROP_ZONE',
                                            'no_duplo': 'EXPLORE_MAP',
                                            'duplo_detected': 'GO_TO_DUPLO',
                                            'pause'   : 'PAUSE_ROBOT'},
                               remapping={'init_time': 'init_time',
                                          'explore_state_in' : 'explore_state'})
    
        smach.StateMachine.add('PAUSE_ROBOT', PauseRobotState(),
                               transitions={'continue'   : 'EXPLORE_MAP'},
                               )
        
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('goombot_fsm_server', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()

    if outcome == 'completed':
        rospy.loginfo("State machine completed successfully.")
        sis.stop()

    

if __name__ == '__main__':
    main()