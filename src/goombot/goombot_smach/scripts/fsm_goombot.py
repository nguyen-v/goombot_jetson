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


def main():
    rospy.init_node('state_machine_node')

    # Set the initial pose
    initial_pose_publisher = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
    set_dynamixel_pos_pub = rospy.Publisher('/set_dynamixel_pos', String, queue_size=1)
    initial_pose_msg = PoseWithCovarianceStamped()
    initial_pose_msg.header.frame_id = 'map'
    initial_pose_msg.pose.pose.position.x = 2.15
    initial_pose_msg.pose.pose.position.y = 2.40
    initial_pose_msg.pose.pose.position.z = 0.0
    initial_pose_msg.pose.pose.orientation.x = 0.0
    initial_pose_msg.pose.pose.orientation.y = 0.0
    initial_pose_msg.pose.pose.orientation.z = -0.707
    initial_pose_msg.pose.pose.orientation.w = 0.707
    msg = String()
    msg.data = 'LIFT_UP'
    set_dynamixel_pos_pub.publish(msg)
    msg = String()
    msg.data = 'GRIP_RELEASE'
    set_dynamixel_pos_pub.publish(msg)


    # Wait for the publisher to connect
    while initial_pose_publisher.get_num_connections() < 1:
        rospy.sleep(0.1)

    rospy.sleep(3)
    # Publish the initial pose
    initial_pose_publisher.publish(initial_pose_msg)

    rospy.sleep(5)

    sm = smach.StateMachine(outcomes=['completed'])
    sm.userdata.init_time = rospy.Time.now()

    with sm:
        smach.StateMachine.add('EXPLORE_MAP', ExploreMapState(),
                               transitions={'duplo_detected': 'GO_TO_DUPLO',
                                            'low_time': 'GO_TO_DROP_ZONE',
                                            'success' : 'ROTATE_IN_PLACE'},
                               remapping={'init_time': 'init_time'})
        
        smach.StateMachine.add('GO_TO_DUPLO', GoToDuploState(),
                               transitions={'success': 'GRASP_DUPLO',
                                            'low_time': 'GO_TO_DROP_ZONE',
                                            'failure': 'ROTATE_IN_PLACE'},
                                remapping={'init_time': 'init_time'})

        smach.StateMachine.add('GRASP_DUPLO', GraspDuploState(),
                               transitions={'retry': 'GO_TO_DUPLO',
                                            'success': 'ROTATE_IN_PLACE',
                                            'failure': 'ROTATE_IN_PLACE',
                                            'success_full': 'GO_TO_DROP_ZONE',
                                            'low_time': 'GO_TO_DROP_ZONE'},
                                remapping={'init_time': 'init_time'})

        smach.StateMachine.add('GO_TO_DROP_ZONE', GoDropZoneState(),
                               transitions={'success': 'DROP_DUPLO'})

        smach.StateMachine.add('DROP_DUPLO', DropDuploState(),
                               transitions={'success': 'EXPLORE_MAP'})

        smach.StateMachine.add('ROTATE_IN_PLACE', RotateInPlaceState(),
                               transitions={'low_time': 'GO_TO_DROP_ZONE',
                                            'no_duplo': 'EXPLORE_MAP',
                                            'duplo_detected': 'GO_TO_DUPLO'},
                               remapping={'init_time': 'init_time'})
    
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('goombot_fsm_server', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()

    if outcome == 'completed':
        rospy.loginfo("State machine completed successfully.")
        sis.stop()

    

if __name__ == '__main__':
    main()