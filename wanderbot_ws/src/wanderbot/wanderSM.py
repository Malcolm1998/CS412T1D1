#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# define state MoveForward
class MoveForward(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['start_rotate_right'],
                             input_keys=['state_change_time_in'],
                             output_keys=['state_change_time_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing state MoveForward')
        twist = Twist()
        global g_range_ahead
        global cmd_vel_pub

        while True:
            print(g_range_ahead)
            if (g_range_ahead < 0.8 or rospy.Time.now() > userdata.state_change_time_in):
                userdata.state_change_time_out = rospy.Time.now() + rospy.Duration(5)
                return 'start_rotate_right'
            else:
                twist.linear.x = 1
                cmd_vel_pub.publish(twist)
                time.sleep(0.5)#TODO:


# define state TurnRight
class RotateLeft(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['start_move_forward'],
                             input_keys=['state_change_time_in'],
                             output_keys=['state_change_time_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing state RotateRight')
        twist = Twist()
        global g_range_ahead
        global cmd_vel_pub

        while True:
            print(g_range_ahead)
            if (rospy.Time.now() > userdata.state_change_time_in):
                userdata.state_change_time_out = rospy.Time.now() + rospy.Duration(30)
                return 'start_move_forward'
            else:
                twist.angular.z = 1
                cmd_vel_pub.publish(twist)
                time.sleep(0.5)


def scan_callback(msg):
    global g_range_ahead
    print(min(msg.ranges))
    g_range_ahead = min(msg.ranges)

g_range_ahead = 1
cmd_vel_pub = None
def main():
    global g_range_ahead
    global cmd_vel_pub

    scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.init_node('wanderSM')
    rate = rospy.Rate(10)

    sm = smach.StateMachine(outcomes=['outcome4'])
    sm.userdata.state_change_time = rospy.Time.now();

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Move_Forward', MoveForward(),
                               transitions={'start_rotate_right':'Rotate_Left'},
                               remapping={'state_change_time_in':'state_change_time',
                                          'state_change_time_out':'state_change_time'})
        smach.StateMachine.add('Rotate_Left', RotateLeft(),
                               transitions={'start_move_forward':'Move_Forward'},
                               remapping={'state_change_time_in':'state_change_time',
                                          'state_change_time_out':'state_change_time'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('wander_sm', sm, '/WANDER_SM')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    # Stop smash viewer
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
