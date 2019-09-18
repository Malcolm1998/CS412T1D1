#!/usr/bin/env python
# -*- coding: utf-8 -*-


from __future__ import print_function

import rospy
from rospy import Subscriber
from smach import StateMachine, State
import smach_ros

from std_msgs.msg import String


class Tic(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        self._active = False
        self._ros_rate = rospy.Rate(100)
        self._tic_received = False
        self._listener = Subscriber('tic', String, callback=self._listener_cb)

    def execute(self, ud):
        self._active = True
        while not rospy.is_shutdown() and not self._tic_received:
            self._ros_rate.sleep()
        self._active = False
        self._tic_received = False
        return 'success'

    def _listener_cb(self, msg):
        if self._active:
            self._tic_received = True


if __name__ == '__main__':
    rospy.init_node('test_fsm', anonymous=True)

    top = StateMachine(outcomes=['success'])
    with top:
        StateMachine.add('TIC_A', Tic(), transitions={'success':'TIC_B'})
        StateMachine.add('TIC_B', Tic(), transitions={'success':'TIC_C'})
        StateMachine.add('TIC_C', Tic(), transitions={'success':'TIC_A'})

    sis = smach_ros.IntrospectionServer(str(rospy.get_name()), top, '/SM_ROOT' + str(rospy.get_name()))
    sis.start()

    top.execute()

    rospy.spin()
    sis.stop()
