#!/usr/bin/env python

import rospy
import actionlib
import time
import argparse

from verified_torque_controller.msg import verified_motionAction, verified_motionFeedback, verified_motionResult, verified_motionGoal


def feedback_fn(msg):
    print '-------------------------------------'
    print "We are getting feedback:\n" + str(msg)
    print '-------------------------------------'


if __name__ == '__main__':
    print('Starting client ...')
    rospy.init_node('verify_torque_control_motion_client')
    client = actionlib.SimpleActionClient('execute_verified_motion', verified_motionAction)
    print('Waiting for the server ...')
    client.wait_for_server()
    
    print('Declaring goal message ...')
    goal = verified_motionGoal()
    goal.arm = 'left'
    goal.EFF_movement = [0.0, 0.0, -0.1] # [dx dy dz]
    goal.motion_parameters = [1, 20, 10] # [stiffness, damping, duration]
    
    print('Sending goal message ... ')
    client.send_goal(goal,feedback_cb=feedback_fn)
    print client.get_goal_status_text()
    client.wait_for_result()#rospy.Duration.from_sec(5.0))
    result = client.get_result()
    print result