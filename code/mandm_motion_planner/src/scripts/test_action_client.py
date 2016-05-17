#! /usr/bin/env python

import rospy
import actionlib
import time
import argparse

from mandm_motion_planner.msg import PlanAndExecuteTrajAction, PlanAndExecuteTrajGoal

def feedback_fn(msg):
    print '-------------------------------------'
    print "We are getting feedback:\n" + str(msg)
    print '-------------------------------------'


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Trajectory Client.")
    parser.add_argument('arm', type=str, help='Specify arm. Possible paramenters: left_arm, right_arm and both_arms')

    args = parser.parse_args()

    if args.arm in ['left_arm','right_arm','both_arms']:
        rospy.init_node('plan_and_execute_traj_client_'+args.arm)
        client = actionlib.SimpleActionClient('plan_and_execute_traj_'+args.arm, PlanAndExecuteTrajAction)
        client.wait_for_server()

        goal = PlanAndExecuteTrajGoal()
        goal.tag_name = 'tag_4'
        goal.action_type = 'pickup'
        goal.planning_mode = 'rough_and_fine'
        goal.arm = args.arm
        goal.rotate_ninety = True


        # Fill in the goal here
        client.send_goal(goal,feedback_cb=feedback_fn)
        print client.get_goal_status_text()
        client.wait_for_result()#rospy.Duration.from_sec(5.0))
        result = client.get_result()
        print result

        error_dict = {1:'-no tag_name and target_pose provided.',2:'cannot locate tag in rough mode',\
                     3:'cannot locate tag in fine mode',4:'no rough plan found',\
                     5:'no fine plan found',99:'Unexpected error!'}
        if result.error_type:
            print "ERROR:" + error_dict[result.error_type]

    else:
        mAndm_logger.error("Argument Incorrect!:" + args.arm)

