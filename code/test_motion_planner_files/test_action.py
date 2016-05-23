#! /usr/bin/env python

import rospy
import actionlib
import time

from mandm_motion_planner.msg import PlanAndExecuteTrajAction, PlanAndExecuteTrajFeedback, PlanAndExecuteTrajResult

class PlanAndExecuteTrajServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('plan_and_execute_traj', PlanAndExecuteTrajAction, self.execute, False)
    print "we are starting...!"
    self.feedback = PlanAndExecuteTrajFeedback()
    self.result = PlanAndExecuteTrajResult()

    print "feedback:" + str(self.feedback)
    print type(self.feedback)
    print self.feedback.rough_plan_found
    print self.feedback.executing_rough_plan
    print self.feedback.executing_grasp

    print "result:" + str(self.result)
    print type(self.result)
    print self.result.rough_execution_result
    print self.result.rough_planning_result
    print self.result.grasp_result

    self.server.start()


  def execute(self, goal):
    # Do lots of awesome groundbreaking robot stuff here
    print "we are executing...!"
    print goal.tag_name
    print goal.target_pose
    print goal.action_type
    self.result.grasp_result = True
    self.server.publish_feedback(self.feedback)

    print dir(self.server)
    time.sleep(5)

    self.feedback.rough_plan_found = True
    self.server.publish_feedback(self.feedback)

    self.server.set_succeeded(result=self.result)


if __name__ == '__main__':
  rospy.init_node('plan_and_execute_traj_server')
  server = PlanAndExecuteTrajServer()
  rospy.spin()