import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import baxter_interface
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

"""
Before this, you need to enable robot and run trajectory server.
Then run moveit.

roslaunch baxter_gazebo baxter_world.launch

rosrun baxter_tools enable_robot.py -e
rosrun baxter_interface joint_trajectory_action_server.py

roslaunch baxter_moveit_config demo_baxter.launch right_electric_gripper:=true left_electric_gripper:=true

"""

import os
import signal
import subprocess
# Climb the tree to find out where we are
p = os.path.abspath(__file__)
t = ""
while t != "code":
    (p, t) = os.path.split(p)
    if p == "":
        print "I have no idea where I am; this is ridiculous"
        sys.exit(1)
sys.path.append(os.path.join(p,"code"))

import globalConfig
import logging
mAndm_logger = logging.getLogger('mAndm_logger')

#######################################################################

# remap joint states (moved to launch file)
#remap_joint_states_subprocess = subprocess.Popen(["python","remap.py"])

mAndm_logger.info("============ Starting tutorial setup")
moveit_commander.roscpp_initialize(sys.argv) #sys.argv
rospy.init_node('move_group_python_interface_tutorial',
                anonymous=True)

# TODO: disable
#left_limb = baxter_interface.Limb('left')
#left_limb.move_to_neutral()
#right_limb = baxter_interface.Limb('right')
#right_limb.move_to_neutral()

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_left_arm = moveit_commander.MoveGroupCommander("left_arm")
group_right_arm = moveit_commander.MoveGroupCommander("right_arm")
group_both_arms = moveit_commander.MoveGroupCommander("both_arms")


display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory)

mAndm_logger.info("============ Starting tutorial ")

mAndm_logger.debug("Reference frame-moveGroup planning: %s" % group_left_arm.get_planning_frame())
mAndm_logger.debug("Reference frame-moveGroup pose: %s" % group_left_arm.get_pose_reference_frame())
mAndm_logger.debug("Reference frame-robot planning: %s" %robot.get_planning_frame())
mAndm_logger.debug("moveGroup End-effector-link: %s" % group_left_arm.get_end_effector_link())

mAndm_logger.debug("Robot Groups:" + str(robot.get_group_names()))
mAndm_logger.debug("Robot state:" + str(robot.get_current_state()))
mAndm_logger.debug("current joint values-left_arm:" + str(group_left_arm.get_current_joint_values()))
mAndm_logger.debug("current joint values-both_arms:" + str(group_both_arms.get_current_joint_values()))

mAndm_logger.info("============ Generating plan 1")

mAndm_logger.log(6,"get_goal_tolerance: " +str(group_left_arm.get_goal_tolerance()))
mAndm_logger.log(6,"get_goal_position_tolerance: " +str( group_left_arm.get_goal_position_tolerance()))
mAndm_logger.log(6,"get_goal_orientation_tolerance: " + str( group_left_arm.get_goal_orientation_tolerance()))
mAndm_logger.log(6,"get_goal_joint_tolerance: " +str( group_left_arm.get_goal_joint_tolerance()))

group_left_arm.set_goal_tolerance(0.10)
mAndm_logger.log(4, "get_goal_tolerance-set: " +str(group_left_arm.get_goal_tolerance()))

group_left_arm.set_goal_position_tolerance(0.10)
mAndm_logger.log(4, "get_goal_position_tolerance-set: " +str( group_left_arm.get_goal_position_tolerance()))

group_left_arm.set_goal_orientation_tolerance(0.01)
mAndm_logger.log(4, "get_goal_orientation_tolerance-set: " + str( group_left_arm.get_goal_orientation_tolerance()))

group_left_arm.set_goal_joint_tolerance(0.01)
mAndm_logger.log(4, "get_goal_joint_tolerance-set: " +str( group_left_arm.get_goal_joint_tolerance()))

mAndm_logger.log(2, "DOES THIS CHANGE?get_goal_tolerance-set: " +str(group_left_arm.get_goal_tolerance()))


# set goal pose
pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.x = -0.366894936773
pose_target.orientation.y = 0.885980397775
pose_target.orientation.z = 0.108155782462
pose_target.orientation.w = 0.262162481772
pose_target.position.x = 0.657579481614
pose_target.position.y = 0.851981417433
pose_target.position.z = 0.5388352386502
group_left_arm.set_pose_target(pose_target)


# group_left_arm.set_position_target([0.657579481614, 0.851981417433, 0.0388352386502], group_left_arm.get_end_effector_link())
# group_left_arm.set_random_target()
# group_left_arm.set_planner_id("RRTConnectkConfigDefault")
# group_left_arm.set_goal_tolerance(0.10)
# group_left_arm.set_planning_time(100.0)

###########################
#### plan left arm only ###
###########################
#plan1 = group_left_arm.plan()
#mAndm_logger.debug("Plan1:" + str(plan1))
#mAndm_logger.debug("type(Plan1):" + str(type(plan1)))
#rospy.sleep(3)

# TODO: test without setting all tolerance at the same time
# TODO: try planning for both arms
group_both_arms.set_pose_target(pose_target, 'left_gripper')
import copy

poseStamped_target_right = group_right_arm.get_current_pose()
poseStamped_target_right.pose.position.x = 0.657579481614
poseStamped_target_right.pose.position.y = -0.451981417433
poseStamped_target_right.pose.position.z = -0.2388352386502


group_right_arm.set_goal_position_tolerance(0.10)
group_right_arm.set_goal_orientation_tolerance(0.02)
group_right_arm.set_goal_joint_tolerance(0.01)
group_right_arm.set_planner_id("RRTConnectkConfigDefault")

###########################
#### plan right arm only ###
###########################
#plan3 = group_right_arm.plan(20.0)
#mAndm_logger.debug("Plan3:" + str(plan3))
#mAndm_logger.debug("type(Plan3):" + str(type(plan3)))
#rospy.sleep(3)

group_both_arms.set_goal_position_tolerance(0.10)
group_both_arms.set_goal_orientation_tolerance(0.02)
group_both_arms.set_goal_joint_tolerance(0.01)
group_both_arms.set_planner_id("RRTConnectkConfigDefault")

mAndm_logger.debug("group_both_arms.get_end_effector_link:" + str(group_both_arms.get_end_effector_link())) #empty

group_both_arms.set_pose_target(poseStamped_target_right, 'right_gripper')
#group_both_arms.set_position_target([0.657579481614, -0.851981417433, 0.0388352386502], 'right_gripper')

###########################
#### plan both arms only ###
###########################
plan2 = group_both_arms.plan(20.0)
mAndm_logger.debug("Plan2:" + str(plan2))
mAndm_logger.debug("type(Plan2):" + str(type(plan2)))

group_both_arms.go(wait=True)

#mAndm_logger.info("============ Visualizing plan1")
#display_trajectory = moveit_msgs.msg.DisplayTrajectory()

#display_trajectory.trajectory_start = robot.get_current_state()
#display_trajectory.trajectory.append(plan2)
#display_trajectory_publisher.publish(display_trajectory)

# #print "============ Waiting while plan1 is visualized (again)..."
# #rospy.sleep(5)

# """
# The following works.
# """
# # clean the scene
# scene.remove_world_object("pole")
# scene.remove_world_object("table")
# scene.remove_world_object("part")

# # publish a demo scene
# p = PoseStamped()
# p.header.frame_id = robot.get_planning_frame()
# p.pose.position.x = 0.7
# p.pose.position.y = -0.4
# p.pose.position.z = 0.85
# p.pose.orientation.w = 1.0
# scene.add_box("pole", p, (0.3, 0.1, 1.0))

# p.pose.position.y = -0.2
# p.pose.position.z = 0.175
# scene.add_box("table", p, (0.5, 1.5, 0.35))

# p.pose.position.x = 0.6
# p.pose.position.y = -0.7
# p.pose.position.z = 0.5
# scene.add_box("part", p, (0.15, 0.1, 0.3))

# rospy.sleep(1)

# # pick an object
# #robot.left_arm.pick("part")

rospy.spin()
moveit_commander.roscpp_shutdown()

###################################
############ TODO #################
###################################
# TF
# go to -0.10m in z-axis of Tag
# orientation of the point is negative z
# then transfer to world frame

# moveit with kinect to build depth