import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import baxter_interface
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

import tf
import numpy
import math
import os
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


######################################################################
# currently manually set tagName
tagName = "tag_1"
#####################################################################

mAndm_logger.info("============ Starting MoveIt! Setup ==========================")
moveit_commander.roscpp_initialize(sys.argv) #sys.argv
rospy.init_node('move_group_with_tf', anonymous=True)



robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_left_arm = moveit_commander.MoveGroupCommander("left_arm")
group_right_arm = moveit_commander.MoveGroupCommander("right_arm")
group_both_arms = moveit_commander.MoveGroupCommander("both_arms")


display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory)

mAndm_logger.info("============ Starting planning for left_arm ===================")
mAndm_logger.info("- Setting goal tolerances")

#group_left_arm.set_goal_tolerance(0.10)
#mAndm_logger.log(4, "get_goal_tolerance-set: " +str(group_left_arm.get_goal_tolerance()))
group_left_arm.set_goal_position_tolerance(0.01)
#mAndm_logger.log(4, "get_goal_position_tolerance-set: " +str( group_left_arm.get_goal_position_tolerance()))
group_left_arm.set_goal_orientation_tolerance(0.01)
#mAndm_logger.log(4, "get_goal_orientation_tolerance-set: " + str( group_left_arm.get_goal_orientation_tolerance()))
group_left_arm.set_goal_joint_tolerance(0.01)
#mAndm_logger.log(4, "get_goal_joint_tolerance-set: " +str( group_left_arm.get_goal_joint_tolerance()))
mAndm_logger.log(2, "get_goal_tolerance-set: " +str(group_left_arm.get_goal_tolerance()))


mAndm_logger.info("============ Starting to locate tag (goal position):" + tagName + " ============")

listener = tf.TransformListener()
broadcaster = tf.TransformBroadcaster()

# transform frame
# for API visit http://www.lfd.uci.edu/~gohlke/code/transformations.py.html
z_axis_offset_tf = tf.transformations.translation_matrix([0,0,0.05]) #x,y,z offset
about_x_rotation_qua = tf.transformations.quaternion_about_axis(math.pi,[1,0,0])

mAndm_logger.log(4, "about_x_rotation_qua: " + str(about_x_rotation_qua))

# find tag in world frame (x,y,z,w)
world_tag_pose = None
world_tag_rot  = None
while not (world_tag_pose and world_tag_rot):
    try:
        mAndm_logger.debug("list of frames:" + str(listener.getFrameStrings()))
        (world_tag_pose,world_tag_rot) = listener.lookupTransform('torso', tagName, rospy.Time(0))
        #print "Does " + tagName + " exist?" + str(listener.frameExists(tagName))
    except:
        rospy.sleep(1)
        mAndm_logger.debug("Stilling waiting for tag pose and orientation in the world frame")

mAndm_logger.log(4, "world_original_tag_pose: " + str(world_tag_pose))
mAndm_logger.log(4, "world_original_tag_rot: " + str(world_tag_rot))

world_to_tag_pose_tf = tf.transformations.translation_matrix(list(world_tag_pose))

world_transformed_tag_pose = world_to_tag_pose_tf.dot(z_axis_offset_tf.dot([0,0,0,1]))
world_transformed_tag_rot  = tf.transformations.quaternion_multiply(list(world_tag_rot), about_x_rotation_qua)

mAndm_logger.log(4, "world_transformed_tag_pose: " + str(world_transformed_tag_pose))
mAndm_logger.log(4, "world_transformed_tag_rot: " + str(world_transformed_tag_rot))

mAndm_logger.debug("if you want to see tranformed tag in Rviz, run test_tf_broadcaster.py with tagName replaced in the file")

mAndm_logger.info("============ Setting goal position ======================")

# set goal pose
pose_target = geometry_msgs.msg.Pose()
pose_target.orientation = geometry_msgs.msg.Quaternion(*world_transformed_tag_rot)
pose_target.position = geometry_msgs.msg.Point(*world_transformed_tag_pose[0:3])
group_left_arm.set_pose_target(pose_target)
group_left_arm.set_planner_id("RRTConnectkConfigDefault")

mAndm_logger.log(4, "pose_target: " + str(pose_target))

mAndm_logger.info("============ PLanning to goal position ======================")

plan = group_left_arm.plan(20.0)
mAndm_logger.debug("plan:" + str(plan))

mAndm_logger.info("============ Scaling plan velocity ======================")
def scale_trajectory_speed(traj, scale):
    # Create a new trajectory object
    new_traj = RobotTrajectory()

    # Initialize the new trajectory to be the same as the planned trajectory
    new_traj.joint_trajectory = traj.joint_trajectory

    # Get the number of joints involved
    n_joints = len(traj.joint_trajectory.joint_names)

    # Get the number of points on the trajectory
    n_points = len(traj.joint_trajectory.points)

    # Store the trajectory points
    points = list(traj.joint_trajectory.points)

    # Cycle through all points and scale the time from start, speed and acceleration
    for i in range(n_points):
        point = JointTrajectoryPoint()
        point.time_from_start = traj.joint_trajectory.points[i].time_from_start / scale
        point.velocities = list(traj.joint_trajectory.points[i].velocities)
        point.accelerations = list(traj.joint_trajectory.points[i].accelerations)
        point.positions = traj.joint_trajectory.points[i].positions

        for j in range(n_joints):
            point.velocities[j] = point.velocities[j] * scale
            point.accelerations[j] = point.accelerations[j] * scale * scale

        points[i] = point

    # Assign the modified points to the new trajectory
    new_traj.joint_trajectory.points = points

    # Return the new trajecotry
    return new_traj

scaled_plan = scale_trajectory_speed(plan, 0.3)
mAndm_logger.debug("scaled_plan:" + str(scaled_plan))

mAndm_logger.info("============ Excuting plan ======================")
#group_left_arm.go(wait=True)
group_left_arm.execute(scaled_plan)

mAndm_logger.info("============ Excution is done! Please exit with Ctrl-C ======================")


#rospy.spin()
moveit_commander.roscpp_shutdown()





