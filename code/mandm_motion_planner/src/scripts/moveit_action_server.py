#! /usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import baxter_interface
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import JointState
import trajectory_msgs.msg


import tf
import numpy, math, time, argparse, traceback
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

# for action in ros
import actionlib
# ros_ws/devel/share/mandm_motion_planner
from mandm_motion_planner.msg import PlanAndExecuteTrajAction, PlanAndExecuteTrajFeedback, PlanAndExecuteTrajResult

# NOTE:
# changed ompl_planning.yaml in moveit of baxter longest_valid_segment_fraction: 0.02 for better planning with obstacles
# changed kinect_sensor.yaml in moveit of baxter's padding_scale: 2.0 #3.0

###### restart the rethink software for moveit (ssh)##########
# sudo rc-service rethink stop
# sudo rc-service rethink start
##########################################################

# TODO:
# stuck because moveit doesn't terminate correctly.

class PlanAndExecuteTrajServer:
    def __init__(self, arm):
        try:
            self.server = actionlib.SimpleActionServer('plan_and_execute_traj_'+arm, PlanAndExecuteTrajAction, self.execute, False)
            self.feedback = None
            self.result = None

            # check type of server.
            if arm not in ['left_arm','right_arm','both_arms','any']:
                mAndm_logger.info("===== Arm parameter is not invalid. Setting to any.=====")
                arm = 'any'
            self.arm = arm
            mAndm_logger.info("===== This is a {arm} traj server.=====".format(arm=arm))

            # clear up previous control mode
            mAndm_logger.info("===== Before we start, run clean exit from advanced control modes (joint torque or velocity).=====")
            self.limb = {}
            self.limb["left"] = baxter_interface.Limb('left')
            self.limb["right"] = baxter_interface.Limb('right')
            self.limb["left"].exit_control_mode()
            self.limb["right"].exit_control_mode()

            # setting valid parameters for incoming requests
            if arm == 'any':
                self.possible_arm_groups = ["left_arm", "right_arm", "both_arms"]
            else:
                self.possible_arm_groups = [arm]
            self.possilbe_action_types = ['pickup','drop']
            self.possible_planning_modes = ['rough','fine','rough_and_fine']
            self.planning_timeout = 20.0

            mAndm_logger.info("============ Setting up MoveIt! ==========================")
            moveit_commander.roscpp_initialize(sys.argv) #sys.argv
            self.robot = moveit_commander.RobotCommander()
            self.scene = moveit_commander.PlanningSceneInterface()
            self.group = {}
            for group_name in self.possible_arm_groups:
                mAndm_logger.info("--setting up MoveGroup: {group_name}--".format(group_name=group_name))
                self.group[group_name] = moveit_commander.MoveGroupCommander(group_name)
                self.group[group_name].set_goal_tolerance(0.01)
                self.group[group_name].set_planner_id("RRTConnectkConfigDefault")

            mAndm_logger.info("============ Setting up tf ==========================")
            self.tf_listener = tf.TransformListener()
            self.tf_broadcaster = tf.TransformBroadcaster()

            mAndm_logger.info("============ Start executing ... ==================")
            rospy.on_shutdown(self.shutdown)
            self.server.start()
        except:
            mAndm_logger.error("Unexpected error:")
            exc_type, exc_value, exc_traceback = sys.exc_info()
            traceback.print_tb(exc_traceback, limit=1, file=sys.stdout)
            traceback.print_exception(exc_type, exc_value, exc_traceback,
                              limit=2, file=sys.stdout)
            self.shutdown()

    def shutdown(self):
        # clean up moveit
        mAndm_logger.info("============ shutdown: Shutting down moveit! ==================")
        moveit_commander.roscpp_shutdown()

    #def moveit_shutdown(self):
    #    # clean up moveit
    #    moveit_commander.roscpp_shutdown()
    #    mAndm_logger.warning("============ Shutting down moveit! ==================")

    def execute(self, goal):
        try:
            mAndm_logger.info("======== Planning request received from client ==========")

            # re-initialize feedback and result
            self.feedback = PlanAndExecuteTrajFeedback()
            self.result = PlanAndExecuteTrajResult()

            mAndm_logger.info("========Parameters check: target tag or target pose =========")
            if not goal.tag_name and not (goal.target_pose.position.x or goal.target_pose.position.y or\
                                goal.target_pose.position.z or goal.target_pose.orientation.x or\
                                goal.target_pose.orientation.y or goal.target_pose.orientation.z or\
                                goal.target_pose.orientation.w):
                # return that we fail
                mAndm_logger.error("No Tag Name or Pose is provided. Quitting path planning.")
                self.result.error_type = 1
                self.server.set_aborted(result=self.result)
                return

            # first figure out the target arm
            mAndm_logger.info("========Parameters check: Finding target arm =========")
            if not goal.arm or goal.arm == self.arm:
                target_arm = self.arm
            elif goal.arm in self.possible_arm_groups:
                mAndm_logger.warning("This is a {arm} traj server. We may have a problem!".format(arm=self.arm))
                target_arm = goal.arm
            else:
                mAndm_logger.warning("Arm parameter does not match with expected values: left_arm, right_arm or both_arms. Using both_arms.")
                target_arm = "both_arms"
            mAndm_logger.info("-- Target arm is {target_arm}".format(target_arm=target_arm))

            mAndm_logger.info("========Parameters check: Identifying action type =========")
            if goal.action_type not in self.possilbe_action_types:
                mAndm_logger.warning("Arm parameter does not match with expected values: pickup or drop. Using pickup.")
                action_type = 'pickup'
            else:
                action_type = goal.action_type

            mAndm_logger.info("========Parameters check: Identifying planning mode =========")
            if goal.planning_mode not in self.possible_planning_modes:
                mAndm_logger.warning("Planning mode does not match with expected values: rough, fine or rough_and_fine. Using rough_and_fine.")
                planning_mode = 'rough_and_fine'
            else:
                planning_mode = goal.planning_mode

            ########################
            ###### rough plan ######
            ########################
            if planning_mode in ['rough','rough_and_fine']:
                mAndm_logger.info("============ Rough planning ======================")
                if goal.tag_name:
                    # find target pose
                    pose_target = self.find_target_pose(goal.tag_name,'rough', target_arm)
                else:
                    # use target pose
                    pose_target = goal.target_pose

                # cannot get target pose
                if not pose_target:
                    self.result.error_type = 2
                    self.server.set_aborted(result=self.result)
                    return

                # now choose an arm to plan
                mAndm_logger.info("======== Setting rough goal position:{pose_target} =========".format(pose_target=pose_target))
                self.set_target_pose(pose_target, target_arm)

                mAndm_logger.info("============ Rough planning to goal position ======================")
                plan = self.group[target_arm].plan(self.planning_timeout)
                mAndm_logger.debug(plan)
                if plan is not None:
                    mAndm_logger.info("-- Rough Plan found")
                    self.result.rough_planning_result = True
                    self.feedback.rough_plan_found = True
                    self.server.publish_feedback(self.feedback) # is this necesary?
                else:
                    mAndm_logger.error("No rough plan found.")
                    self.result.error_type = 4
                    self.server.set_aborted(result=self.result)
                    return

                scaled_plan = scale_trajectory_speed(plan, 0.3)
                mAndm_logger.debug("scaled_plan:" + str(scaled_plan))

                mAndm_logger.info("============ Executing rough plan ======================")
                self.feedback.executing_rough_plan = True
                self.server.publish_feedback(self.feedback) # is this necesary?
                self.group[target_arm].execute(scaled_plan)
                self.result.rough_execution_result = True


            ########################
            ###### fine plan ######
            ########################
            if planning_mode in ['fine','rough_and_fine']:
                mAndm_logger.info("============ Fine planning ======================")
                if goal.tag_name:
                    # find target pose
                    pose_target = self.find_target_pose(goal.tag_name,'fine', target_arm)
                else:
                    # use target pose
                    pose_target = goal.target_pose

                # cannot get target pose
                if not pose_target:
                    self.result.error_type = 3
                    self.server.set_aborted(result=self.result)
                    return

                mAndm_logger.info("======== Setting fine goal position:{pose_target} =========".format(pose_target=pose_target))
                self.set_target_pose(pose_target, target_arm)
                mAndm_logger.info("============ Fine planning to goal position ======================")
                plan = self.group[target_arm].plan(self.planning_timeout)
                mAndm_logger.debug(plan)
                if plan is not None:
                    mAndm_logger.info("-- Fine Plan found")
                    self.result.fine_planning_result = True
                    self.feedback.fine_plan_found = True
                    self.server.publish_feedback(self.feedback) # is this necesary?
                else:
                    mAndm_logger.error("No find plan found.")
                    self.result.error_type = 5
                    self.server.set_aborted(result=self.result)
                    return

                mAndm_logger.info("============ Executing fine plan ======================")
                scaled_plan = scale_trajectory_speed(plan, 0.3)
                self.feedback.executing_fine_plan = True
                self.server.publish_feedback(self.feedback) # is this necesary?
                self.group[target_arm].execute(scaled_plan)
                self.result.fine_execution_result = True

            mAndm_logger.info("============ Motion Planning Execution Done! ====================")

            # now send to scott
            self.feedback.executing_grasp = True
            self.server.publish_feedback(self.feedback) # is this necesary?

            self.result.grasp_result = True
            self.server.set_succeeded(result=self.result)

        except:
            mAndm_logger.error("Unexpected error:"+str(sys.exc_info()[0]))
            self.result.error_type = 99
            self.server.set_aborted(result=self.result)


    def set_target_pose(self, target_pose, arm_name):
        """
        This funciton takes in the target_pose and arm_name.
        target_pose (geometry_msgs.msg.Pose): pose object
        arm_name (string): name of arm
        """
        if arm_name in ["left_arm","right_arm"]:
            self.group[arm_name].set_pose_target(target_pose)
        else: # both arms
            # first figure out which arm is closer
            left_endpose_dict = self.limb["left"].endpoint_pose()
            right_endpose_dict = self.limb["right"].endpoint_pose()

            left_pose_distance = math.sqrt(math.pow(left_endpose_dict['position'].x - target_pose.position.x,2)+\
                                           math.pow(left_endpose_dict['position'].y - target_pose.position.y,2)+\
                                           math.pow(left_endpose_dict['position'].z - target_pose.position.z,2))
            right_pose_distance =math.sqrt(math.pow(right_endpose_dict['position'].x - target_pose.position.x,2)+\
                                           math.pow(right_endpose_dict['position'].y - target_pose.position.y,2)+\
                                           math.pow(right_endpose_dict['position'].z - target_pose.position.z,2))
            # now set the pose
            if left_pose_distance > right_pose_distance:
                mAndm_logger.debug('-- Planning for right arm')
                self.group[arm_name].set_pose_target(target_pose, 'right_gripper')
            else:
                mAndm_logger.debug('-- Planning for left arm')
                self.group[arm_name].set_pose_target(target_pose, 'left_gripper')


    def find_target_pose(self, tagName, mode, arm_name):
        """
        This function takes in a tagName. Find its location with transform,
        then return a Pose object.
        tagName (string): name of tag
        mode(string): rough or fine
        """
        mAndm_logger.info("========= Finding goal position from tagName {tagName}===============".format(tagName=tagName))

        # find tag in world frame (x,y,z,w)
        world_tag_pose = None
        world_tag_rot  = None

        # choose a mode
        if mode == 'rough':
            reference_frame = 'torso'  # kinect -> world
            z_offset = 0.2
            tagNameList = [tagName]
        elif mode == 'fine':
            reference_frame = 'torso'  # left_hand_camera -> torso
            if arm_name in ["left_arm", "right_arm"]:
                tagNameList = [arm_name.replace("arm","hand")+"_"+tagName]
            else: # both arms
                tagNameList = [x.replace("arm","hand")+"_"+tagName for x in ["left_arm", "right_arm"]]
            z_offset = -0.05

        startTime = time.time()
        while not (world_tag_pose and world_tag_rot) and time.time()-startTime < 10.0:
            #mAndm_logger.debug("list of frames:" + str(self.tf_listener.getFrameStrings()))
            for tagName in tagNameList:
                #mAndm_logger.debug("tagName:" + str(tagName))
                try:
                    mAndm_logger.debug( "Does " + tagName + " exist?" + str(self.tf_listener.frameExists(tagName)))
                    (world_tag_pose,world_tag_rot) = self.tf_listener.lookupTransform(reference_frame, tagName, rospy.Time(0))
                except:
                    rospy.sleep(1)
                    mAndm_logger.debug("Stilling waiting for tag pose and orientation in the world frame")

        mAndm_logger.log(4, "world_original_tag_pose: " + str(world_tag_pose))
        mAndm_logger.log(4, "world_original_tag_rot: " + str(world_tag_rot))

        if world_tag_pose and world_tag_rot:
            # transform frame
            # for API visit http://www.lfd.uci.edu/~gohlke/code/transformations.py.html
            z_axis_offset_tf = tf.transformations.translation_matrix([0,0,z_offset]) #x,y,z offset
            about_x_rotation_qua = tf.transformations.quaternion_about_axis(math.pi,[1,0,0]) # about x

            world_to_tag_pose_tf = tf.transformations.translation_matrix(list(world_tag_pose)) # convert pose to translation matrix

            world_transformed_tag_pose = world_to_tag_pose_tf.dot(z_axis_offset_tf.dot([0,0,0,1])) # offset pose with z_offset
            world_transformed_tag_rot  = tf.transformations.quaternion_multiply(list(world_tag_rot), about_x_rotation_qua) # transform quaternion

            mAndm_logger.log(4, "world_transformed_tag_pose: " + str(world_transformed_tag_pose))
            mAndm_logger.log(4, "world_transformed_tag_rot: " + str(world_transformed_tag_rot))

            mAndm_logger.debug("if you want to see tranformed tag in Rviz, run test_tf_broadcaster.py with tagName replaced in the file")

            # set goal pose
            pose_target = geometry_msgs.msg.Pose()
            pose_target.orientation = geometry_msgs.msg.Quaternion(*world_transformed_tag_rot)
            pose_target.position = geometry_msgs.msg.Point(*world_transformed_tag_pose[0:3])
            return pose_target
        else:
            mAndm_logger.error("Cannot get tag: {tagName}".format(tagName=tagName))
            return None

    def add_modules(self):
        """
        This function updates the workspace of baxter based on modules detected
        """
        for frame in self.tf_listener.getFrameStrings():
            # this is a tag
            if 'tag' in frame:
                # clean the scene
                #scene.remove_world_object("frame")

                tag_pose = None
                tag_rot  = None
                while not (tag_pose and tag_rot):
                    try:
                        mAndm_logger.debug("list of frames:" + str(listener.getFrameStrings()))
                        (tag_pose,tag_rot) = self.tf_listener.lookupTransform('world', frame, rospy.Time(0))
                    except:
                        rospy.sleep(1)
                        mAndm_logger.debug("Stilling waiting for tag pose and orientation in the world frame")

                z_axis_offset_tf = tf.transformations.translation_matrix([0,0,-0.375]) #x,y,z offset
                world_to_tag_pose_tf = tf.transformations.translation_matrix(list(tag_pose))
                transformed_tag_pose = world_to_tag_pose_tf.dot(z_axis_offset_tf.dot([0,0,0,1]))

                # publish a demo scene
                p = PoseStamped()
                p.header.frame_id = robot.get_planning_frame()
                p.pose.position.x = transformed_tag_pose[0]
                p.pose.position.y = transformed_tag_pose[1]
                p.pose.position.z = transformed_tag_pose[2]
                p.pose.orientation.w = tag_rot[3]
                self.scene.add_box(frame, p, (0.65, 0.65, 0.65))


def scale_trajectory_speed(traj, scale):
    # Create a new trajectory object
    new_traj = moveit_msgs.msg.RobotTrajectory()

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
        point = trajectory_msgs.msg.JointTrajectoryPoint()
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


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Trajectory Planner and Executor.")
    parser.add_argument('arm', type=str, help='Specify arm. Possible paramenters: left_arm, right_arm, both_arms or any')

    args = parser.parse_args()

    if args.arm in ['left_arm','right_arm','both_arms']:
        rospy.init_node('plan_and_execute_traj_server_'+args.arm)
        server = PlanAndExecuteTrajServer(args.arm)
        rospy.spin()
    else:
        mAndm_logger.error("Argument Incorrect!:" + args.arm)
        mAndm_logger.info("============ Shutting down moveit! ==================")
        moveit_commander.roscpp_shutdown()
