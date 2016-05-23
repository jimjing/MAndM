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
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene


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

# camera poses
#limb['left'].endpoint_pose()
#{'position': Point(x=0.7544123864868437, y=0.12425564429666114, z=0.19748198671622685), 'orientation': Quaternion(x=0.7458686725005649, y=0.6486596280713739, z=-0.0903643866478993, w=0.12146969958924195)}
#limb['right'].endpoint_pose()
#{'position': Point(x=0.6953452819130829, y=-0.24940577526211638, z=0.22988738078528453), 'orientation': Quaternion(x=-0.6544312721463218, y=0.7199562711595843, z=0.17773956466961444, w=0.14761884976516265)}

##########################################################
class PlanAndExecuteTrajServer:
    def __init__(self, arm):
        try:
            self.server = actionlib.SimpleActionServer('plan_and_execute_traj_'+arm, PlanAndExecuteTrajAction, self.execute, False)
            self.feedback = None
            self.result = None
            self.using_octamap = False


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
            self.gripper = {}
            self.gripper['left'] = baxter_interface.Gripper('left')
            self.gripper['right'] = baxter_interface.Gripper('right')
            self.gripper['left'].calibrate()
            self.gripper['right'].calibrate()

            # setting valid parameters for incoming requests
            if arm == 'any':
                self.possible_arm_groups = ["left_arm", "right_arm", "both_arms"]
            else:
                self.possible_arm_groups = [arm]
            self.possilbe_action_types = ['move_to','open_gripper','close_gripper','move_to_camera_pos','get_tag_pose']
            self.possible_planning_modes = ['rough','fine','rough_and_fine']
            self.planning_timeout = 30.0
            self.allow_replanning = True

            mAndm_logger.info("===== Setting up MoveIt! ==========================")
            moveit_commander.roscpp_initialize(sys.argv) #sys.argv
            self.robot = moveit_commander.RobotCommander()
            self.scene = moveit_commander.PlanningSceneInterface()
            self.group = {}
            for group_name in self.possible_arm_groups:
                mAndm_logger.info("--setting up MoveGroup: {group_name}--".format(group_name=group_name))
                self.group[group_name] = moveit_commander.MoveGroupCommander(group_name)
                self.group[group_name].set_goal_tolerance(0.01)
                self.group[group_name].set_planner_id("RRTConnectkConfigDefault")
                self.group[group_name].allow_replanning(self.allow_replanning)
                self.group[group_name].set_planning_time(self.planning_timeout)

            mAndm_logger.info("===== Setting up tf ==========================")
            self.tf_listener = tf.TransformListener()
            self.tf_broadcaster = tf.TransformBroadcaster()

            if self.using_octamap:
                mAndm_logger.info("===== Setting up service for excluding objects ====")
                self._pubPlanningScene = rospy.Publisher('planning_scene', PlanningScene)
                rospy.wait_for_service('/get_planning_scene', 10.0)
                self.get_planning_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)

            mAndm_logger.info("===== Setting up move to camera poses =========")
            self.move_to_camera_poses = {}
            #left
            #'position': Point(x=0.75, y=0.12, z=0.20),
            #'orientation': Quaternion(x=0.75, y=0.65, z=-0.09, w=0.12)
            self.move_to_camera_poses['left'] = geometry_msgs.msg.Pose()
            self.move_to_camera_poses['left'].orientation = geometry_msgs.msg.Quaternion(*[0.7458686725005649,0.6486596280713739,-0.0903643866478993,0.12146969958924195])
            self.move_to_camera_poses['left'].position = geometry_msgs.msg.Point(*[0.7544123864868437,0.12425564429666114,0.19748198671622685])
            # right
            #'position': Point(x=0.70, y=-0.25, z=0.23),
            #'orientation': Quaternion(x=-0.65, y=0.72, z=0.18, w=0.15)
            self.move_to_camera_poses['right'] = geometry_msgs.msg.Pose()
            self.move_to_camera_poses['right'].orientation = geometry_msgs.msg.Quaternion(*[-0.6544312721463218,0.7199562711595843,0.17773956466961444,0.14761884976516265])
            self.move_to_camera_poses['right'].position = geometry_msgs.msg.Point(*[0.6953452819130829,-0.24940577526211638,0.22988738078528453])

            if not self.using_octamap:
                time.sleep(1)
                mAndm_logger.info("===== Adding table to scene ==========")
                self.add_table()
                self.add_fake_module()
                time.sleep(1)

            mAndm_logger.info("===== Start executing ... ==================")
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
        mAndm_logger.info("===== shutdown: Shutting down moveit! ==================")
        moveit_commander.roscpp_shutdown()

    def execute(self, goal):
        try:

            mAndm_logger.info("===== Planning request received from client ==========")
            mAndm_logger.info(goal)

            # re-initialize feedback and result
            self.feedback = PlanAndExecuteTrajFeedback()
            self.result = PlanAndExecuteTrajResult()

            mAndm_logger.info("===== Parameters check: Identifying action type =========")
            if goal.action_type not in self.possilbe_action_types:
                mAndm_logger.error("Action parameter: {action_type} does not match with expected values."\
                                    .format(action_type=goal.action_type))
                self.result.error_type = 6
                self.server.set_aborted(result=self.result)
                return
            else:
                action_type = goal.action_type
            mAndm_logger.info("- Action is: {action_type}".format(action_type=goal.action_type))


            mAndm_logger.info("===== Parameters check: target tag or target pose =========")
            if action_type == 'move_to' and not goal.tag_name and not (goal.target_pose.position.x or goal.target_pose.position.y or\
                                goal.target_pose.position.z or goal.target_pose.orientation.x or\
                                goal.target_pose.orientation.y or goal.target_pose.orientation.z or\
                                goal.target_pose.orientation.w):
                # return that we fail
                mAndm_logger.error("No Tag Name or Pose is provided for action move_to. Quitting path planning.")
                self.result.error_type = 1
                self.server.set_aborted(result=self.result)
                return
            elif action_type == 'get_tag_pose' and not goal.tag_name:
              # return that we fail
                mAndm_logger.error("No Tag Name is provided for action get_tag_pose. Quitting..")
                self.result.error_type = 10
                self.server.set_aborted(result=self.result)
                return
            mAndm_logger.info("- Tag Name is: {tag_name}".format(tag_name=goal.tag_name))
            mAndm_logger.info("- Tag Name is: {target_pose}".format(target_pose=goal.target_pose))

            # first figure out the target arm
            mAndm_logger.info("===== Parameters check: Finding target arm =========")
            if not goal.arm or goal.arm == self.arm:
                target_arm = self.arm
            elif goal.arm in self.possible_arm_groups:
                mAndm_logger.warning("This is a {arm} traj server. We may have a problem!".format(arm=self.arm))
                target_arm = goal.arm
            else:
                mAndm_logger.warning("Arm parameter:{arm} does not match with expected values:left_arm, right_arm or both_arms. Using both_arms.".format(arm=goal.arm))
                #target_arm = "both_arms"
                self.result.error_type = 7
                self.server.set_aborted(result=self.result)
                return
            mAndm_logger.info("-- Target arm is {target_arm}".format(target_arm=target_arm))
            self.feedback.arm = target_arm

            mAndm_logger.info("===== Parameters check: Identifying planning mode =========")
            if goal.planning_mode not in self.possible_planning_modes:
                mAndm_logger.warning("Planning mode: {planning_mode} does not match with expected values: rough, fine or rough_and_fine. Using rough_and_fine.".format(planning_mode=goal.planning_mode))
                planning_mode = 'rough_and_fine'
            else:
                planning_mode = goal.planning_mode
            mAndm_logger.info("Planning mode is: {planning_mode}".format(planning_mode=planning_mode))

            #############################
            ########## MODES ###########
            ############################

            # ------------ close_gripper
            # position: 72.65 for module, start with 100, closed 3.2
            if action_type == 'close_gripper':
                self.feedback.executing_grasp = True
                mAndm_logger.info("===== Closing gripper ======================")
                # operations on gripper if pickup or drop
                # open gripper before fine plan
                if target_arm in ['left_arm','right_arm']:
                    mAndm_logger.debug(target_arm.replace('_arm',''))
                    self.gripper[target_arm.replace('_arm','')].close()
                    time.sleep(2) # so that internal data is updated
                    # feeback
                    if self.gripper[target_arm.replace('_arm','')].gripping():
                        gripping_successful = True
                    else:
                        gripping_successful = False

                else: # both arms
                    self.gripper['left'].close()
                    self.gripper['right'].close()
                    time.sleep(2) # so that internal data is updated
                    # feeback
                    if self.gripper['left'].gripping() and self.gripper['right'].gripping():
                        gripping_successful = True
                    else:
                        gripping_successful = False

                #mAndm_logger.debug("Position when gripper is closed: {pos}".format(pos=self.gripper['right'].position()))
                #mAndm_logger.debug("Force when gripper is closed: {pos}".format(pos=self.gripper['right'].force()))
                #mAndm_logger.debug("Gripping: {pos}".format(pos=self.gripper['right'].gripping()))
                #mAndm_logger.debug("Missed: {pos}".format(pos=self.gripper['right'].missed()))
                # send result
                self.result.grasp_result = gripping_successful
                mAndm_logger.debug("--Grasping module : {status}".format(status=gripping_successful))
                if gripping_successful:
                    self.server.set_succeeded(result=self.result)
                    mAndm_logger.info("===== Closing gripper: successful ======================")
                else:
                    self.server.set_aborted(result=self.result)
                    mAndm_logger.error("===== Closing gripper: failed - no object in hand =======")


            # ------------ open_gripper
            elif action_type == 'open_gripper':
                self.feedback.executing_grasp = True
                mAndm_logger.info("===== Opening gripper ======================")
                # operations on gripper if pickup or drop
                # open gripper before fine plan
                if target_arm in ['left_arm','right_arm']:
                    mAndm_logger.debug(target_arm.replace('_arm',''))
                    self.gripper[target_arm.replace('_arm','')].open()
                else: # both arms
                    self.gripper['left'].open()
                    self.gripper['right'].open()

                #self.limb['left'].set_joint_velocities({'left_w2':0.3})
                #time.sleep(7)
                #self.limb['left'].set_joint_velocities({'left_w2':0.0})

                # feedback on whether grasp is good?
                self.result.grasp_result = True
                self.server.set_succeeded(result=self.result)
                mAndm_logger.info("===== Opening gripper: successful ======================")

            # ------------ move_to
            elif action_type == 'move_to':
                mAndm_logger.info("===== Mode move_to ======================")
                ########################
                ###### rough plan ######
                ########################
                mAndm_logger.debug("planning_mode:" + str(planning_mode))
                if planning_mode in ['rough','rough_and_fine']:
                    #rough plan
                    if not self.planning_traj(goal.tag_name, goal.target_pose, target_arm, 'rough'):
                        return

                ########################
                ###### add module ######
                ########################
                if self.using_octamap:
                    mAndm_logger.info("===== Adding module ======================")
                    # add module:
                    if goal.tag_name:
                        self.add_module(goal.tag_name)

                    mAndm_logger.info("===== Excluding tag object ======================")
                    request = PlanningSceneComponents(components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
                    response = self.get_planning_scene(request)
                    acm = response.scene.allowed_collision_matrix
                    # add gripper fingers to allowed collision matrix
                    acm.default_entry_names += [goal.tag_name]
                    acm.default_entry_values += [True]
                    planning_scene_diff = PlanningScene(is_diff=True, allowed_collision_matrix=acm)
                    #mAndm_logger.log(4, "acm: {acm}".format(acm=acm))
                    self._pubPlanningScene.publish(planning_scene_diff)

                ########################
                ###### fine plan ######
                ########################
                if planning_mode in ['fine','rough_and_fine']:
                    #fine plan
                    if not self.planning_traj(goal.tag_name, goal.target_pose, target_arm, 'fine'):
                        return

                ###########################
                ###### remove module ######
                ###########################
                if self.using_octamap:
                    mAndm_logger.info("===== Removing tag object ======================")
                    # remove gripper fingers to allowed collision matrix
                    acm.default_entry_names = []
                    acm.default_entry_values = []
                    planning_scene_diff = PlanningScene(is_diff=True, allowed_collision_matrix=acm)
                    #mAndm_logger.log(4, "acm: {acm}".format(acm=acm))
                    self._pubPlanningScene.publish(planning_scene_diff)
                    self.scene.remove_world_object(goal.tag_name)

                self.server.set_succeeded(result=self.result)

            # ------------ move_to_camera_pos
            elif action_type == 'move_to_camera_pos':
                mAndm_logger.info("===== Mode move_to_camera_pos ==============")
                #fine plan
                if target_arm not in ['left_arm','right_arm']:
                    mAndm_logger.error('move to camera pose only works for left_arm or right_arm.Given {target_arm}'.format(target_arm=target_arm))
                    self.result.error_type = 8
                    self.server.set_aborted(result=self.result)
                    return

                target_pose = self.move_to_camera_poses[target_arm.replace('_arm','')]
                #mAndm_logger.debug(target_pose)
                if not self.planning_traj("", target_pose, target_arm, 'rough'):
                    return
                self.server.set_succeeded(result=self.result)


            # ------------ get_tag_pose
            elif action_type == 'get_tag_pose':
                mAndm_logger.info("===== Mode get_tag_pose ======================")
                if self.find_target_pose(goal.tag_name, 'rough', target_arm):
                    self.server.set_succeeded(result=self.result)
                else:
                    self.server.set_aborted(result=self.result)


            mAndm_logger.info("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
            mAndm_logger.info("++++++++++++ Motion Planning Execution Done! +++++++++++++++++")
            mAndm_logger.info("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n")

        except:
            mAndm_logger.error("Unexpected error:")
            exc_type, exc_value, exc_traceback = sys.exc_info()
            traceback.print_tb(exc_traceback, limit=1, file=sys.stdout)
            traceback.print_exception(exc_type, exc_value, exc_traceback,
                              limit=2, file=sys.stdout)
            self.result.error_type = 99
            self.server.set_aborted(result=self.result)

    def planning_traj(self, tag_name, target_pose, target_arm, mode):
        """
        reuse planning
        mode: rough or fine
        goal: goal object
        target_arm: left_arm, right_arm or both_arms
        if return false, then abort action
        """
        mAndm_logger.info("===== {mode} planning ======================".format(mode=mode))

        if tag_name:
            # find target pose
            pose_target = self.find_target_pose(tag_name, mode, target_arm)
        else:
            # use target pose
            pose_target = target_pose

        # cannot get target pose
        if not pose_target:
            if mode == 'rough':
                self.result.error_type = 2
            else:
                self.result.error_type = 3
            self.server.set_aborted(result=self.result)
            return False

        # now choose an arm to plan
        mAndm_logger.info("===== Setting {mode} goal position:{pose_target} =========".format(mode=mode, pose_target=pose_target))
        self.set_target_pose(pose_target, target_arm)

        mAndm_logger.info("===== {mode} planning to goal position ======================".format(mode=mode))
        plan = self.group[target_arm].plan()
        if plan and plan.joint_trajectory.points:
            mAndm_logger.info("-- {mode} Plan found".format(mode=mode))
            #mAndm_logger.debug(plan)
            if mode == 'rough':
                self.result.rough_planning_result = True
                self.feedback.rough_plan_found = True
            else:
                self.result.fine_planning_result = True
                self.feedback.fine_plan_found = True
            self.server.publish_feedback(self.feedback) # is this necesary?
        else:
            mAndm_logger.error("No {mode} plan found.".format(mode=mode))
            if mode == 'rough':
                self.result.error_type = 4
            else:
                self.result.error_type = 5
            self.server.set_aborted(result=self.result)
            return False

        scaled_plan = scale_trajectory_speed(plan, 0.3)
        #mAndm_logger.debug("scaled_plan:" + str(scaled_plan))

        mAndm_logger.info("===== Executing {mode} plan ======================".format(mode=mode))
        if mode == 'rough':
            self.feedback.executing_rough_plan = True
        else:
            self.feedback.executing_fine_plan = True
        self.server.publish_feedback(self.feedback)

        # now execute
        self.group[target_arm].execute(scaled_plan)
        if mode == 'rough':
            self.result.rough_execution_result = True
        else:
            self.result.fine_execution_result = True

        return True

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
        mAndm_logger.info("===== Finding goal position from tagName {tagName}===============".format(tagName=tagName))

        # find tag in world frame (x,y,z,w)
        world_tag_pose = None
        world_tag_rot  = None

        # choose a mode
        if mode == 'rough':
            reference_frame = 'torso'  # kinect -> world
            z_offset = 0.15
            tagNameList = [tagName]
        elif mode == 'fine':
            reference_frame = 'torso'  # left_hand_camera -> torso
            if arm_name in ["left_arm", "right_arm"]:
                tagNameList = [arm_name.replace("arm","hand")+"_"+tagName]
            else: # both arms
                tagNameList = [x.replace("arm","hand")+"_"+tagName for x in ["left_arm", "right_arm"]]
            z_offset = -0.08

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

        # second time searching for all poses
        if not (world_tag_pose and world_tag_rot):
            mAndm_logger.warning('No tag pose from {tagNameList}. Using full list'.format(tagNameList=tagNameList))
            tagNameList = ['left_hand_'+tagName, 'right_hand_'+tagName, tagName]
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
            # set to result
            self.result.tag_pose = geometry_msgs.msg.Pose()
            self.result.tag_pose.orientation = geometry_msgs.msg.Quaternion(*world_tag_rot)
            self.result.tag_pose.position = geometry_msgs.msg.Point(*world_tag_pose)

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

    def add_module(self, givenTagName):
        """
        This function updates the workspace of baxter based on modules detected
        """
        tagNameList = ['left_hand_'+givenTagName, 'right_hand_'+givenTagName]
        tag_pose = None
        tag_rot  = None
        startTime = time.time()
        while not (tag_pose and tag_rot) and time.time()-startTime < 10.0:
            for tagName in tagNameList:
                try:
                    mAndm_logger.debug("tagName:" + str(tagName))
                    #mAndm_logger.debug("list of frames:" + str(listener.getFrameStrings()))
                    (tag_pose,tag_rot) = self.tf_listener.lookupTransform('torso', tagName, rospy.Time(0))
                except:
                    rospy.sleep(1)
                    mAndm_logger.debug("Stilling waiting for tag pose and orientation in the torso frame")

        if tag_pose and tag_rot:
            mAndm_logger.debug("tag_pose:" + str(tag_pose))
            cube_size = 0.15
            z_axis_offset_tf = tf.transformations.translation_matrix([0,0,-0.0375]) #x,y,z offset
            world_to_tag_pose_tf = tf.transformations.translation_matrix(list(tag_pose))
            transformed_tag_pose = world_to_tag_pose_tf.dot(z_axis_offset_tf.dot([0,0,0,1]))

            # publish a demo scene
            p = PoseStamped()
            p.header.frame_id = self.robot.get_planning_frame()
            p.pose.position.x = transformed_tag_pose[0]
            p.pose.position.y = transformed_tag_pose[1]
            p.pose.position.z = transformed_tag_pose[2]
            p.pose.orientation.w = tag_rot[3]
            self.scene.add_box(givenTagName, p, (cube_size, cube_size, cube_size))
            #self.scene.add_box(givenTagName, p, (cube_size, cube_size, 10*cube_size))

    def add_table(self):
        #also rotate about y 15 degrees?
        #about_y_rotation_qua = tf.transformations.quaternion_about_axis(-math.pi/10,[0,1,0]) # about x
        table_height = 1.0 # height of table
        p = PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()
        p.pose.position.x = 0.65
        p.pose.position.y = -0.14
        p.pose.position.z = -0.29-table_height/2
        #p.pose.orientation = geometry_msgs.msg.Quaternion(*about_y_rotation_qua)
        #mAndm_logger.debug("table: {p}".format(p=p))
        self.scene.add_box('table', p, (0.70, 2.0, table_height))

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
        mAndm_logger.info("===== Shutting down moveit! ==================")
        moveit_commander.roscpp_shutdown()
