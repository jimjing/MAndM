#!/usr/bin/env python

import os, sys
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
#logging.basicConfig(level=logging.DEBUG)
#mAndm_logger = logging.getLogger(__name__)

import rospy
import time

from dynamic_reconfigure.server import Server

from baxter_interface.cfg import (
    PositionJointTrajectoryActionServerConfig,
    VelocityJointTrajectoryActionServerConfig,
    PositionFFJointTrajectoryActionServerConfig,
)
from joint_trajectory_action.joint_trajectory_action import (
    JointTrajectoryActionServer,
)

from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

import operator
import sys
import threading
import subprocess

from bisect import bisect
from copy import copy
from os import path

import actionlib

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
import baxter_interface
from baxter_interface import CHECK_VERSION


class _TrajectoryServer(object):
    def __init__(self, limb, rate, mode):
        """
        Initalization of the two arms and grippers

        limb (string): joint trajectory action limb. You can enter both,left or right (default="both")
        rate (float): trajectory control rate in Hz. (default=100.0)
        mode (string): Control mode for trajectory execution. You can enter position_w_id,position or velocity (default="position_w_id")
        """

        self.dyn_cfg_srv = None # for traj action server
        self.jtas = [] # for track which limbs are used

        # initialize dictionaries
        rospy.on_shutdown(self._cleanup)

        mAndm_logger.info("Initializing joint trajectory action server...")

        if mode == 'velocity':
            self.dyn_cfg_srv = Server(VelocityJointTrajectoryActionServerConfig,
                                 lambda config, level: config)
        elif mode == 'position':
            self.dyn_cfg_srv = Server(PositionJointTrajectoryActionServerConfig,
                                 lambda config, level: config)
        else:
            self.dyn_cfg_srv = Server(PositionFFJointTrajectoryActionServerConfig,
                                 lambda config, level: config)

        if limb == 'both':
            self.jtas.append(JointTrajectoryActionServer('right', self.dyn_cfg_srv, rate, mode))
            self.jtas.append(JointTrajectoryActionServer('left', self.dyn_cfg_srv, rate, mode))
        else:
            self.jtas.append(JointTrajectoryActionServer(limb, self.dyn_cfg_srv, rate, mode))

    def _cleanup(self):
        mAndm_logger.info("Running. Ctrl-c to quit")
        #rospy.spin()
        #mAndm_logger.info("Cleaning jtas...")
        #for j in self.jtas:
        #    j.clean_shutdown()

class _TrajectoryExecutor(object):
    """
    This class comes from _joint_trajectory_file_playback
    """
    def __init__(self):
        self.setupConnection()

        self.now_from_start = 0.0
        mAndm_logger.log(1,'initalizing')
        self.pause = False
        self.aggregate_time = 0.0

        # Timing offset to prevent gripper playback before trajectory has started
        self._slow_move_offset = 0.0
        self._trajectory_start_offset = rospy.Duration(0.0)
        self._trajectory_actual_offset = rospy.Duration(0.0)

    def setupConnection(self):
       #create our action server clients
        self._left_client = actionlib.SimpleActionClient(
            'robot/limb/left/follow_joint_trajectory',
            FollowJointTrajectoryAction,
        )
        self._right_client = actionlib.SimpleActionClient(
            'robot/limb/right/follow_joint_trajectory',
            FollowJointTrajectoryAction,
        )

        #verify joint trajectory action servers are available
        l_server_up = self._left_client.wait_for_server(rospy.Duration(10.0))
        r_server_up = self._right_client.wait_for_server(rospy.Duration(10.0))
        if not l_server_up or not r_server_up:
            msg = ("Action server not available."
                   " Verify action server availability.")
            rospy.logerr(msg)
            rospy.signal_shutdown(msg)
            sys.exit(1)
        #create our goal request
        self._l_goal = FollowJointTrajectoryGoal()
        self._r_goal = FollowJointTrajectoryGoal()

        #limb interface - current angles needed for start move
        self._l_arm = baxter_interface.Limb('left')
        self._r_arm = baxter_interface.Limb('right')
        self._l_goal.trajectory.joint_names.extend(self._l_arm.joint_names())
        self._r_goal.trajectory.joint_names.extend(self._r_arm.joint_names())

        #gripper interface - for gripper command playback
        self._l_gripper = baxter_interface.Gripper('left', CHECK_VERSION)
        self._r_gripper = baxter_interface.Gripper('right', CHECK_VERSION)

        #flag to signify the arm trajectories have begun executing
        self._arm_trajectory_started = False
        #reentrant lock to prevent same-thread lockout
        self._lock = threading.RLock()

        # Verify Grippers Have No Errors and are Calibrated
        if self._l_gripper.error():
            self._l_gripper.reset()
        if self._r_gripper.error():
            self._r_gripper.reset()
        if (not self._l_gripper.calibrated() and
            self._l_gripper.type() != 'custom'):
            self._l_gripper.calibrate()
        if (not self._r_gripper.calibrated() and
            self._r_gripper.type() != 'custom'):
            self._r_gripper.calibrate()

        #gripper goal trajectories
        self._l_grip = FollowJointTrajectoryGoal()
        self._r_grip = FollowJointTrajectoryGoal()

        #param namespace
        self._param_ns = '/rsdk_joint_trajectory_action_server/'

        #gripper control rate
        self._gripper_rate = 20.0  # Hz

    def _pause(self):
        """
        pause current action
        """
        # here we stop the action
        self.stop()
        self.pause = True


    def _execute_gripper_commands(self):
        start_time = rospy.get_time() - self._trajectory_actual_offset.to_sec()
        r_cmd = self._r_grip.trajectory.points
        l_cmd = self._l_grip.trajectory.points
        pnt_times = [pnt.time_from_start.to_sec() for pnt in r_cmd]
        end_time = pnt_times[-1]
        rate = rospy.Rate(self._gripper_rate)
        self.now_from_start = rospy.get_time() - start_time
        while(self.now_from_start < end_time + (1.0 / self._gripper_rate) and
              not rospy.is_shutdown() and not self.pause):
            idx = bisect(pnt_times, self.now_from_start) - 1
            if self._r_gripper.type() != 'custom':
                self._r_gripper.command_position(r_cmd[idx].positions[0])
            if self._l_gripper.type() != 'custom':
                self._l_gripper.command_position(l_cmd[idx].positions[0])
            rate.sleep()
            self.now_from_start = rospy.get_time() - start_time
            #mAndm_logger.log(2,'self.now_from_start:' + str(self.now_from_start))

    def _add_point(self, positions, side, time):
        """
        Appends trajectory with new point

        @param positions: joint positions
        @param side: limb to command point
        @param time: time from start for point in seconds
        """
        #creates a point in trajectory with time_from_start and positions
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.time_from_start = rospy.Duration(time)
        if side == 'left':
            self._l_goal.trajectory.points.append(point)
        elif side == 'right':
            self._r_goal.trajectory.points.append(point)
        elif side == 'left_gripper':
            self._l_grip.trajectory.points.append(point)
        elif side == 'right_gripper':
            self._r_grip.trajectory.points.append(point)


    def _feedback(self, data):
        # Test to see if the actual playback time has exceeded
        # the move-to-start-pose timing offset
        #mAndm_logger.log(4,'(>0)data.actual.time_from_start- self._trajectory_start_offset:' + str(data.actual.time_from_start-self._trajectory_start_offset))

        if (not self._get_trajectory_flag() and
              data.actual.time_from_start >= self._trajectory_start_offset):
            self._set_trajectory_flag(value=True)
            self._trajectory_actual_offset = data.actual.time_from_start

    def _set_trajectory_flag(self, value=False):
        with self._lock:
            # Assign a value to the flag
            self._arm_trajectory_started = value

    def _get_trajectory_flag(self):
        temp_flag = False
        with self._lock:
            # Copy to external variable
            temp_flag = self._arm_trajectory_started
        #mAndm_logger.log(2,"temp_flag:" + str(temp_flag))
        return temp_flag

    def start(self):
        """
        Sends FollowJointTrajectoryAction request
        """
        self._left_client.send_goal(self._l_goal, feedback_cb=self._feedback)
        self._right_client.send_goal(self._r_goal, feedback_cb=self._feedback)
        # Syncronize playback by waiting for the trajectories to start
        while not rospy.is_shutdown() and not self._get_trajectory_flag() and not self.pause:
            rospy.sleep(0.05)
            mAndm_logger.log(2,'in start')
        #self._execute_gripper_commands()

    def stop(self):
        """
        Preempts trajectory execution by sending cancel goals
        """
        if (self._left_client.gh is not None and
            self._left_client.get_state() == actionlib.GoalStatus.ACTIVE):
            self._left_client.cancel_goal()

        if (self._right_client.gh is not None and
            self._right_client.get_state() == actionlib.GoalStatus.ACTIVE):
            self._right_client.cancel_goal()

        self.pause = True # stop start while loop
        #delay to allow for terminating handshake
        rospy.sleep(0.1)

    def wait(self):
        """
        Waits for and verifies trajectory execution result
        """
        #create a timeout for our trajectory execution
        #total time trajectory expected for trajectory execution plus a buffer
        last_time = self._r_goal.trajectory.points[-1].time_from_start.to_sec()
        time_buffer = rospy.get_param(self._param_ns + 'goal_time', 0.0) + 1.5
        timeout = rospy.Duration(self._slow_move_offset +
                                 last_time +
                                 time_buffer)

        l_finish = self._left_client.wait_for_result(timeout)
        r_finish = self._right_client.wait_for_result(timeout)
        l_result = (self._left_client.get_result().error_code == 0)
        r_result = (self._right_client.get_result().error_code == 0)

        mAndm_logger.debug('l_finish:' + str(l_finish) + 'r_finish:' +str(r_finish) + 'l_result:' + str(l_result) + 'r_result:' + str(r_result))

        #verify result
        if all([l_finish, r_finish, l_result, r_result]):
            return True
        else:
            msg = ("Trajectory action failed or did not finish before "
                   "timeout/interrupt.")
            rospy.logwarn(msg)
            return False

class WayPointExecution(object):
    def __init__(self, limb):
        self._accuracy = baxter_interface.settings.JOINT_ANGLE_TOLERANCE
        mAndm_logger.log(2,self._accuracy) # about 0.5degrees / 0.008 radians
        self._speed = 0.3
        self._waypoints = [] # list of dict
        self._limb = baxter_interface.Limb(limb)

    def addWaypoint(self, jointAnglesList):
        mAndm_logger.log(4, dict(zip(self._limb.joint_names(), jointAnglesList)))
        self._waypoints.append(dict(zip(self._limb.joint_names(), jointAnglesList)))

    def playWaypoints(self, timeout=20.0):
        """
        Execute waypoints
        timeout: time before execution of the current waypoint ends.
        """
        rospy.sleep(1.0)

        rospy.loginfo("Waypoint Playback Started")
        print("  Press Ctrl-C to stop...")

        # Set joint position speed ratio for execution
        self._limb.set_joint_position_speed(self._speed)

        mAndm_logger.debug("Waypoint playback started")
        for waypoint in self._waypoints:
            mAndm_logger.debug("Playing waypoint" + str(waypoint.values()))

            if rospy.is_shutdown():
                break
            self._limb.move_to_joint_positions(waypoint, timeout,
                                               threshold=self._accuracy)

            # calculate timeout based on different in joint pos

            # check if joint angle matches
            current_joint_angles = [self._limb.joint_angle(jnt) for jnt in self._limb.joint_names()]
            mAndm_logger.debug("Destination joint angles" + str(current_joint_angles))
            mAndm_logger.debug("Destination joint angles" + str(self._limb.joint_angles()))
            mAndm_logger.debug("waypoint angles" + str(waypoint))


        # Set joint position speed back to default
        self._limb.set_joint_position_speed(0.3)

        mAndm_logger.debug('We are done!')


