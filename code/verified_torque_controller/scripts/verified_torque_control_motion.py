#! /usr/bin/env python


import rospy

import actionlib

import matlab.engine

from dynamic_reconfigure.server import (
    Server,
)
from std_msgs.msg import (
    Empty,
)

import baxter_interface

from baxter_examples.cfg import (
    JointSpringsExampleConfig,
)
from baxter_interface import CHECK_VERSION

from baxter_pykdl import baxter_kinematics

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from test_package.msg import verified_motionAction, verified_motionFeedback, verified_motionResult


class verifiedMotionServer:
    def __init__(self, reconfig_server):
        self.server = actionlib.SimpleActionServer('execute_verified_motion', verified_motionAction, self.execute, False)
        self.feedback = None
        self.result = None
        
        self._dyn = reconfig_server        
        
        # control parameters
        self._rate = 1000.0  # Hz
        self._missed_cmds = 20.0  # Missed cycles before triggering timeout
        
        # Include gripper(s)
        #self._left_gripper = baxter_interface.gripper.Gripper("left")
        
        # Limbs,limb parameters
        self.limb = {}
        self.limb["left"] = baxter_interface.Limb('left')
        self.limb["right"] = baxter_interface.Limb('right')
        self._left_arm = baxter_interface.Limb('left')
        
        self._kin = {}
        self._kin["left"] = baxter_kinematics('left')
        self._kin["right"] = baxter_kinematics('right')        
        
        self._springs = dict()
        self._damping = dict()
        self._start_angles = dict()
        self._goal_angles = dict()
        
        self._spring_modifier = []
        self._damping_modifier = []
        self._sim_time = []
        
        # Initialize Matlab
        # Start matlab
        print("Starting MATLAB...")
        self.eng = matlab.engine.start_matlab() # start matlab
        print("Matlab started.")
        print("Initializing robotic toolbox files ...")
        self.eng.baxter_matlab_robotics_toolbox_setup(nargout = 0)
        print("Initialized.")
        
        
        # Is this necessary?
        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        
        self.move_to_neutral('left')
        self.move_to_neutral('right')
        
        print('Starting server ...')
        self.server.start()
        
        
    def execute(self, goal):
        
        print('======= motion request recieved from client ...')
        
        self.feedback = verified_motionFeedback()   
        self.result = verified_motionResult()
        
        #self.move_to_neutral(goal.arm)
        
        self._spring_modifier = goal.motion_parameters[0]
        self._damping_modifier = goal.motion_parameters[1]
        self._sim_time = goal.motion_parameters[2]
        #print(self._spring_modifier)
        #print(self._damping_modifier)
        
        # Generate goal angles (inverse kinematics)
        self._start_angles = self.limb[goal.arm].joint_angles()
        found_solution = self.generate_target_angles(goal.arm,goal.EFF_movement)
        
        if not found_solution:
            self.result.error_message = "No inverse kinematics solution found."
            self.server.set_aborted(result=self.result)
            return
        
        # Verify the motion
        # Compute flowstar file - verify the controller
        print("Computing Flow* file, verifying the motion ...")
        self._update_parameters(goal.arm)
        timeEnd = matlab.double([self._sim_time])
        springMod = matlab.double([self._spring_modifier])
        dampingMod = matlab.double([self._damping_modifier])
        maxMinValues = self.eng.generate_flowstar_file(self._start_angles,self._goal_angles,self._springs,self._damping,timeEnd,springMod, dampingMod)
        print(maxMinValues)
        print("... verified.")
        
        # set control rate
        control_rate = rospy.Rate(self._rate)
        
        # for safety purposes, set the control rate command timeout.
        # if the specified number of command cycles are missed, the robot
        # will timeout and disable
        self.limb[goal.arm].set_command_timeout((1.0 / self._rate) * self._missed_cmds)
        
        #print('you made it this far?')
        
        ctrlTime = self._sim_time # in seconds
        ctrlTrigger = True
        start = rospy.Time.now()

        print("Implementing torque control ...")        
        
        while ctrlTrigger:
            nowTime = rospy.Time.now()
            timeDiff = nowTime.secs - start.secs
            if timeDiff >= ctrlTime:
                ctrlTrigger = False
            self._update_forces(goal.arm)
            control_rate.sleep()
            
        print("Exiting torque control ...")
        self.limb[goal.arm].exit_control_mode()
            
        print("Torqe control complete.")
        self.result.motion_complete = True
        self.server.set_succeeded(result=self.result)
        
    def generate_target_angles(self,arm,targetXYZ):
        print("Generating target joint angles ...")
        
        # Matlab - inverse kinematics
        #   Find EFF Position
        Eff_Start_baxter_7 = self.limb[arm].endpoint_pose() # EFF Position here is different than that used by the DH parameters
        #print('Baxter start EFF') 
        #print(Eff_Start_baxter_7)
        
        #EFF_left = self._left_arm.endpoint_pose()
        #print('Alternate EFF:')
        #print(EFF_left)
        
        #print('Arm start angles:')
        #print(self.limb[arm].joint_angles())
        
        move_x = targetXYZ[0]
        move_y = targetXYZ[1]
        move_z = targetXYZ[2]
        
        pykdl_end_point = [Eff_Start_baxter_7['position'].x + move_x, Eff_Start_baxter_7['position'].y + move_y, Eff_Start_baxter_7['position'].z + move_z]
        pykdl_end_orientation = [Eff_Start_baxter_7['orientation'].x, Eff_Start_baxter_7['orientation'].y, Eff_Start_baxter_7['orientation'].z, Eff_Start_baxter_7['orientation'].w]
        
        #print('pykdl end point')
        #print(pykdl_end_point)
        #print('pykdl end orientation')
        #print(pykdl_end_orientation)
        
        #print("KDL forward kinematics test:")
        #print(self._kin[arm].forward_position_kinematics())

        pykdl_end_angles = self._kin[arm].inverse_kinematics(pykdl_end_point,pykdl_end_orientation)
        
        #print('PYKDL angles:')
        #print(pykdl_end_angles)
        
        if len(pykdl_end_angles) == 0:
            # Error - no solution found
            ikin_solution_found = False
            return ikin_solution_found
        else:
            ikin_solution_found = True
            
        
        angles = self.limb[arm].joint_angles()
        if arm == 'left':
            angles['left_s0'] = pykdl_end_angles[0]
            angles['left_s1'] = pykdl_end_angles[1]
            angles['left_e0'] = pykdl_end_angles[2]
            angles['left_e1'] = pykdl_end_angles[3]
            angles['left_w0'] = pykdl_end_angles[4]
            angles['left_w1'] = pykdl_end_angles[5]
            angles['left_w2'] = pykdl_end_angles[6]
        elif arm == 'right':
            angles['right_s0'] = pykdl_end_angles[0]
            angles['right_s1'] = pykdl_end_angles[1]
            angles['right_e0'] = pykdl_end_angles[2]
            angles['right_e1'] = pykdl_end_angles[3]
            angles['right_w0'] = pykdl_end_angles[4]
            angles['right_w1'] = pykdl_end_angles[5]
            angles['right_w2'] = pykdl_end_angles[6]
        
        self._goal_angles = angles 
        print("Goal angles determined.")
        #print('Final angle positions:')        
        #print(angles)
        
        return ikin_solution_found
        
        
    def _update_forces(self,arm):
        """
        Calculates the current angular difference between the start position
        and the current joint positions applying the joint torque spring forces
        as defined on the dynamic reconfigure server.
        """
        # get latest spring constants
        self._update_parameters(arm)

        # disable cuff interaction
        #self._pub_cuff_disable.publish()

        # create our command dict
        cmd = dict()
        # record current angles/velocities
        cur_pos = self.limb[arm].joint_angles()
        cur_vel = self.limb[arm].joint_velocities()
        # calculate current forces
        for joint in self._start_angles.keys():
            # spring portion
            #cmd[joint] = self._springs[joint] * (self._start_angles[joint] - cur_pos[joint])
            cmd[joint] = self._springs[joint] * self._spring_modifier * (self._goal_angles[joint] - cur_pos[joint])
            # damping portion
            cmd[joint] -= self._damping[joint] * self._damping_modifier * cur_vel[joint]
        # command new joint torques
        self.limb[arm].set_joint_torques(cmd)
        
        
    
    def _update_parameters(self,arm):
        for joint in self.limb[arm].joint_names():
            self._springs[joint] = self._dyn.config[joint[-2:] + '_spring_stiffness']
            self._damping[joint] = self._dyn.config[joint[-2:] + '_damping_coefficient']
            
            
    def move_to_neutral(self,arm):
        """
        Moves the limb to neutral location.
        """
        print("Moving to the neutral arm position")
        self.limb[arm].move_to_neutral()
            
    def clean_shutdown(self):
        """
        Switches out of joint torque mode to exit cleanly
        """
        print("\nExiting process...")
        self.limb["left"].exit_control_mode()
        self.limb["right"].exit_control_mode()
        #if not self._init_state and self._rs.state().enabled:
        #    print("Disabling robot...")
        #    self._rs.disable()
        
def main():
    
    print("Initializing node... ")
    rospy.init_node("verified_arm_motion")
    dynamic_cfg_srv = Server(JointSpringsExampleConfig, lambda config, level: config)
    server = verifiedMotionServer(dynamic_cfg_srv)
    rospy.on_shutdown(server.clean_shutdown)
    rospy.spin()
    
if __name__ == "__main__":
    main()
        
        
        
        
        
        
        
        
        
        
        