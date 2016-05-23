import rospy
from baxter_pykdl import baxter_kinematics
import time

import baxter_interface
from baxter_interface import CHECK_VERSION

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

import trajHelper
import numpy
import time

"""
This python file gives an overview of how to do planning
"""

# inputs: current joint location, goal position of gripper
# outputs return trajectory


def set_j(limb, joint_name, desired_pos):
    current_position = limb.joint_angle(joint_name)
    joint_command = {joint_name: desired_pos}
    limb.set_joint_positions(joint_command)

def inverse_kinematics_wrapper(position, orientation=[], seed=[]):
    """
    takes in seed in radians. convert to degrees and feed into inverse_kinematics in pyKDL.
    then convert outputto radians and return.
    """
    seed_degrees = []
    if seed:
        #seed = seed_degrees # in radians now
        for x in seed:
            seed_degrees.append(x*180/numpy.pi) # convert from radians to degree
    mAndm_logger.log(2,"seed_degrees:" + str(seed_degrees))

    if orientation and seed_degrees:
        output_angles_degrees = kin.inverse_kinematics(position, orientation=orientation, seed=seed_degrees)
    elif orientation:
        output_angles_degrees = kin.inverse_kinematics(position, orientation=orientation)
    elif seed_degrees:
        output_angles_degrees = kin.inverse_kinematics(position, seed=seed_degrees)
    else:
        output_angles_degrees = kin.inverse_kinematics(position)
    mAndm_logger.log(2,"output_angles_degrees:" + str(output_angles_degrees))

    output_angles_radians = []
    if output_angles_degrees is not None:
        for x in output_angles_degrees:
            output_angles_radians.append(x/180*numpy.pi)
    else:
        output_angles_radians = None
    mAndm_logger.log(2,"output_angles_radians:" + str(output_angles_radians))

    return output_angles_radians

limb = 'right'
# enable robot
rospy.init_node('baxter_kinematics')
rs = baxter_interface.RobotEnable(CHECK_VERSION)
mAndm_logger.info("Enabling robot... ")
rs.enable()

waypointEx = trajHelper.WayPointExecution(limb)
#waypointEx._limb.move_to_neutral()

#do IK
kin = baxter_kinematics(limb)

# find current position
mAndm_logger.debug('kin.forward_position_kinematics():' + str(kin.forward_position_kinematics()))
current_gripper_pos = kin.forward_position_kinematics()

current_gripper_pos = [0.582583, -0.180819, 0.216003] #[x, y, z]
rot = [0.03085, 0.9945, 0.0561, 0.0829]
# later on only test it here. inverse to forward
mAndm_logger.debug('current_gripper_pos:' + str(current_gripper_pos))
mAndm_logger.debug('current_gripper_pos[:3]:' + str(current_gripper_pos[:3]))
mAndm_logger.debug('rot:' + str(rot))


current_joint_angles = [waypointEx._limb.joint_angle(jnt) for jnt in waypointEx._limb.joint_names()]
mAndm_logger.debug('current_joint_angles:' + str(current_joint_angles))

output_angles_radians = kin.inverse_kinematics(current_gripper_pos[:3], orientation=rot, seed=current_joint_angles)
print output_angles_radians

#next_joint_angles = inverse_kinematics_wrapper(current_gripper_pos[:3], seed=current_joint_angles)
#mAndm_logger.debug('next_joint_angles:' + str(next_joint_angles))

output_angles_radians = ( output_angles_radians + numpy.pi) % (2 * numpy.pi ) - numpy.pi
#output_angles_radians = numpy.unwrap(output_angles_radians, axis=0) # wrap around 0 to 2 pi
print output_angles_radians

# now make sure the angles are within range
# radians
jointRanges = {'S0':[-1.7016 ,+1.7016],\
           'S1':[-2.147  ,+1.047],\
           'E0':[-3.0541 , +3.0541],\
           'E1':[-0.05   ,+2.618 ],\
           'W0':[-3.059  ,+3.059],\
           'W1':[-1.5707 ,+2.094],\
           'W2':[-3.059  ,+3.059]}
# some angles are out of range.. don't know what to do..
# i guess try their IK example provided. not with pyKDL

next_joint_angles_dict = dict(zip(waypointEx._limb.joint_names(), output_angles_radians))
mAndm_logger.debug('next_joint_angles_dict:' + str(next_joint_angles_dict))

current_gripper_pos = kin.forward_position_kinematics(next_joint_angles_dict)
mAndm_logger.debug('from inverse_gripper_pos:' + str(current_gripper_pos))

# traj accuracy problem
waypointEx._limb.set_joint_positions(next_joint_angles_dict) #need to check proximity yourselves
time.sleep(10.0)
current_joint_angles = waypointEx._limb.joint_angles()
mAndm_logger.debug('current_joint_angles:' + str(current_joint_angles))
current_gripper_pos = kin.forward_position_kinematics(current_joint_angles)
mAndm_logger.debug('from inverse_gripper_pos:' + str(current_gripper_pos))

# constraints wasn't too right, as in it is over (KDL returns things that cannot be executed.)


# # !!! replace with random pose?
# pos = [0.582583, -0.180819, 0.216003] #[x, y, z]
# #pos = [0.582583, -0.90819, 0.216003] #[x, y, z]
# rot = [0.03085, 0.9945, 0.0561, 0.0829] # [i, j, k, w]

# stepPose = []
# noOfSteps = 10
# inverseTrialsThres = 10

# # now separate pos into steps
# for idx, coordPose in enumerate(pos):
#     stepPose.append(numpy.linspace(current_gripper_pos[idx], coordPose, noOfSteps))

# mAndm_logger.debug('stepPose:' + str(stepPose))

# # now append waypoints
# #mAndm_logger.debug("waypointEx._limb.joint_names():" + str(waypointEx._limb.joint_names()))
# #mAndm_logger.debug("waypointEx._limb.joint_angles():" + str(waypointEx._limb.joint_angles()))

# current_joint_angles = [waypointEx._limb.joint_angle(jnt) for jnt in waypointEx._limb.joint_names()]
# step_joint_angles = current_joint_angles
# for x in range(noOfSteps):
#     # can also add seed
#     currentStep = [subArray[x] for subArray in stepPose]
#     mAndm_logger.log(4, 'currentStep:' + str(currentStep))
#     mAndm_logger.log(6, 'step_joint_angles:' + str(step_joint_angles))
#     #mAndm_logger.debug(kin.inverse_kinematics(pos, orientation=rot, seed=cur_cmd)) # return joint angles
#     next_joint_angles = inverse_kinematics_wrapper(currentStep, seed=step_joint_angles)
#     mAndm_logger.log(6, "next_joint_angles:" + str(next_joint_angles)) # return joint angles

#     # inverseTrailsCount = 0
#     # while inverseTrailsCount < inverseTrialsThres:
#     #     next_joint_angles = inverse_kinematics_wrapper(currentStep, seed=step_joint_angles)
#     #     #next_joint_angles = inverse_kinematics_wrapper(currentStep)
#     #     mAndm_logger.log(6, "next_joint_angles:" + str(next_joint_angles)) # return joint angles
#     #     # try a couple times.
#     #     if next_joint_angles is not None:
#     #         inverseTrailsCount += inverseTrialsThres
#     #     else:
#     #         inverseTrailsCount += 1


#     # add new joints if it exists
#     if next_joint_angles is not None:
#         waypointEx.addWaypoint(next_joint_angles)
#         step_joint_angles = next_joint_angles


# waypointEx.playWaypoints(timeout=20.0)
# mAndm_logger.debug('kin.forward_position_kinematics():' + str(kin.forward_position_kinematics()))

# rospy.spin()

# # radians
# jointRanges = {'S0':[-1.7016 ,+1.7016],\
#            'S1':[-2.147  ,+1.047],\
#            'E0':[-3.0541 , +3.0541],\
#            'E1':[-0.05   ,+2.618 ],\
#            'W0':[-3.059  ,+3.059],\
#            'W1':[-1.5707 ,+2.094],\
#            'W2':[-3.059  ,+3.059]}


# # what have done
# # try different methods to move arm (trajectory executor with server/client)
# # ended up with just move_to_joint positions (blocking)

# # problems
# # not accurate end pos: accuracy problem (set_positions - non-blocking)
# # radians/degrees not sure (testing with forward inverse kinamatices that requires
# # modification of the library)

# # moving on
# # check object/arm collisions
# # RRT
# # also find out how far each arm can reach