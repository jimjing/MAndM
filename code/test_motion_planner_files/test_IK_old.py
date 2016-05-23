"""
Baxter RSDK Inverse Kinematics Example
"""
import argparse
import struct
import sys

import rospy

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from sensor_msgs.msg import (
    JointState)

from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import baxter_interface
from baxter_interface import CHECK_VERSION

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

# enable robot
rospy.init_node("rsdk_ik_service_client")
rs = baxter_interface.RobotEnable(CHECK_VERSION)
mAndm_logger.info("Enabling robot... ")
rs.enable()

# get current pose (left, right)
left_limb = baxter_interface.Limb('left')
right_limb = baxter_interface.Limb('right')

mAndm_logger.debug("right_limb.endpoint_pose():" + str(right_limb.endpoint_pose()))
mAndm_logger.debug("left_limb.endpoint_pose():" + str(left_limb.endpoint_pose()))

right_limbPose = Pose(position= right_limb.endpoint_pose()['position'],
     orientation=right_limb.endpoint_pose()['orientation'])

ns_left = "ExternalTools/left/PositionKinematicsNode/IKService"
ns_right = "ExternalTools/right/PositionKinematicsNode/IKService"

iksvc_left = rospy.ServiceProxy(ns_left, SolvePositionIK)
iksvc_right = rospy.ServiceProxy(ns_right, SolvePositionIK)

ikreq = SolvePositionIKRequest()
hdr = Header(stamp=rospy.Time.now(), frame_id='base')

poses = {
    'left': PoseStamped(
        header=hdr,
        pose=Pose(
            position=Point(
                x=0.657579481614,
                y=0.851981417433,
                z=0.0388352386502,
            ),
            orientation=Quaternion(
                x=-0.366894936773,
                y=0.885980397775,
                z=0.108155782462,
                w=0.262162481772,
            ),
        ),
    ),
    'right': PoseStamped(
        header=hdr,
        pose=Pose(
            position=Point(
                x=0.656982770038,
                y=-0.852598021641,
                z=0.0388609422173,
            ),
            orientation=Quaternion(
                x=0.367048116303,
                y=0.885911751787,
                z=-0.108908281936,
                w=0.261868353356,
            ),
        ),
    ),
    'left2': PoseStamped(
        header=hdr,
        pose=Pose(
            position=Point(
                x=0.657579481614,
                y=0.851981417433,
                z=0.5088352386502,
            ),
            orientation=Quaternion(
                x=-0.366894936773,
                y=0.885980397775,
                z=0.108155782462,
                w=0.262162481772,
            ),
        ),
    )
}

ikreq.pose_stamp.append(poses['left'])

left_joint_state =  JointState(name=left_limb.joint_angles().keys(),
           position=left_limb.joint_angles().values())

mAndm_logger.debug("left_limb.joint_angles():" + str(left_limb.joint_angles()))

ikreq.seed_angles.append(left_joint_state)

ikreq.pose_stamp.append(poses['left2'])
left_joint_state =  JointState(name=left_limb.joint_angles().keys(),
           position=left_limb.joint_angles().values())

mAndm_logger.debug("left_limb.joint_angles():" + str(left_limb.joint_angles()))

ikreq.seed_angles.append(left_joint_state)

# ikreq.pose_stamp.append(poses['right'])

# right_joint_state =  JointState(name=right_limb.joint_angles().keys(),
#           position=right_limb.joint_angles().values())

# mAndm_logger.debug("right_limb.joint_angles():" + str(right_limb.joint_angles()))
# ikreq.seed_angles.append(right_joint_state)

mAndm_logger.info("ikreq:" + str(ikreq))
try:
    rospy.wait_for_service(ns_left, 5.0)
    resp = iksvc_left(ikreq)
    #resp = iksvc_right(ikreq)
except (rospy.ServiceException, rospy.ROSException), e:
    rospy.logerr("Service call failed: %s" % (e,))
    mAndm_logger.warning('return 1')
    #return 1

# Check if result valid, and type of seed ultimately used to get solution
# convert rospy's string representation of uint8[]'s to int's
resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                           resp.result_type)
if (resp_seeds[0] != resp.RESULT_INVALID):
    seed_str = {
                ikreq.SEED_USER: 'User Provided Seed',
                ikreq.SEED_CURRENT: 'Current Joint Angles',
                ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
               }.get(resp_seeds[0], 'None')
    print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
          (seed_str,))
    # Format solution into Limb API-compatible dictionary
    limb_joints_list = []
    for x in resp.joints:
        limb_joints_list.append(dict(zip(x.name, x.position)))
    print "\nIK Joint Solution:\n", limb_joints_list
    print "------------------"
    print "Response Message:\n", resp
else:
    print("INVALID POSE - No Valid Joint Solution Found.")

mAndm_logger.warning('return 0')
#return 0

current_gripper_pos = [0.582583, -0.180819, 0.216003] #[x, y, z]
rot = [0.03085, 0.9945, 0.0561, 0.0829]

#output_angles_radians = ( output_angles_radians + numpy.pi) % (2 * numpy.pi ) - numpy.pi
#print output_angles_radians

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

mAndm_logger.debug("limb_joints_list:" + str(limb_joints_list))
import time
# traj accuracy problem
#left_limb.set_joint_position_speed(0.3)

for limb_joints in limb_joints_list:
    left_limb.move_to_joint_positions(limb_joints) #need to check proximity yourselves
    current_joint_angles = left_limb.joint_angles()
    mAndm_logger.debug('current_joint_angles:' + str(current_joint_angles))
    mAndm_logger.debug('left limb endpoint_pose:' + str(left_limb.endpoint_pose()))

# import time
# # traj accuracy problem
# right_limb.set_joint_position_speed(0.5)
# right_limb.move_to_joint_positions(limb_joints) #need to check proximity yourselves
# current_joint_angles = right_limb.joint_angles()
# mAndm_logger.debug('current_joint_angles:' + str(current_joint_angles))
# mAndm_logger.debug('right limb endpoint_pose:' + str(right_limb.endpoint_pose()))