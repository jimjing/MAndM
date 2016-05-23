"""
Baxter Inverse Kinematics Handler (adapt from official IK example)
"""
import struct
import sys

import rospy

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,)

from sensor_msgs.msg import (JointState)

from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,)

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


class InverseKinematicsHandler(object):
    """
    this class in charge of kinematics related things
    it takes it a limb name and the associated object
    """
    def __init__(self, limb_name, limb_object):
        """
        limb_name: should be 'left' or 'right'
        """
        self.limb_name = limb_name
        self.limb_object = limb_object

        self.ns = "ExternalTools/"+self.limb_name+"/PositionKinematicsNode/IKService"
        self.iksvc = rospy.ServiceProxy(self.ns, SolvePositionIK)
        self.ikreq = SolvePositionIKRequest()

    def createPoseCartesianPoint(self, position, orientation):
        """
        create pose point.
        position = [x,y,z]
        orientation:[x,y,z,w]
        """
        return Pose(position=Point(*position), orientation=Quaternion(*orientation))

    def add_poseStamped(self, position, orientation):
        # TODO:
        # allow to add more postampes?
        self.ikreq.pose_stamp.append(PoseStamped(
            header=Header(stamp=rospy.Time.now(), frame_id='base'),
            pose=self.createPoseCartesianPoint(position,orientation)))

    def prepareIK(self, poses, joints=[]):
        """
        add in waypoints
        poses = [[position, orientation], [position, orientation], ...]
        position = [x,y,z]
        orientation = [x,y,z,w]
        joints = [dict, dict, ...]
        """
        # clear ikreq (recreate)
        self.ikreq = SolvePositionIKRequest()

        # check if poses length = joints length,
        # yes, use joints as seed, no: no seed
        useSeed = False
        if len(poses) == len(joints):
            useSeed = True

        # add waypoints ([0]=pose, [1]=orientation)
        # add seed if len > 0
        for idx,waypoint in enumerate(poses):
            self.add_poseStamped(waypoint[0], waypoint[1])
            if useSeed:
                self.ikreq.seed_angles.append(JointState(name=joints[idx].keys(),
                                                    position=joints[idx].values()))

        # final form of ikreq
        mAndm_logger.debug("self.ikreq:" + str(self.ikreq))

    def runIK(self):
        try:
            rospy.wait_for_service(self.ns, 5.0)
            resp = self.iksvc(self.ikreq)
            #resp = iksvc_right(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            mAndm_logger.warning('return 1')
            return 1, []

        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                                   resp.result_type)
        mAndm_logger.debug("resp_seeds:" + str(resp_seeds))
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                       self.ikreq.SEED_USER: 'User Provided Seed',
                       self.ikreq.SEED_CURRENT: 'Current Joint Angles',
                       self.ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                      }.get(resp_seeds[0], 'None')
            mAndm_logger.info("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
                 (seed_str,))
            # Format solution into Limb API-compatible dictionary
            limb_joints_list = []
            for x in resp.joints:
                limb_joints_list.append(dict(zip(x.name, x.position)))
            mAndm_logger.log(2, "\nIK Joint Solution:\n"+ str(limb_joints_list))
            mAndm_logger.log(2, "------------------")
            mAndm_logger.log(2, "Response Message:\n" +  str(resp))
        else:
            mAndm_logger.info("INVALID POSE - No Valid Joint Solution Found.")

        mAndm_logger.warning('return 0')
        return 0, limb_joints_list


if __name__ == "__main__":
    # enable robot
    rospy.init_node("rsdk_ik_service_client")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    mAndm_logger.info("Enabling robot... ")
    rs.enable()

    # get current pose (left, right)
    left_limb = baxter_interface.Limb('left')
    right_limb = baxter_interface.Limb('right')

    a = InverseKinematicsHandler('left',left_limb)

    # list of pose
    # one seed one pose to make sure that it works
    """SAME SEED
    left_poses = [[[0.657579481614, 0.851981417433, 0.0388352386502],
                   [-0.366894936773, 0.885980397775, 0.108155782462, 0.262162481772]],
                  [[0.657579481614, 0.851981417433, 0.5088352386502],
                   [-0.366894936773, 0.885980397775, 0.108155782462, 0.262162481772]],
                  [[0.257579481614, 0.851981417433, 0.5088352386502],
                   [-0.366894936773, 0.885980397775, 0.108155782462, 0.262162481772]]]

    #left_joints = []
    left_joints = [left_limb.joint_angles(), left_limb.joint_angles(),
                   left_limb.joint_angles()]

    a.prepareIK(left_poses, left_joints)
    _, limb_joints_list =  a.runIK()
    """

    # new one pose at a time (better)
    left_poses = [[[0.657579481614, 0.851981417433, 0.0388352386502],
                   [-0.366894936773, 0.885980397775, 0.108155782462, 0.262162481772]],
                  [[0.657579481614, 0.851981417433, 0.5088352386502],
                   [-0.366894936773, 0.885980397775, 0.108155782462, 0.262162481772]],
                  [[0.257579481614, 0.851981417433, 0.5088352386502],
                   [-0.366894936773, 0.885980397775, 0.108155782462, 0.262162481772]]]

    #left_joints = []
    limb_joints_list = []
    current_joints = [left_limb.joint_angles()]
    for x in left_poses:
        mAndm_logger.debug(current_joints)
        a.prepareIK([x], current_joints)
        _, new_joints =  a.runIK()
        limb_joints_list.extend(new_joints)
        current_joints = new_joints

    mAndm_logger.debug("limb_joints_list:" + str(limb_joints_list))
    #left_limb.set_joint_position_speed(0.3)
    mAndm_logger.debug('original joint angles:' + str(left_limb.joint_angles()))

    for limb_joints in limb_joints_list:
        left_limb.move_to_joint_positions(limb_joints) #need to check proximity yourselves
        mAndm_logger.debug('current_joint_angles:' + str(left_limb.joint_angles()))
        mAndm_logger.debug('left limb endpoint_pose:' + str(left_limb.endpoint_pose()))

# now make sure the angles are within range(radians)
jointRanges = {'S0':[-1.7016 ,+1.7016],\
           'S1':[-2.147  ,+1.047],\
           'E0':[-3.0541 , +3.0541],\
           'E1':[-0.05   ,+2.618 ],\
           'W0':[-3.059  ,+3.059],\
           'W1':[-1.5707 ,+2.094],\
           'W2':[-3.059  ,+3.059]}