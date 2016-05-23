#!/usr/bin/env python

import rospy
import tf
import geometry_msgs.msg
import numpy
import math

rospy.init_node('test_tf_transform')

listener = tf.TransformListener()
broadcaster = tf.TransformBroadcaster()

tagName = "right_hand"
tagName2 = "left_hand"

# first get tag location in camera frame
print "tagName:" +str(tagName)


# transform frame
z_axis_offset_tf = tf.transformations.translation_matrix([0,0,0.2])
about_x_rotation_qua = tf.transformations.quaternion_about_axis(math.pi,[1,0,0])

print "about_x_rotation_qua: " + str(about_x_rotation_qua)

# find tag in world frame (x,y,z,w)
torso_tag_pose = None
torso_tag_rot  = None
while not (torso_tag_pose and torso_tag_rot):
    try:
        (torso_tag_pose,torso_tag_rot) = listener.lookupTransform('torso', tagName, rospy.Time(0))
        #print "list of frames:" + str(listener.getFrameStrings())
        #print "Does " + tagName + " exist?" + str(listener.frameExists(tagName))
    except:
        rospy.sleep(1)
        #print "Stilling waiting for tag pose and orientation in the torso frame"

#torso_tag_pose = (0.2064564758793131, -0.3881725683716438, -0.5539859494136887)
#torso_tag_rot = (0.4073767265386651, 0.9132375743568946, 0.00590749675790219, 0.0025371124202168555)

print "torso_original_tag_pose: " + str(torso_tag_pose)
print "torso_original_tag_rot: " + str(torso_tag_rot)

torso_to_tag_pose_tf = tf.transformations.translation_matrix(list(torso_tag_pose))

torso_transformed_tag_pose = torso_to_tag_pose_tf.dot(z_axis_offset_tf.dot([0,0,0,1]))
torso_transformed_tag_rot  = tf.transformations.quaternion_multiply(list(torso_tag_rot), about_x_rotation_qua)


print "torso_transformed_tag_pose: " + str(torso_transformed_tag_pose)
print "torso_transformed_tag_rot: " + str(torso_transformed_tag_rot)

# check with rviz
while not rospy.is_shutdown():
    broadcaster.sendTransform(tuple(torso_transformed_tag_pose[0:3]), tuple(torso_transformed_tag_rot), rospy.Time.now(), "moveit_goal_"+tagName, 'torso')

rospy.spin()
