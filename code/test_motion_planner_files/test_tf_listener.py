#!/usr/bin/env python
import rospy
import tf
import geometry_msgs.msg

rospy.init_node('test_tf_listener')

listener = tf.TransformListener()

tagName = "right_hand"
tagName2 = "left_hand"

# first get tag location in camera frame
print "tagName:" +str(tagName)

# find tag in world frame (x,y,z,w)
while not rospy.is_shutdown():
    try:
        (torso_tag_pose,torso_tag_rot) = listener.lookupTransform('torso', "moveit_goal_"+tagName, rospy.Time(0))
        print "list of frames:" + str(listener.getFrameStrings())
        print "Does " + tagName + " exist?" + str(listener.frameExists(tagName))
    except:
        rospy.sleep(1)
        #print "Stilling waiting for tag pose and orientation in the torso frame"


