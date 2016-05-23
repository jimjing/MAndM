#!/usr/bin/env python
import rospy
import tf
import geometry_msgs.msg

rospy.init_node('test_tf_broadcaster')

broadcaster = tf.TransformBroadcaster()
#listener = tf.TransformListener()


tagName = "tag_4"
tagName2 = "right_hand"


# translate point 0.20m in z + flip orientation. (x,y,z,w)
# axis/angle rotation (a,x,y,z) is equal to quaternion (cos(a/2),xsin(a/2),ysin(a/2),z*sin(a/2))
while not rospy.is_shutdown():
    broadcaster.sendTransform((0.0, 0.0, 0.20), (1.0, 0.0, 0.0, 0.0), rospy.Time.now(), "moveit_goal_"+tagName, tagName)
    broadcaster.sendTransform((0.0, 0.0, 0.20), (1.0, 0.0, 0.0, 0.0), rospy.Time.now(), "moveit_goal_"+tagName2, tagName2)
    #try:
    #    (trans,rot) = listener.lookupTransform('torso', "moveit_goal_"+tagName, rospy.Time(0))
    #    print "trans:" + str(trans)
    #except:
    #    print "cannot transform"

    # tf.transformation.py

    #rospy.sleep(0.05)
#rospy.spin()




