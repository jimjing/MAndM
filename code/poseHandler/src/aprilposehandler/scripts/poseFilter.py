#!/usr/bin/env python
import rospy
import tf
import numpy
from functools import partial
from std_msgs.msg import String
import std_msgs.msg
from geometry_msgs.msg import PoseArray, PointStamped, Pose
from apriltags_ros.msg import AprilTagDetectionArray, AprilTagDetection

class PoseFilter:
    def __init__(self):
        rospy.init_node('pose_filter', anonymous=True)
        self.sub_topics = ["/camera/tag_detections","/cameraUP/tag_detections"]
        self.subs = {}
        self.tf = None
        self.camera_data = {}
        self.tag_list = []

    def initialize(self):
        self.tf = tf.TransformListener()
        for name in self.sub_topics:
            cb = partial(self.callback, topic=name)
            self.subs[name] = rospy.Subscriber(name, AprilTagDetectionArray, cb)
            self.camera_data[name] = {}

    def talker(self):
        pub = rospy.Publisher('tag_detections_merged', AprilTagDetectionArray, queue_size=10)
        rate = rospy.Rate(1) # 10hz
        while not rospy.is_shutdown():

            detection_array = AprilTagDetectionArray()
            for tag in self.tag_list:
                detections_from_cameras = []
                for topic in self.camera_data.keys():
                    if (tag in self.camera_data[topic].keys()):
                        if (self.camera_data[topic][tag] != None):
                            detections_from_cameras.append(self.camera_data[topic][tag])
                            self.camera_data[topic][tag] = None

                if (len(detections_from_cameras)>0):
                    merged_detection = AprilTagDetection()
                    merged_detection.id = tag
                    merged_detection.size = detections_from_cameras[0].size
                    merged_detection.pose.header = detections_from_cameras[0].pose.header

                    pose_list = [d.pose.pose for d in detections_from_cameras]
                    merged_detection.pose.pose = self.averagePose (pose_list)

                    detection_array.detections.append(merged_detection)

            pub.publish(detection_array)
            rate.sleep()

    def averagePose (self, pose_list):
        p = Pose()
        p.position.x = numpy.mean([pose.position.x for pose in pose_list])
        p.position.y = numpy.mean([pose.position.y for pose in pose_list])
        p.position.z = numpy.mean([pose.position.z for pose in pose_list])
        p.orientation = pose_list[0].orientation
        return p

    def callback(self, data, topic):
        for detection in data.detections:
            if (detection.id not in self.tag_list):
                self.tag_list.append(detection.id)
            self.camera_data[topic][detection.id] = detection

        #point_in_world = self.tf.transformPoint("/world", ps)

    def listener(self):
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    try:
        pf = PoseFilter()
        pf.initialize()
        pf.talker()
    except rospy.ROSInterruptException:
        pass
