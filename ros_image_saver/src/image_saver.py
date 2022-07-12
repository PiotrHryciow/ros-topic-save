#!/usr/bin/env python

from pickle import NONE
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import os
from datetime import datetime

class image_saver:
    def __init__(self):
        rospy.init_node('image_saver', anonymous=True)
        self.bridge = CvBridge()
        #self.time = 0
        self.time = datetime.today().strftime('%m_%d_%H_%M_%S')

        #importing topic names for each camera
        self.ros_topic_camera_front = rospy.get_param("~ros_topic_camera_front", "test")
        self.ros_topic_camera_left = rospy.get_param("~ros_topic_camera_left", "test")
        self.ros_topic_camera_right = rospy.get_param("~ros_topic_camera_right", "test")
        #importing msg types for each camera
        self.msg_type_camera_front = rospy.get_param("~msg_type_camera_front", "std_msgs/String")
        self.msg_type_camera_left = rospy.get_param("~msg_type_camera_left", "std_msgs/String")
        self.msg_type_camera_right = rospy.get_param("~msg_type_camera_right", "std_msgs/String")
        #converting msg type from string to actual type
        self.msg_type_camera_front = self.import_msg_type(self.msg_type_camera_front)
        self.msg_type_camera_left = self.import_msg_type(self.msg_type_camera_left)
        self.msg_type_camera_right = self.import_msg_type(self.msg_type_camera_right)
        #imporint frequency of saving photos
        self.frequency = rospy.get_param("~frequency", "test")
        #importing path to save the photos
        self.path = rospy.get_param("~path")
        self.path_front = self.path + "/camera_front/front_{}.jpg"
        self.path_left = self.path + "/camera_left/left_{}.jpg"
        self.path_right = self.path + "/camera_right/right_{}.jpg"
        self.paths()



        self.camera_front = message_filters.Subscriber(self.ros_topic_camera_front, self.msg_type_camera_front)
        self.camera_left = message_filters.Subscriber(self.ros_topic_camera_left, self.msg_type_camera_left)
        self.camera_right = message_filters.Subscriber(self.ros_topic_camera_right, self.msg_type_camera_right)
        self.synchronizer = message_filters.ApproximateTimeSynchronizer([self.camera_front, self.camera_left, self.camera_right], 10, 0.1)
        #self.get_time = message_filters.Cache(self.camera_front)
        self.synchronizer.registerCallback(self.synchronize_callback)
        self.i=0
        self.prev_i=1
        rospy.Timer(rospy.Duration(1/self.frequency), self.save_callback)
           

    def save_callback(self, timer):
        self.i += 1

    def get_params(self):
        #reading input values from .launch file
        #importing topic names for each camera
        self.ros_topic_camera_front = rospy.get_param("~ros_topic_camera_front", "test")
        self.ros_topic_camera_left = rospy.get_param("~ros_topic_camera_left", "test")
        self.ros_topic_camera_right = rospy.get_param("~ros_topic_camera_right", "test")
        #importing msg types for each camera
        self.msg_type_camera_front = rospy.get_param("~msg_type_camera_front", "std_msgs/String")
        self.msg_type_camera_left = rospy.get_param("~msg_type_camera_left", "std_msgs/String")
        self.msg_type_camera_right = rospy.get_param("~msg_type_camera_right", "std_msgs/String")
        #converting msg type from string to actual type
        self.msg_type_camera_front = self.import_msg_type(self.msg_type_camera_front)
        self.msg_type_camera_left = self.import_msg_type(self.msg_type_camera_left)
        self.msg_type_camera_right = self.import_msg_type(self.msg_type_camera_right)

        #imporint frequency of saving photos
        self.frequency = rospy.get_param("~frequency", "test")

        #importing path to save the photos
        self.path = rospy.get_param("~path")

    def import_msg_type(self, msg_type):
        if msg_type == "sensors_msgs/Image":
            from sensor_msgs.msg import Image
            subscriber_msg = Image
        elif msg_type == "sensor_msgs/CompressedImage":
            from sensor_msgs.msg import CompressedImage
            subscriber_msg = CompressedImage
        return subscriber_msg

    def synchronize_callback(self, camera_front, camera_left, camera_right):
        try:
            #for other image formats then rgb change second paramiter "bgr8"
            self.image_front = self.bridge.compressed_imgmsg_to_cv2(camera_front, "bgr8")
            self.image_left = self.bridge.compressed_imgmsg_to_cv2(camera_left, "bgr8")
            self.image_right = self.bridge.compressed_imgmsg_to_cv2(camera_right, "bgr8")
            #self.time = self.get_time.getLatestTime()
            self.time = datetime.today().strftime('%m_%d_%H_%M_%S')
            if self.i != self.prev_i:
                cv2.imwrite(self.path_front.format(self.time), self.image_front)
                cv2.imwrite(self.path_left.format(self.time), self.image_left)
                cv2.imwrite(self.path_right.format(self.time), self.image_right)
        except CvBridgeError as e:
            print(e)

    def paths(self):
        if not(os.path.isdir(self.path)):
            os.mkdir(self.path)
        path_front = self.path + "/camera_front"
        path_left = self.path + "/camera_left"
        path_right = self.path + "/camera_right"
        if not(os.path.isdir(path_front)):
            os.mkdir(path_front)
        if not(os.path.isdir(path_left)):
            os.mkdir(path_left)
        if not(os.path.isdir(path_right)):
            os.mkdir(path_right)

if __name__ == '__main__':
    try:
        image_saver()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")