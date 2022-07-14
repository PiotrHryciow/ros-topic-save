#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import os
import csv
from datetime import datetime
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
#open source library from :https://github.com/dimatura/pypcd
from pypcd import pypcd


class data_saver:
    def __init__(self):
        rospy.init_node('data_saver', anonymous=True)
        self.bridge = CvBridge()
        #importing current date[%m=months, %d=days, %H=hours, %M=minutes, %S=seconds, %f=microseconds]
        self.date_format = '%m_%d_%H_%M_%S_%f'
        self.time = datetime.today().strftime(self.date_format)

        #importing each topic path
        self.ros_topic_camera_front = rospy.get_param("~ros_topic_camera_front", "test")
        self.ros_topic_camera_left = rospy.get_param("~ros_topic_camera_left", "test")
        self.ros_topic_camera_right = rospy.get_param("~ros_topic_camera_right", "test")
        self.ros_topic_point_cloud = rospy.get_param("~ros_topic_point_cloud", "test")
        self.ros_topic_imu = rospy.get_param("~ros_topic_imu", "test")
        self.ros_topic_cmd_vel = rospy.get_param("~ros_topic_cmd_vel", "test")

        #importing each msg type
        self.msg_type_camera_front = rospy.get_param("~msg_type_camera_front", "std_msgs/String")
        self.msg_type_camera_left = rospy.get_param("~msg_type_camera_left", "std_msgs/String")
        self.msg_type_camera_right = rospy.get_param("~msg_type_camera_right", "std_msgs/String")
        self.msg_type_point_cloud = rospy.get_param("~msg_type_point_cloud", "std_msgs/String")
        self.msg_type_imu = rospy.get_param("~msg_type_imu", "std_msgs/String")
        self.msg_type_cmd_vel = rospy.get_param("~msg_type_cmd_vel", "std_msgs/String")

        #converting msg type from string to actual type from ros topic
        self.msg_type_camera_front = self.import_msg_type(self.msg_type_camera_front)
        self.msg_type_camera_left = self.import_msg_type(self.msg_type_camera_left)
        self.msg_type_camera_right = self.import_msg_type(self.msg_type_camera_right)
        self.msg_type_point_cloud = self.import_msg_type(self.msg_type_point_cloud)
        self.msg_type_imu = self.import_msg_type(self.msg_type_imu)
        self.msg_type_cmd_vel = self.import_msg_type(self.msg_type_cmd_vel)

        #importing frequency of saving photos
        self.frequency = rospy.get_param("~frequency", "test")

        #importing path to where save the photos
        self.path = rospy.get_param("~path")
        self.path_camera_front = self.path + "/camera_front/front_{}.jpg"
        self.path_camera_left = self.path + "/camera_left/left_{}.jpg"
        self.path_camera_right = self.path + "/camera_right/right_{}.jpg"
        self.path_point_cloud = self.path + "/point_cloud/point_cloud_{}.pcd"
        self.path_imu = self.path + "/imu/imu_{}.csv"
        self.path_cmd_vel = self.path + "/cmd_vel/cmd_vel_{}.csv"
        self.mkdirs()

        #initialize message_filters which enables to synchronize timings of incoming messages from various topics
        self.camera_front = message_filters.Subscriber(self.ros_topic_camera_front, self.msg_type_camera_front)
        self.camera_left = message_filters.Subscriber(self.ros_topic_camera_left, self.msg_type_camera_left)
        self.camera_right = message_filters.Subscriber(self.ros_topic_camera_right, self.msg_type_camera_right)
        self.point_cloud = message_filters.Subscriber(self.ros_topic_point_cloud, self.msg_type_point_cloud)
        self.imu = message_filters.Subscriber(self.ros_topic_imu, self.msg_type_imu)
        self.cmd_vel = message_filters.Subscriber(self.ros_topic_cmd_vel, self.msg_type_cmd_vel)


        ###################### second parameter is queue, third is allowed time difference between each topic ############################## 
        self.synchronizer = message_filters.ApproximateTimeSynchronizer([self.camera_front, self.camera_left, self.camera_right, self.imu, self.point_cloud, self.cmd_vel],
                                                                        1000, 0.5, allow_headerless=True)
        self.synchronizer.registerCallback(self.synchronize_callback)


        #end of initialization, goes to infinite loop, where it can recive callbacks  
        self.sleep()  

    def sleep(self):
        rate = rospy.Rate(self.frequency)
        while not rospy.is_shutdown():
            rate.sleep()            

    def import_msg_type(self, msg_type):
        if msg_type == "sensors_msgs/Image":
            subscriber_msg = Image
        elif msg_type == "sensor_msgs/CompressedImage":
            subscriber_msg = CompressedImage
        elif msg_type == "sensor_msgs/PointCloud2":
            from sensor_msgs.msg import PointCloud2
            subscriber_msg = PointCloud2
        elif msg_type == "sensor_msgs/Imu":
            from sensor_msgs.msg import Imu
            subscriber_msg = Imu
        elif msg_type == "geometry_msgs/Twist":
            from geometry_msgs.msg import Twist
            subscriber_msg = Twist
        return subscriber_msg

    def synchronize_callback(self, camera_front, camera_left, camera_right, imu, point_cloud, cmd_vel):
        try:
            #getting current time
            self.time = datetime.today().strftime(self.date_format)
            #for other image formats then rgb change second paramiter "bgr8"
            #checking what type of format is the image
            if self.msg_type_camera_front == CompressedImage:
                self.image_front = self.bridge.compressed_imgmsg_to_cv2(camera_front, "bgr8")
            if self.msg_type_camera_front == Image:
                self.image_front = self.bridge.imgmsg_to_cv2(camera_front, "bgr8")
            if self.msg_type_camera_left == CompressedImage:
                self.image_left = self.bridge.compressed_imgmsg_to_cv2(camera_left, "bgr8")
            if self.msg_type_camera_left == Image:
                self.image_left = self.bridge.imgmsg_to_cv2(camera_left, "bgr8")
            if self.msg_type_camera_right == CompressedImage:
                self.image_right = self.bridge.compressed_imgmsg_to_cv2(camera_right, "bgr8")
            if self.msg_type_camera_right == Image:
                self.image_right = self.bridge.imgmsg_to_cv2(camera_right, "bgr8")
            #saving the images
            cv2.imwrite(self.path_camera_front.format(self.time), self.image_front)
            cv2.imwrite(self.path_camera_left.format(self.time), self.image_left)
            cv2.imwrite(self.path_camera_right.format(self.time), self.image_right)
            #saving pointCloud
            pc = pypcd.PointCloud.from_msg(point_cloud)
            pc.save_pcd(self.path_point_cloud.format(self.time), compression='binary_compressed')
            #saving imu
            imu_list = [self.time, imu.orientation.x,imu.orientation.y,imu.orientation.z,
                        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z,
                        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z]
            self.imu_writer.writerow(imu_list)
            #saving cmd_vel
            cmd_vel_list = [self.time, cmd_vel.linear.x,cmd_vel.linear.y, cmd_vel.linear.z,
                        cmd_vel.angular.x, cmd_vel.angular.y, cmd_vel.angular.z]
            self.cmd_vel_writer.writerow(cmd_vel_list)
        except CvBridgeError as e:
            print(e)

    def mkdirs(self):
        if not(os.path.isdir(self.path)):
            os.mkdir(self.path)
        path_camera_front = self.path + "/camera_front"
        path_camera_left = self.path + "/camera_left"
        path_camera_right = self.path + "/camera_right"
        path_point_cloud = self.path + "/point_cloud"
        path_imu = self.path + "/imu"
        path_cmd_vel = self.path + "/cmd_vel"
        if not(os.path.isdir(path_camera_front)):
            os.mkdir(path_camera_front)
        if not(os.path.isdir(path_camera_left)):
            os.mkdir(path_camera_left)
        if not(os.path.isdir(path_camera_right)):
            os.mkdir(path_camera_right)
        if not(os.path.isdir(path_point_cloud)):
            os.mkdir(path_point_cloud)
        if not(os.path.isdir(self.path_imu)):
            if not(os.path.isdir(path_imu)):
                os.mkdir(path_imu)
            self.imu_open = open(self.path_imu.format(self.time), "a")
            self.imu_writer = csv.writer(self.imu_open)
            self.imu_writer.writerow(['time', 'orientation x', 'orientation y', 'orientation z',
                                    "angular velocity x", "angular velocity y", "angular velocity z",
                                    "linear_acceleration x", "linear_acceleration y", "linear_acceleration z"])
        if not(os.path.isdir(self.path_cmd_vel)):
            if not(os.path.isdir(path_cmd_vel)):
                os.mkdir(path_cmd_vel)
            self.cmd_vel = open(self.path_cmd_vel.format(self.time), "a")
            self.cmd_vel_writer = csv.writer(self.imu_open)
            self.cmd_vel_writer.writerow(['time', 'linear x', 'linear y', 'linear z',
                                    "angular x", "angular y", "angular z"])

if __name__ == '__main__':
    try:
        data_saver()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
