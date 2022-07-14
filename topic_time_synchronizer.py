#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import CompressedImage


class synchronizer:

    def __init__(self):
        
        self.point_cloud = PointCloud2
        rospy.init_node('saver', anonymous=True)
        self.time = rospy.get_rostime()
        rospy.Subscriber("/os_cloud_node/points", PointCloud2, self.callback)
        rospy.Subscriber("/camera/image_front/compressed", CompressedImage, self.callback2)
        self.pub = rospy.Publisher("cloud/points", PointCloud2, queue_size=5)
        self.sleep()


    def sleep(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.pub.publish(self.point_cloud)
            rate.sleep()

    def callback(self, msg):
        self.point_cloud = msg
        self.point_cloud.header.stamp = self.time
    
    def callback2(self, msg):
        self.time = msg.header.stamp


if __name__ == '__main__':

    synchronizer()
    rospy.spin()