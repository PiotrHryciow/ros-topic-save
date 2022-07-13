#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2

#open source library from :https://github.com/dimatura/pypcd
from pypcd import pypcd

def callback(msg):
    pc = pypcd.PointCloud.from_msg(msg)
    pc.save_pcd('/home/piotr/Desktop/foo.pcd', compression='binary_compressed')

if __name__ == '__main__':
    rospy.init_node('saver', anonymous=True)
    rospy.Subscriber("/os_cloud_node/points", PointCloud2, callback)
    rospy.spin()
