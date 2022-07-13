#!/usr/bin/env python

from re import L
import rospy
from sensor_msgs.msg import Imu
import csv

def callback(imu_data):
    f = open("/home/piotr/Desktop/Imu.csv", 'a')
    #print("orientation = ", imu_data.orientation)
    imu_list = [imu_data.orientation.x,imu_data.orientation.y,imu_data.orientation.z]
    writer = csv.writer(f)
    writer.writerow(imu_list)

if __name__ == '__main__':
    rospy.init_node('saver', anonymous=True)
    rospy.Subscriber("/imu/raw_data", Imu, callback)
    rospy.spin()
