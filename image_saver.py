#!/usr/bin/env python
#for reading input commands
import sys
import getopt
#standard rospy and opencv libraries
import rospy
import cv2
#for converting image from rostopic to opencv
from cv_bridge import CvBridge, CvBridgeError
#for reciving image from ros topic
from sensor_msgs.msg import CompressedImage

#initialize bridge
bridge = CvBridge()

class image_saver:
    def __init__(self):
        #define the frequency of saving the images
        self.frequency = 1
        #define argument lists for inputs
        self.argument_list = sys.argv[1:]
        self.short_options = "hp:t:f:"
        self.long_options = ["help", "path=", "topic=", "frequency="]
        self.path = "/home"
        self.topic = "/camera"
        self.argument_reader()
        #define name of the node
        rospy.init_node('image_saver', anonymous=True)
        #define path to the topic with published images
        rospy.Subscriber(self.topic, CompressedImage, self.image_callback)
        #function which handles saving files
        self.i=0
        rospy.Timer(rospy.Duration(self.frequency), self.save_callback)
        

    def image_callback(self, ros_image):
        try:
            #for other image formats then rgb change second paramiter "bgr8"
            self.image = bridge.compressed_imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError as e:
            print(e)

    def save_callback(self, timer):
        #incrementation for unique names, and than saving in given path
        self.i=self.i+1
        cv2.imwrite(self.path.format(self.i), self.image)

    def argument_reader(self):
        #reading input arguments and assigning their's values to "self.""
        try:
            arguments, values = getopt.getopt(self.argument_list, self.short_options, self.long_options)
        except getopt.error as err:
            print (str(err))
            sys.exit(2)
        else:
            for current_argument, current_value in arguments:
                if current_argument in ("-h", "--help"):
                    print("--path= to specify path")
                    print("--topic= to specify subsribed topic")
                    print("--frequency= to specify frequency")
                elif current_argument in ("-p", "--path"):
                    print(current_value)
                    self.path = current_value
                    self.path += "/image{}.jpg"
                elif current_argument in ("-t", "--topic"):
                    print(current_value)
                    self.topic = current_value
                elif current_argument in ("-f", "--frequency"):
                    self.frequency = current_value

if __name__ == '__main__':
    try:
        image_saver()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")