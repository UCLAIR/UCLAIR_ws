#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class Test:
    
    def __init__(self):
        self.frame = Image()
        
        self.sub = rospy.Subscriber("webcam/image_raw", Image, self.get_image)
    
        
    def get_image(self, data):
        bridge = CvBridge()
        #self.frame = bridge.imgmsg_to_cv2(data,"bgr8")
        self.frame = data.header
        #rospy.loginfo(self.frame)
        #print(data.header)
        # cv2.imshow('frame',frame)
        #cv2.waitKey(17)  # 1 millisecond
        #rospy.sleep(1) # wait for 1 second
    
    

if __name__ == "__main__":
    rospy.init_node('cameraSub')
    
    
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        Te = Test()
        
        rospy.loginfo(Te.frame)
        
        rate.sleep()
