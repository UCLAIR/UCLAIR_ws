#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import time

frame = []

def callback_frame(data):
    global frame
    frame = bridge.imgmsg_to_cv2(data,"bgr8")
    # cv2.imshow('frame',frame)
    cv2.waitKey(17)  # 1 millisecond
    rospy.sleep(2) # wait for 1 second

def strategy2():

    if len(frame) != 0:
        cv2.imshow('frame',frame)


if __name__ == '__main__':
    rospy.init_node('cameraSub')
    bridge = CvBridge()
    rospy.Subscriber("/camera", Image, callback_frame)
    #rospy.spin()
    
    while not rospy.is_shutdown():
        strategy2()
