#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge



def callback_frame(data):
    global frame
    frame = bridge.imgmsg_to_cv2(data,"bgr8")
    cv2.imshow('frame',frame)
    cv2.waitKey(1)  # 1 millisecond

if __name__ == "__main__":
    rospy.init_node('printer')
    bridge = CvBridge()
    rospy.Subscriber("/camera", Image, callback_frame)
    rospy.spin()
    #pub = rospy.Publisher('/yolov5', PoseStamped, queue_size=10)


