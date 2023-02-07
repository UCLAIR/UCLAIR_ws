#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO
from sensor_msgs.msg import Image, CompressedImage
from detection_msgs.msg import BoundingBox, BoundingBoxes




class Yolov8:
    def __init__(self):
        self.model = YOLO('yolov8n.pt')
        self.sub = rospy.Subscriber("camera_raw", Image, self.get_image)

    def get_image(self, data):
        bridge = CvBridge()
        self.im = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        self.yolo(self.im)

    def yolo(self,x):
        #model = YOLO('yolov8n.pt')

        res = self.model.predict(
            source = x,
            conf = 0.25,
            show = True
        )

if __name__ == "__main__":
    rospy.init_node('yolov8')
    
    RESULTS = Yolov8()
    rospy.spin()




    


