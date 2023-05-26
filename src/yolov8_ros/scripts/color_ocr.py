#!/usr/bin/env python3

import rospy
import cv2
from detection_msgs.msg import BoundingBox, BoundingBoxes, DetectionBB
from alphanumeric_detectionv2 import alphanumeric_detection2
from colour_detectionv2 import color_detection
from sensor_msgs.msg import Image
import pandas as pd
from getpass import getuser
from cv_bridge import CvBridge



class OCR_COLOUR:
    def __init__(self):
        self.image = Image()
        self.picture_number = 1
        self.pred_pub = rospy.Publisher('BoundingBoxes', BoundingBoxes, queue_size = 10)
        self.dataframe = pd.DataFrame()
        self.yolo_subscriber = rospy.Subscriber('YOLO_results', DetectionBB, self.publish)
        
        
        
    def publish(self, YOLO):
        bridge = CvBridge()
        self.image = bridge.imgmsg_to_cv2(YOLO.image, desired_encoding="bgr8")

        
        BB = BoundingBoxes()
        bb = BoundingBox()
        
        bb.Class = YOLO.Class
        bb.probability = bb.probability
        bb.xmin = YOLO.xmin
        bb.ymin = YOLO.ymin
        bb.xmax = YOLO.xmax
        bb.ymax = YOLO.ymax
        bb.long = YOLO.long
        bb.lat =  YOLO.lat
        bb.xDISTANCE = YOLO.xDISTANCE
        bb.yDISTANCE = YOLO.yDISTANCE
        
        [bb.color_shape, bb.color_char] = color_detection(self.image[bb.ymin:bb.ymax,bb.xmin:bb.xmax])
                
        try:
            bb.character = alphanumeric_detection2(self.image[bb.ymin:bb.ymax,bb.xmin:bb.xmax])
        except:
            pass
        
        BB.bounding_boxes.append(bb)
        
        self.pred_pub.publish(BB)
        
if __name__ == "__main__":
    rospy.init_node('OCR_Colour')
    
    RESULTS = OCR_COLOUR()
    rospy.spin()