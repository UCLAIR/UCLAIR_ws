#!/usr/bin/env python3

import rospy
import cv2
from detection_msgs.msg import BoundingBox, BoundingBoxes
from alphanumeric_detectionv2 import alphanumeric_detection2
from colour_detectionv2 import color_detection
import pandas as pd
from getpass import getuser



class OCR_COLOUR:
    def __init__(self):
        self.image = None
        self.picture_number = 1
        self.pred_pub = rospy.Publisher('BoundingBoxes', BoundingBoxes, queue_size = 10)
        self.dataframe = pd.DataFrame()
        
        
        
    def publish(self):
        self.dataframe = pd.read_csv(f'/home/{getuser()}/UCLAIR_ws/src/yolov8_ros/database/yolo_database.csv')
        current_pic = self.dataframe.loc[self.dataframe["ID"] == self.picture_number]
        self.image = None
        self.image = cv2.imread(f'/home/{getuser()}/UCLAIR_ws/src/yolov8_ros/database/images/{self.picture_number}.png')

        
        BB = BoundingBoxes()
        
        check = False
        for i, row in current_pic.iterrows() :
                bb = BoundingBox()
                bb.xmin = current_pic.loc[i]["xmin"]
                bb.ymin = current_pic.loc[i]["ymin"]
                bb.xmax = current_pic.loc[i]["xmax"]
                bb.ymax = current_pic.loc[i]["ymax"]

                bb.long = current_pic.loc[i]["long"]
                bb.lat = current_pic.loc[i]["lat"]
                bb. xDISTANCE = current_pic.loc[i]["xDISTANCE"]
                bb.yDISTANCE = current_pic.loc[i]["yDISTANCE"]
                
                try:
                    [bb.color_shape, bb.color_char] = color_detection(self.image[bb.ymin:bb.ymax,bb.xmin:bb.xmax])
                except:
                    pass
                    
                try:
                    bb.character = alphanumeric_detection2(self.image[bb.ymin:bb.ymax,bb.xmin:bb.xmax])
                except:
                    pass
            
                BB.bounding_boxes.append(bb)
                check = True
        
        
        if check and self.image != None:
            self.picture_number = self.picture_number + 1
            self.pred_pub.publish(BB)
        
if __name__ == "__main__":
    rospy.init_node('OCR_Colour')
    
    RESULTS = OCR_COLOUR()
    
    while not rospy.is_shutdown():
        try:
            RESULTS.publish()
        except:
            pass