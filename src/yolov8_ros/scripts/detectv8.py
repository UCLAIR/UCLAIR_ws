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
        self.sub = rospy.Subscriber("/webcam/image_raw", Image, self.get_image)
        self.pred_pub = rospy.Publisher('publisss', BoundingBoxes, queue_size = 10)

    def get_image(self, data):
        bridge = CvBridge()
        self.im = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        self.yolo(self.im)
        
        BB = BoundingBoxes()
        BB.header = data.header
        BB.image_header = data.header
        
        for r in res:
            x = 0
            for xyxy in r.boxes.xyxy :
                bb = BoundingBox()
                bb.xmin = int(xyxy[0])
                bb.ymin = int(xyxy[1])
                bb.xmax = int(xyxy[2])
                bb.ymax = int(xyxy[3])
                bb.conf = int(r.boxes.conf[x])
                cls = r.boxes.cls[x]
                bb.Class = model.names[int(cls)]
                x = x + 1
                BB.bounding_boxes.append(bb)
                
        self.pred_pub.publish(BB)

                

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




    


