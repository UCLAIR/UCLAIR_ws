#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO
from sensor_msgs.msg import Image, CompressedImage, NavSatFix
from detection_msgs.msg import BoundingBox, BoundingBoxes
from geographic_msgs.msg import GeoPoseStamped, GeoPoint
import math
from pygeodesy.geoids import GeoidPGM



class Yolov8:
    def __init__(self):
        self.model = YOLO('yolov8n.pt')
        self.current_global_position = NavSatFix() # Latitude, Longitude, WGS-84

        # Subcribing the global_position/global topic to know the global location (GPS) of the UAV
        # The altitude is WGS84 Ellipsoid
        self.current_global_position_sub = rospy.Subscriber(
            name="mavros/global_position/global",
            data_class=NavSatFix,
            queue_size=10,
            callback=self.current_global_position_cb
        )
        self.sub = rospy.Subscriber("camera_raw",Image, self.get_image)
        self.pred_pub = rospy.Publisher('publisss', BoundingBoxes, queue_size = 10)
        
        self.fx = 347.9976
        self.fy = 347.9976
        self.cx = 320
        self.cy = 320

    def get_image(self, data):
        bridge = CvBridge()
        self.im = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        self.yolo(self.im)
        
        BB = BoundingBoxes()
        BB.header = data.header
        BB.image_header = data.header
        
        for r in self.res:
            x = 0
            for xyxy in r.boxes.xyxy :
                bb = BoundingBox()
                bb.xmin = int(xyxy[0])
                bb.ymin = int(xyxy[1])
                bb.xmax = int(xyxy[2])
                bb.ymax = int(xyxy[3])
                bb.long, bb.lat = self.localisation(bb.xmin,bb.ymin,bb.xmax,bb.ymax)
                bb.probability = float(r.boxes.conf[x])
                cls = r.boxes.cls[x]
                bb.Class = self.model.names[int(cls)]
                x = x + 1
                BB.bounding_boxes.append(bb)
                
        self.pred_pub.publish(BB)

                

    def yolo(self,x):
        #model = YOLO('yolov8n.pt')

        self.res = self.model.predict(
            source = x,
            conf = 0.25,
            show = True
        )

    def localisation(self,x1,y1,x2,y2,long_drone,lat_drone,alt_drone):
        #https://snehilsanyal.github.io/files/paper1.pdf
        x_center = int((xyxy[0] + xyxy[2])/2)
        y_center = int((xyxy[1] + xyxy[3])/2)
        X = alt_drone*(x_center - cx)/fx
        Y = alt_drone*(y_center - cy)/fy
        
        b = math.atan(abs(X/Y))
        s = math.sqrt(math.pow(X,2)+math.pow(Y,2))
        dX = s*math.sin(b)
        dY = s*math.cos(b)

        deltalong = dX/(11320*math.cos(lat_drone))
        deltalat = dY/(110540)

        long = long_drone + deltalong
        lat = lat_drone + deltalat

        return long, lat


if __name__ == "__main__":
    rospy.init_node('yolov8')
    
    RESULTS = Yolov8()
    rospy.spin()




    


