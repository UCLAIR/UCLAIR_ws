#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO
from sensor_msgs.msg import Image, CompressedImage, NavSatFix
from detection_msgs.msg import BoundingBox, BoundingBoxes, DetectionBB
from geographic_msgs.msg import GeoPoseStamped, GeoPoint
import math
from pygeodesy.geoids import GeoidPGM
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import TerrainReport
from std_msgs.msg import Float32, Float64
from getpass import getuser
import glob, os
import numpy as np


def splitter():
    file = open(f"/home/{getuser()}/UCLAIR_ws/src/yolov8_ros/database/BOTTLESDATA.txt", "r")
    data = file.read()
    
    shapes = []
    
    for line in data.split("\n"):
        columns = line.split(",")
        try:
            shapes.append(columns[2])
        except:
            pass
    return shapes

class Yolov8:
    def __init__(self):
        self.source = rospy.get_param("~source")
        self.weights = rospy.get_param("~weights")
        self.model = YOLO(f'/home/{getuser()}/UCLAIR_ws/best.pt')
        self.current_global_position = NavSatFix() # Latitude, Longitude, WGS-84
        self.longitude = Float64()
        self.latitude = Float64()
        self.altitude = Float32()
        self.current_compass = Float64()
        # Subcribing the global_position/global topic to know the global location (GPS) of the UAV
        # The altitude is WGS84 Ellipsoid
        
        try:
            self.shapes = splitter()
        except:
            pass
        
        self.current_global_position_sub = rospy.Subscriber(
            name="mavros/global_position/global",
            data_class=NavSatFix,
            queue_size=10,
            callback=self.current_global_position_cb
        )

        self.current_altitude_sub = rospy.Subscriber(
            name="mavros/terrain/report",
            data_class=TerrainReport,
            callback=self.current_terrain_report_sub_cb
        )
        
        self.current_compass = rospy.Subscriber(
            name="mavros/global_position/compass_hdg",
            data_class=Float64,
            callback=self.current_compass_heading_cb
        )
        
        self.sub = rospy.Subscriber(self.source,Image, self.get_image)
        self.pred_pub = rospy.Publisher('YOLO_results', DetectionBB, queue_size = 10)
        
        #https://shop.siyi.biz/products/zr10?VariantsId=10623
        #Focal Length: 5.15±5% to 47.38±5% mm
        self.fx = 515
        self.fy = 515
        self.cx = 320
        self.cy = 320

    def current_global_position_cb(self, msg):
        self.longitude = msg.longitude
        self.latitude = msg.latitude

    def current_terrain_report_sub_cb(self, msg):
        self.altitude = msg.current_height
        
    def current_compass_heading_cb(self, msg):
        self.current_compass = msg

    def get_image(self, data):
        bridge = CvBridge()
        self.image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        self.yolo(self.image)
        
        
        for r in self.results:
            x = 0
            for xyxy in r.boxes.xyxy :
                bb = DetectionBB()
                bb.xmin = int(xyxy[0])
                bb.ymin = int(xyxy[1])
                bb.xmax = int(xyxy[2])
                bb.ymax = int(xyxy[3])
                
                try:
                    [bb.long, bb.lat, bb.xDISTANCE, bb.yDISTANCE] = self.localisation(bb.xmin,bb.ymin,bb.xmax,bb.ymax,self.longitude,self.latitude,self.altitude)
                except:
                    [bb.long, bb.lat, bb.xDISTANCE, bb.yDISTANCE] = self.localisation(bb.xmin,bb.ymin,bb.xmax,bb.ymax,5,5,5)
                    
                
                bb.image = data
                bb.probability = float(r.boxes.conf[x])
                cls = r.boxes.cls[x]
                bb.Class = self.model.names[int(cls)]
                x = x + 1
                
                if bb.Class in self.shapes:
                    self.pred_pub.publish(bb)
                
                            
            
                

                

    def yolo(self, video_source):
        if self.source == '/static_image':
            show_results = True
        else:
            show_results = False
        
        self.results = self.model.predict(
            source = video_source,
            conf = 0.8,
            show = show_results
        )

    def localisation(self,x1,y1,x2,y2,long_drone,lat_drone,alt_drone):
        #https://snehilsanyal.github.io/files/paper1.pdf
        
        x_center = ((x1 + x2)/2)
        y_center = ((y1 + y2)/2)
        X = (alt_drone)*(x_center - self.cx)/self.fx
        Y = (alt_drone)*(y_center - self.cy)/self.fy
        Y = -Y
        
        bear = self.current_compass
        XY = np.array([[X],[Y]])
        try:
            bear = -bear*(180/math.pi)
            trans = np.array([[math.cos(bear), -math.sin(bear)], [math.sin(bear), math.cos(bear)]])
            new = np.matmul(trans, XY)
        except:
            bear = 0
            trans = np.array([[math.cos(bear), -math.sin(bear)], [math.sin(bear), math.cos(bear)]])
            new = np.matmul(trans, XY)
            
        X = float(new[0])
        Y = float(new[1])
        
            
        
        
        
        b = math.atan(abs(X/Y))
        s = math.sqrt(math.pow(X,2)+math.pow(Y,2))
        dX = s*math.sin(b)
        dY = s*math.cos(b)

        # deltalong = dX/(11320*math.cos(lat_drone))
        # deltalat = dY/(110540)

        # long = long_drone + deltalong
        # lat = lat_drone + deltalat
        
        # return long, lat, X, Y
        
        ########################################################################
        #https://dronekit-python.readthedocs.io/en/latest/examples/mission_basic.html
        
        earth_radius = 6378137
        
        dlat = Y/earth_radius
        dlon = X/(earth_radius*math.cos(math.pi*lat_drone/180))
        
        newlat = lat_drone + (dlat * 180/math.pi)
        newlon = long_drone + (dlon * 180/math.pi)
        
        return newlon, newlat, X, Y      
    


if __name__ == "__main__":
    files = glob.glob(f'/home/{getuser()}/UCLAIR_ws/src/yolov8_ros/database/images/*')
    for f in files:
        os.remove(f)
    rospy.init_node('yolov8')
    
    RESULTS = Yolov8()
    rospy.spin()
    
    #rate = rospy.Rate(10)
        
    #while not rospy.is_shutdown():
    #    pass