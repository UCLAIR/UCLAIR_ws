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
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import TerrainReport
from std_msgs.msg import Float32, Float64
from alphanumeric_detectionv2 import alphanumeric_detection2
from colour_detectionv2 import color_detection
from getpass import getuser


class Yolov8:
    def __init__(self):
        self.source = rospy.get_param("~source")
        self.weights = rospy.get_param("~weights")
        self.model = YOLO(f'/home/{getuser()}/UCLAIR_ws/best.pt')
        self.model = YOLO('yolov8n.pt')
        self.current_global_position = NavSatFix() # Latitude, Longitude, WGS-84
        self.longitude = Float64()
        self.latitude = Float64()
        self.altitude = Float32()
        # Subcribing the global_position/global topic to know the global location (GPS) of the UAV
        # The altitude is WGS84 Ellipsoid
        
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
        
        
        self.sub = rospy.Subscriber(self.source,Image, self.get_image)
        self.pred_pub = rospy.Publisher('BoundingBoxes', BoundingBoxes, queue_size = 10)
        
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

    def get_image(self, data):
        bridge = CvBridge()
        self.image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        self.yolo(self.image)
        
        BB = BoundingBoxes()
        BB.header = data.header
        
        for r in self.results:
            x = 0
            for xyxy in r.boxes.xyxy :
                bb = BoundingBox()
                bb.xmin = int(xyxy[0])
                bb.ymin = int(xyxy[1])
                bb.xmax = int(xyxy[2])
                bb.ymax = int(xyxy[3])
                
                try:
                    [bb.long, bb.lat, bb.xDISTANCE, bb.yDISTANCE] = self.localisation(bb.xmin,bb.ymin,bb.xmax,bb.ymax,self.longitude,self.latitude,self.altitude)
                except:
                    [bb.long, bb.lat, bb.xDISTANCE, bb.yDISTANCE] = self.localisation(bb.xmin,bb.ymin,bb.xmax,bb.ymax,5,5,5)
                
                [bb.color_shape, bb.color_char] = color_detection(self.image[bb.ymin:bb.ymax,bb.xmin:bb.xmax])
                bb.character = alphanumeric_detection2(self.image[bb.ymin:bb.ymax,bb.xmin:bb.xmax])
                
                bb.probability = float(r.boxes.conf[x])
                cls = r.boxes.cls[x]
                bb.Class = self.model.names[int(cls)]
                x = x + 1
                BB.bounding_boxes.append(bb)
                
        self.pred_pub.publish(BB)

                

    def yolo(self, video_source):
        if self.source == '/static_image':
            show_results = True
        else:
            show_results = True
        
        self.results = self.model.predict(
            source = video_source,
            conf = 0.1,
            show = show_results
        )

    def localisation(self,x1,y1,x2,y2,long_drone,lat_drone,alt_drone):
        #https://snehilsanyal.github.io/files/paper1.pdf
        
        x_center = ((x1 + x2)/2)
        y_center = ((y1 + y2)/2)
        X = (alt_drone)*(x_center - self.cx)/self.fx
        Y = (alt_drone)*(y_center - self.cy)/self.fy
        Y = -Y
        
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
    rospy.init_node('yolov8')
    
    RESULTS = Yolov8()
    rospy.spin()
    
    #rate = rospy.Rate(10)
        
    #while not rospy.is_shutdown():
    #    pass




    


