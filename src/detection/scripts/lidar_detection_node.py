#!/usr/bin/env
import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)
import rospy
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import LaserScan
import numpy as np
import pandas as pd

class LidarDetection:

    def __init__(self):
        self.obstacles = pd.DataFrame(columns=list('θR'))

        self.obstacles_r_pub = rospy.Publisher(
            name="lidar/obstacles_r",
            data_class=Float64MultiArray,
            queue_size=10
        )

        self.obstacles_theta_pub = rospy.Publisher(
            name="lidar/obstacles_theta",
            data_class=Float64MultiArray,
            queue_size=10
        )

        self.lidar_sub = rospy.Subscriber(
            name="/spur/laser/scan",
            data_class=LaserScan,
            callback=self.lidar_sub_cb
        )

    def lidar_sub_cb(self, data):
        dis = data.ranges
        list = []
        for i in range(len(dis)):
            if dis[i]/dis[i] == 1:
                list.append(dis[i])
            else:
                list.append(30)
        x = np.linspace(data.angle_min,data.angle_max,num=len(list))
        self.obstacles = self.lidar_segmentation(x,list)
        # rospy.loginfo("Number of Obstacles: %f", len(self.obstacles))
        # rospy.loginfo("-------------------------")
        # rospy.loginfo("Angle θ")
        # rospy.loginfo(self.obstacles.θ.to_numpy())
        # rospy.loginfo("               ")
        # rospy.loginfo("Radius R")
        # rospy.loginfo(self.obstacles.R.to_numpy())
        # rospy.loginfo("_________________________________")

    def lidar_segmentation(self, x, y):
        d=np.array(y)
        dif=np.absolute(np.diff(d))
        df = pd.DataFrame(columns=list('θR'))
        cc = 0
        for i in range(len(dif)):
            if dif[i]>1 or i==len(dif)-1:
                df2 = pd.DataFrame([[np.average(x[cc:i+1]), np.average(y[cc:i+1])]], columns=list('θR'))
                df = df.append(df2, ignore_index=True)
                cc=i+1 
        df = df.drop(df[df.R == 30].index)
        return df

    def publish_float64multiarray_data(self, d):
        publishing_data = Float64MultiArray()

        publishing_data.data = d

        return publishing_data


# global df

# def segment(x,y):
#     d=np.array(y)
#     dif=np.absolute(np.diff(d))
#     df = pd.DataFrame(columns=list('θR'))
#     cc = 0
#     for i in range(len(dif)):
#         if dif[i]>1 or i==len(dif)-1:
#             df2 = pd.DataFrame([[np.average(x[cc:i+1]), np.average(y[cc:i+1])]], columns=list('θR'))
#             df = df.append(df2, ignore_index=True)
#             cc=i+1 
#     df = df.drop(df[df.R == 30].index)
#     return df

# def callback(data):
#     dis = data.ranges
#     list = []
#     for i in range(len(dis)):
#         if dis[i]/dis[i] == 1:
#             list.append(dis[i])
#         else:
#             list.append(30)
#     x = np.linspace(data.angle_min,data.angle_max,num=len(list))
#     df = segment(x,list)
#     rospy.loginfo("Number of Obstacles: %f", len(df))
#     rospy.loginfo("-------------------------")
#     rospy.loginfo("Angle θ")
#     rospy.loginfo(df.θ.to_numpy())
#     rospy.loginfo("               ")
#     rospy.loginfo("Radius R")
#     rospy.loginfo(df.R.to_numpy())
#     rospy.loginfo("_________________________________")
    
    
    
    
# def listener():
#     rospy.init_node('subscribbb')
#     rospy.Subscriber('/spur/laser/scan', LaserScan, callback)
#     rospy.spin()
    
    
    
if __name__ == '__main__':
    
    try:

        rospy.init_node("obstacle_detection_node")

        lidar_detection = LidarDetection()

        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():

            rospy.loginfo("Number of Obstacles: %f", len(lidar_detection.obstacles))
            rospy.loginfo("-------------------------")
            rospy.loginfo("Angle θ")
            rospy.loginfo(lidar_detection.obstacles.θ.to_numpy())
            rospy.loginfo("               ")
            rospy.loginfo("Radius R")
            rospy.loginfo(lidar_detection.obstacles.R.to_numpy())
            rospy.loginfo("_________________________________")
            
            radii_list = lidar_detection.obstacles['R'].values.tolist()
            theta_list = lidar_detection.obstacles['θ'].values.tolist()
            publishable_radii_list = lidar_detection.publish_float64multiarray_data(radii_list)
            publishable_theta_list = lidar_detection.publish_float64multiarray_data(theta_list)

            lidar_detection.obstacles_r_pub.publish(publishable_radii_list)
            lidar_detection.obstacles_theta_pub.publish(publishable_theta_list)
    
    except KeyboardInterrupt:
        exit()



    