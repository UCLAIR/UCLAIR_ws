#!/usr/bin/env
import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import numpy as np
import pandas as pd

global df

def segment(x,y):
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

def callback(data):
    dis = data.ranges
    list = []
    for i in range(len(dis)):
        if dis[i]/dis[i] == 1:
            list.append(dis[i])
        else:
            list.append(30)
    x = np.linspace(data.angle_min,data.angle_max,num=len(list))
    df = segment(x,list)
    rospy.loginfo("Number of Obstacles: %f", len(df))
    rospy.loginfo("-------------------------")
    rospy.loginfo("Angle θ")
    rospy.loginfo(df.θ.to_numpy())
    rospy.loginfo("               ")
    rospy.loginfo("Radius R")
    rospy.loginfo(df.R.to_numpy())
    rospy.loginfo("_________________________________")
    
    
    
    
def listener():
    rospy.init_node('subscribbb')
    rospy.Subscriber('/spur/laser/scan', LaserScan, callback)
    rospy.spin()
    
    
    
if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
    