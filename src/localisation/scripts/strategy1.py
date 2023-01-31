#!/usr/bin/env python3
#import getBottlesData

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
global frame

def getCamera(data):
    global frame
    frame = bridge.imgmsg_to_cv2(data,"bgr8")
    cv2.waitKey(17)  # 1 millisecond
    #rospy.sleep(1) # wait for 1 second
    

def strategy1():

    # INPUT BOTTLES DATA.
    #letter_list,color_list,shape_list = getBottlesData()
    #print (letter_list)

    # GET FRAME FROM CAMERA
    
    cv2.imshow('frame',frame)
    
    # Wait for a key press
    while(True):
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # GET CORD OF DRONE WHEN IMAGE WAS TAKEN


    # INPUT: FRAME 
    # OUTPUT: BOOL (DETECTED TRUE OR FALSE)



    # IF DETECTED
    # GET COORD
        # GET MIDPOINT OF DETECTED THING


    # CROP AND ZOOM FRAME

    # RUN SHAPE
    # RUN CHAR
    # RUN COLOR

    # STORE SHAPE, CHAR, COLOR, COORD
    pass

if __name__ == '__main__':

    rospy.init_node('cameraSub')
    bridge = CvBridge()
    rospy.Subscriber("/camera", Image, getCamera)
    rospy.spin()
    
    strategy1()
    cv2.destroyAllWindows()


