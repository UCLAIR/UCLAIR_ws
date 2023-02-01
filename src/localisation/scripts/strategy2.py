#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import time
import sys
import numpy as np

class image_converter:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera",Image,self.callback)
       
    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)
    

def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)










#  # Subscribers
#         self.camera_sub = rospy.Subscriber(
#             name="/camera",
#             data_class=Image,
#             callback=self.camera_sub_cb
#         )


#     def camera_sub_cb(self, msg):
#         self.frame = msg
#         # rospy.loginfo(type(self.frame))
#         # bridge = CvBridge()
#         # rospy.loginfo(type(bridge.imgmsg_to_cv2(msg, "bgr8")))
#         # # cv2.waitKey(10)

# if __name__ == "__main__":

#     rospy.init_node("Controller")

#     gui = GUI()
#     bridge = CvBridge()
#     # cv2.imshow('frame', gui.frame)
#     # rospy.spin()


#     rate = rospy.Rate(10)
#     # gui = GUI()

#     while (not rospy.is_shutdown()):

#         feed = bridge.imgmsg_to_cv2(gui.frame, "bgr8")

#         #rospy.loginfo(gui.frame)
#         cv2.waitKey(17)
#         cv2.imshow('frame', feed)
#         rate.sleep()





















# bridge = CvBridge()

# frame = []

# class GUI(QDialog):

#     first_time = True

#     def __init__(self):
#         if len(frame) != 0:
#             cv2.imshow('frame',frame)

# def callback_frame(data):
#     global frame
#     frame = bridge.imgmsg_to_cv2(data,"bgr8")
#     # cv2.imshow('frame',frame)
#     cv2.waitKey(17)  # 1 millisecond
#     rospy.sleep(2) # wait for 1 second

# if __name__ == '__main__':
#     app = QApplication(sys.argv)
#     window = GUI()
#     rospy.init_node('cameraSub')
#     rospy.Subscriber("/camera", Image, callback_frame)
#     window.show()
#     sys.exit(app.exec_())

