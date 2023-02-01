#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import time
import numpy as np

class GUI:
    def __init__(self):

        self.bridge = CvBridge()
        self.frame = np.ndarray([])

        # Subscribers
        self.camera_sub = rospy.Subscriber(
            name="/camera",
            data_class=Image,
            callback=self.camera_sub_cb
        )

    def say_hello(self):
        print("Hello")
    
    def camera_sub_cb(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.waitKey(17)
        rospy.sleep(1) # wait for 1 second
        cv2.imshow('frame',self.frame)
        #rospy.loginfo(type(self.bridge.imgmsg_to_cv2(msg,"bgr8")))

    


if __name__ == "__main__":

    rospy.init_node("Controller")

    # rate = rospy.Rate(10)
    # gui = GUI()
    # rospy.spin()

    while (not rospy.is_shutdown()):

        gui = GUI()

        # rospy.loginfo(gui.frame)

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

