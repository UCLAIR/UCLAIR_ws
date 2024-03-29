#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge,CvBridgeError




def talker():

	cap = cv2.VideoCapture('rtsp://192.168.144.25:8554/main.264')
	print(cap.isOpened())
	bridge = CvBridge()
 
	frame_width = int(cap.get(3))
	frame_height = int(cap.get(4))
 
	out = cv2.VideoWriter('recording.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 10, (frame_width,frame_height))

    
	pub = rospy.Publisher('/camera_raw',Image,queue_size = 1)
	rospy.init_node('webcam',anonymous = False)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		ret,frame = cap.read()
  
		if ret:
			out.write(frame)
   
		if not ret:
			break
		msg = bridge.cv2_to_imgmsg(frame,"bgr8")
		pub.publish(msg)
		
		if cv2.waitKey(1) == ord('q'):
			break
		
		if rospy.is_shutdown():
			cap.release()
			out.release()


if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
