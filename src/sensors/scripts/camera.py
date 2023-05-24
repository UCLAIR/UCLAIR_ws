#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge,CvBridgeError
from camera2 import RTSPVideoWriterObject




def talker():

	rtsp_stream_link = "rtsp://192.168.144.25:8554/main.264"
	video_stream_widget = RTSPVideoWriterObject(rtsp_stream_link)
	bridge = CvBridge()

	
	pub = rospy.Publisher('/camera_raw',Image,queue_size = 1)
	rospy.init_node('webcam',anonymous = False)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		rate.sleep()
		#video_stream_widget.show_frame()
		ret,frame = video_stream_widget.status, video_stream_widget.frame
		frame = frame[0:720, 280:1000]
		if not ret:
			break
		msg = bridge.cv2_to_imgmsg(frame,"bgr8")
		pub.publish(msg)
		
		if cv2.waitKey(1) == ord('q'):
			break
		
		if rospy.is_shutdown():
			frame.release()


if __name__ == '__main__':
	
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
