#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge,CvBridgeError
import getpass




def talker():

	cap = cv2.imread(f'/home/{getpass.getuser()}/UCLAIR_ws/src/sensors/images/K6.png')
	bridge = CvBridge()
	
	pub = rospy.Publisher('/static_image',Image,queue_size = 1)
	rospy.init_node('static_image_publisher',anonymous = False)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		frame = cap
		msg = bridge.cv2_to_imgmsg(frame,"bgr8")
		pub.publish(msg)
		
		# if cv2.waitKey(1) == ord('q'):
		# 	break
		
		# if rospy.is_shutdown():
		# 	cap.release()


if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
