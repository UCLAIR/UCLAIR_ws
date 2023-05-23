import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from camera2 import RTSPVideoWriterObject


def talker():
    rtsp_stream_link = "rtsp://192.168.144.25:8554/main.264"
    video_stream_widget = RTSPVideoWriterObject(rtsp_stream_link)

    colours = {
        'blue': ((90, 50, 70), (128, 255, 255)),
        'red': ((0, 180, 255), (179, 255, 255)),
        'grey': ((0, 0, 0), (178, 53, 185)),
        'green': ((36, 50, 70), (94, 255, 255)),
        'yellow': ((25, 50, 70), (35, 255, 255)),
        'purple': ((20, 0, 0), (179, 255, 80)),
        'orange': ((0, 139, 107), (20, 255, 255)),
        'brown': ((10, 100, 20), (20, 255, 200)),
        'white': ((0, 0, 180), (255, 30, 255)),
        'black': ((0, 0, 0), (180, 255, 30))
    }

    # User choice of color to recognize/filter
    user_input = input("Enter a list of colors separated by commas: ")
    colours_list = user_input.split(',')

    pub = rospy.Publisher('/camera_raw', Image, queue_size=1)
    rospy.init_node('webcam', anonymous=False)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        ret, frame = video_stream_widget.status, video_stream_widget.frame
        if ret:
            # Creating copy of image
            original = frame.copy()

            # Convert to HSV color space
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            midpoints = []

            # Loop through all the selected colors
            for color in colours_list:
                lower, upper = colours[color.strip().lower()]
                # Create a mask for the selected color
                mask = cv2.inRange(frame, np.array(lower), np.array(upper))
                # Morphological operations to remove noise and fill small holes
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
                # Find contours in the mask
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for cnt in contours:
                    # Calculate the center of the contour
                    M = cv2.moments(cnt)
                    if M["m00"] > 0:
                        area = cv2.contourArea(cnt)
                        if area > 1:
                            cX = int(M["m10"] / M["m00"])
                            cY = int(M["m01"] / M["m00"])
                            # Draw the contour on the original image
                            cv2.drawContours(original, [cnt], -1, (255, 0, 0), 2)
                            # Write the color name next to the center of the contour
                            cv2.putText(original, color, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                            midpoints.append(((cX, cY), color))

            if midpoints:
                print(midpoints)

        # Show the final
        cv2.imshow("Result", original)

        if not ret:
            break
        msg = bridge.cv2_to_imgmsg(frame, "bgr8")
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
