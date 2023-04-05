#!/usr/bin/env python3

# import subprocess

# # Define the command to run
# command = ["rosrun", "payload_dropping", "payload_dropping_node.py"]

# # Run the command using subprocess
# process = subprocess.Popen(command)

# # Wait for the command to complete
# process.wait()

import rospy
from std_msgs.msg import Bool

def callback(data):
    rospy.loginfo(data.data)
    
def listener():

    rospy.init_node('listener')

    rospy.Subscriber("message", Bool, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()