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

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener')

    rospy.Subscriber("drop_complete", Bool, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()