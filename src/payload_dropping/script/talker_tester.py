#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool

def talker():
    pub = rospy.Publisher('message', Bool, queue_size=10)
    rospy.init_node('chatter_node')

    execute_dropping = True # from the navigation code
    counter = 0

    while not rospy.is_shutdown():

        while counter < 1 and execute_dropping:
            rospy.loginfo("Dropping bottle")
            pub.publish(True)

            counter += 1

        if execute_dropping == False:
            counter = 0

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass