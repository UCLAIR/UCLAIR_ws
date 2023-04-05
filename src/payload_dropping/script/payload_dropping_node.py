#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool


class PayloadDropping:
    def __init__(self):
        self.execute_drop = False
        
        # ROS Publishers
        self.bottle_dropping_execution_pub = rospy.Publisher(
            name="/servo_motor_control",
            data_class=Bool,
            queue_size=10
        )

        # ROS Subscribers
        self.execute_drop_sub = rospy.Subscriber(
            name="execute_drop_bottle",
            data_class=Bool,
            callback=self.execute_drop_sub_cb
        )


    def execute_drop_sub_cb(self, msg):
        self.execute_drop = msg.data


if __name__ == "__main__":
    try:
        rospy.init_node("payload_dropping_node")
        
        counter = 0

        payload_dropping = PayloadDropping()

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            while counter < 1 and payload_dropping.execute_drop:

                rate.sleep()
                
                rospy.loginfo("Dropping bottle")

                payload_dropping.bottle_dropping_execution_pub.publish(True)
                
                counter += 1

            if payload_dropping.execute_drop == False:
                counter = 0

    except rospy.ROSInterruptException:
        pass


        


