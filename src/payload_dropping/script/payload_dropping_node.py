#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool


class PayloadDropping:
    def __init__(self):
        self.execute_drop = Bool
        self.drop_complete = Bool
        
        # ROS Publishers
        self.bottle_dropping_execution_pub = rospy.Publisher(
            name="/servo_motor_control",
            data_class=Bool,
            queue_size=10
        )

        # ROS Subscribers

        self.drop_complete_sub = rospy.Subscriber(
            name="drop_complete",
            data_class=Bool,
            callback=self.drop_complete_sub_cb
        )

    def drop_complete_sub_cb(self, msg):
        self.drop_complete = msg.data


if __name__ == "__main__":
    try:
        rospy.init_node("payload_dropping_node")
        
        counter = 0

        payload_dropping = PayloadDropping()

        while not rospy.is_shutdown():

            while counter < 1 and payload_dropping.execute_drop:
                rospy.loginfo("Dropping bottle")

                payload_dropping.publish(True)
                
                counter += 1

            if payload_dropping.execute_drop == False:
                counter = 0

    except rospy.ROSInterruptException:
        pass


        


