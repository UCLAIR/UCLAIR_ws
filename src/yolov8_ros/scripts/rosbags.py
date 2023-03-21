#!/usr/bin/env python3

import rospy
import rosbag
from detection_msgs.msg import BoundingBoxes

class BoundingBoxesRecorder:
    def __init__(self):
        self.bag = rosbag.Bag('bounding_boxes.bag', 'w')
        self.sub = rospy.Subscriber('BoundingBoxes', BoundingBoxes, self.record_bbox)

    def record_bbox(self, msg):
        self.bag.write('BoundingBoxes', msg)

if __name__ == '__main__':
    rospy.init_node('bounding_boxes_recorder')
    recorder = BoundingBoxesRecorder()
    rospy.spin()
