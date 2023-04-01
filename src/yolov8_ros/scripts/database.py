#!/usr/bin/env python3
import rospy
from detection_msgs.msg import BoundingBox, BoundingBoxes
import pandas as pd

class Dataf:
    def __init__(self):
        self.source = rospy.get_param("~source")

        self.column_names=["Class","probability","long","lat","xDISTANCE","yDISTANCE","character","color_shape","color_char"]
        self.dataf=pd.DataFrame(columns=self.column_names)

        self.current_altitude_sub = rospy.Subscriber(
            name="BoundingBoxes",
            data_class=BoundingBoxes,
            callback=self.data_sub_cb
        )

    def data_sub_cb(self, msg):
        for bb in msg.bounding_boxes:
            datafadd=pd.DataFrame({"Class":[bb.Class],"probability":[bb.probability],"long":[bb.long],"lat":[bb.lat],"xDISTANCE":[bb.xDISTANCE],"yDISTANCE":[bb.yDISTANCE],"character":[bb.character],"color_shape":[bb.color_shape],"color_char":[bb.color_char]})        
            self.dataf=self.dataf.append(datafadd)
            print(self.dataf.to_string())

if __name__ == "__main__":
    rospy.init_node('Dataf')
    RESULTS=Dataf()
    rospy.spin()
    RESULTS.dataf.to_csv('/home/jetson/UCLAIR_ws/src/yolov8_ros/database/data.csv')
    



    


