#!/usr/bin/env python3
import rospy
from detection_msgs.msg import BoundingBox, BoundingBoxes
import pandas as pd
import math

class Dataf:
    def __init__(self):
        self.source = rospy.get_param("~source")

        self.column_names=["Class","probability","long","lat","Distance","character","color_shape","color_char"]
        self.dataf=pd.DataFrame(columns=self.column_names)

        self.current_altitude_sub = rospy.Subscriber(
            name="BoundingBoxes",
            data_class=BoundingBoxes,
            callback=self.data_sub_cb
        )

    def data_sub_cb(self, msg):
        for bb in msg.bounding_boxes:
            dist=math.sqrt(pow(bb.xDISTANCE,2)+pow(bb.yDISTANCE,2))
            row_dict = {"Class":[bb.Class],"probability":[bb.probability],"long":[bb.long],"lat":[bb.lat],"Distance":[dist],"character":[bb.character],"color_shape":[bb.color_shape],"color_char":[bb.color_char]}
            # Check if row with same combination of class, character, color_shape, and color_char exists in the dataframe
            if not any(value == "null" for value in [bb.character, bb.color_shape, bb.color_char]):
                if self.dataf.empty:
                    self.dataf = pd.DataFrame(row_dict)
                else:
                    existing_rows = self.dataf.loc[(self.dataf["Class"] == bb.Class) & (self.dataf["character"] == bb.character) & (self.dataf["color_shape"] == bb.color_shape) & (self.dataf["color_char"] == bb.color_char)]
                    if existing_rows.empty or dist < existing_rows["Distance"].min():
                        self.dataf = self.dataf.append(row_dict, ignore_index=True)
            print(self.dataf.to_string())

if __name__ == "__main__":
    rospy.init_node('Finaldatabase')
    RESULTS=Dataf()
    rospy.spin()
    RESULTS.dataf.to_csv('/home/jetson/UCLAIR_ws/src/yolov8_ros/database/finaldata.csv')
    



    


