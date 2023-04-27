#!/usr/bin/env python3
import rospy
from detection_msgs.msg import BoundingBox, BoundingBoxes
import pandas as pd
import math
from getpass import getuser


class Dataf:
    def __init__(self):

        self.column_names=["Class","probability","long","lat","Distance","character","color_shape","color_char"]
        self.dataf=pd.DataFrame(columns=self.column_names)
        
        self.bottles_df = pd.read_csv(f'/home/{getuser()}/UCLAIR_ws/src/yolov8_ros/database/BOTTLESDATA.txt', sep=',', header=None, names=['Class', 'character', 'color_shape', 'color_char'])
        self.matches_df=pd.DataFrame(columns=self.column_names)
        self.matching_rows=pd.DataFrame(columns=self.column_names)

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
            

if __name__ == "__main__":
    rospy.init_node('yolo_data_sorter')
    RESULTS=Dataf()
    
    while not rospy.is_shutdown():

        RESULTS.dataf.to_csv(f'/home/{getuser()}/UCLAIR_ws/src/yolov8_ros/database/data.csv')

        for i, row in RESULTS.bottles_df.iterrows():
            # Find the matching row(s) in dataf
            RESULTS.matches_df = RESULTS.dataf.loc[
            (RESULTS.dataf['Class'] == row['Class']) &
            (RESULTS.dataf['character'] == row['character']) &
            (RESULTS.dataf['color_shape'] == row['color_shape']) &
            (RESULTS.dataf['color_char'] == row['color_char'])]
            # If there are no matches, continue to the next row
            if not RESULTS.matches_df.empty:
                #RESULTS.matching_rows=RESULTS.matching_rows.append({'Class': row['Class'], 'character': 'not found', 'color_shape': 'not found', 'color_char': 'not found', 'Distance': 0}, ignore_index=True)
                RESULTS.matching_rows = RESULTS.matching_rows.append(RESULTS.matches_df, ignore_index=True)
                #continue
            # Otherwise, find the row with the minimum Distance value
            #min_dist_row = matches_df.loc[matches_df['Distance'].min()]
        
            # Add the matching row to the new dataframe
            #RESULTS.matching_rows = RESULTS.matching_rows.append(RESULTS.matches_df, ignore_index=True)

        RESULTS.matching_rows.to_csv(f'/home/{getuser()}/UCLAIR_ws/src/yolov8_ros/database/list.csv')





    


