#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
from detection_msgs.msg import BoundingBox, BoundingBoxes
import pandas as pd
import math
from getpass import getuser
import warnings

warnings.simplefilter(action='ignore', category=FutureWarning)


class Dataf:
    def __init__(self):

        self.column_names=["Class","probability","long","lat","Distance","character","color_shape","color_char"]
        self.dataf=pd.DataFrame(columns=self.column_names)
        self.rowdf=pd.DataFrame(columns=self.column_names)
        
        self.bottles_df = pd.read_csv(f'/home/{getuser()}/UCLAIR_ws/src/yolov8_ros/database/BOTTLESDATA.txt', sep=',', header=None, names=['color_shape', 'color_char', 'Class', 'character'])
        self.matches_df=pd.DataFrame(columns=self.column_names)
        self.matching_rows=pd.DataFrame(columns=self.column_names)
        
        self.GPS_drop_1_pub = rospy.Publisher(
            name="GPS_drop_1",
            data_class=Float64MultiArray,
            queue_size=10
        )
        
        self.GPS_drop_2_pub = rospy.Publisher(
            name="GPS_drop_2",
            data_class=Float64MultiArray,
            queue_size=10
        )

        self.GPS_drop_3_pub = rospy.Publisher(
            name="GPS_drop_3",
            data_class=Float64MultiArray,
            queue_size=10
        )

        self.GPS_drop_4_pub = rospy.Publisher(
            name="GPS_drop_4",
            data_class=Float64MultiArray,
            queue_size=10
        )

        self.GPS_drop_5_pub = rospy.Publisher(
            name="GPS_drop_5",
            data_class=Float64MultiArray,
            queue_size=10
        )

        self.current_altitude_sub = rospy.Subscriber(
            name="BoundingBoxes",
            data_class=BoundingBoxes,
            callback=self.data_sub_cb
        )

        # Convert the data into publisherable data for Float64MultiArray()
    def publish_float64multiarray_data(self, d):
        publishing_data = Float64MultiArray()

        publishing_data.data = d

        return publishing_data

    def publish_data(self, i, lat, lon):
        pub_GPS = self.publish_float64multiarray_data([lat, lon])


        if i == 0:
            self.GPS_drop_1_pub.publish(pub_GPS)

        elif i == 1:
            self.GPS_drop_2_pub.publish(pub_GPS)

        elif i == 2:
            self.GPS_drop_3_pub.publish(pub_GPS)

        elif i == 3:
            self.GPS_drop_4_pub.publish(pub_GPS)
        
        elif i == 4:
            self.GPS_drop_5_pub.publish(pub_GPS)

    def data_sub_cb(self, msg):
        for bb in msg.bounding_boxes:
            dist=math.sqrt(pow(bb.xDISTANCE,2)+pow(bb.yDISTANCE,2))
            row_dict = {"Class":bb.Class,"probability":bb.probability,"long":bb.long,"lat":bb.lat,"Distance":bb.xDISTANCE,"character":bb.character,"color_shape":bb.color_shape,"color_char":bb.color_char}
            # Check if row with same combination of class, character, color_shape, and color_char exists in the dataframe
            if not any(value == "null" for value in [bb.character, bb.color_shape, bb.color_char]):
                if self.dataf.empty:
                    self.dataf = pd.DataFrame(row_dict, index=[0])
                else:
                    existing_rows = self.dataf.loc[(self.dataf["Class"] == bb.Class) & (self.dataf["character"] == bb.character) & (self.dataf["color_shape"] == bb.color_shape) & (self.dataf["color_char"] == bb.color_char)]
                    if existing_rows.empty:
                        self.dataf = self.dataf.append(row_dict, ignore_index=True)
                    elif dist < existing_rows["Distance"].min():
                        existing_rows_idx = existing_rows['Distance'].idxmin()
                        self.dataf = self.dataf.drop(existing_rows_idx)
                        self.dataf = self.dataf.append(row_dict, ignore_index=True)
                        
                        
                        
            

if __name__ == "__main__":
    rospy.init_node('yolo_data_sorter')
    RESULTS=Dataf()
    
    while not rospy.is_shutdown():

        RESULTS.dataf.to_csv(f'/home/{getuser()}/UCLAIR_ws/src/yolov8_ros/database/data.csv')
        RESULTS.matching_rows = pd.DataFrame(columns=RESULTS.column_names)

        for i, row in RESULTS.bottles_df.iterrows():
            #print(row)
            # Find the matching row(s) in dataf
            RESULTS.matches_df = RESULTS.dataf.loc[(
            (RESULTS.dataf['Class'] == row['Class']) &
            (RESULTS.dataf['character'] == row['character']) &
            (RESULTS.dataf['color_shape'] == row['color_shape']) &
            (RESULTS.dataf['color_char'] == row['color_char']))]

            if RESULTS.matches_df.empty:
                RESULTS.matches_df = RESULTS.dataf.loc[
                (RESULTS.dataf['Class'] == row['Class']) &
                (RESULTS.dataf['color_shape'] == row['color_shape'])
                ]

                if RESULTS.matches_df.empty:
                    RESULTS.publish_data(i, 0, 0)
                else:
                    lon = float(RESULTS.matches_df["long"].values[0])
                    lat = float(RESULTS.matches_df["lat"].values[0])

                    RESULTS.publish_data(i, lat, lon)


            else:
                lon = float(RESULTS.matches_df["long"])
                lat = float(RESULTS.matches_df["lat"])

                RESULTS.publish_data(i, lat, lon)

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





    


