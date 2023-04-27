import pandas as pd
from getpass import getuser


csv = pd.read_csv(f'/home/{getuser()}/UCLAIR_ws/src/yolov8_ros/database/list.csv', index_col=0)
bottles_df = pd.read_csv(f'/home/{getuser()}/UCLAIR_ws/src/yolov8_ros/database/BOTTLESDATA.txt', sep=',', header=None, names=['Class', 'character', 'color_shape', 'color_char'])


longgg = []
lattt = []
for i, row in bottles_df.iterrows():
    long = csv.loc[(
        (csv['Class'] == row['Class']) &
        (csv['character'] == row['character']) &
        (csv['color_shape'] == row['color_shape']) &
        (csv['color_char'] == row['color_char'])
    )]
    try:
        longgg.append(float(long['long']))
        lattt.append(float(long['lat']))
    except:
        longgg.append("null")
        lattt.append("null")
        
print(longgg)
print(lattt)