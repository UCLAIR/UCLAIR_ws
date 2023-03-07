from collections import Counter
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt
import numpy as np
import cv2

def rgb_to_hex(rgb_color):
    hex_color = '#'
    for i in rgb_color:
        i = int(i)
        hex_color += ('{:02x}'.format(i))
    return hex_color


img_name = 'progress-pride-2021.jpg'
raw_img = cv2.imread(img_name)
raw_img = cv2.cvtColor(raw_img, cv2.COLOR_BGR2RGB)

img = cv2.resize(raw_img, (900, 600), interpolation=cv2.INTER_AREA)
img = img.reshape(img.shape[0] * img.shape[1], 3)

filtered_image = [pixel for pixel in img if not (pixel == [0, 0, 0]).all()]

clf = KMeans(n_clusters=5).fit(filtered_image)
color_labels = clf.fit_predict(filtered_image)
center_colors = clf.cluster_centers_

counts = Counter(color_labels)
ordered_colors = [center_colors[i] for i in counts.keys()]
hex_colors = [rgb_to_hex(ordered_colors[i]) for i in counts.keys()]

shape_colour = hex_colors[0]
text_colour = hex_colors[1]





