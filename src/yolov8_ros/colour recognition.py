from collections import Counter
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt
import numpy as np
import cv2
import webcolors

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

# Define reference colors
color_names = {
    (255, 255, 255): 'white',
    (0, 0, 0): 'black',
    (128, 128, 128): 'gray',
    (255, 0, 0): 'red',
    (0, 0, 255): 'blue',
    (0, 255, 0): 'green',
    (255, 255, 0): 'yellow',
    (128, 0, 128): 'purple',
    (165, 42, 42): 'brown',
    (255, 165, 0): 'orange'
}

# Get the closest reference color and its name for each color in the image
color_names = []
for hex_color in hex_colors:
    rgb_color = webcolors.hex_to_rgb(hex_color)
    closest_color = None
    closest_distance = float('inf')
    for ref_color, name in color_names.items():
        distance = np.linalg.norm(np.array(rgb_color) - np.array(ref_color))
        if distance < closest_distance:
            closest_distance = distance
            closest_color = name
    color_names.append(closest_color)

# Print the detected colors and their corresponding reference colors
for i in range(len(color_names)):
    print("Detected color: " + hex_colors[i] + ", Reference color: " + color_names[i])
