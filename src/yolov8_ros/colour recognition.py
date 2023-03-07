from collections import Counter
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt
import numpy as np
import cv2
import webcolors

def closest(colors, color):
    colors = np.array(colors)
    color = np.array(color)
    distances = np.sqrt(np.sum((colors-color)**2, axis=1))
    index_of_smallest = np.where(distances == np.amin(distances))
    smallest_distance = colors[index_of_smallest][0]
    return smallest_distance

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
hex_colors = ['#' + ''.join([format(int(c), '02x') for c in color]) for color in ordered_colors]

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
detected_colors = []
for hex_color in hex_colors:
    rgb_color = webcolors.hex_to_rgb(hex_color)
    closest_color = closest(color_names.keys(), rgb_color)
    color_name = color_names[closest_color]
    detected_colors.append((closest_color, color_name))

print(detected_colors)
