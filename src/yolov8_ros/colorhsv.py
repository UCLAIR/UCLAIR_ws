# Import necessary libraries
from collections import Counter
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt
import numpy as np
import cv2
import webcolors

# Define a function to find the closest color in a list of colors to a given color
def closest(colors, color):
    return min(colors, key=lambda x: np.linalg.norm(np.array(color)-np.array(x)))

# Load the image and convert it to RGB color space
img_name = 'progress-pride-2021.jpg'
raw_img = cv2.imread(img_name)
raw_img = cv2.cvtColor(raw_img, cv2.COLOR_BGR2RGB)

# Resize the image to a smaller size for faster processing
img = cv2.resize(raw_img, (900, 600), interpolation=cv2.INTER_AREA)

# Reshape the image to a 2D array of pixels
img = img.reshape(img.shape[0] * img.shape[1], 3)

# Filter out black pixels
filtered_image = [pixel for pixel in img if not (pixel == [0, 0, 0]).all()]

# Perform K-Means clustering to find the most common colors in the image
clf = KMeans(n_clusters=5).fit(filtered_image)
color_labels = clf.fit_predict(filtered_image)
center_colors = clf.cluster_centers_

# Count the number of pixels in each cluster
counts = Counter(color_labels)

# Order the cluster centers by frequency
ordered_colors = [center_colors[i] for i in counts.keys()]

# Convert the cluster centers to hex color codes
hex_colors = ['#' + ''.join([format(int(c), '02x') for c in color]) for color in ordered_colors]

# Define a dictionary of reference colors
color_names = {
    range(0, 30): 'black',
    range(30, 60): 'gray',
    range(60, 90): 'silver',
    range(90, 120): 'maroon',
    range(120, 150): 'red',
    range(150, 180): 'olive',
    range(180, 210): 'yellow',
    range(210, 240): 'lime',
    range(240, 270): 'green',
    range(270, 300): 'teal',
    range(300, 330): 'blue',
    range(330, 360): 'purple',
}

# Get the closest reference color and its name for each color in the image
detected_colors = []
for hex_color in hex_colors:
    rgb_color = webcolors.hex_to_rgb(hex_color) # convert the hex color code to an RGB color
    h, s, v = webcolors.rgb_to_hsv(*rgb_color) # convert the RGB color to HSV color
    for color_range, color_name in color_names.items():
        if h in color_range:
            detected_colors.append((rgb_color, color_name))
            break

# Print the list of detected colors
print(detected_colors)
