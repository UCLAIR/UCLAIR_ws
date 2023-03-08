# Import necessary libraries
from collections import Counter
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt
import numpy as np
import cv2
import webcolors

# Define a function to find the closest color in a list of colors to a given color
def closest(colors, color):
    colors = np.array(colors)
    color = np.array(color)
    distances = np.sqrt(np.sum((colors-color)**2, axis=1)) # calculate the Euclidean distance between each color in the list and the given color
    index_of_smallest = np.where(distances == np.amin(distances)) # find the index of the color with the smallest distance
    smallest_distance = colors[index_of_smallest[0][0]] # get the closest color
    return smallest_distance

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
    rgb_color = webcolors.hex_to_rgb(hex_color) # convert the hex color code to an RGB color
    closest_color = closest(list(color_names.keys()), rgb_color) # find the closest reference color
    color_name = color_names[closest_color] # get the name of the closest reference color
    detected_colors.append((closest_color, color_name)) # add the detected color and its name to the list

# Print the list of detected colors
print(detected_colors)
