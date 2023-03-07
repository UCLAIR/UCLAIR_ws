import numpy as np
from PIL import Image
from sklearn.cluster import KMeans
import webcolors

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

# Load image and convert it to a numpy array
img = Image.open('image.jpg')
img_data = np.array(img)

# Flatten the image data
img_data = img_data.reshape((-1, 3))

# Use KMeans clustering to find the three main colors
kmeans = KMeans(n_clusters=3, random_state=0).fit(img_data)

# Get the three main colors as RGB values
main_colors = kmeans.cluster_centers_.astype(int)

# Initialize variables for closest color and distance for each main color
closest_colors = []
closest_distances = [float('inf')] * 3

# Iterate over reference colors and compute distance for each main color
for i, color in enumerate(main_colors):
    closest_color = None
    closest_distance = float('inf')
    for ref_color, name in color_names.items():
        distance = np.linalg.norm(color - ref_color)
        if distance < closest_distance:
            closest_distance = distance
            closest_color = name
    closest_colors.append(closest_color)

# Print the three main colors
print("The three main colors in the image are:", closest_colors)
