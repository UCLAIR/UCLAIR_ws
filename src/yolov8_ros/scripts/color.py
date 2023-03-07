import numpy as np
from PIL import Image
from sklearn.cluster import KMeans

# Load image and convert it to a numpy array
img = Image.open('image.jpg')
img_data = np.array(img)

# Flatten the image data
img_data = img_data.reshape((-1, 3))

# Use KMeans clustering to find the three main colors
kmeans = KMeans(n_clusters=3, random_state=0).fit(img_data)

# Get the three main colors as RGB values
main_colors = kmeans.cluster_centers_.astype(int)

# Print the main colors
print("The three main colors are:")
for color in main_colors:
    print(f"({color[0]}, {color[1]}, {color[2]})")
