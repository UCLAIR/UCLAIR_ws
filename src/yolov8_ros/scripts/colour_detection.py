import sys
from PIL import Image
import numpy as np
from sklearn.cluster import KMeans
import cv2


class ImageProcessingClassifier:
    COLORS = {
        "White": [255, 255, 255],
        "Black": [0, 0, 0],
        "Grey": [128, 128, 128],
        "Red": [255, 0, 0],
        "Blue": [0, 0, 255],
        "Yellow": [255, 255, 0],
        "Purple": [128, 0, 128],
        "Brown": [165, 42, 42],
        "Orange": [255, 165, 0]
    }

    def __init__(self, image):
        self.image = crop_center(image)

    @staticmethod
    def crop_center(image_cv2):
         """Convert cv2 Image into PIL Image"""
        # Convert the color format from BGR to RGB
        cv2_img = cv2.cvtColor(image_cv2, cv2.COLOR_BGR2RGB)
        # Convert the cv2 image to a PIL image
        image = Image.fromarray(cv2_img)
        
      """Crop the given PIL Image from the center to 4 times smaller."""
      width, height = image.size
      new_width, new_height = width // 2, height // 2
      left = (width - new_width) // 2
      top = (height - new_height) // 2
      right = (width + new_width) // 2
      bottom = (height + new_height) // 2
      return image.crop((left, top, right, bottom))
      
    def crop_center(self):
      """Crop the given PIL Image from the center to 4 times smaller."""
      width, height = self.image.size
      new_width, new_height = width // 2, height // 2
      left = (width - new_width) // 2
      top = (height - new_height) // 2
      right = (width + new_width) // 2
      bottom = (height + new_height) // 2
      return self.image.crop((left, top, right, bottom))

    def get_dominant_color(self, k=3):

        im_small = self.image.resize((150, 150))
        im_arr = np.array(im_small)

        im_flat = im_arr.reshape((im_arr.shape[0] * im_arr.shape[1]), im_arr.shape[2])

        kmeans = KMeans(n_clusters=k)
        kmeans.fit(im_flat)
        colors = kmeans.cluster_centers_

        hex_colors = []
        for color in colors:
            hex_colors.append(self.convert_rgb_to_hex(color))

        color_names = []
        for hex_color in hex_colors:
            closest_name = ""
            closest_distance = sys.float_info.max
            for name, rgb in self.COLORS.items():
                distance = self.compute_distance(rgb, self.convert_hex_to_rgb(hex_color))
                if distance < closest_distance:
                    closest_name = name
                    closest_distance = distance
            if closest_name == "":
                raise ValueError(f"No color name found for hex color {hex_color}")
            color_names.append(closest_name)

        pixel_counts = []
        for i in range(k):
            pixel_counts.append(list(kmeans.labels_).count(i))

        color_map = {}
        for i in range(k):
            color_map[color_names[i]] = pixel_counts[i]

        sorted_colors = sorted(color_map.items(), key=lambda x: x[1], reverse=True)

        main_colors = []

        for color in sorted_colors[:3]:
            main_colors.append(color[0])

        return main_colors

    @staticmethod
    def convert_rgb_to_hex(rgb):
        return "#{:02x}{:02x}{:02x}".format(int(rgb[0]), int(rgb[1]), int(rgb[2]))

    @staticmethod
    def convert_hex_to_rgb(hex_color):
        hex_color = hex_color.lstrip('#')
        return tuple(int(hex_color[i:i + 2], 16) for i in (0, 2, 4))

    @staticmethod
    def compute_distance(rgb1, rgb2):
        r1, g1, b1 = rgb1
        r2, g2, b2 = rgb2
        return ((r1 - r2) ** 2 + (g1 - g2) ** 2 + (b1 - b2) ** 2) ** 0.5

def color_detection(image):
    cropped_classifier = ImageProcessingClassifier(image)
    cropped_dominant_colors = cropped_classifier.get_dominant_color()
    #print("Cropped Image Shape:", cropped_dominant_colors[0], "Cropped Image Text:", cropped_dominant_colors[1])
    return cropped_dominant_colors[0], cropped_dominant_colors[1]
