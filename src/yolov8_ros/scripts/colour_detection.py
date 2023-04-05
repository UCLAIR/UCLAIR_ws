import sys
import numpy as np
from sklearn.cluster import KMeans
import cv2


class ImageProcessingClassifier:
    COLORS = {
        "white": [255, 255, 255],
        "black": [0, 0, 0],
        "grey": [169, 169, 169],
        "red": [255, 0, 0],
        "blue": [0, 0, 255],
        "yellow": [255, 255, 0],
        "purple": [128, 0, 128],
        "brown": [92, 64, 51],
        "orange": [255, 165, 0],
        "pink": [255, 145, 175]
    }


    def __init__(self, image_path):
        self.image_path = image_path
        self.image = cv2.imread(image_path)

    def get_dominant_color(self, k=3):
        rgb_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)

        blurred = cv2.GaussianBlur(rgb_image, (17, 17), 0)

        gray = cv2.cvtColor(blurred, cv2.COLOR_RGB2GRAY)

        thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 5, 1)

        cnts = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]

        max_contour = max(cnts, key=cv2.contourArea)

        mask = np.zeros_like(thresh)
        cv2.drawContours(mask, [max_contour], 0, (255, 255, 255), -1)

        mask_rgb = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
        mask_rgb[np.where((mask_rgb == [0, 0, 0]).all(axis=2))] = [255, 145, 175]

        image_small = cv2.bitwise_and(rgb_image, mask_rgb)

        image_small = cv2.resize(image_small, (50, 50))

        image_array = np.array(image_small)

        im_flat = image_array.reshape((image_array.shape[0] * image_array.shape[1]), image_array.shape[2])

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
            if color_names[i] != 'pink':
                color_map[color_names[i]] = pixel_counts[i]

        sorted_colors = sorted(color_map.items(), key=lambda x: x[1], reverse=True)

        dominant_colors = []
        for color in sorted_colors[:2]:
            dominant_colors.append(color[0])

        if len(dominant_colors) > 1:
            return dominant_colors[0], dominant_colors[1]
        else:
            return 'null', 'null'

    @staticmethod
    def convert_rgb_to_hex(rgb):
        # Convert RGB to hex
        hex_color = '#{:02x}{:02x}{:02x}'.format(int(rgb[0]), int(rgb[1]), int(rgb[2]))
        return hex_color

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
    classifier = ImageProcessingClassifier(image)
    dominant_colors = classifier.get_dominant_color()
    return dominant_colors