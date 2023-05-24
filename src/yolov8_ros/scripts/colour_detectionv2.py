import sys
import numpy as np
from sklearn.cluster import KMeans
import cv2


class ImageProcessingClassifier:
    COLORS = {
        "white": (255, 255, 255),
        "black": (0, 0, 0),
        "grey": (169, 169, 169),
        "red": (255, 0, 0),
        "blue": (0, 0, 255),
        "green": (0, 170, 20),
        "yellow": (255, 255, 0),
        "purple": (128, 0, 128),
        "brown": (92, 64, 51),
        "orange": (255, 165, 0),
        "pink": (255, 145, 175)
    }

    def __init__(self, image_path):
        self.image = image_path

    def get_dominant_color(self, k=3):

        rgb_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)

        blurred = cv2.GaussianBlur(rgb_image, (17, 17), 0)

        gray = cv2.cvtColor(blurred, cv2.COLOR_RGB2GRAY)

        thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 13, 1)

        cnts = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]

        max_contour = max(cnts, key=cv2.contourArea)

        mask = np.zeros(gray.shape, dtype=np.uint8)
        cv2.drawContours(mask, [max_contour], -1, 255, cv2.FILLED)

        new_image = np.zeros_like(rgb_image)
        new_image[:] = (175, 145, 255)

        new_image[mask != 0] = self.image[mask != 0]

        new_image_resized = cv2.resize(new_image, (0, 0), fx=0.5, fy=0.5)

        new_image_resized = cv2.cvtColor(new_image_resized, cv2.COLOR_BGR2RGB)

        image_array = np.array(new_image_resized)

        image_flat = image_array.reshape((image_array.shape[0] * image_array.shape[1]), image_array.shape[2])

        kmeans = KMeans(n_clusters=k)
        kmeans.fit(image_flat)
        colours = kmeans.cluster_centers_

        hex_colors = []
        for colour in colours:
            hex_colors.append(self.convert_rgb_to_hex(colour))

        colour_names = []
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
            colour_names.append(closest_name)

        pixel_counts = []
        for i in range(k):
            pixel_counts.append(list(kmeans.labels_).count(i))

        colour_map = {}
        for i in range(k):
            if colour_names[i] != 'pink':
                colour_map[colour_names[i]] = pixel_counts[i]

        sorted_colours = sorted(colour_map.items(), key=lambda x: x[1], reverse=True)

        dominant_colours = []
        for colour in sorted_colours[:2]:
            dominant_colours.append(colour[0])

        if len(dominant_colours) > 1:
            return dominant_colours[0], dominant_colours[1]
        else:
            return 'null', 'null'

    @staticmethod
    def convert_rgb_to_hex(rgb):
        hex_colour = '#{:02x}{:02x}{:02x}'.format(int(rgb[0]), int(rgb[1]), int(rgb[2]))
        return hex_colour

    @staticmethod
    def convert_hex_to_rgb(hex_colour):
        hex_colour = hex_colour.lstrip('#')
        return tuple(int(hex_colour[i:i + 2], 16) for i in (0, 2, 4))

    @staticmethod
    def compute_distance(rgb1, rgb2):
        r1, g1, b1 = rgb1
        r2, g2, b2 = rgb2
        return ((r1 - r2) ** 2 + (g1 - g2) ** 2 + (b1 - b2) ** 2) ** 0.5

    @staticmethod
    def convert_rgb_to_hex(rgb):
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
