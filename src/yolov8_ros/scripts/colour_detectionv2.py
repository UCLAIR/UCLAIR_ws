import sys
import numpy as np
from sklearn.cluster import KMeans
import cv2


class ImageProcessingClassifier:
    COLORS = {
        "WHITE": (255, 255, 255),
        "BLACK": (0, 0, 0),
        "GRAY": (169, 169, 169),
        "RED": (255, 0, 0),
        "BLUE": (0, 0, 255),
        "GREEN": (0, 170, 20),
        "YELLOW": (255, 255, 0),
        "PURPLE": (128, 0, 128),
        "BROWN": (92, 64, 51),
        "ORANGE": (255, 165, 0),
        "PINK": (255, 145, 175)
    }

    def __init__(self, image_path):
        self.image = image_path
        self.image = cv2.resize(self.image, (0, 0), fx=0.25, fy=0.25)

    def get_dominant_color(self, k=3):

        image_new = self.adjust_contrast(self.image, alpha=1.5, beta=0)

        blurred = cv2.GaussianBlur(image_new, (3, 3), 0)

        gray = cv2.cvtColor(blurred, cv2.COLOR_RGB2GRAY)

        thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 21, 1)

        cnts = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]

        max_contour = max(cnts, key=cv2.contourArea)

        mask = np.zeros(gray.shape, dtype=np.uint8)
        cv2.drawContours(mask, [max_contour], -1, 255, cv2.FILLED)

        new_image = np.zeros_like(self.image)
        new_image[:] = (255, 145, 175)

        new_image[mask != 0] = self.image[mask != 0]

        image_array = np.array(new_image)

        image_flat = image_array.reshape((image_array.shape[0] * image_array.shape[1]), image_array.shape[2])

        kmeans = KMeans(n_clusters=k)
        kmeans.fit(image_flat)
        colours = kmeans.cluster_centers_

        colour_names = []
        for colour in colours:
            closest_name = ""
            closest_distance = sys.float_info.max
            for name, rgb in self.COLORS.items():
                distance = self.compute_distance(rgb, colour)
                if distance < closest_distance:
                    closest_name = name
                    closest_distance = distance
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
    def compute_distance(rgb1, rgb2):
        return np.linalg.norm(np.array(rgb1) - np.array(rgb2))

    @staticmethod
    def adjust_contrast(image, alpha, beta):
        adjusted_image = cv2.convertScaleAbs(image, alpha=alpha, beta=beta)
        return adjusted_image


def color_detection(image):
    classifier = ImageProcessingClassifier(image)
    dominant_colors = classifier.get_dominant_color()
    return dominant_colors
