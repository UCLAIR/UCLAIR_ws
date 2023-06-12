import sys
import numpy as np
from sklearn.cluster import KMeans
import cv2
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers


class ImageProcessingClassifier:
    model = tf.keras.models.load_model("colormodel_trained_90.h5")

    color_dict={
        0 : 'RED',
        1 : 'GREEN',
        2 : 'BLUE',
        3 : 'YELLOW',
        4 : 'ORANGE',
        5 : 'PINK',
        6 : 'PURPLE',
        7 : 'BROWN',
        8 : 'GREY',
        9 : 'BLACK',
        10 : 'WHITE'
    }

    def __init__(self, image_path):
        self.image = image_path
        self.image = cv2.resize(self.image, (0, 0), fx=0.25, fy=0.25)

    def get_dominant_color(self, k=3):

        image_new = self.adjust_contrast(self.image, alpha=3, beta=1.5)

        blurred = cv2.GaussianBlur(image_new, (3, 3), 0)

        gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)

        thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 21, 1)

        cnts = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]

        max_contour = max(cnts, key=cv2.contourArea)

        mask = np.zeros(gray.shape, dtype=np.uint8)
        cv2.drawContours(mask, [max_contour], -1, 255, cv2.FILLED)

        new_image = np.zeros_like(self.image)
        new_image[:] = (175, 145, 255)

        new_image[mask != 0] = self.image[mask != 0]

        new_image = cv2.cvtColor(new_image, cv2.COLOR_BGR2RGB)

        image_array = np.array(new_image)

        image_flat = image_array.reshape((image_array.shape[0] * image_array.shape[1]), image_array.shape[2])

        kmeans = KMeans(n_clusters=k)
        kmeans.fit(image_flat)
        colours = kmeans.cluster_centers_
        
        output_colours = []

        for i in range(len(colours)):
            predicted_colour = self.predict_colour(colours[i][0], colours[i][1], colours[i][2])
            if predicted_colour == 'PINK':
                continue
            existing_colors = [color for color in output_colours if color[0] == predicted_colour]
            if existing_colors:
                existing_colors[0][1] += list(kmeans.labels_).count(i)
            else:
                output_colours.append([predicted_colour, list(kmeans.labels_).count(i)])

        ordered = sorted(output_colours, key=lambda x: x[1], reverse=True)
        colour_names = [color[0] for color in ordered[:2]]

        return colour_names

    def predict_colour(self, Red, Green, Blue):
        rgb = np.asarray((Red, Green, Blue)) #rgb tuple to numpy array
        input_rgb = np.reshape(rgb, (-1,3)) #reshaping as per input to ANN model
        color_class_confidence = self.model.predict(input_rgb) # Output of layer is in terms of Confidence of the 11 classes
        color_index = np.argmax(color_class_confidence, axis=1) #finding the color_class index from confidence
        color = self.color_dict[int(color_index)]
        return color

    @staticmethod
    def adjust_contrast(image, alpha, beta):
        adjusted_image = cv2.convertScaleAbs(image, alpha=alpha, beta=beta)
        return adjusted_image


def color_detection(image):
    classifier = ImageProcessingClassifier(image)
    dominant_colors = classifier.get_dominant_color()
    return dominant_colors
