import cv2
import numpy as np
from getBottlesData import color_list

# Dictionary of possible colors listed in SUAS rules
colours = {
    'blue': ((90, 50, 70), (128, 255, 255)),
    'red': ((0, 150, 150), (179, 255, 255)),
    'grey': ((0, 0, 0), (178, 53, 185)),
    'green': ((36, 50, 70), (94, 255, 255)),
    'yellow': ((25, 50, 70), (35, 255, 255)),
    'purple': ((20, 0, 0), (179, 255, 80)),
    'orange': ((0, 139, 107), (20, 255, 255)),
    'brown': ((10, 100, 20), (20, 255, 200)),
    'white': ((0, 0, 180), (255, 30, 255)),
    'black': ((0, 0, 0), (180, 255, 30))
}


def colourdetected(frame):

    img_name = frame

    # Read image
    image = cv2.imread(img_name)

    # Convert to HSV color space
    image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # User choice of color to recognize/filter
    user_input = color_list

    midpoints = []

    # Loop through all the selected colors
    for color in user_input:
        lower, upper = colours[color.strip().lower()]
        # Create a mask for the selected color
        mask = cv2.inRange(image, np.array(lower), np.array(upper))
        # Morphological operations to remove noise and fill small holes
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            # Calculate the center of the contour
            M = cv2.moments(cnt)
            if M["m00"] > 0:
                area = cv2.contourArea(cnt)
                if area > 2000:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])

                    midpoints.append(((cX, cY), color))

                    print(midpoints)
