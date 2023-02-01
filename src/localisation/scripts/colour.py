import cv2
from getBottlesData import color_list

# Dictionary of possible colours listed in SUAS rules
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


def colordetected(frame):
    user_input = color_list

    image = cv2.imread(frame)

    # Creating copy of image
    original = image.copy()

    # Conv to HSV colour space
    image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    colours_mask = []

    # Masking image
    for colour in user_input:
        lower, upper = colours[colour.strip()]

        mask = cv2.inRange(image, lower, upper)

        colours_mask.append(mask)

    final_mask = colours_mask[0]

    for i in range(1, len(colours_mask)):
        final_mask = cv2.bitwise_or(final_mask, colours_mask[i])

    # Finds contours of shape with same colour
    cnts = cv2.findContours(final_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    cnts = cnts[0] if len(cnts) == 2 else cnts[1]

    midpoints = []

    # Draws bounding boxes on original image
    for c in cnts:

        # Calculate the midpoint of the contour
        M = cv2.moments(c)

        if M["m00"] > 0:

            area = cv2.contourArea(c)

            if area > 1000:

                cv2.drawContours(original, [c], -1, (0, 0, 255), 5)

                midpoint = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                if midpoint[0] != 0 and midpoint[1] != 0:
                    midpoints.append(midpoint)

    # Fills shape made from contour
    cv2.fillPoly(final_mask, cnts, (255, 255, 255))

    # Check if final_mask has any non-zero values

    if cv2.countNonZero(final_mask) > 0:
        return True
