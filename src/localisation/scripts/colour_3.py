import cv2
import os

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

# User choice of colour to recognise/filter
user_input = input("Enter a list of colours separated by commas: ")

colours_list = user_input.split(',')

# folder path containing the images
input = 'output'
# folder path to save the results
output = 'results'

# Create output folder if it doesn't exist
if not os.path.exists(output):
    os.makedirs(output)

for img_name in os.listdir(input):
    # Read image
    image = cv2.imread(os.path.join(input, img_name))

    # Creating copy of image
    original = image.copy()

    # Convert to HSV colour space
    image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    colours_mask =[]

    # Masking image
    for colour in colours_list:

        lower, upper = colours[colour.strip()]

        mask = cv2.inRange(image, lower, upper)

        colours_mask.append(mask)

    final_mask = colours_mask[0]

    for i in range(1, len(colours_mask)):

        final_mask = cv2.bitwise_or(final_mask, colours_mask[i])

    # Finds contours of shape with same colour
    cnts = cv2.findContours(final_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    cnts = cnts[0] if len(cnts) == 2 else cnts[1]

    # Fills shape made from contour
    cv2.fillPoly(final_mask, cnts, (255, 255, 255))

    # Combines Mask & Original image
    result = cv2.bitwise_and(original, original, mask=final_mask)

    # Save the result
    cv2.imwrite(os.path.join(output, img_name), result)

