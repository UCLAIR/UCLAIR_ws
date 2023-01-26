import cv2
import os

# Dictionary of possible colours listed in SUAS rules
colours = {
    'blue': ((90, 50, 70), (128, 255, 255)),
    'red': ((0, 100, 100), (179, 255, 255)),
    'grey': ((0, 0, 0), (178, 53, 185)),
    'green': ((36, 50, 70), (94, 255, 255)),
    'yellow': ((25, 50, 70), (35, 255, 255)),
    'purple': ((20, 0, 0), (179, 255, 80)),
    'orange': ((0, 139, 107), (20, 255, 255)),
    'brown': ((10, 100, 20), (20, 255, 200)),
    'white': ((0, 0, 180), (255, 30, 255)),
    'black': ((0, 0, 0), (180, 255, 30))
}

# Initialize the video capture
cap = cv2.VideoCapture(0)

# Defining the colour
user_input = input("Enter a colour: ")
if user_input in colours:
    lower, upper = colours[user_input]
else:
    user_input = input("Enter a colour: ")
    lower, upper = colours[user_input]

# Create the output directory if it does not exist
if not os.path.exists('output'):
    os.makedirs('output')

# Setting counter
count = 1

# Maximum counter
max_count = 300

while True:

    # Read a frame from the video capture
    _, frame = cap.read()

    # Convert the frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create the mask
    mask = cv2.inRange(hsv, lower, upper)

    # Finds contours of shape with same colour
    cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    cnts = cnts[0] if len(cnts) == 2 else cnts[1]

    # Fills shape made from contour
    cv2.fillPoly(mask, cnts, (255, 255, 255))

    # Mask the frame
    masked_frame = cv2.bitwise_and(frame, frame, mask=mask)

    # Gaussian Filter
    # blur = cv2.GaussianBlur(masked_frame,(15,15),0)
    # cv2.imshow('Gaussian Blurring', blur)

    # Bilateral blur

    bilateral = cv2.bilateralFilter(masked_frame, 15, 75, 75)

    cv2.imshow('Bilateral Blur', bilateral)

    file_name = f"{user_input}_{count}.jpg"

    file_path = os.path.join('output', file_name)

    # Show the masked frame
    # cv2.imshow('Masked Frame', masked_frame)

    # Save the frame if the specified color is detected
    if cv2.countNonZero(mask) > 0:
        cv2.imwrite(file_path, bilateral)
        count += 1

    # Take a max amount of frames for each colour
    if count > max_count:
        user_input = input("Enter a colour: ")
        if user_input in colours:
            lower, upper = colours[user_input]
        count = 1
        continue

    key = cv2.waitKey(1)

    # If space pressed, release the video capture object and destroy all windows
    if key == ord(' '):
        break

    if key == ord('r'):
        user_input = input("Enter a colour: ")
        lower, upper = colours[user_input]
        continue

cap.release
cv2.destroyAllWindows()
