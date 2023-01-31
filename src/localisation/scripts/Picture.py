import cv2
import time

# initialize the camera
cap = cv2.VideoCapture(0)

# set the counter and start time
counter = 0
start_time = time.time()

# set the output folder path
output_folder = "output/"

while True:
    # capture frame
    ret, frame = cap.read()
    if not ret:
        break

    # check if 3 seconds have passed
    elapsed_time = time.time() - start_time
    if elapsed_time > 2:
        # save the image
        file_name = output_folder + "image_" + str(counter) + ".jpg"
        cv2.imwrite(file_name, frame)
        counter += 1
        start_time = time.time()

# release the camera
cap.release