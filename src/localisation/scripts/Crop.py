import cv2
import os.path

# Create the cropped folder if it doesn't exist
if not os.path.exists("cropped"):
    os.makedirs("cropped")

# Find all the image files in the output folder
image_files = [f for f in os.listdir("output") if os.path.isfile(os.path.join("output", f)) and f.endswith(".jpg")]

for file in image_files:
    # Load the images
    image = cv2.imread(os.path.join("output", file))

    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Find all the non-black pixels in the image
    indices = cv2.findNonZero(gray)

    # Get the bounding box for these pixels
    x, y, w, h = cv2.boundingRect(indices)

    # Crop the image to the bounding box
    cropped_image = image[y:y + h, x:x + w]

    # Save the cropped image to the output folder
    cv2.imwrite("cropped/" + file, cropped_image)
