import cv2


def cropmain(frame):
    # Load the images
    image = cv2.imread(frame)

    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Find all the non-black pixels in the image
    indices = cv2.findNonZero(gray)

    # Get the bounding box for these pixels
    x, y, w, h = cv2.boundingRect(indices)

    # Crop the image to the bounding box
    cropped_image = image[y:y + h, x:x + w]

    return cropped_image
