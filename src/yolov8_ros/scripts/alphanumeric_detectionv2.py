import cv2
import imutils
import pytesseract


class OCRDetectionClassifier:

    def __init__(self, image_path):
        self.image = image_path

    @staticmethod
    def image_processing(image):

        img = cv2.medianBlur(image, 3)

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 51, 3)

        return thresh

    @staticmethod
    def center_crop(image, dim):

        width, height = image.shape[1], image.shape[0]

        crop_width = dim[0] if dim[0] < image.shape[1] else image.shape[1]

        crop_height = dim[1] if dim[1] < image.shape[0] else image.shape[0]

        mid_x, mid_y = int(width / 2), int(height / 2)

        cw2, ch2 = int(crop_width / 2), int(crop_height / 2)

        crop_img = image[mid_y - ch2:mid_y + ch2, mid_x - cw2:mid_x + cw2]

        return crop_img

    def OCR_image_rotation(self):

        resized_img = imutils.resize(self.image, 600, 600,inter=cv2.INTER_AREA)

        filtered_img = self.image_processing(resized_img)

        cropped = self.center_crop(filtered_img, (300, 400))

        max_confidence = 0

        confidence_scores = []

        max_confidence_index = []

        results = []

        pytesseract.pytesseract.tesseract_cmd = r"/usr/bin/tesseract"

        config = '-c tessedit_char_whitelist=ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789 --psm 10'

        for i in range(0, 6):

            rotated = imutils.rotate(cropped, i * 60)

            data = pytesseract.image_to_data(rotated, lang='eng', config=config)

            last = data.split('\n')[-2]
            split = last.split('\t')

            results.append(split[-1])

            confidence_scores.append(split[-2])

            if float(split[-2]) > max_confidence:
                max_confidence = float(split[-2])
                max_confidence_index = i

        max_confidence_result = results[max_confidence_index]

        return max_confidence_result


def alphanumeric_detection2(image):
    classifier = OCRDetectionClassifier(image)

    alphanumeric = classifier.OCR_image_rotation()

    return alphanumeric
