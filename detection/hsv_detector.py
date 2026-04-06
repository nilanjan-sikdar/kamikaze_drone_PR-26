import cv2
import numpy as np
import config
class HSVDetector:
    def __init__(self):
        self.lower1 = np.array(config.HSV_LOWER1)
        self.upper1 = np.array(config.HSV_UPPER1)
        self.lower2 = np.array(config.HSV_LOWER2)
        self.upper2 = np.array(config.HSV_UPPER2)

        self.min_contour_area = 500

    def process_frame(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask1 = cv2.inRange(hsv, self.lower1, self.upper1)
        mask2 = cv2.inRange(hsv, self.lower2, self.upper2)

        mask = cv2.bitwise_or(mask1, mask2)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None, None, 0.0, 0.0, mask

        largest = max(contours, key=cv2.contourArea)

        if cv2.contourArea(largest) < self.min_contour_area:
            return None, None, 0.0, 0.0, mask

        x, y, w, h = cv2.boundingRect(largest)
        cx = x + w / 2.0
        cy = y + h / 2.0

        return cx, cy, w, h, mask