import cv2
import numpy as np

def nothing(x):
    pass
def detect_green_laser(frame):
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2Lab)
    LMin, LMax, aMin, aMax, bMin, bMax = 50, 255, 0, 110, 130, 200
    lower = np.array([LMin, aMin, bMin], dtype=np.uint8)
    upper = np.array([LMax, aMax, bMax], dtype=np.uint8)
    mask = cv2.inRange(lab, lower, upper)

    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=1)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        max_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(max_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            return cx, cy

    return -1, -1