import cv2
from cap_module import ZoomedCamera
import numpy as np

def nothing(x):
    pass

#红：40 240 145 255 0 255
#绿：50 255 0 110 130 200
cv2.namedWindow("Trackbars")
cv2.createTrackbar("LMin", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("LMax", "Trackbars", 240, 255, nothing)
cv2.createTrackbar("aMin", "Trackbars", 140, 255, nothing)
cv2.createTrackbar("aMax", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("bMin", "Trackbars", 0,   255, nothing)
cv2.createTrackbar("bMax", "Trackbars", 255, 255, nothing)

cam = ZoomedCamera(zoom_slider=10)

while True:
    frame = cam.read_zoomed_frame()
    
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2Lab)
    
    LMin = cv2.getTrackbarPos("LMin", "Trackbars")
    LMax = cv2.getTrackbarPos("LMax", "Trackbars")
    aMin = cv2.getTrackbarPos("aMin", "Trackbars")
    aMax = cv2.getTrackbarPos("aMax", "Trackbars")
    bMin = cv2.getTrackbarPos("bMin", "Trackbars")
    bMax = cv2.getTrackbarPos("bMax", "Trackbars")
    #LMin, LMax, aMin, aMax, bMin, bMax = 150, 240, 130, 255, 0, 255
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
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            cv2.circle(frame, (cX, cY), 5, (0, 255, 0), -1)
            cv2.putText(frame, f"({cX}, {cY})", (cX + 10, cY), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    
    cv2.imshow("Frame", frame)
    cv2.imshow("Mask(Lab)", mask)
    if cv2.waitKey(1) & 0xFF == 27:  #ESC
        break

cv2.destroyAllWindows()
cam.release()