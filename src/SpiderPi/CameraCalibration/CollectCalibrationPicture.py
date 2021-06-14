#!/usr/bin/env python3
# encoding:utf-8
import os
import cv2
from CalibrationConfig import *

# Collect calibration images and save them in calib folder
# Press the Space key on the keyboard to store the image, and press ESC to exit

cap = cv2.VideoCapture(-1)

# If the calib folder does not exist, create a new one
if not os.path.exists(save_path):
    os.mkdir(save_path)

# Count the number of images stored
num = 0
while True:
    ret, frame = cap.read()
    if ret:
        Frame = frame.copy()
        cv2.putText(Frame, str(num), (10, 50), cv2.FONT_HERSHEY_COMPLEX, 2.0, (0, 0, 255), 5)
        cv2.imshow("Frame", Frame)
        key = cv2.waitKey(1)
        if key == 27:
            break
        if key == 32:
            num += 1
            # Image name format：current number of images.jpg
            cv2.imwrite(save_path + str(num) + ".jpg", frame) 

cap.release()
cv2.destroyAllWindows()
