#!/usr/bin/python3
#coding=utf8
import os
import sys
import cv2
import import_path
import time
import Camera
import threading
import pandas as pd
import numpy as np
import kinematics as kinematics
import HiwonderSDK.Board as Board

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

ik = kinematics.IK()

HWSONAR = None

Threshold = 40.0 # Default threshold 40cm
TextColor = (0, 255, 255)
TextSize = 12

__isRunning = False
distance = 0

def reset():
    Board.setPWMServoPulse(1, 1500, 500)
    Board.setPWMServoPulse(2, 1500, 500)    

def init():
    reset()
    print('Avoidance Init')

def exit():
    global __isRunning
    
    HWSONAR.setRGBMode(0)
    HWSONAR.setRGB(1, (0, 0, 0))
    HWSONAR.setRGB(2, (0, 0, 0))
    __isRunning = False
    print('Avoidance Exit')

def setThreshold(args):
    global Threshold
    Threshold = args[0]
    return (True, (Threshold,))

def getThreshold(args):
    global Threshold
    return (True, (Threshold,))

def start():
    global __isRunning
    __isRunning = True
    print('Avoidance Start')

def stop():
    global __isRunning
    __isRunning = False
    ik.stand(ik.initial_pos)
    print('Avoidance Stop')

def move():
    while True:
        if __isRunning:
            if 0 < distance < Threshold:
                while distance < 25: # Less than 25cm backward
                    ik.back(ik.initial_pos, 2, 80, 50, 1)
                for i in range(6): # Turn left 6 times, at 15 degrees each, 90 degrees in total
                    if __isRunning:
                        ik.turn_left(ik.initial_pos, 2, 15, 50, 1)
            else: 
                ik.go_forward(ik.initial_pos, 2, 80, 50, 1)
        else:
            time.sleep(0.01)

threading.Thread(target=move, daemon=True).start()

distance_data = []

def run(img):
    global __isRunning
    global Threshold
    global distance
    global distance_data

    if __isRunning:
        
        # Data processing, filtering outliers
        distance_ = HWSONAR.getDistance() / 10.0
        distance_data.append(distance_)
        data = pd.DataFrame(distance_data)
        data_ = data.copy()
        u = data_.mean()  # Calculate average 
        std = data_.std()  # Calculate standard deviation

        data_c = data[np.abs(data - u) <= std]
        distance = data_c.mean()[0]
        if len(distance_data) == 5:
            distance_data.remove(distance_data[0])

        cv2.putText(img, "Dist:%.1fcm" % distance, (30, 480 - 30), cv2.FONT_HERSHEY_SIMPLEX, 1.2, TextColor, 2)
    return img

if __name__ == '__main__':
    import HiwonderSDK.Sonar as Sonar
    from CameraCalibration.CalibrationConfig import *

    # Loading parameter
    param_data = np.load(calibration_param_path + '.npz')
    
    # Get parameter
    mtx = param_data['mtx_array']
    dist = param_data['dist_array']
    newcameramtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, (640, 480), 0, (640, 480))
    mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (640, 480), 5)

    HWSONAR = Sonar.Sonar()
    init()
    start()
    my_camera = Camera.Camera()
    my_camera.camera_open()
    while True:
        img = my_camera.frame
        if img is not None:
            frame = img.copy()
            frame = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)  # Distortion correction
            Frame = run(frame)
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            time.sleep(0.01)
    my_camera.camera_close()
    cv2.destroyAllWindows()
