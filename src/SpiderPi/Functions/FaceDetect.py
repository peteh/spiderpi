#!/usr/bin/python3
# coding=utf8
import sys
import import_path
import cv2
import math
import time
import Camera
import threading
import numpy as np
from LABConfig import *
import kinematics as kinematics
import HiwonderSDK.Misc as Misc
import HiwonderSDK.Board as Board
import HiwonderSDK.ActionGroupControl as AGC

#  Facial Recognition

HWSONAR = None
ik = kinematics.IK()

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

# Threshold value
conf_threshold = 0.6

# Model poaition
modelFile = "/home/pi/SpiderPi/models/res10_300x300_ssd_iter_140000_fp16.caffemodel"
configFile = "/home/pi/SpiderPi/models/deploy.prototxt"
net = cv2.dnn.readNetFromCaffe(configFile, modelFile)

servo2_pulse = servo2
# Initial position
def initMove():
    HWSONAR.setRGBMode(0)
    HWSONAR.setRGB(1, (0, 0, 0))
    HWSONAR.setRGB(2, (0, 0, 0))     
    Board.setPWMServoPulse(1, 1800, 500)
    Board.setPWMServoPulse(2, servo2_pulse, 500)    

d_pulse = 10
start_greet = False
action_finish = True
# Variable reset
def reset():
    global d_pulse
    global start_greet
    global servo2_pulse    
    global action_finish

    d_pulse = 10
    start_greet = False
    action_finish = True
    servo2_pulse = servo2    
    initMove()  
    
# app initial call
def init():
    print("FaceDetect Init")
    reset()

__isRunning = False
# app start games call
def start():
    global __isRunning
    __isRunning = True
    print("FaceDetect Start")

# app stop games call
def stop():
    global __isRunning
    __isRunning = False
    reset()
    print("FaceDetect Stop")

# app exit games call
def exit():
    global __isRunning
    __isRunning = False
    ik.stand(ik.initial_pos)
    print("FaceDetect Exit")

def move():
    global start_greet
    global action_finish
    global d_pulse, servo2_pulse    
    
    while True:
        if __isRunning:
            if start_greet:
                start_greet = False
                action_finish = False
                AGC.runActionGroup('wave') # When a face is recognized and action performed
                action_finish = True
                time.sleep(0.5)
            else:
                if servo2_pulse > 2000 or servo2_pulse < 1000:
                    d_pulse = -d_pulse
            
                servo2_pulse += d_pulse       
                Board.setPWMServoPulse(2, servo2_pulse, 50)
                time.sleep(0.05)
        else:
            time.sleep(0.01)
            
# run thread
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()

size = (320, 240)
def run(img):
    global start_greet
    global action_finish
       
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]

    if not __isRunning:
        return img

    blob = cv2.dnn.blobFromImage(img_copy, 1, (250, 250), [104, 117, 123], False, False)
    net.setInput(blob)
    detections = net.forward() # Calculated recognition
    for i in range(detections.shape[2]):
        confidence = detections[0, 0, i, 2]
        if confidence > conf_threshold:
            #Each coordinate of the person identified will be converted without scaling the previous coordinate
            x1 = int(detections[0, 0, i, 3] * img_w)
            y1 = int(detections[0, 0, i, 4] * img_h)
            x2 = int(detections[0, 0, i, 5] * img_w)
            y2 = int(detections[0, 0, i, 6] * img_h)             
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2, 8) # Frame out the recognized face
            if abs((x1 + x2)/2 - img_w/2) < img_w/4:
                if action_finish:
                    start_greet = True
    
    return img

if __name__ == '__main__':
    import HiwonderSDK.Sonar as Sonar
    from CameraCalibration.CalibrationConfig import *
    
    # Loading parameter
    param_data = np.load(calibration_param_path + '.npz')

    # Get parameter
    mtx = param_data['mtx_array']
    dist = param_data['dist_array']
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (640, 480), 0, (640, 480))
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
