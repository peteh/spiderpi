#!/usr/bin/python3
# coding=utf8
import sys
import import_path
import cv2
import math
import time
import Camera
import apriltag
import threading
import numpy as np
import HiwonderSDK.Misc as Misc
import HiwonderSDK.Board as Board
import HiwonderSDK.ActionGroupControl as AGC

#apriltag detects and takes correspond actions

debug = False
HWSONAR = None

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

# Initial position
def initMove():
    HWSONAR.setRGBMode(0)
    HWSONAR.setRGB(1, (0, 0, 0))
    HWSONAR.setRGB(2, (0, 0, 0))    
    Board.setPWMServoPulse(1, 1500, 500)
    Board.setPWMServoPulse(2, 1500, 500)    

tag_id = None
__isRunning = False
action_finish = True
# Variable reset
def reset():      
    global tag_id
    global action_finish
    
    tag_id = 0
    action_finish = True
    
# app initialization call
def init():
    print("Apriltag Init")
    initMove()

# app start games call
def start():
    global __isRunning
    reset()
    __isRunning = True
    print("Apriltag Start")

# app stop games call
def stop():
    global __isRunning
    __isRunning = False
    print("Apriltag Stop")

# app stop games call
def exit():
    global __isRunning
    __isRunning = False
    AGC.runActionGroup('stand_low')
    print("Apriltag Exit")

def move():
    global tag_id
    global action_finish  
    
    while True:
        if debug:
            return
        if __isRunning:
            if tag_id is not None:
                action_finish = False
                time.sleep(0.5)
                if tag_id == 1:               
                    AGC.runActionGroup('wave')# Wave
                    tag_id = None
                    time.sleep(1)                  
                    action_finish = True                
                elif tag_id == 2:                    
                    AGC.runActionGroup('stepping')# No progress
                    tag_id = None
                    time.sleep(1)
                    action_finish = True          
                elif tag_id == 3:                   
                    AGC.runActionGroup('twist_l')# Twist
                    tag_id = None
                    time.sleep(1)
                    action_finish = True
                else:
                    action_finish = True
                    time.sleep(0.01)
            else:
               time.sleep(0.01)
        else:
            time.sleep(0.01)

# Run thread
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()

#  Detect apriltag
detector = apriltag.Detector(searchpath=apriltag._get_demo_searchpath())
def apriltagDetect(img):   
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    detections = detector.detect(gray, return_image=False)

    if len(detections) != 0:
        for detection in detections:                       
            corners = np.rint(detection.corners)  # Get the four corner points
            cv2.drawContours(img, [np.array(corners, np.int)], -1, (0, 255, 255), 2)

            tag_family = str(detection.tag_family, encoding='utf-8')  # Get tag_family
            tag_id = int(detection.tag_id)  # Get tag_id

            object_center_x, object_center_y = int(detection.center[0]), int(detection.center[1])  # Center point
            
            object_angle = int(math.degrees(math.atan2(corners[0][1] - corners[1][1], corners[0][0] - corners[1][0])))  # Calculate the rotation angle
            
            return tag_family, tag_id
            
    return None, None

def run(img):
    global tag_id
    global action_finish
     
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]

    if not __isRunning:
        return img
    
    tag_family, tag_id = apriltagDetect(img) # apriltag detect
    
    if tag_id is not None:
        cv2.putText(img, "tag_id: " + str(tag_id), (10, img.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, [0, 255, 255], 2)
        cv2.putText(img, "tag_family: " + tag_family, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, [0, 255, 255], 2)
    else:
        cv2.putText(img, "tag_id: None", (10, img.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, [0, 255, 255], 2)
        cv2.putText(img, "tag_family: None", (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, [0, 255, 255], 2)
    
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
    
    debug = False
    if debug:
        print('Debug Mode')
    
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
