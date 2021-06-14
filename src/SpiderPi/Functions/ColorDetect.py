#!/usr/bin/python3
# coding=utf8
import sys
import cv2
import math
import time
import import_path
import Camera
import threading
import kinematics
import numpy as np
from LABConfig import *
import HiwonderSDK.Misc as Misc
import HiwonderSDK.Board as Board

ik = kinematics.IK()

debug = False
HWSONAR = None

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

# Find the contour with the largest area
# The parameter is list of contours to be compared
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0

    area_max_contour = None
    max_area = 0

    for c in contours:  # Traverse all contours
        contour_area_temp = math.fabs(cv2.contourArea(c))  # Calculate the contour area
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp >= 100:  # Only when the area is larger than the setting, the contour of the largest area is effective to filter interference
                area_max_contour = c
                max_area = contour_area_temp

    return area_max_contour, max_area  # Returns the largest contour area

# Initial position
def initMove():
    HWSONAR.setRGBMode(0)
    HWSONAR.setRGB(1, (0, 0, 0))
    HWSONAR.setRGB(2, (0, 0, 0))
    Board.setPWMServoPulse(1, 1500, 500)
    Board.setPWMServoPulse(2, servo2, 500)

color_list = []
detect_color = 'None'
action_finish = True
draw_color = range_rgb["black"]
# Variable reset
def reset():
    global draw_color
    global color_list
    global detect_color
    global action_finish
    
    color_list = []
    detect_color = 'None'
    action_finish = True
    draw_color = range_rgb["black"]
    
# app initialize call
def init():
    print("ColorDetect Init")
    initMove()

__isRunning = False
# app start the games call
def start():
    global __isRunning
    reset()
    __isRunning = True
    print("ColorDetect Start")

# app stop the games call
def stop():
    global __isRunning
    __isRunning = False
    print("ColorDetect Stop")

# app exit games call
def exit():
    global __isRunning
    __isRunning = False
    ik.stand(ik.initial_pos)
    print("ColorDetect Exit")

def move():
    global draw_color
    global detect_color
    global action_finish

    while True:
        if debug:
            return
        if __isRunning:
            if detect_color != 'None':
                action_finish = False
                if detect_color == 'red':
                    Board.setPWMServoPulse(1, 1800, 200)
                    time.sleep(0.2)
                    Board.setPWMServoPulse(1, 1200, 200)
                    time.sleep(0.2)
                    Board.setPWMServoPulse(1, 1800, 200)
                    time.sleep(0.2)
                    Board.setPWMServoPulse(1, 1200, 200)
                    time.sleep(0.2)
                    Board.setPWMServoPulse(1, 1500, 100)
                    time.sleep(0.1)
                    detect_color = 'None'
                    draw_color = range_rgb["black"]                    
                    time.sleep(1)
                elif detect_color == 'green' or detect_color == 'blue':
                    Board.setPWMServoPulse(2, 1800, 200)
                    time.sleep(0.2)
                    Board.setPWMServoPulse(2, 1200, 200)
                    time.sleep(0.2)
                    Board.setPWMServoPulse(2, 1800, 200)
                    time.sleep(0.2)
                    Board.setPWMServoPulse(2, 1200, 200)
                    time.sleep(0.2)
                    Board.setPWMServoPulse(2, 1500, 100)
                    time.sleep(0.1)
                    detect_color = 'None'
                    draw_color = range_rgb["black"]                    
                    time.sleep(1)
                else:
                    time.sleep(0.01)                
                action_finish = True                
                detect_color = 'None'
            else:
               time.sleep(0.01)
        else:
            time.sleep(0.01)

# Run Thread
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()

size = (320, 240)
def run(img):
    global draw_color
    global color_list
    global detect_color
    global action_finish
    
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]

    if not __isRunning:
        return img

    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)      
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # Conver the image to LAB space

    max_area = 0
    color_area_max = None    
    areaMaxContour_max = 0
    
    if action_finish:
        for i in color_range:
            if i != 'black' and i != 'white':
                frame_mask = cv2.inRange(frame_lab, color_range[i][0], color_range[i][1])  # The original image and mask are bitwise operated
                eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # Corrosion
                dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))) # Inflation
                if debug:
                    cv2.imshow(i, dilated)
                contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # Find out the outline 
                areaMaxContour, area_max = getAreaMaxContour(contours)  # Find the maximum contour
                if areaMaxContour is not None:
                    if area_max > max_area:# Find maximum area
                        max_area = area_max
                        color_area_max = i
                        areaMaxContour_max = areaMaxContour
        if max_area > 100:  # Have found the maximum area
            ((centerX, centerY), radius) = cv2.minEnclosingCircle(areaMaxContour_max)  # Gets the minimum circumferential circle
            centerX = int(Misc.map(centerX, 0, size[0], 0, img_w))
            centerY = int(Misc.map(centerY, 0, size[1], 0, img_h))
            radius = int(Misc.map(radius, 0, size[0], 0, img_w))            
            cv2.circle(img, (centerX, centerY), radius, range_rgb[color_area_max], 2)# Draw circle

            if color_area_max == 'red':  # Red biggest
                color = 1
            elif color_area_max == 'green':  # Green biggest
                color = 2
            elif color_area_max == 'blue':  # Blue largest
                color = 3
            else:
                color = 0
            color_list.append(color)

            if len(color_list) == 3:  # Multiple judgment
                # Average
                color = int(round(np.mean(np.array(color_list))))
                color_list = []
                if color == 1:
                    detect_color = 'red'
                    draw_color = range_rgb["red"]
                elif color == 2:
                    detect_color = 'green'
                    draw_color = range_rgb["green"]
                elif color == 3:
                    detect_color = 'blue'
                    draw_color = range_rgb["blue"]
                else:
                    detect_color = 'None'
                    draw_color = range_rgb["black"]               
        else:
            detect_color = 'None'
            draw_color = range_rgb["black"]
            
    cv2.putText(img, "Color: " + detect_color, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, draw_color, 2)
    
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
