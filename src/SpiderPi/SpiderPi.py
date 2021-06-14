#!/usr/bin/python3
# coding=utf8
import sys
import os
sys.path.append(os.path.dirname(os.path.realpath(__file__)))
import cv2
import time
import queue
import Camera
import logging
import threading
import RPCServer
import MjpgServer
import numpy as np
import HiwonderSDK.Sonar as Sonar
import Functions.Running as Running
import Functions.Transport as Transport
import Functions.Avoidance as Avoidance
import Functions.kinematics as kinematics
import Functions.ColorTrack as ColorTrack
import Functions.FaceDetect as FaceDetect
import Functions.ColorDetect as ColorDetect
import Functions.VisualPatrol as VisualPatrol
import Functions.RemoteControl as RemoteControl
import Functions.ApriltagDetect as ApriltagDetect
from CameraCalibration.CalibrationConfig import *

# main thread,has been started as backend mode
# Self-starter mode systemd，Self-starter file/etc/systemd/system/spiderpi.service
# sudo systemctl stop spiderpi  Closed
# sudo systemctl disable spiderpi Permanently closed
# sudo systemctl enable spiderpi Permanently open
# sudo systemctl start spiderpi open

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

QUEUE_RPC = queue.Queue(10)
HWSONAR = Sonar.Sonar()  # ultrasonic sensor

# Loading parameter
param_data = np.load(calibration_param_path + '.npz')

# Get parameter
mtx = param_data['mtx_array']
dist = param_data['dist_array']

def startSpiderPi():
    RPCServer.QUEUE = QUEUE_RPC
    RPCServer.HWSONAR = HWSONAR
    RPCServer.Avoidance = Avoidance
    RPCServer.Running = Running
    RPCServer.Transport = Transport
    RPCServer.FaceDetect = FaceDetect
    RPCServer.ColorTrack = ColorTrack
    RPCServer.ColorDetect = ColorDetect   
    RPCServer.VisualPatrol = VisualPatrol
    RPCServer.RemoteControl = RemoteControl
    RPCServer.ApriltagDetect = ApriltagDetect

    Running.FUNCTIONS[1] = RemoteControl
    Running.FUNCTIONS[2] = Transport
    Running.FUNCTIONS[3] = ColorDetect
    Running.FUNCTIONS[4] = VisualPatrol
    Running.FUNCTIONS[5] = ColorTrack
    Running.FUNCTIONS[6] = FaceDetect
    Running.FUNCTIONS[7] = ApriltagDetect
    Running.FUNCTIONS[8] = Avoidance
   
    Avoidance.HWSONAR = HWSONAR
    Transport.HWSONAR = HWSONAR
    ColorTrack.HWSONAR = HWSONAR
    FaceDetect.HWSONAR = HWSONAR
    ColorDetect.HWSONAR = HWSONAR
    VisualPatrol.HWSONAR = HWSONAR
    RemoteControl.HWSONAR = HWSONAR
    ApriltagDetect.HWSONAR = HWSONAR
    
    RemoteControl.init()

    threading.Thread(target=RPCServer.startRPCServer,
                     daemon=True).start()  # rpc server
    threading.Thread(target=MjpgServer.startMjpgServer,
                     daemon=True).start()  # mjpg streamer server
    
    loading_picture = cv2.imread('./loading.jpg')
    cam = Camera.Camera()  # camera reading
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (cam.width, cam.height), 0, (cam.width, cam.height))
    mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (cam.width, cam.height), 5)    
    
    Running.cam = cam
    
    while True:
        time.sleep(0.03)

        # executive RPC command that need to be executed in this thread
        while True:
            try:
                req, ret = QUEUE_RPC.get(False)
                event, params, *_ = ret
                ret[2] = req(params)  # executive RPC command
                event.set()
            except BaseException as e:
                #print(e)
                break
        #####
        # executive function games program：
        try:
            if Running.RunningFunc > 0 and Running.RunningFunc <= 8:
                if cam.frame is not None:
                    frame = cv2.remap(cam.frame.copy(), mapx, mapy, cv2.INTER_LINEAR)  # Distortion correction
                    MjpgServer.img_show = Running.CurrentEXE().run(frame)
                else:
                    MjpgServer.img_show = loading_picture
            else:
                cam.frame = None
        except KeyboardInterrupt:
            break
        except BaseException as e:
            print(e)

if __name__ == '__main__':
    logging.basicConfig(level=logging.ERROR)
    startSpiderPi()
