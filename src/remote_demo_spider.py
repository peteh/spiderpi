#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import numpy as np
import threading
import paho.mqtt.client as mqtt
import json
from SpiderPi.Functions import kinematics  # The Kinematics library is encrypted and can only be called
'''
class kinematics():

    class IK():
        def stand(self, pos, dt):
            pass
'''


class GamePad():
    def __init__(self):
        self._joyState = {}
        # buttons
        self._joyState["mode"] = "stop"
    
    def update(self, key, value):
        self._joyState[key] = value
    
    def getValue(self, key):
        return self._joyState[key]


class HexapodRunner():
    def __init__(self, gamepad):
        self._gamepad = gamepad
        self._ik = kinematics.IK()  # Instabtiate the inverse kinematics library
        self._ik.stand(self._ik.initial_pos, t=500)  # Attention, attitude is IK.INITIAL_POS, time 500ms
    
    def loop(self):
        # forward
        if(self._gamepad.getValue("mode") == "forward"):
            self._ik.go_forward(self._ik.current_pos, 2, 50, 80, 1)  # Go straight ahead for 50mm
        # backward
        elif(self._gamepad.getValue("mode") == "backward"):
            self._ik.back(self._ik.current_pos, 2, 50, 80, 1)  # Go back ahead for 50mm
        #right strafe
        elif(self._gamepad.getValue("mode") == "strafe_r"):
            self._ik.right_move(self._ik.current_pos, 2, 50, 80, 1)  # Right 50mm
        #left strafe
        elif(self._gamepad.getValue("mode") == "strafe_l"):
            self._ik.left_move(self._ik.current_pos, 2, 50, 80, 1)  # Go left ahead for 50mm
        #right turn
        elif(self._gamepad.getValue("mode") == "turn_r"):
            self._ik.turn_right(self._ik.current_pos, 2, 30, 80, 1)  # Turn right 30 degrees
        #left turn
        elif(self._gamepad.getValue("mode") == "turn_l"):
            self._ik.turn_left(self._ik.current_pos, 2, 30, 80, 1)  # Turn left 30 degrees



class HotwordBeep(object):
    def __init__(self, gamepad):
        self._msgThread = threading.Thread(target = self._run)
        
        self._mqtt_client = mqtt.Client()
        self._mqtt_client.on_connect = self._onConnect
        self._mqtt_client.on_message = self._onMessage
        self._gamepad = gamepad
        
    def _onConnect(self, client, userdata, flags, rc):
        # subscribe to all messages
        client.subscribe("remote/spiderpi")
        pass
    
    def start(self):
        self._mqtt_client.connect('rhasspy.local', 1883)
        self._msgThread.start()
        

    def _run(self):
        self._mqtt_client.loop_forever()
        print("Ended Skill")
    
    def sendMessage(self, msg):
        msg = json.dumps(msg)
        #print(msg)
        self._mqtt_client.publish("remote", msg)

    def stop(self):
        print("Skill should end")
        self._mqtt_client.disconnect()
        print("mqtt disconnected")
        
    def _onMessage(self, client, userdata, msg):
        if msg.topic == "remote/spiderpi":
            msgPayload = json.loads(msg.payload.decode("utf-8"))
            for element in msgPayload:
                gamepad.update(element, msgPayload[element])
            

gamepad = GamePad()
skill = HotwordBeep(gamepad)
runner = HexapodRunner(gamepad)
skill.start()


while(True):
    try:
        runner.loop()
        #time.sleep(0.02)
    except KeyboardInterrupt:
        break
skill.stop()
