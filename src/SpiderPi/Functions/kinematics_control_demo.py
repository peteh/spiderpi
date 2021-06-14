#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import numpy as np
import kinematics  # The Kinematics library is encrypted and can only be called

ik = kinematics.IK()  # Instabtiate the inverse kinematics library
ik.stand(ik.initial_pos, t=500)  # Attention, attitude is IK.INITIAL_POS, time 500ms
print('姿态:\n', np.array(ik.initial_pos))  # Print and check Pose
# The attitude data is a 3x6 array representing the x, y, and z coordinates of the ends of the six legs, in mm
# The head forward direction is the X-axis, the head forward position is the negative direction, the right side is the Y-axis positive, and the vertical up is the Z-axis positive. A vertical line is drawn from the midpoint of the connection between the two middle legs to intersect the upper and lower plates, and the center of the connection is set as zero
# The first leg is the upper left leg when the head is facing forward, and the counterclockwise leg is 1-6
# [[-199.53, -177.73, -100.0],
#  [0.0,     -211.27, -100.0],
#  [199.53,  -177.73, -100.0],
#  [199.53,  177.73,  -100.0],
#  [0.0,     211.27,  -100.0],
#  [-199.53, 177.73,  -100.0]]

# Parameter 1: Attitude; Parameter 2: mode, 2 is hexapod mode, 4 is quadruped mode, and when is four-legged mode, the corresponding posture shall be initial_pos_quadruped
# Parameter 3: stride length, unit of mm (Angle, unit of degree when turning); Parameter 4: speed, unit mm/s; Parameter 5: Number of executions. When the unit is 0, it means infinite loop

ik.go_forward(ik.initial_pos, 2, 50, 80, 1)  # Go straight ahead for 50mm

ik.back(ik.initial_pos, 2, 100, 80, 1)  # Go backward 100mm

ik.turn_left(ik.initial_pos, 2, 30, 100, 1)  # Turn left 30 degrees

ik.turn_right(ik.initial_pos, 2, 30, 100, 1)  # Turn right 30 degrees

ik.left_move(ik.initial_pos, 2, 100, 100, 1)  # left 100mm

ik.right_move(ik.initial_pos, 2, 100, 100, 1)  # Right 100mm

ik.stand(ik.initial_pos, t=500)
