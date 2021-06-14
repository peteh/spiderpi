import os, sys
dir_path = os.path.dirname(os.path.realpath(__file__))

# The actual distance between two adjacent corners, in cm 
corners_length = 2.1

# The side length of the block is 3cm
square_length = 3

# Calibration checkerboar size, column, row, refers to the number of corner points，non checkerboard
calibration_size = (7, 7)

# Collect and calibrate image storage path
save_path = dir_path + '/calibration_images/'

# Calibration parameter storage path
calibration_param_path = dir_path + '/calibration_param'

# Map parameter store path
map_param_path = dir_path + '/map_param'
