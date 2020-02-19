"""
CAR CONFIG

This file is read by your car application's manage.py script to change the car
performance.

EXMAPLE
-----------
import dk
cfg = dk.load_config(config_path='~/d2/config.py')
print(cfg.CAMERA_RESOLUTION)

"""


import os

#PATHS
CAR_PATH = PACKAGE_PATH = os.path.dirname(os.path.realpath(__file__))
DATA_PATH = os.path.join(CAR_PATH, 'data')
MODELS_PATH = os.path.join(CAR_PATH, 'models')

#VEHICLE
DRIVE_LOOP_HZ = 20
MAX_LOOPS = 100000

#CAMERA
IMAGE_W = 160 #(120, 160) #(height, width)
IMAGE_H = 120
CAMERA_FRAMERATE = DRIVE_LOOP_HZ

#STEERING
STEERING_CHANNEL = 1
STEERING_LEFT_PWM = 310         # PGPS: 276, Kiwi:310
STEERING_RIGHT_PWM = 580        # PGPS: 450, Kiwi:580

#THROTTLE
THROTTLE_CHANNEL = 2
THROTTLE_FORWARD_PWM = 425      # PGPS: 480, Kiwi:435
THROTTLE_STOPPED_PWM = 405      # PGPS: 391, Kiwi:405
THROTTLE_REVERSE_PWM = 270      # PGPS: 270, Kiwi:275

################ GPS PROJECT ###################
#CONTROLLER
THROTTLE_P_GAIN = 1
STEERING_P_GAIN = 0.75           # Changing from 0.5 to 0.75

#GPS SERIAL
PORT = '/dev/ttyS0'
BAUD_RATE = 9600
TIMEOUT =  1  # seconds
################ GPS PROJECT ###################

#TRAINING
BATCH_SIZE = 128
TRAIN_TEST_SPLIT = 0.8


#JOYSTICK
USE_JOYSTICK_AS_DEFAULT = True
JOYSTICK_MAX_THROTTLE = 0.3
JOYSTICK_STEERING_SCALE = 0.85
AUTO_RECORD_ON_THROTTLE = True

#RNN or 3D
SEQUENCE_LENGTH = 3
IMAGE_DEPTH = 3

#IMU
HAVE_IMU = False

#BEHAVIORS
TRAIN_BEHAVIORS = False
BEHAVIOR_LIST = ['Left_Lane', "Right_Lane"]
BEHAVIOR_LED_COLORS =[ (0, 10, 0), (10, 0, 0) ] #RGB tuples 0-100 per chanel
