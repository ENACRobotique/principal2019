"""
   motorControl.cpp
  
    Created on: 17 mars 2019
        Author: clubRobot
"""

from math import pi

NAVIGATOR_TIME_PERIOD = 0.05


#Variables de communications avec la Teensy
RADIAN_TO_MSG_ADDER = pi
RADIAN_TO_MSG_FACTOR = 1000
SPEED_ADDER = 1<<15
XY_ADER = 1<<15
ANGULAR_SPEED_TO_MSG_FACTOR = (1<<15)/30 #max omega=30rad/s  
ANGULAR_SPEED_TO_MSG_ADDER = (1<<15) #split uint16 in two    
SERIAL_BAUDRATE = 115200
SERIAL_PATH = "/dev/ttyAMA0"
TIMEOUT = 0.01

#Variables de naviagtion et de pure pursuit
MAX_ACCEL = 600
SPEED_MAX = 500
ADMITTED_POSITION_ERROR = 5
NAVIGATOR_TIME_PERIOD = 0.05
#Definition of look_ahead distance : L=L0+k*speed
L0 = 100
k = 0.23