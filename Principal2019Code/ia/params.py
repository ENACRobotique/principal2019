"""
   motorControl.cpp
  
    Created on: 17 mars 2019
        Author: clubRobot
"""

from math import pi
from threading import RLock

VERROU = RLock()
OBSTACLE_DETECTED = False

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
STOP_DELAY = 1
MATCH_DURATION = 100

#Couleurs des côtés du terrain et tirette
UNDEFINED = 0
VIOLET = 1
JAUNE = 2

#Variables de navigation et de pure pursuit
MAX_ACCEL = 700
BRAKE_ACCEL = 2400
SPEED_MAX = 300
SPEED_MAX_DECER = 100
SPEED_MAX_PRECISE = 50
OMEGA_MAX = 0.5
OMEGA_MAX_DECER = 0.1
MAX_ACCEL_OMEGA = 1
ADMITTED_ANGLE_ERROR = 0.01
ADMITTED_SPEED_ERROR = 10;
ADMITTED_POSITION_ERROR = 15
NAVIGATOR_TIME_PERIOD = 0.15
#Definition of look_ahead distance : L=L0+k*speed
L0 = 70 #100
k = 0.4 #0.23

#Dimensions du robot pour aider au positionnement
ROBOT_TAILLE_X = 30
ROBOT_TAILLE_Y = 26