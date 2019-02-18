# -*- coding: utf-8 -*-

import numpy as np
from math import sin, cos, pi, sqrt


class Point:
    
    def __init__(self,x,y):
        self.x = x
        self.y = y

class RobotState:

    def __init__(self, x, y, theta, v, omega):
        self.x = x
        self.y = y
        self.theta = thetat
        self.v = v
        self.omega = omega

class Path:
    
    def __init__(self):
        self.path = np.array([])
        
     
