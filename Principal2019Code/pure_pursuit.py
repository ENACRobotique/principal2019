# -*- coding: utf-8 -*-

import numpy as np
from math import sin, cos, pi, sqrt


class Point:
    
    def __init__(self,x,y):
        self.x = x
        self.y = y
        
    def __repr__(self):
        print("x = : ", self.x, "y = : ", self.y, "\n")

class Robot:

    def __init__(self, x, y, theta, v, omega):
        self.x = x
        self.y = y
        self.theta = theta
        self.v = v
        self.omega = omega
        
    def update(self, x, y, theta, v, omega):
        self.x = x
        self.y = y
        self.theta = theta
        self.v = v
        self.omega = omega
        
    def set_path(self, path):
        self.path = path
        
    def __repr__(self):
        print("robot : ", self.x, self.y, self.theta, self.v, self.omega, "\n")
        

        #self.goal = Point(0,0)
        
    def calculate_goal_point(self, robot):
        dx = [robot.x - point.x for point in self.path]     
        dy = [robot.y - point.y for point in self.path] 
        d = [sqrt(dx**2 
    
     
