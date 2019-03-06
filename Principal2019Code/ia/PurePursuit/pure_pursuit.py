# -*- coding: utf-8 -*-

from math import sin, cos

from path import Path, Point
from robot import RobotPosition

"""
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
        
    def __repr__(self):
        print("robot : ", self.x, self.y, self.theta, self.v, self.omega, "\n")
        
"""

class PurePursuit:

    def __init__(self, path):
        self.path = path

    def compute(self, current_robot, look_ahead_distance, loop=False):
        p_robot = Point(current_robot.x, current_robot.y)
        theta = current_robot.theta
        if not loop:
            index, p_goal = self.path.find_goal_point(p_robot, look_ahead_distance)
        else:
            index, p_goal = self.path.find_goal_point_loop(p_robot, look_ahead_distance)
        x_body = -sin(theta)*(p_goal.x-p_robot.x) + cos(theta)*(p_goal.y-p_robot.y)
        omega_control = ((2*current_robot.speed)/(look_ahead_distance**2))*x_body
        return omega_control
