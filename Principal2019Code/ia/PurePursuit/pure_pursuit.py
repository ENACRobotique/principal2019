# -*- coding: utf-8 -*-
import sys
path = "../Communication"
sys.path.append(path)

from math import sin, cos

from path import Point, Path, dist, delta
from robot import RobotPosition

MAX_ACCEL = 800
SPEED_MAX = 150
ADMITTED_POSITION_ERROR = 5
NAVIGATOR_TIME_PERIOD = 0.05


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

    def compute(self, current_robot, look_ahead_distance, break_distance,loop=False):
        p_robot = Point(current_robot.x, current_robot.y)
        theta = current_robot.theta
        if not loop:
            _, p_goal = self.path.find_goal_point(p_robot, look_ahead_distance)
        else:
            _, p_goal = self.path.find_goal_point_loop(p_robot, look_ahead_distance)
        x_body = -sin(theta)*(p_goal.x-p_robot.x) + cos(theta)*(p_goal.y-p_robot.y)
        
        #dist_to_end = dist(p_robot, self.path.points[-1])
        
        t_stop = current_robot.speed / MAX_ACCEL
        dist_fore = current_robot.speed*t_stop-1/2*MAX_ACCEL*t_stop**2
        
        dist_to_end = dist(p_robot, self.path.points[-1])
        
        print("dist_fore-dist_to_end = ", dist_fore-dist_to_end)

        #Si le point estimé est suffisamment proche du point voulu, on décélére, sinon on accélére jusqu'à la vitesse maximale.
        if abs(dist_fore - dist_to_end) < ADMITTED_POSITION_ERROR :
            speed_cons = max(0, -2*MAX_ACCEL*NAVIGATOR_TIME_PERIOD + abs(current_robot.speed))
            print("proche")
            
        else:
            if dist_fore - dist_to_end > 0:
                speed_cons = max(0,abs(current_robot.speed) - MAX_ACCEL*NAVIGATOR_TIME_PERIOD)
                print("1")
            
            else:
                speed_cons = min(SPEED_MAX,abs(current_robot.speed) + MAX_ACCEL*NAVIGATOR_TIME_PERIOD)
                print("2")
        
        
        omega_cons = ((2*current_robot.speed)/(look_ahead_distance**2))*x_body
        return speed_cons, omega_cons
