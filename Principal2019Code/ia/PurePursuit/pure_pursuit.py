# -*- coding: utf-8 -*-
import sys
#from Communication.main_communication import look_ahead_distance
path = "../Communication"
sys.path.append(path)
path = "../../ia"
sys.path.append(path)

from math import sin, cos, sqrt

from path import Point, Path, dist, delta
from robot import RobotPosition

import params as p


def slope_model(x0, x1, slope, dt):
    x_dot = (x0-x1)/dt
    if x_dot < slope:
        x = x0+slope*dt
    else:
        x = x0
    return x
    
    

class PurePursuit:

    def __init__(self, path):
        self.path = path
        self.look_min = min(p.L0,sqrt(path.dists[-1]/p.MAX_ACCEL)/2)
        #self.must_brake = False

    def compute(self, current_robot,loop=False):
        
        look_ahead_distance = p.L0 + p.k*current_robot.speed
        
        #Test : looke_ahead_distance égale à la distance de freiange à vitesse max
        t_stop = current_robot.speed / p.MAX_ACCEL
        dist_foresee = 4*(current_robot.speed*t_stop-0.5*p.MAX_ACCEL*t_stop**2)
        
        #look_ahead_distance = max(self.look_min,dist_foresee)
        
        print("L = ", look_ahead_distance)
        
        p_robot = Point(current_robot.x, current_robot.y)
        theta = current_robot.theta
        if not loop:
            _, p_goal = self.path.find_goal_point(p_robot, look_ahead_distance)
            closest_point_index = self.path.find_closest_point(p_robot)
        else:
            _, p_goal = self.path.find_goal_point_loop(p_robot, look_ahead_distance)
            closest_point_index = self.path.find_closest_point_loop(p_robot)
            
        x_body = -sin(theta)*(p_goal.x-p_robot.x) + cos(theta)*(p_goal.y-p_robot.y)
        
        #print(p_goal.x, p_goal.y)
        
        #dist_to_end = dist(p_robot, self.path.points[-1])
        
        #dist_to_end = dist(p_robot, self.path.points[-1])
        dist_to_end = self.path.dists[-1]-self.path.dists[closest_point_index]
        
        
        print("dist_foresee-dist_to_end = ", dist_foresee-dist_to_end)
        
        """
        #another test
        if abs(dist_foresee - dist_to_end) <= p.ADMITTED_POSITION_ERROR :
            self.must_brake = True or self.must_brake
            
        if self.must_brake:
            speed_cons = max(0, abs(current_robot.speed) - p.MAX_ACCEL*p.NAVIGATOR_TIME_PERIOD)
        else :
            speed_cons = min(p.SPEED_MAX,abs(current_robot.speed) + p.MAX_ACCEL*p.NAVIGATOR_TIME_PERIOD)
            
        omega_cons = ((2*speed_cons)/(look_ahead_distance**2))*x_body"""
            
        
        """
        #Si le point estimé est suffisamment proche du point voulu, on décélére, sinon on accélére jusqu'à la vitesse maximale.
        if abs(dist_foresee - dist_to_end) <= p.ADMITTED_POSITION_ERROR :
            speed_cons = max(0, abs(current_robot.speed) - p.MAX_ACCEL*p.NAVIGATOR_TIME_PERIOD)
            omega_cons = ((2*speed_cons)/(look_ahead_distance**2))*x_body
            print("proche")
            
        else:
            if dist_foresee - dist_to_end > 0:
                speed_cons = max(0,abs(current_robot.speed) - p.MAX_ACCEL*p.NAVIGATOR_TIME_PERIOD)
                omega_cons = ((2*speed_cons)/(look_ahead_distance**2))*x_body
                print("1")
            
            else:
                speed_cons = min(p.SPEED_MAX,abs(current_robot.speed) + p.MAX_ACCEL*p.NAVIGATOR_TIME_PERIOD)
                omega_cons = ((2*speed_cons)/(look_ahead_distance**2))*x_body
                print("2")
        """
        
        #Third test
        if(p_goal == self.path.points[-1]):
            speed_cons = max(0, abs(current_robot.speed) - p.MAX_ACCEL*p.NAVIGATOR_TIME_PERIOD)
            omega_cons = ((2*speed_cons)/(look_ahead_distance**2))*x_body
            print("proche")
        else:
            if dist_foresee - dist_to_end > 0:
                speed_cons = max(0,abs(current_robot.speed) - p.MAX_ACCEL*p.NAVIGATOR_TIME_PERIOD)
                omega_cons = ((2*speed_cons)/(look_ahead_distance**2))*x_body
                print("1")
            
            else:
                speed_cons = min(p.SPEED_MAX,abs(current_robot.speed) + p.MAX_ACCEL*p.NAVIGATOR_TIME_PERIOD)
                omega_cons = ((2*speed_cons)/(look_ahead_distance**2))*x_body
                print("2")
        
        return speed_cons, omega_cons
