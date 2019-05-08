# -*- coding: utf-8 -*-
import sys
#from Communication.main_communication import look_ahead_distance
path = "../Communication"
sys.path.append(path)
path = "../../ia"
sys.path.append(path)

from math import sin, cos, sqrt

from PurePursuit.path_manager import Point, Path, dist, delta
#from robot import RobotPosition

import params as p


class PurePursuit:

    def __init__(self, path):
        self.path = path
        #self.look_min = min(p.L0,sqrt(path.dists[-1]/p.MAX_ACCEL)/2)
        #self.must_brake = False

    def compute(self, current_robot,loop=False):
        
        look_ahead_distance = p.L0 + p.k*current_robot.speed
        
        #Test : looke_ahead_distance égale à la distance de freinage à vitesse max
        #t_stop = current_robot.speed / p.MAX_ACCEL
        #dist_foresee = 4*(current_robot.speed*t_stop-0.5*p.MAX_ACCEL*t_stop**2)
        
        #look_ahead_distance = max(self.look_min,dist_foresee)
        
        
        p_robot = Point(current_robot.x, current_robot.y)
        theta = current_robot.theta
        
        if not loop:
            p_goal_index, p_goal = self.path.find_goal_point(p_robot, look_ahead_distance)
            closest_point_index = self.path.find_closest_point(p_robot)
        else:
            p_goal_index, p_goal = self.path.find_goal_point_loop(p_robot, look_ahead_distance)
            closest_point_index = self.path.find_closest_point_loop(p_robot)
            
        x_body = -sin(theta)*(p_goal.x-p_robot.x) + cos(theta)*(p_goal.y-p_robot.y)
        
        #print(p_goal.x, p_goal.y)
        
        #dist_to_end = dist(p_robot, self.path.points[-1])
        
        #dist_to_end = dist(p_robot, self.path.points[-1])
        #dist_to_end = self.path.dists[-1]-self.path.dists[closest_point_index]

        #print("closest_index : ", closest_point_index)
        #print("goal_point_index : ", p_goal_index)
        #print("Goal : ({}, {})".format(p_goal.x, p_goal.y))
        speed_cons = self.path.speed[closest_point_index]#150
        if speed_cons <= 1:
            speed_cons = 0
        omega_cons = ((2*speed_cons)/(look_ahead_distance**2))*x_body
        
        return omega_cons, speed_cons
