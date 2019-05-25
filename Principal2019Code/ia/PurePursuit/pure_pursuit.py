# -*- coding: utf-8 -*-
import sys
from tkinter.constants import CURRENT
from locale import currency
# from Communication.main_communication import look_ahead_distance
path = "../Communication"
sys.path.append(path)
path = "../../ia"
sys.path.append(path)

from math import sin, cos, sqrt, pi

from PurePursuit.path_manager import Point, Path, dist, delta
from MainRobot import robot
from enum import Enum
from time import time
import params as p


class Trajectory(Enum):
    STOPPED = 0
    PATH = 1
    TURNING = 2
    ACCELERATE = 3
    DECELERATE = 4
    
class MoveSet(Enum):
    PATH_ACCEL = 0
    PATH_PURSUIT = 1
    PATH_DECEL = 2
    PATH_FINAL = 3
    TURN_BEGIN = 4
    TURN_FINISH = 5


class PurePursuit:

    def __init__(self, robot):
        self.is_stopped = False
        self.time_stop = 0;
        self.move = Trajectory.STOPPED
        self.robot = robot
        self.path = []
        self.theta_target = 0
        self.recalcul = False
        self.previous_cons = 0
        self.previous_omega = 0
        # self.must_brake = False


    def move_finished(self):
        return self.move == Trajectory.STOPPED


    def add_path(self, path):
        self.path = path
        self.move = Trajectory.PATH
        self.look_min = p.L0
        self.move_set = MoveSet.PATH_ACCEL
        
        
    def add_turn(self, turn):
        self.theta_target = turn
        self.move = Trajectory.TURNING
        self.move_set = MoveSet.TURN_BEGIN
        if (self.center_radian(self.theta_target - self.robot.theta) > 0):
            self.sgn = 1
        else:
            self.sgn = -1


    def compute(self, loop=False):
        # print("Debut compute {}".format(time()))
        if self.move == Trajectory.STOPPED:
            speed_cons = 0
            omega_cons = 0
        elif self.move == Trajectory.TURNING:
            if self.move_set == MoveSet.TURN_BEGIN:
                omega_cons, speed_cons = self.begin_turn()
            elif self.move_set == MoveSet.TURN_FINISH:
                omega_cons, speed_cons = self.turn()
        elif self.move == Trajectory.PATH:
            self.lidar_check()
            if not self.is_stopped:
                if self.move_set == MoveSet.PATH_ACCEL:
                    omega_cons, speed_cons = self.accelerate()
                elif self.move_set == MoveSet.PATH_DECEL:
                    omega_cons, speed_cons = self.decelerate_to_low_speed()
                elif self.move_set == MoveSet.PATH_PURSUIT:
                    omega_cons, speed_cons = self.compute_pure_pursuit(loop)
                elif self.move_set == MoveSet.PATH_FINAL:
                    omega_cons, speed_cons = self.decelerate(self.path.points[-1])
            else:
                omega_cons, speed_cons = self.brake()
        return omega_cons, speed_cons
    
    
    def begin_turn(self):
        speed_cons = 0
        omega_cons = self.sgn*min(p.OMEGA_MAX, p.NAVIGATOR_TIME_PERIOD * p.MAX_ACCEL_OMEGA + abs(self.robot.omega))
        if abs(self.theta_target - self.robot.theta) < 0.1:
            self.move_turn = MoveSet.TURN_FINISH
        return omega_cons, speed_cons
    
    
    def turn(self):
        speed_cons = 0
        t_rotation_stop = abs(self.robot.omega) / p.MAX_ACCEL_OMEGA
        angle_fore = self.center_radian(self.robot.theta + self.sgn * (abs(self.robot.omega) * t_rotation_stop - 1 / 2 * p.MAX_ACCEL_OMEGA * pow(t_rotation_stop, 2)))
        if abs(self.center_radian(angle_fore - self.theta_target)) < p.ADMITTED_ANGLE_ERROR:
            omega_cons = self.sgn * max(0, abs(self.robot.omega) - p.NAVIGATOR_TIME_PERIOD * p.MAX_ACCEL_OMEGA)
        else:
            if(self.sgn * (self.center_radian(self.theta_target - angle_fore)) > 0):
                omega_cons = self.sgn * min(p.OMEGA_MAX_DECER, p.NAVIGATOR_TIME_PERIOD * p.MAX_ACCEL_OMEGA + abs(self.robot.omega))
            else:
                omega_cons = self.sgn * max(0, abs(self.robot.omega) - p.NAVIGATOR_TIME_PERIOD * p.MAX_ACCEL_OMEGA)
        print("theta {}, theta target {}, omega cons {}".format( self.robot.theta, self.theta_target, omega_cons))
        return omega_cons, speed_cons
        
    
    
    def lidar_check(self):
        if self.robot._lidarZone.activated_zone3():
            self.time_stop = time()
            if not self.is_stopped:
                print("Obstacle détecté")
                self.is_stopped = True
                self.recalcul = True
                self.move_set = MoveSet.PATH_ACCEL
        elif self.is_stopped:
            if time() - self.time_stop > p.STOP_DELAY:
                    print("Obstacle fini")
                    self.is_stopped = False
    
    
    def accelerate(self):
        omega_cons = 0
        speed_cons = min(p.SPEED_MAX, self.robot.speed + p.MAX_ACCEL*p.NAVIGATOR_TIME_PERIOD)
        if speed_cons == p.SPEED_MAX:
            self.move_set = MoveSet.PATH_PURSUIT
            print("-----------------------PURSUIT------------------------")
        return omega_cons, speed_cons
    
    def brake(self):
        omega_cons = 0
        speed_cons = max(0,self.robot.speed - p.BRAKE_ACCEL*p.NAVIGATOR_TIME_PERIOD)
        return omega_cons, speed_cons
    
    
    def decelerate_to_low_speed(self):
        omega_cons = 0
        speed_cons = max(p.SPEED_MAX_DECER, self.robot.speed - p.MAX_ACCEL*p.NAVIGATOR_TIME_PERIOD)
        if speed_cons == p.SPEED_MAX_DECER:
            self.move_set = MoveSet.PATH_FINAL
        return omega_cons, speed_cons
    
    def decelerate(self, target):
        omega_cons = 0
        t_stop = self.robot.speed/(p.MAX_ACCEL-1000);
        dist_fore = (self.robot.speed*t_stop-1/2*(p.MAX_ACCEL-1000)*pow(t_stop,2))
        
        dist_objective = sqrt(pow(target.x - self.robot.x,2) + pow(target.y - self.robot.y,2));
        
        #print(" fore : {}, objective : {}".format(dist_fore,dist_objective))
        
        if abs( dist_fore - dist_objective ) < p.ADMITTED_POSITION_ERROR:
            speed_cons = max(0,-p.MAX_ACCEL*p.NAVIGATOR_TIME_PERIOD + abs(self.robot.speed))
        else:
            if(dist_fore - dist_objective > 0):
                speed_cons = max(0,abs(self.robot.speed) - p.MAX_ACCEL*p.NAVIGATOR_TIME_PERIOD)
            else:
                speed_cons = min(p.SPEED_MAX_DECER,abs(self.robot.speed) + p.MAX_ACCEL*p.NAVIGATOR_TIME_PERIOD)
                
        if dist_objective < p.ADMITTED_POSITION_ERROR and self.robot.speed < p.ADMITTED_SPEED_ERROR:
            self.move = Trajectory.STOPPED
            print("-------------------- STOPPED -------------------")
        return omega_cons, speed_cons
    
    
    def compute_pure_pursuit(self, loop):
        look_ahead_distance = p.L0 + p.k * self.robot.speed
                
        # Test : look_ahead_distance égale à la distance de freinage à vitesse max
        # t_stop = current_robot.speed / p.MAX_ACCEL
        # dist_foresee = (current_robot.speed*t_stop-0.5*p.MAX_ACCEL*t_stop**2)
        
        # look_ahead_distance = max(self.look_min,dist_foresee)
        # print("L = {}, speed = {}".format(look_ahead_distance,current_robot.speed))
        
        p_robot = Point(self.robot.x, self.robot.y)
        theta = self.robot.theta
        
        if not loop:
            closest_point_index, p_goal_index, p_goal = self.path.find_goal_point(p_robot, look_ahead_distance)
        else:
            closest_point_index, p_goal_index, p_goal = self.path.find_goal_point_loop(p_robot, look_ahead_distance)
            
        x_body = -sin(theta) * (p_goal.x - p_robot.x) + cos(theta) * (p_goal.y - p_robot.y)
        
        # print(p_goal.x, p_goal.y)
        
        # dist_to_end = dist(p_robot, self.path.points[-1])
        
        # dist_to_end = dist(p_robot, self.path.points[-1])
        # dist_to_end = self.path.dists[-1]-self.path.dists[closest_point_index]

        # print("closest_index : ", closest_point_index)
        # print("goal_point_index : ", p_goal_index)
        # print("Goal : ({}, {})".format(p_goal.x, p_goal.y))
        
        # if p_goal == self.path.points[-1] :
            # speed_cons = max(0, self.previous_cons- p.MAX_ACCEL*p.NAVIGATOR_TIME_PERIOD)
        # else:
            # speed_cons = min(p.SPEED_MAX, self.previous_cons + p.MAX_ACCEL*p.NAVIGATOR_TIME_PERIOD)
        speed_cons = p.SPEED_MAX
        omega_cons = ((2 * speed_cons) / (look_ahead_distance ** 2)) * x_body
        
        closest_point = self.path.find_closest_point(p_robot)
        
        #print("Closest : {}, total : {}".format(closest_point, self.path.length))
        
        if(self.path.length - closest_point)/self.path.length < 0.10:
            self.move_set = MoveSet.PATH_DECEL
            print("---------------------DECEL-----------------------")
        
        #print("speed : {}, L : {}, x : {}, omega : {}".format(speed_cons, look_ahead_distance, x_body, omega_cons))
        return omega_cons, speed_cons
    
    def center_radian(self, angle):
        if abs(angle) > pi:
            if angle < 0:
                while abs(angle) > pi:
                    angle += pi * 2
            else:
                while abs(angle) > pi:
                    angle -= pi * 2
        return angle
                    
