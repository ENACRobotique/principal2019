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

"""Classe des différentes trajectoires que le robot peut suivre"""
class Trajectory(Enum):
    STOPPED = 0
    PATH = 1
    TURNING = 2
    ACCELERATE = 3
    DECELERATE = 4
    PRECISE_MOVE = 5
    
"""Différents mouvements que le robot peut effectuer
Ces mouvements s'inscrivent dans des trajectoires"""
class MoveSet(Enum):
    PATH_ACCEL = 0
    PATH_PURSUIT = 1
    PATH_DECEL = 2
    PATH_FINAL = 3
    
    TURN_BEGIN = 4
    TURN_TRANSITION = 5
    TURN_FINISH = 6
    
    PRECISE_MOVE = 7
    

"""Classe de locomotion générale.
Permet d'ajouter des chemins, des rotations et des parcours précis.
Permet de calculer les consignes à envoyer à la teensy"""
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
        self.depassement = False
        # self.must_brake = False



    def move_finished(self):
        return self.move == Trajectory.STOPPED



    def add_path(self, path, forward = True):
        self.path = path
        self.move = Trajectory.PATH
        #self.look_min = p.L0
        self.move_set = MoveSet.PATH_ACCEL
        self.forward = forward
        self.fini = False
        
        
        
    def add_turn(self, turn):
        self.theta_target = pi*turn/180
        self.move = Trajectory.TURNING
        self.move_set = MoveSet.TURN_BEGIN
        if (self.center_radian(self.theta_target - self.robot.theta) > 0):
            self.sgn = 1
        else:
            self.sgn = -1
            
    def add_precise_path(self, distance, forward = True):
        self.forward = forward
        self.move = Trajectory.PRECISE_MOVE
        self.distance = distance
        self.target = Point(self.robot.x + distance * cos(self.robot.theta), self.robot.y + distance*sin(self.robot.theta))


    def compute(self, loop=False):
        # print("Debut compute {}".format(time()))
        if self.move == Trajectory.STOPPED:
            speed_cons = 0
            omega_cons = 0
        elif self.move == Trajectory.TURNING:
            if self.move_set == MoveSet.TURN_BEGIN:
                omega_cons, speed_cons = self.begin_turn()
            elif self.move_set == MoveSet.TURN_TRANSITION:
                omega_cons, speed_cons = self.transition_turn()
            elif self.move_set == MoveSet.TURN_FINISH:
                omega_cons, speed_cons = self.turn()
        elif self.move == Trajectory.PATH:
            self.lidar_check()
            if not self.is_stopped:
                if self.move_set == MoveSet.PATH_ACCEL:
                    omega_cons, speed_cons = self.compute_pure_pursuit(loop)
                elif self.move_set == MoveSet.PATH_DECEL:
                    omega_cons, speed_cons = self.decelerate_to_low_speed()
                elif self.move_set == MoveSet.PATH_PURSUIT:
                    omega_cons, speed_cons = self.compute_pure_pursuit(loop)
                elif self.move_set == MoveSet.PATH_FINAL:
                    omega_cons, speed_cons = self.low_speed_move(self.path.points[-1], p.SPEED_MAX_DECER)
                if not self.forward:
                    speed_cons = -speed_cons
                    omega_cons = -omega_cons
            else:
                omega_cons, speed_cons = self.brake()
        elif self.move == Trajectory.PRECISE_MOVE:
            omega_cons, speed_cons = self.low_speed_move(self.target, p.SPEED_MAX_PRECISE)
            if not self.forward:
                speed_cons = -speed_cons
                omega_cons = - omega_cons
        return omega_cons, speed_cons
    
    
    
    def begin_turn(self):
        speed_cons = 0
        omega_cons = self.sgn*min(p.OMEGA_MAX, p.NAVIGATOR_TIME_PERIOD * p.MAX_ACCEL_OMEGA + abs(self.previous_omega))
        if abs(self.theta_target - self.robot.theta) < 0.5:
            self.move_set = MoveSet.TURN_TRANSITION
            print("-----------------FIN------------------------")
        print("theta {}, theta target {}".format( self.robot.theta, self.theta_target, omega_cons))
        #print("speed {} omega_cons {}".format(self.robot.omega, omega_cons))
        self.previous_omega = omega_cons
        return omega_cons, speed_cons
    
    def transition_turn(self):
        print("TRANSITION")
        speed_cons = 0
        omega_cons = self.sgn * max(p.OMEGA_MAX_DECER, abs(self.robot.omega) - p.NAVIGATOR_TIME_PERIOD * p.MAX_ACCEL_OMEGA)
        if abs(omega_cons) == p.OMEGA_MAX_DECER:
            self.move_set = MoveSet.TURN_FINISH
        return omega_cons, speed_cons
    
    
    
    def turn(self):
        #print("---------------------TURN FINISH--------------------")
        speed_cons = 0
        t_rotation_stop = abs(self.robot.omega) / p.MAX_ACCEL_OMEGA
        angle_fore = self.center_radian(self.robot.theta + self.sgn * (abs(self.robot.omega) * t_rotation_stop - 1 / 2 * p.MAX_ACCEL_OMEGA * pow(t_rotation_stop, 2)))
        if abs(self.center_radian(angle_fore - self.theta_target)) < p.ADMITTED_ANGLE_ERROR:
            print("On est dedans")
            omega_cons = self.sgn * max(0, abs(self.robot.omega) - p.NAVIGATOR_TIME_PERIOD * p.MAX_ACCEL_OMEGA)
        else:
            if(self.sgn * (self.center_radian(self.theta_target - angle_fore)) > 0):
                omega_cons = self.sgn * min(p.OMEGA_MAX_DECER, p.NAVIGATOR_TIME_PERIOD * p.MAX_ACCEL_OMEGA + abs(self.robot.omega))
            else:
                omega_cons = self.sgn * max(0, abs(self.robot.omega) - p.NAVIGATOR_TIME_PERIOD * p.MAX_ACCEL_OMEGA)
        print("theta {}, theta target {}, omega cons {}".format( self.robot.theta, self.theta_target, omega_cons))
        return omega_cons, speed_cons
    
    
    
    def lidar_check(self):
        if self.robot._lidarZone.activated_zone1():
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
        speed_cons = min(p.SPEED_MAX, abs(self.robot.speed) + p.MAX_ACCEL*p.NAVIGATOR_TIME_PERIOD)
        if speed_cons > (p.SPEED_MAX/2):
            self.move_set = MoveSet.PATH_PURSUIT
            print("-----------------------PURSUIT------------------------")
        return omega_cons, speed_cons
    
    
    
    def brake(self):
        omega_cons = 0
        speed_cons = max(0,abs(self.robot.speed) - p.BRAKE_ACCEL*p.NAVIGATOR_TIME_PERIOD)
        return omega_cons, speed_cons
    
    
    
    def decelerate_to_low_speed(self):
        omega_cons = 0
        speed_cons = max(p.SPEED_MAX_DECER, abs(self.robot.speed) - p.MAX_ACCEL*p.NAVIGATOR_TIME_PERIOD)
        if speed_cons == p.SPEED_MAX_DECER:
            self.move_set = MoveSet.PATH_FINAL
        return omega_cons, speed_cons
    
    
    
    def low_speed_move(self, target, maximum_speed):
        omega_cons = 0
        t_stop = abs(self.robot.speed)/(p.MAX_ACCEL);
        dist_fore = (abs(self.robot.speed)*t_stop-1/2*(p.MAX_ACCEL)*pow(t_stop,2))
        
        dist_objective = sqrt(pow(target.x - self.robot.x,2) + pow(target.y - self.robot.y,2));
        
        #print(" fore : {}, objective : {}".format(dist_fore,dist_objective))
        
        if abs( dist_fore - dist_objective ) < p.ADMITTED_POSITION_ERROR:
            print("On y est")
            self.fini = True
            speed_cons = max(0,-p.MAX_ACCEL*p.NAVIGATOR_TIME_PERIOD + abs(self.robot.speed))
        else:
            if(dist_fore - dist_objective > 0):
                speed_cons = max(0,abs(self.robot.speed) - p.MAX_ACCEL*p.NAVIGATOR_TIME_PERIOD)
            else:
                speed_cons = min(maximum_speed,abs(self.robot.speed) + p.MAX_ACCEL*p.NAVIGATOR_TIME_PERIOD)
                
        if dist_objective < p.ADMITTED_POSITION_ERROR:
            self.move = Trajectory.STOPPED
            print("-------------------- STOPPED -------------------")
        return omega_cons, speed_cons
    
    
    
    def compute_pure_pursuit(self, loop):
        look_ahead_distance = p.L0 + p.k * abs(self.robot.speed)
                
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
        speed_cons = min(p.SPEED_MAX, abs(self.robot.speed) + p.MAX_ACCEL*p.NAVIGATOR_TIME_PERIOD)
        omega_cons = ((2 * abs(speed_cons)) / (look_ahead_distance ** 2)) * x_body
        
        #print("Closest : {}, total : {}".format(closest_point, self.path.length))
        
        if dist(p_robot,self.path.points[-1]) < 250:
            self.move_set = MoveSet.PATH_DECEL
            print("---------------------DECEL-----------------------")
        
        #print("speed : {}, L : {}, x : {}, omega : {}".format(speed_cons, look_ahead_distance, x_body, omega_cons))
        return omega_cons, speed_cons
    
    
    
    def center_radian(self, angle):
        #print(" angle avant {}".format(180*angle/pi))
        if abs(angle) > pi:
            if angle < 0:
                while abs(angle) > pi:
                    angle += pi * 2
            else:
                while abs(angle) > pi:
                    angle -= pi * 2
        #print(" angle après {}".format(180*angle/pi))
        return angle
                    
