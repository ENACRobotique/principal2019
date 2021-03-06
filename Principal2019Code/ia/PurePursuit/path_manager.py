# -*- coding: utf-8 -*-
import sys
from params import NAVIGATOR_TIME_PERIOD
path="../../ia"
sys.path.append(path)

import numpy as np
from math import sqrt
from time import time

import params as p

class Point:
    def __init__(self,x,y):
        self.x = x
        self.y = y
        
        
    def __repr__(self):
        print("x = : ", self.x, "y = : ", self.y, "\n")



def delta(p1, p2):
    dx = p2.x - p1.x
    dy = p2.y - p1.y
    return Point(dx, dy)


def dist(p1, p2):
    dx = p2.x - p1.x
    dy = p2.y - p1.y
    dist = sqrt(dx**2 + dy**2)
    return dist



class Path:

    def __init__(self, points, top_of_climb, top_of_descent, Vmax):
        self.points = points  #array of points from class Point
        self.length = len(self.points)
        #self.headings = self.compute_headings()
        self.compute_dist()
        #self.compute_curvature()
        self.last_passed_index = 0
        
        self.last_goal_index = 0
        
        self.compute_speed(top_of_climb, top_of_descent, Vmax)
        """if len(headings) > 0:
            self.headings = headings
        else:
            self.compute_headings()
        if len(dists) > 0:
            self.dists = dists
        else:
            self.compute_dists()"""


    def compute_headings(self):
        self.headings = np.zeros(len(self.points))
        for i, _ in enumerate(self.points):
            if i==0:
                dp = delta(self.points[0], self.points[1])
            elif i==len(self.points)-1:
                dp = delta(self.points[-2], self.points[-1])
            else:
                dp = delta(self.points[i-1], self.points[i+1])
                self.headings[i] = np.arctan2(dp.y, dp.x)   
         
                           
    def compute_dist(self):
        n = len(self.points)
        self.dists = np.zeros(n)
        self.dists[0] = 0
        for i in range(0, n-1):
            self.dists[i+1] = self.dists[i] + dist(self.points[i+1], self.points[i])


    def compute_curvature(self):
        self.curvature = np.zeros(len(self.points))
        for i in range(1, len(self.points) - 1):
            (x1, y1) = self.points[i-1].x, self.points[i-1].y
            (x2, y2) = self.points[i].x, self.points[i].y
            (x3, y3) = self.points[i+1].x, self.points[i+1].y
            d1, d2, d3 = x1**2+y1**2, x2**2+y2**2, x3**2+y3**2
            A = np.linalg.det(np.array([[x1, y1, 1], [x2, y2, 1], [x3, y3, 1]]))
            B = -np.linalg.det(np.array([[d1, y1, 1], [d2, y2, 1], [d3, y3, 1]]))
            C = np.linalg.det(np.array([[d1, x1, 1], [d2, x2, 1], [d3, x3, 1]]))
            D = -np.linalg.det(np.array([[d1, x1, y1], [d2, x2, y2], [d3, x3, y3]]))
            if abs(A) < 1e-9:
                self.curvature[i] = 0.
            else:
                curve = (A ** 2)/(0.25*(B ** 2 + C ** 2 - 4 * A * D))
                curve = np.sqrt(curve)
                self.curvature[i] = curve
                a, b = self.points[i].x-self.points[i-1].x, self.points[i].y-self.points[i-1].y
                c, d = self.points[i+1].x-self.points[i-1].x, self.points[i+1].y-self.points[i-1].y
                cross = np.linalg.det([[a, c],[b, d]])
                self.curvature[i] *= np.sign(cross)
        self.curvature[0] = self.curvature[1]
        self.curvature[-1] = self.curvature[-2]


    def compute_speed(self, top_of_climb, top_of_descent, Vmax):# TOC, TOD percentage [0,1]
        self.speed = np.array([])
        nb_points = len(self.points)
        iTOC = int(nb_points*top_of_climb)
        iTOD = int(nb_points*top_of_descent)

        #parabole
        """for i in range(iTOC+1):
            a = Vmax/iTOC**2
            self.speed = np.append(self.speed, a*i**2)
            
        for i in range(iTOC+1,iTOD):
            self.speed = np.append(self.speed, p.SPEED_MAX)   
             
        for i in range(iTOD, nb_points):
            a = Vmax/(iTOD-(nb_points-1))**2
            self.speed = np.append(self.speed, a*(i-(nb_points-1))**2)"""
            
        #trapeze
        for i in range(iTOC+1):
            a = Vmax/iTOC
            self.speed = np.append(self.speed, min(Vmax,a*i+40))
            
        for i in range(iTOC+1,iTOD):
            self.speed = np.append(self.speed, p.SPEED_MAX)   
             
        for i in range(iTOD, nb_points):
            a = Vmax/(iTOD-(nb_points-1))
            self.speed = np.append(self.speed, a*(i-(nb_points-1)))
            
    def compute_speed_2(self):
        nb_points = len(self.points)
        self.speed = np.array([0]*nb_points)
        for i in range(1,int(nb_points/2)):
            t_move_1 = dist(self.points[i], self.points[i-1])/self.speed[i-1]
            self.speed[i] = min(self.speed[i-1] + p.MAX_ACCEL*t_move_1, p.SPEED_MAX)
            t_move_2 = dist(self.points[nb_points -1 -i], self.points[nb_points -i])/self.speed[nb_points -i]
            self.speed[nb_points -1 -i] = min(self.speed[nb_points -i] + p.MAX_ACCEL*t_move_2, p.SPEED_MAX)
        self.speed[int(nb_points/2)] = self.speed[int(nb_points/2)-1]
        
    def troncate_path(self, top_of_climb, top_of_descent, Vmax = p.SPEED_MAX):
        print("TRONCATE")
        self.points = self.points[self.last_passed_index:]
        self.last_passed_index = 0
        self.last_goal_index = 0
        print("length_direct : {}\n".format(len(self.points)))
        self.length = len(self.points)
        print("length : {}\n".format(self.length))
        self.compute_dist()
        if self.length > 1:
            self.compute_speed(top_of_climb,top_of_descent,Vmax)

    def find_closest_point(self, p0, max_index=100):
        dist_to_p0 = np.array([dist(p0, p) for p in self.points[self.last_passed_index:self.last_passed_index+max_index]])
        i = np.argmin(dist_to_p0) + self.last_passed_index
        self.last_passed_index = i
        return i
    
        
    def find_closest_point_loop(self, p0, max_index=100):
        i = self.find_closest_point(p0, max_index)
        end_reached = False
        if i == len(self.points)-1 and dist(p0, self.points[0]) <= dist(p0, self.points[-1]):
            i = 0
            end_reached = True
            self.last_passed_index = 0
        return i


    def find_goal_point(self, p0, look_ahead_distance):
        #index_begin = self.find_closest_point(p0) #index at the start
        index_begin = self.last_goal_index
        index = index_begin
        p_start = self.points[index]
        dist_to_p0 = dist(p0, p_start)
        path_length = len(self.points)
        #print("Begin while : {}".format(time()))
        while index < path_length-1 and dist_to_p0 < look_ahead_distance and (index - index_begin < 200):
            index += 1
            dist_to_p0 = dist(p0, self.points[index])
        #print("indexs : begin {}, end {} ".format(index_begin,index))
        #print("End while : {}".format(time()))
        self.last_goal_index = index
        return index_begin, index, self.points[index]
      
        
    def find_goal_point_loop(self, p0, look_ahead_distance):
        #index_begin = self.find_closest_point_loop(p0) #index at the start
        index_begin = self.last_goal_index
        index = index_begin
        p_start = self.points[index]
        dist_to_p0 = dist(p0, p_start)
        path_length = len(self.points)
        while index < path_length-1 and dist_to_p0 < look_ahead_distance:
            index += 1
            dist_to_p0 = dist(p0, self.points[index])
        if index == len(self.points)-1:
            index = 0
            dist_to_p0 = dist(p0, self.points[index])
            while dist_to_p0 < look_ahead_distance:
                index += 1
                dist_to_p0 = dist(p0, self.points[index])
        self.last_goal_index
        return index_begin, index, self.points[index]
