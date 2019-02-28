# -*- coding: utf-8 -*-

import numpy as np
from math import sqrt

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

    def __init__(self, points):
        self.points = points  #array of points from class Point
        self.headings = self.compute_headings()
        self.dists = self.compute_dist()
        self.last_passed_index = 0
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

    def find_closest_point(self, p0):
        dist_to_p0 = np.array([dist(p0, p) for p in self.points[self.last_passed_index:]])
        i = np.argmin(dist_to_p0) + self.last_passed_index
        return i

    def find_goal_point(self, p0, look_ahead_distance):
        index_start = self.find_closest_point(p0)
        p_start = self.points[index_start]
        index = index_start
        dist_to_p0 = dist(p0, p_start)
        path_length = len(self.points)
        while index < path_length and dist_to_p0 < look_ahead_distance:
            index += 1
            dist_to_p0 = dist(p0, self.points[index])
        return index, self.points[index]
