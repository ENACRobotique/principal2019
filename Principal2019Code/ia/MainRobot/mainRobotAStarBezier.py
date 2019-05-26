import serial

import sys
path = "../PathFinder"
sys.path.append(path)
path = "../Capteurs"
sys.path.append(path)
path = "../../ia"
sys.path.append(path)
path = "../Communication"
sys.path.append(path)
path = "../StateMachine"
sys.path.append(path)
path = "../PurePursuit"
sys.path.append(path)


import numpy as np
from math import sin, pi
from time import time

from Communication import communication as com
from Communication import main_communication
from StateMachine import state_machine
from MainRobot import robot

from PurePursuit import pure_pursuit as pp
from PurePursuit import path_factory as pf
from shapely.geometry import Point
from PathFinder import AStar
from PathFinder import Bezier
from PathFinder import TestPP
from PathFinder import Map
import math
import matplotlib.pyplot as plt

from PurePursuit import pure_pursuit as pp
from PurePursuit import path_factory as pf
from PurePursuit import path_manager as pm
from Capteurs import USThread
from PurePursuit.path_manager import Path, Point
import params as p

if __name__ == '__main__':
    
    #goal and start
    sx = 300.0  # [mm]
    sy = 450.0  # [mm]
    gx = 1300.0  # [mm]
    gy = 450.0  # [mm]
    robot = robot.RobotPosition(300, 450, math.pi/2, 0, 0)

    comm = main_communication.CommManager(robot)
    behaviour = state_machine.FSMMatch(robot)
    comm.sendPositionMessage()
    comm.sendLidarMessage(1, 0, 0, 0, 0)
    
        #Map
    MyMap = Map.Map()
    obstacle = []#rectangle x = 700 - 800 y = 300 - 600
    for i in range(300,600,10) :
        obstacle.append(Point(700,i))
    for i in range(300,600,10) :
        obstacle.append(Point(800,i))
    for i in range(700,800,10) :
        obstacle.append(Point(i,300))
    for i in range(700,800,10) :
        obstacle.append(Point(i,600))  
    MyMap.add_obstacles(obstacle)
    

    #MyMap.plot_map(sx, sy, gx, gy)
    
    # initial state
    
    #path Creation
    
    #AStar parameters
    grid_size = 70  # [mm] #précision du A*
    robot_size = 150  # [mm] #distance de sécurité avec les obstacles i.e. taille du robot/2
    
    #AStar planning
    ox,oy = MyMap.get()
    rx, ry = AStar.a_star_planning(sx, sy, gx, gy, ox, oy, grid_size, robot_size)
    
    #calcul controlPoints used for Bezier
    controlPoints = AStar.calc_control_points(rx,ry,sx,sy,gx,gy)
    NPoints = 100 # number of points(for Bezier) between each control points calculated by Astar
    
    #Bezier
    Path_truc = Bezier.calc_bezier_path(controlPoints, n_points=NPoints*len(controlPoints))
    
    truc = []
    for point in Path_truc:
        truc.append(Point(point[0],point[1]))
    
    
    #calcul cx,cy use for Pure Poursuit
    #cx,cy = Bezier.calc_cxcy(Path)
    
    new_path = pm.Path(truc)
    new_path.compute_speed(0.2,0.85,p.SPEED_MAX)
    
    tracking = pp.PurePursuit(new_path)
    
    #parameter for Pure-Poursuite
    k = 0.1  # look forward gain
    Lfc = 2.0  # look-ahead distance
    Kp = 1.0  # speed proportional gain
    dt = 0.1  # [s] time increment for simulation
    L = 2.9  # [mm] wheel base of vehicle
    target_speed = 300.0 / 3.6  # [mm/s]
    
    
    #tracking = TestPP.PurePursuit(cx,cy,robot)
    
    x = []
    y = []
    
    t0 = time()
    while True:#time()-t0<35:
        
        comm.receive_message()
                

        omega, speed = tracking.compute(robot,False)
        print("omega_cons = ", omega, "speed_cons = ", speed)
        #print("lidar : {}, {}, {}".format(robot._lidarZone.activated_zone1(),robot._lidarZone.activated_zone2(),robot._lidarZone.activated_zone3()))
            
        comm.sendVelocityMessage(speed, omega)