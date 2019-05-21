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
from time import time,sleep

from Communication import communication as com
from Communication import main_communication
from StateMachine import state_machine
from MainRobot import robot

from PurePursuit import pure_pursuit as pp
from PurePursuit import path_factory as pf
from Capteurs import USThread
from PurePursuit.path_manager import Path, Point
import params as p

if __name__ == '__main__':
    robot_pas_le_meme_nom_que_la_classe = robot.RobotPosition(0, 0, 0, 0, 0)
    print(robot_pas_le_meme_nom_que_la_classe)
    comm = main_communication.CommManager(robot_pas_le_meme_nom_que_la_classe)
    behaviour = state_machine.FSMMatch(robot_pas_le_meme_nom_que_la_classe)
    comm.sendPositionMessage()
    comm.sendLidarMessage(1, 0, 0, 0, 0)
    
    comm.start_receive_thread()
    
    Nbpoints = 10000
    path = pf.line(Nbpoints, Point(0,0), Point(750,0))
    #path = pf.polyline(Nbpoints, Point(0,0), Point(1500,500), Point(3000-500,0))
    #path = pf.circle(Nbpoints, Point(0,600), 600)
    
    #list = [Point(i, -350*sin(2*pi*i/2000)) for i in np.linspace(0,2000, Nbpoints)]
    #list += [Point(i, 350*sin(2*pi*i/2000)) for i in np.linspace(2000,0, Nbpoints)]
    #path = Path(list)
    
    #path = Path([Point(i, -300*sin(2*pi*i/2500)) for i in np.linspace(0,2500, Nbpoints)])
    
    #path = pf.lemniscate(Nbpoints, 1000, 600)
    
    
    #path = pf.polyline(Nbpoints, Point(0,0), Point(0, 500), Point(500,500), Point(500,0),Point(0,0))

    tracking = pp.PurePursuit()
    tracking.add_path(path)
    while True:#time()-t0<35:
                

        omega, speed = tracking.compute(robot_pas_le_meme_nom_que_la_classe, False)
        print("omega_cons = ", omega, "speed_cons = ", speed)
        print(robot_pas_le_meme_nom_que_la_classe)
        #print("lidar : {}, {}, {}".format(robot._lidarZone.activated_zone1(),robot._lidarZone.activated_zone2(),robot._lidarZone.activated_zone3()))
            
        comm.sendVelocityMessage(speed, omega)
        
        sleep(0.05)
            
            
            
    #np.savez('outfile.npz', x=np.array(x), y=np.array(y))
    
    