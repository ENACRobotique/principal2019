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
from multiprocessing import Process 

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
    robot = robot.RobotPosition(0, 0, 0, 0, 0)
    comm = main_communication.CommManager(robot)
    tracking = pp.PurePursuit(robot)
    behaviour = state_machine.FSMMatch(robot, tracking, comm)
    comm.sendPositionMessage()
    #comm.sendLidarMessage(1, 0, 0, 0, 0)
    
    sleep(2)
    #Evite les problèmes de messages parasites après un test
    robot.update(885,223,0,0,0)
    comm.sendPositionMessage()
    comm.sendLidarMessage(1, 0, 0, 0, 0)
    
    Nbpoints = 2500
    #path = pf.line(Nbpoints, Point(0,0), Point(2000,0))
    #path = pf.polyline(Nbpoints, Point(0,0), Point(800,300), Point(1600,-200))
    path = pf.polyline(Nbpoints, Point(885,223), Point(1000,223),Point(1200,240), Point(1250,280), Point(1350,350), Point(1450,500), Point(1520,600),  Point(1520,900)) 
    #path = pf.circle(Nbpoints, Point(0,400), 400)
    
    #list = [Point(i, -350*sin(2*pi*i/2000)) for i in np.linspace(0,2000, Nbpoints)]
    #list += [Point(i, 350*sin(2*pi*i/2000)) for i in np.linspace(2000,0, Nbpoints)]
    #path = Path(list)
    
    #path = Path([Point(i, -300*sin(2*pi*i/2500)) for i in np.linspace(0,2500, Nbpoints)])
    
    #path = pf.lemniscate(Nbpoints, 1000, 600)
    
    
    #path = pf.polyline(Nbpoints, Point(0,0), Point(0, 500), Point(500,500), Point(500,0),Point(0,0))
    
    

    #tracking.add_turn(-90)
    tracking.add_path(path)
    comm.flush()
    
    #comm.start_receive_thread()
    time_update = time()
    while True:#time()-t0<35:
        #d =  Process(target=comm.start_receive_thread)
        
        #d.start()
        #print("Dans la boucle !")
        comm.receive_message()
        robot.updateLidarGPIO()
        
        
        if time() - time_update > p.NAVIGATOR_TIME_PERIOD:
            #print("TEMPS DEBUT : {}".format(time()))
            time_update = time()
            
            behaviour.loop()
            
            omega, speed = tracking.compute(False)
            #print("omega_cons = ", omega, "speed_cons = ", speed)
            print(robot)
            #print(robot._lidarZone)
            #print("lidar : {}, {}, {}".format(robot._lidarZone.activated_zone1(),robot._lidarZone.activated_zone2(),robot._lidarZone.activated_zone3()))
            
            comm.sendVelocityMessage(speed, omega)
            #print("TEMPS FIN : {}".format(time()))
            #d.join()
            
            
    #np.savez('outfile.npz', x=np.array(x), y=np.array(y))

    
    

    