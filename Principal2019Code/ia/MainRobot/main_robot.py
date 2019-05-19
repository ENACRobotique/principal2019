import serial


import numpy as np
from math import sin, pi
from time import time

from Communication import communication as com
from Communication import main_communication
from StateMachine import state_machine
from MainRobot import robot

import sys
path = "../PurePursuit"
sys.path.append(path)
path = "../Ultrason"
sys.path.append(path)
path = "../../ia"
sys.path.append(path)

from PurePursuit import pure_pursuit as pp
from PurePursuit import path_factory as pf
from Capteurs import USThread
from PurePursuit.path_manager import Path, Point
import params as p

if __name__ == '__main__':
    robot = robot.RobotPosition(0, 0, 0, 0, 0)
    print(robot)
    comm = main_communication.CommManager(robot)
    behaviour = state_machine.FSMMatch(robot)
    comm.sendPositionMessage()
    comm.sendLidarMessage(1, 0, 0, 0, 0)
    
    print("ici")
    
    print("la")
    # messageVelocity.update(100, -1)
    # downCommunication.send_message(messageVelocity.serial_encode())
    
    Nbpoints = 10000
    path = pf.line(Nbpoints, Point(0,0), Point(3000-500,0))
    #path = pf.polyline(Nbpoints, Point(0,0), Point(1500,500), Point(3000-500,0))
    #path = pf.circle(Nbpoints, Point(0,600), 600)
    
    #list = [Point(i, -350*sin(2*pi*i/2000)) for i in np.linspace(0,2000, Nbpoints)]
    #list += [Point(i, 350*sin(2*pi*i/2000)) for i in np.linspace(2000,0, Nbpoints)]
    #path = Path(list)
    
    #path = Path([Point(i, -300*sin(2*pi*i/2500)) for i in np.linspace(0,2500, Nbpoints)])
    
    #path = pf.lemniscate(Nbpoints, 1000, 600)
    
    
    #path = pf.polyline(Nbpoints, Point(0,0), Point(0, 500), Point(500,500), Point(500,0),Point(0,0))

    path.compute_speed(0.2,0.85,p.SPEED_MAX)

    tracking = pp.PurePursuit(path)
    
    x = []
    y = []
    
    t0 = time()
    while True:#time()-t0<35:
        
        comm.receive_message()
                

        omega, speed = tracking.compute(robot, False)
        print("omega_cons = ", omega, "speed_cons = ", speed)
        #print("lidar : {}, {}, {}".format(robot._lidarZone.activated_zone1(),robot._lidarZone.activated_zone2(),robot._lidarZone.activated_zone3()))
            
        comm.sendVelocityMessage(speed, omega)
            
            
            
    #np.savez('outfile.npz', x=np.array(x), y=np.array(y))
    
    