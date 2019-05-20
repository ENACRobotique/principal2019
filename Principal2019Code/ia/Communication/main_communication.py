import serial, bitstring


import numpy as np
from math import sin, pi
from time import time

from Communication import communication as com
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
from Ultrason import USThread
from PurePursuit.path_manager import Path, Point
import params as p

if __name__ == '__main__':
    robot = robot.RobotPosition(0, 0, 0, 0, 0)
    print(robot)
    
<<<<<<< Updated upstream

    upCommunication = com.CommunicationReceived()
    downCommunication = com.CommunicationSend()

    positionReceived = com.PositionReceived()
    messagePosition = com.MakePositionMessage()
    messageVelocity = com.MakeVelocityMessage()
    
    messagePump = com.MakePumpMessage()
    
    print("ici")
    
    print("la")
=======
        self.positionReceived = com.PositionReceived()
        self.messagePosition = com.MakePositionMessage()
        self.messageZoneLidar = com.MakeLidarMessage()
        self.ZoneLidarReceived = com.LidarZoneReceive()
        self.messageVelocity = com.MakeVelocityMessage()
        
        self.messagePump = com.MakePumpMessage()
>>>>>>> Stashed changes
    # messageVelocity.update(100, -1)
    # downCommunication.send_message(messageVelocity.serial_encode())

    messagePosition.update(robot.x, robot.y, robot.theta)
    downCommunication.send_message(messagePosition.serial_encode())
    
    Nbpoints = 10000
    path = pf.line(Nbpoints, Point(0,0), Point(3000-500,0))
    #path = pf.polyline(Nbpoints, Point(0,0), Point(1500,500), Point(3000-500,0))
    #path = pf.circle(Nbpoints, Point(0,600), 600)
    
    #list = [Point(i, -350*sin(2*pi*i/2000)) for i in np.linspace(0,2000, Nbpoints)]
    #list += [Point(i, 350*sin(2*pi*i/2000)) for i in np.linspace(2000,0, Nbpoints)]
    #path = Path(list)
    
    #path = Path([Point(i, -300*sin(2*pi*i/2500)) for i in np.linspace(0,2500, Nbpoints)])
    
    path = pf.lemniscate(Nbpoints, 1000, 600)
    
    
    #path = pf.polyline(Nbpoints, Point(0,0), Point(0, 500), Point(500,500), Point(500,0),Point(0,0))

    path.compute_speed(0.2,0.85,p.SPEED_MAX)

    tracking = pp.PurePursuit(path)
    
    x = []
    y = []
    
    t0 = time()
    while True:#time()-t0<35:
        
        #sleep(params.NAVIGATOR_TIME_PERIOD)
        
        """if time()-t0<10:
            if time()-t0<5:
                messagePump.update(1)
            else :
                messagePump.update(1)
        else:
            t0 = time()"""
        messagePump.update(0)
        downCommunication.send_message(messagePump.serial_encode())
        
        receive_message = upCommunication.receive_message()

        if receive_message is not None:
            id_message, payload = receive_message
            
            

            if id_message == com.Type.POS_VEL.value:
<<<<<<< Updated upstream
                positionReceived.serial_decode(payload)
                robot.update(positionReceived.x, positionReceived.y, positionReceived.theta, positionReceived.speed, positionReceived.omega)
                x.append(positionReceived.x)
                y.append(positionReceived.y)
                print("({}, {}, {}, {}, {})".format(positionReceived.x, positionReceived.y, positionReceived.theta, positionReceived.speed, positionReceived.omega))
                #print(robot)

            omega, speed = tracking.compute(robot, False)
            #print("omega_cons = ", omega, "speed_cons = ", speed)
=======
                self.positionReceived.serial_decode(payload)
                self.robot.update(self.positionReceived.x, self.positionReceived.y, self.positionReceived.theta, self.positionReceived.speed, self.positionReceived.omega)
                print(robot)
                
            if id_message == com.Type.LID_UP.value:
                self.ZoneLidarReceived.serial_decode(payload);
                self.robot.updateZones(self.ZoneLidarReceived.get_zone1,self.ZoneLidarReceived.get_zone2,self.ZoneLidarReceived.get_zone3)
>>>>>>> Stashed changes
            
            messageVelocity.update(speed, omega)
            downCommunication.send_message(messageVelocity.serial_encode())
            
            
            
    #np.savez('outfile.npz', x=np.array(x), y=np.array(y))
