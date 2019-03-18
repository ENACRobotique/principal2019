import serial, bitstring
import numpy as np
from math import sin, pi
from time import sleep

import communication as com
import robot

import sys
path = "../PurePursuit"
sys.path.append(path)
path = "../PurePursuit"
sys.path.append(path)
path = "../../ia"
sys.path.append(path)

import pure_pursuit as pp
import path_factory as pf
from path import Path, Point
import params as p

if __name__ == '__main__':
    robot = robot.RobotPosition(0, 0, 0, 0, 0)
    print(robot)

    upCommunication = com.CommunicationReceived()
    downCommunication = com.CommunicationSend()

    positionReceived = com.PositionReceived()
    messagePosition = com.MakePositionMessage()
    messageVelocity = com.MakeVelocityMessage()
    # messageVelocity.update(100, -1)
    # downCommunication.send_message(messageVelocity.serial_encode())

    messagePosition.update(robot.x, robot.y, robot.theta)
    downCommunication.send_message(messagePosition.serial_encode())
    
    Nbpoints = 5000
    #path = pf.line(Nbpoints, Point(0,0), Point(3000-300-305-150,0))
    #path = pf.polyline(Nbpoints, Point(0,0), Point(1500,500), Point(3000-500,0))
    #path = pf.circle(Nbpoints, Point(0,0), Point(0,800))
    
    #list = [Point(i, -350*sin(2*pi*i/2000)) for i in np.linspace(0,2000, Nbpoints)]
    #list += [Point(i, 350*sin(2*pi*i/2000)) for i in np.linspace(2000,0, Nbpoints)]
    #path = Path(list)
    
    #path = Path([Point(i, -300*sin(2*pi*i/2500)) for i in np.linspace(0,2500, Nbpoints)])
    
    path = pf.lemniscate(Nbpoints, 1000, 600)
    
    
    #path = pf.polyline(Nbpoints, Point(0,0), Point(0, 500), Point(500,500), Point(500,0),Point(0,0))

    tracking = pp.PurePursuit(path)

    while True:
        
        #sleep(params.NAVIGATOR_TIME_PERIOD)

        receive_message = upCommunication.receive_message()

        if receive_message is not None:
            id_message, payload = receive_message
            print(payload)

            if id_message == com.Type.POS_VEL.value:
                positionReceived.serial_decode(payload)
                robot.update(positionReceived.x, positionReceived.y, positionReceived.theta, positionReceived.speed, positionReceived.omega)
                #print(robot)

            speed, omega = tracking.compute(robot, False)
            print("omega_cons = ", omega, "speed_cons = ", speed)
            
            messageVelocity.update(speed, omega)
            downCommunication.send_message(messageVelocity.serial_encode())
    
