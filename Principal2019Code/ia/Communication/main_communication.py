import serial, bitstring
import numpy as np

import communication as com
import robot

import sys
path = "../PurePursuit"
sys.path.append(path)
path = "../PurePursuit"
sys.path.append(path)
import pure_pursuit as pp
import path_factory as pf
from path import Path, Point

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
    
    Nbpoints = 1000
    path = pf.line(Nbpoints, Point(0,0), Point(1500,0))
    #path = pf.circle(Nbpoints, Point(0,0), Point(0,800))
    
    #path = pf.polyline(Nbpoints, Point(0,0), Point(0, 500), Point(500,500), Point(500,0),Point(0,0))

    tracking = pp.PurePursuit(path)
    look_ahead_distance = 150
    _speed_tracking = 100

    while True:

        receive_message = upCommunication.receive_message()

        if receive_message is not None:
            id_message, payload = receive_message

            if id_message == com.Type.POS_VEL.value:
                positionReceived.serial_decode(payload)
                robot.update(positionReceived.x, positionReceived.y, positionReceived.theta, positionReceived.speed, positionReceived.omega)
                #print(robot)

            speed, omega = tracking.compute(robot, look_ahead_distance, False)
            print("omega_cons = ", omega, "speed_cons = ", speed)
            
            messageVelocity.update(speed, omega)
            downCommunication.send_message(messageVelocity.serial_encode())