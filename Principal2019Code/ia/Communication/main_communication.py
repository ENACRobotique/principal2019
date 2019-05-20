import serial


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
from Capteurs import USThread
from PurePursuit.path_manager import Path, Point
import params as p

class CommManager():
    def __init__(self,robot):
        self.robot = robot
        self.upCommunication = com.CommunicationReceived()
        self.downCommunication = com.CommunicationSend()
    
        self.positionReceived = com.PositionReceived()
        self.messagePosition = com.MakePositionMessage()
        self.messageZoneLidar = com.MakeLidarMessage()
        self.ZoneLidarReceived = com.LidarZoneReceive()
        self.messageVelocity = com.MakeVelocityMessage()
        
        self.messagePump = com.MakePumpMessage()
    
    print("ici")
    
    print("la")
    # messageVelocity.update(100, -1)
    # downCommunication.send_message(messageVelocity.serial_encode())

    def sendPositionMessage(self):
        self.messagePosition.update(self.robot.x, self.robot.y, self.robot.theta)
        self.downCommunication.send_message(self.messagePosition.serial_encode())
    
    def sendLidarMessage(self,pin1,pin2,pin3,pin4,pin5):
        self.messageZoneLidar.update(pin1,pin2,pin3,pin4,pin5)
        self.downCommunication.send_message(self.messageZoneLidar.serial_encode())
    
    
    def receive_message(self):
        
        receive_message = self.upCommunication.receive_message()

        if receive_message is not None:
            id_message, payload = receive_message

            if id_message == com.Type.POS_VEL.value:
                self.positionReceived.serial_decode(payload)
                self.robot.update(self.positionReceived.x, self.positionReceived.y, self.positionReceived.theta, self.positionReceived.speed, self.positionReceived.omega)
                print("({}, {}, {}, {}, {})".format(self.positionReceived.x, self.positionReceived.y, self.positionReceived.theta, self.positionReceived.speed, self.positionReceived.omega))
                #print(robot)
                
            if id_message == com.Type.LID_UP.value:
                self.ZoneLidarReceived.serial_decode(payload);
                self.robot.updateZones(self.ZoneLidarReceived.get_zone1,self.ZoneLidarReceived.get_zone2,self.ZoneLidarReceived.get_zone3)
            
    def sendVelocityMessage(self,speed,omega):
            self.messageVelocity.update(speed, omega)
            self.downCommunication.send_message(self.messageVelocity.serial_encode())
            print("Message envoyé")
            
            
            
    #np.savez('outfile.npz', x=np.array(x), y=np.array(y))
