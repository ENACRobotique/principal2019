import serial


import numpy as np
from threading import Thread
from math import sin, pi
from time import time, sleep

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


"""Classe d'interface entre les messages et le code en lui-même
Permet d'utiliser facilement les bons arguments des messages que l'on cherche et de masquer l'interface des messages"""
class CommManager():
    def __init__(self,robot):
        self.robot = robot
        self.downCommunication = com.CommunicationSend()
        #self.receiveCommunication = ReceiveMessageThread(self.robot)
        self.upCommunication = com.CommunicationReceived()
        self.ZoneLidarReceived = com.LidarZoneReceive()
        self.positionReceived = com.PositionReceived()
    #upCommunication = com.CommunicationReceived()
    #downCommunication = com.CommunicationSend()

    #positionReceived = com.PositionReceived()
    #messagePosition = com.MakePositionMessage()
    #messageVelocity = com.MakeVelocityMessage()
    
    #messagePump = com.MakePumpMessage()
        
        self.messageTiretteDown = com.MakeTiretteQuestion()
        self.messageColorDown = com.MakeColorQuestion()
        self.messageLocker = com.MakeLockerMessage()
        self.messageHolder = com.MakeHolderMessage()
        self.messageDynamicHolder = com.MakeDynamicHolderMessage()
        self.messageEar = com.MakeEarMessage()
        self.messagePosition = com.MakePositionMessage()
        self.messageZoneLidar = com.MakeLidarMessage()
        self.messageVelocity = com.MakeVelocityMessage()
        
        self.messagePump = com.MakePumpMessage()
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
        #self.upCommunication.print_in_waiting()

        if receive_message is not None:
            id_message, payload = receive_message

            if id_message == com.Type.POS_VEL.value:
                #print("message position reçu")
                self.positionReceived.serial_decode(payload)
                self.robot.update(self.positionReceived.x, self.positionReceived.y, self.positionReceived.theta, self.positionReceived.speed, self.positionReceived.omega)
                #print("Nouvelles données !")
                #print("Message reçu avec speed, omega = {}, {}".format(self.positionReceived.speed, self.positionReceived.omega))
                #print(self.robot)
                
            if id_message == com.Type.LID_UP.value:
                #print("message lidar reçu")
                self.ZoneLidarReceived.serial_decode(payload);
                #print("1 : {}, 2 : {}, 3 : {}".format(self.ZoneLidarReceived.get_zone1,self.ZoneLidarReceived.get_zone2,self.ZoneLidarReceived.get_zone3))
                self.robot.updateZones(self.ZoneLidarReceived.get_zone1,self.ZoneLidarReceived.get_zone2,self.ZoneLidarReceived.get_zone3)
            
    def sendVelocityMessage(self,speed,omega):
            #print("MessageVelocity : {}, {}".format(speed,omega))
            self.messageVelocity.update(speed, omega)
            self.downCommunication.send_message(self.messageVelocity.serial_encode())
            #print("Message envoyé avec speed, omega = {}, {}".format(speed,omega))
            
            
    def sendEarUpMessage(self):
        self.messageEar.update(1);
        self.downCommunication.send_message(self.messageEar.serial_encode())
        
        
    def sendEarDownMessage(self):
        self.messageEar.update(0);
        self.downCommunication.send_message(self.messageEar.serial_encode())
        
        
    def sendLockerUpMessage(self):
        self.messageLocker.update(1);
        self.downCommunication.send_message(self.messageLocker.serial_encode())
        
        
    def sendLockerDownMessage(self):
        self.messageLocker.update(0);
        self.downCommunication.send_message(self.messageLocker.serial_encode())
        
    
    def sendHolderUpMessage(self):
        self.messageHolder.update(1);
        self.downCommunication.send_message(self.messageHolder.serial_encode())
        
        
    def sendHolderDownMessage(self):
        self.messageHolder.update(0);
        self.downCommunication.send_message(self.messageHolder.serial_encode())
        
    def sendDynamicHolderUpMessage(self):
        self.messageDynamicHolder.update(1);
        self.downCommunication.send_message(self.messageDynamicHolder.serial_encode())
        
        
    def sendDynamicHolderDownMessage(self):
        self.messageDynamicHolder.update(0);
        self.downCommunication.send_message(self.messageDynamicHolder.serial_encode())
        
    def sendTiretteQuestion(self):
        self.downCommunication.send_message(self.messageTiretteDown.serial_encode())
        
    def sendColorQuestion(self):
        self.downCommunication.send_message(self.messageColorDown.serial_encode())
        
        
    def start_receive_thread(self):
        self.receiveCommunication.start()
        
    def flush(self):
        self.upCommunication.flush()
            
         
"""Classe de thread de réception des messages
A ne pas utiliser car les threads en python ne marchent pas très bien
Privilégier l'utilisation de processus (module multiprocessing par exemple)"""   
class ReceiveMessageThread(Thread):
    def __init__(self, robot):
        Thread.__init__(self);
        self.upCommunication = com.CommunicationReceived()
        self.ZoneLidarReceived = com.LidarZoneReceive()
        self.positionReceived = com.PositionReceived()
        self.robot = robot
        
    def run(self):
        print("-------------------- Thread de réception commencé ! ----------------")
        while True:
            print("temps : {}".format(time()))
            #self.upCommunication.print_in_waiting()
            receive_message = self.upCommunication.receive_message()
           
            if receive_message is not None:
                id_message, payload = receive_message
              
                if id_message == com.Type.POS_VEL.value:
                    self.positionReceived.serial_decode(payload)
                    self.robot.update(self.positionReceived.x, self.positionReceived.y, self.positionReceived.theta, self.positionReceived.speed, self.positionReceived.omega)
                   
            
                    
                if id_message == com.Type.LID_UP.value:
                    self.ZoneLidarReceived.serial_decode(payload);
                    self.robot.updateZones(self.ZoneLidarReceived.get_zone1,self.ZoneLidarReceived.get_zone2,self.ZoneLidarReceived.get_zone3)
                    
            sleep(0.1)
                    
                    
                
        
