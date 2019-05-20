import sys

import bitstring
from math import pi
from enum import Enum
import serial
#import robot
import random
import sys
from threading import Thread
from time import time
#from builtins import None
#from builtins import import None

path = "../../ia"
sys.path.append(path)

import params

#---------------------------------------------Types de trames up and down-------------------------------------------

class Type(Enum):
    #up messages
    POS_VEL = 0 
    BUTTONS = 1 
    VOLTAGE = 2
    ACK = 3 
    #down messages
    VELOCITY = 4 
    POSITION = 5
    PUMP = 6
    GATE = 7
    DYN = 8
    
    LID_UP = 10
    LID_DOWN = 11


#----------------------------------------------Trames down (Raspi->Teensy)----------------------------------------
class MakeVelocityMessage:

    def __init__(self):
        self._speed = None
        self._omega = None

    @property
    def speed(self):
        return int(self._speed + params.SPEED_ADDER)

    @property
    def omega(self):
        return int((self._omega * params.ANGULAR_SPEED_TO_MSG_FACTOR) + params.ANGULAR_SPEED_TO_MSG_ADDER)

    def update(self, speed, omega):
        self._speed = speed
        self._omega = omega

    def serial_encode(self):
        id_message = Type.VELOCITY
        length = 6
        header = bitstring.pack('uintle:8, uintle:8', 0xFF, 0xFF)
        print("Message pos_vel préparé avec id = {}".format(id_message.value))
        
        s = bitstring.pack('uintle:8, uintle:8, uintle:16, uintle:16', length, id_message.value, self.speed, self.omega)
        checksum = bitstring.pack('uintle:8', (~sum(s.tobytes()) & 0xFF))
        #print(checksum)
        #checksum = ~(np.uint8(length)+np.uint8(id_message.value)+np.uint8(self.speed)+np.uint8(self.omega)) & 0xFF
        #print(length, id_message.value, self.speed, self.omega)
        data = header+s+checksum
        return data

class MakePositionMessage:

    def __init__(self):
        self._x = None
        self._y = None
        self._theta = None

    @property
    def x(self):
        return int(self._x + params.XY_ADER)

    @property
    def y(self):
        return int(self._y + params.XY_ADER)

    @property
    def theta(self):
        return int((self._theta+params.RADIAN_TO_MSG_ADDER)*params.RADIAN_TO_MSG_FACTOR)

    def update(self, x, y, theta):
        self._x = x
        self._y = y
        self._theta = theta

    def serial_encode(self):
        id_message = Type.POSITION
        lenght = 8
        header = bitstring.pack('uintle:8, uintle:8', 0xFF, 0xFF)
        s = bitstring.pack('uintle:8, uintle:8, uintle:16, uintle:16, uintle:16', lenght, id_message.value, self.x, self.y, self.theta)
        checksum = bitstring.pack('uintle:8', (~sum(s.tobytes()) & 0xFF))
        data = header+s+checksum
        return data
    
class MakePumpMessage:
    
    def __init__(self):
        self._pump_activated = None 
    
    @property
    def pump_activated(self):
        if self._pump_activated != 0:
            return 1
        else :
            return 0
        
    def update(self, activation):
        self._pump_activated = activation #activation is an int. 0 if no activation. 
        
    def serial_encode(self):
        id_message = Type.PUMP
        length = 3
        header = bitstring.pack('uintle:8, uintle:8', 0xFF, 0xFF)
        s = bitstring.pack('uintle:8, uintle:8, uintle:8', length, id_message.value, self.pump_activated)
        checksum = bitstring.pack('uintle:8', (~sum(s.tobytes()) & 0xFF))
        data = header+s+checksum
        return data

class MakeLidarMessage:
    
    def __init__(self):
        self.pin1 = 0
        self.pin2 = 0
        self.pin3 = 0
        self.pin4 = 0
        self.pin5 = 0
        
    def update(self, pin1,pin2,pin3,pin4,pin5):
        self.pin1 = pin1
        self.pin2 = pin2
        self.pin3 = pin3
        self.pin4 = pin4
        self.pin5 = pin5
        
    def serial_encode(self):
        id_message = Type.LID_DOWN
        length = 7
        header = bitstring.pack('uintle:8, uintle:8', 0xFF, 0xFF)
        s = bitstring.pack('uintle:8, uintle:8, uintle:8, uintle:8, uintle:8, uintle:8, uintle:8', length, id_message.value, self.pin1,self.pin2,self.pin3,self.pin4,self.pin5)
        checksum = bitstring.pack('uintle:8', (~sum(s.tobytes()) & 0xFF))
        data = header+s+checksum
        return data

class MakeGateMessage:
    
    def __init__(self):
        self._gate_activated = None
        
    @property
    def gate_activated(self):
        if self._gate_activated != 0:
            return 1
        else:
            return 0
        
    def update(self, activation):
        self._gate_activated = activation
        
    def serial_encode(self):
        id_message = Type.GATE
        length = 3
        header = bitstring.pack('uintle:8, uintle:8', 0xFF, 0xFF)
        s = bitstring.pack('uintle:8, uintle:8, uintle:8', length, id_message.value, self.gate_activated)
        checksum = bitstring.pack('uintle:8', (~sum(s.tobytes()) & 0xFF))
        data = header+s+checksum
        return data
    
class MakeDynamixelMessage:
    
    def __init__(self):
        self._angle = None #a float in degrees
        self._speed = None
        
    @property
    def angle(self):
        b = 350 #number of angle when trunk at bottom
        a = 1024/300 #300 degrees made in 1024 bytes
        alpha = self._angle
        if alpha<=0:
            return int(b-a*alpha)
        else:
            if alpha<b/a:
                return int(b-a*alpha)
            if alpha >= b/a and alpha<360+(b-1023)/a:
                if alpha>= b/a and alpha<(b/a+360+(b-1023)/a)/2:
                    return 0
                else:
                    return 1023
            else:
                return int(b-a*(alpha-360)) #return byte [0;1023]
    
    @property
    def speed(self):
        return self._speed
    
    def update(self, angle, speed):
        """
        angle is an angle (float) in degrees [-180;180]
        speed is an speed between 0 and 1023
        """
        self._angle = angle
        self._speed = speed
        
    def serial_encode(self):
        id_message = Type.DYN
        length = 6
        header = bitstring.pack('uintle:8, uintle:8', 0xFF, 0xFF)
        print("angle envoye :", self.angle)
        s = bitstring.pack('uintle:8, uintle:8, uintle:16, uintle:16', length, id_message.value, self.angle, self.speed)
        checksum = bitstring.pack('uintle:8', (~sum(s.tobytes()) & 0xFF))
        data = header+s+checksum
        return data

#----------------------------------------------Trames up (Teensy->Raspi)----------------------------------------

class PositionReceived:

    def __init__(self):
        self._x = None
        self._y = None
        self._theta = None
        self._speed = None
        self._omega = None              

    @property
    def x(self):
        return self._x - params.XY_ADER

    @property
    def y(self):
        return self._y - params.XY_ADER

    @property
    def theta(self):
        return (self._theta / params.RADIAN_TO_MSG_FACTOR) - params.RADIAN_TO_MSG_ADDER

    @property
    def speed(self):
        return self._speed - params.SPEED_ADDER

    @property
    def omega(self):
        return (self._omega - params.ANGULAR_SPEED_TO_MSG_ADDER) / params.ANGULAR_SPEED_TO_MSG_FACTOR

    def serial_decode(self, payload):
        s = bitstring.BitStream(payload)
        self._x, self._y, self._theta, self._speed, self._omega = s.unpack('uintle:16, uintle:16, uintle:16, uintle:16, uintle:16')

    

class LidarZoneReceive:
    
    def __init__(self):
        self.zone1 = None
        self.zone2 = None
        self.zone3 = None 
        
    @property
    def get_zone1(self):
        return self.zone1
    
    @property
    def get_zone2(self):
        return self.zone2
    
    @property
    def get_zone3(self):
        return self.zone3
    
    def serial_decode(self, payload):
        s = bitstring.BitStream(payload)
        self.zone1, self.zone2, self.zone3= s.unpack('uintle:8, uintle:8, uintle:8')

#--------------------------------------------Lecture des trames----------------------------------------------


class StateReceive(Enum):
    IDLE = 0
    INIT1 = 1
    INIT2 = 2
    READ = 3

class CommunicationReceived:

    def __init__(self):
        self.ser = serial.Serial(params.SERIAL_PATH,params.SERIAL_BAUDRATE, timeout = params.TIMEOUT)
        self.state = StateReceive.IDLE
        self.lenght = 0
        self.checksum = 0
        self.id_message = 0

    def receive_message(self):

        if self.ser.in_waiting > self.lenght:

            if self.state == StateReceive.IDLE :
                b = bitstring.BitStream(self.ser.read()).unpack('uint:8')[0]
                if b == 0xFF:
                    self.state = StateReceive.INIT1

            if self.state == StateReceive.INIT1:
                b = bitstring.BitStream(self.ser.read()).unpack('uint:8')[0]
                if b == 0xFF:
                    self.state = StateReceive.INIT2
                else:
                    self.state = StateReceive.IDLE

            if self.state == StateReceive.INIT2:
                self.lenght = bitstring.BitStream(self.ser.read()).unpack('uint:8')[0]
                self.checksum += self.lenght
                self.state = StateReceive.READ

            if self.state == StateReceive.READ:
                self.id_message = bitstring.BitStream(self.ser.read()).unpack('uint:8')[0]
                print("id_message : {}".format(self.id_message))
                self.checksum += self.id_message
                payload = self.ser.read(self.lenght-2)
                self.checksum += sum(payload) 
                """for i, b in enumerate(payload):
                    print("indice : ", i, b)
                    self.checksum += bitstring.BitStream(b).unpack('uint:8')[0]"""
                checksum_readed = bitstring.BitStream(self.ser.read()).unpack('uint:8')[0]
                self.state = StateReceive.IDLE
                self.lenght = 0
                idmessage = self.id_message
                self.id_message = 0
                calculated_checksum = ~(self.checksum) & 0xFF
                if calculated_checksum == checksum_readed:
                    self.checksum = 0
                    print("Message bien lu")
                    return idmessage, payload
                else :
                    print("Message PAS bien lu")
                    self.checksum = 0
                    return None #checksum invalide

        return None #message pas encore entierement lu






#--------------------------------------------Envoie des trames----------------------------------------------



class CommunicationSend():

    def __init__(self):
        self.ser = serial.Serial(params.SERIAL_PATH,params.SERIAL_BAUDRATE, timeout = params.TIMEOUT)


    def send_message(self, message):
        self.ser.write(message.tobytes())
        print(message.tobytes())
        
    
    
class CommunicationSendWithAck(Thread):

    def __init__(self,message):
        Thread.__init__(self)
        self.ser = serial.Serial(params.SERIAL_PATH,params.SERIAL_BAUDRATE, timeout = params.TIMEOUT)
        self.message = message
    
    def send_message(self, message):
        self.ser.write(message.tobytes())
    
    
    def run(self):
        upCommunication = CommunicationReceived()
        self.send_message(self.message)
        ack = False
        valeur_au_pif = 2
        t0 = time()
        while not ack:
            if time()-t0 > valeur_au_pif:
                self.send_message(self.message)
                print("je renvoie")
                t0 = time()
            receive_message = upCommunication.receive_message()
            
            if receive_message is not None:
                print(receive_message)
                id_message, payload = receive_message
                if id_message == Type.ACK.value:
                    ack = True
                    print("c'est ackte")
        