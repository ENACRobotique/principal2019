import sys

import bitstring
from math import pi
from enum import Enum
import serial
import robot
#from builtins import None

path = "../../ia"
sys.path.append(path)

import params


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


class PositionReceived:

    def __init__(self):
        self._x = None
        self._y = None
        self._theta = None
        self._speed = None
        self._omega = None
        """self._us_front_right = None;
        self._us_front_left = None;
        self._us_rear_right = None;
        self._us_rear_left = None;"""

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
    
    """@property
    def us_front_right(self):
        return (self._us_front_right)
    
    @property
    def us_front_left(self):
        return (self._us_front_left)
    
    @property
    def us_rear_right(self):
        return (self._us_rear_right)
    
    @property
    def us_rear_left(self):
        return (self._us_rear_left)"""


    def serial_decode(self, payload):
        s = bitstring.BitStream(payload)
        self._x, self._y, self._theta, self._speed, self._omega = s.unpack('uintle:16, uintle:16, uintle:16, uintle:16, uintle:16')

    


class Type(Enum):
    #up messages
    POS_VEL = 0 
    BUTTONS = 1 
    VOLTAGE = 2 
    #down messages
    VELOCITY = 3 
    POSITION = 4
    PUMP = 5



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
                self.checksum += self.id_message
                payload = self.ser.read(self.lenght-2)
                self.checksum += sum(payload) 
                """for i, b in enumerate(payload):
                    print("indice : ", i, b)
                    self.checksum += bitstring.BitStream(b).unpack('uint:8')[0]"""
                checksum_readed = bitstring.BitStream(self.ser.read()).unpack('uint:8')[0]
                self.state = StateReceive.IDLE
                self.lenght = 0
                self.id_message = 0
                calculated_checksum = ~(self.checksum) & 0xFF
                if calculated_checksum == checksum_readed:
                    self.checksum = 0
                    return self.id_message, payload
                else :
                    self.checksum = 0
                    return None #checksum invalide

        return None #message pas encore entièrement lu






#--------------------------------------------Envoie des trames----------------------------------------------



class CommunicationSend:

    def __init__(self):
        self.ser = serial.Serial(params.SERIAL_PATH,params.SERIAL_BAUDRATE, timeout = params.TIMEOUT)


    def send_message(self, message):
        self.ser.write(message.tobytes())
        