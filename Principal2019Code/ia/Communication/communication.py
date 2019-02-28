import bitstring
from math import pi
from enum import Enum
import serial
import robot

RADIAN_TO_MSG_ADDER = pi
RADIAN_TO_MSG_FACTOR = 1000
SPEED_ADDER = 1<<15
XY_ADER = 1<<15
ANGULAR_SPEED_TO_MSG_FACTOR = (1<<15)/30 #max omega=30rad/s  
ANGULAR_SPEED_TO_MSG_ADDER = (1<<15) #split uint16 in two    
SERIAL_BAUDRATE = 115200
SERIAL_PATH = "/dev/ttyAMA0"
TIMEOUT = 0.01


class MakeVelocityMessage:

    def __init__(self):
        self._speed = None
        self._omega = None

    @property
    def speed(self):
        return int(self._speed + SPEED_ADDER)

    @property
    def omega(self):
        return int((self._omega * ANGULAR_SPEED_TO_MSG_FACTOR) + ANGULAR_SPEED_TO_MSG_ADDER)

    def update(self, speed, omega):
        self._speed = speed
        self._omega = omega

    def serial_encode(self):
        id_message = Type.VELOCITY
        lenght = 6
        header = bitstring.pack('uintle:8, uintle:8', 0xFF, 0xFF)
        s = bitstring.pack('uintle:8, uintle:8, uintle:16, uintle:16', lenght, id_message.value, self.speed, self.omega)
        checksum = bitstring.pack('uintle:8', (~sum(s.tobytes()) & 0xFF))
        #print(checksum)
        #checksum = ~(np.uint8(lenght)+np.uint8(id_message.value)+np.uint8(self.speed)+np.uint8(self.omega)) & 0xFF
        #print(lenght, id_message.value, self.speed, self.omega)
        data = header+s+checksum
        return data

class MakePositionMessage:

    def __init__(self):
        self._x = None
        self._y = None
        self._theta = None

    @property
    def x(self):
        return int(self._x + XY_ADER)

    @property
    def y(self):
        return int(self._y + XY_ADER)

    @property
    def theta(self):
        return int((self._theta+RADIAN_TO_MSG_ADDER)*RADIAN_TO_MSG_FACTOR)

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



class PositionReceived:

    def __init__(self):
        self._x = None
        self._y = None
        self._theta = None
        self._speed = None
        self._omega = None

    @property
    def x(self):
        return self._x - XY_ADER

    @property
    def y(self):
        return self._y -XY_ADER

    @property
    def theta(self):
        return (self._theta / RADIAN_TO_MSG_FACTOR) - RADIAN_TO_MSG_ADDER

    @property
    def speed(self):
        return self._speed - SPEED_ADDER

    @property
    def omega(self):
        return (self._omega - ANGULAR_SPEED_TO_MSG_ADDER) / ANGULAR_SPEED_TO_MSG_FACTOR


    def serial_decode(self, payload):
        s = bitstring.BitStream(payload)
        self._x, self._y, self._theta, self._speed, self._omega = s.unpack(
            'uintle:16, uintle:16, uintle:16, uintle:16, uintle:16')

    


class Type(Enum):
    #up messages
    POS_VEL = 0 
    BUTTONS = 1 
    VOLTAGE = 2 
    #down messages
    VELOCITY = 3 
    POSITION = 4



#--------------------------------------------Lecture des trames----------------------------------------------


class StateReceive(Enum):
    IDLE = 0
    INIT1 = 1
    INIT2 = 2
    READ = 3

class CommunicationReceived:

    def __init__(self):
        self.ser = serial.Serial(SERIAL_PATH,SERIAL_BAUDRATE, timeout = TIMEOUT)
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
        self.ser = serial.Serial(SERIAL_PATH,SERIAL_BAUDRATE, timeout = TIMEOUT)


    def send_message(self, message):
        self.ser.write(message.tobytes())
        