import bitstring
from math import pi
from enum import Enum
import serial

RADIAN_TO_MSG_ADDER = pi
RADIAN_TO_MSG_FACTOR = 1000
SPEED_ADDER = 1<<15
XY_ADER = 1<<15
ANGULAR_SPEED_TO_MSG_FACTOR = (1<<15)/30 #max omega=30rad/s  
ANGULAR_SPEED_TO_MSG_ADDER = (1<<15) #split uint16 in two    
SERIAL_BAUDRATE = 115200
SERIAL_PATH = "/dev/ttyUSB0"
TIMEOUT = 0.01

class RobotPosition:

    def __init__(self, x=0, y=0, theta=0, speed=0, omega=0):
        self._x = x
        self._y = y 
        self._theta = theta
        self._speed = speed
        self._omega = omega

    def __repr__(self):
        return('x={} ; y={} ; theta={} ; speed={} ; omega={}'.format(self.x, self.y, self.theta, self.speed, self.omega))
    
    def update(self, x, y, theta, speed, omega):
        self._x = x
        self._y = y
        self._theta = theta
        self._speed = speed
        self._omega = omega

    @property
    def x(self):
        return self._x

    @property
    def y(self):
        return self._y

    @property
    def theta(self):
        return self._theta

    @property
    def speed(self):
        return self._speed

    @property
    def omega(self):
        return self._omega


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

    def serial_encode(self):
        s = bitstring.BitStream('uintle:16, uintle:16, uintle:16, uintle:16, uintle:16',
        self._x, self._y, self._theta, self._speed, self._omega )


class Type(Enum):
    POS_VEL = 0
    BUTTONS = 1
    VOLTAGE = 2



#--------------------------------------------Lecture des trames----------------------------------------------


class StateReceive(Enum):
    IDLE = 0
    INIT1 = 1
    INIT2 = 2
    READ = 3

class Communication:

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
                    print("passe en état INIT1")

            if self.state == StateReceive.INIT1:
                b = bitstring.BitStream(self.ser.read()).unpack('uint:8')[0]
                if b == 0xFF:
                    self.state = StateReceive.INIT2
                    print("passe en état INIT2")
                else:
                    self.state = StateReceive.IDLE
                    print("passe en état IDLE depuis INIT1")

            if self.state == StateReceive.INIT2:
                self.lenght = bitstring.BitStream(self.ser.read()).unpack('uint:8')[0]
                self.checksum += self.lenght
                self.state = StateReceive.READ
                print("passe en état READ")

            if self.state == StateReceive.READ:
                self.id_message = bitstring.BitStream(self.ser.read()).unpack('uint:8')[0]
                self.checksum += self.id_message
                payload = self.ser.read(self.lenght-2)
                print(payload)
                self.checksum += sum(payload) 
                """for i, b in enumerate(payload):
                    print("indice : ", i, b)
                    self.checksum += bitstring.BitStream(b).unpack('uint:8')[0]"""
                checksum_readed = bitstring.BitStream(self.ser.read()).unpack('uint:8')[0]
                self.state = StateReceive.IDLE
                print("passe en état IDLE après ok")
                self.lenght = 0
                self.id_message = 0
                calculated_checksum = ~(self.checksum) & 0xFF
                print("readed: ", checksum_readed, "  calculated: ", calculated_checksum)
                if calculated_checksum == checksum_readed:
                    self.checksum = 0
                    return self.id_message, payload
                else :
                    self.checksum = 0
                    print("checksum false")
                    return None #checksum invalide

        return None #message pas encore entièrement lu








