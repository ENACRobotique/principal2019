
import bitstring
from math import pi
from enum import Enum
import serial

RADIAN_TO_MSG_ADDER = pi
RADIAN_TO_MSG_FACTOR = 1000
SERIAL_BAUDRATE = 115200
SERIAL_PATH = "/dev/ttyAMA0"
TIMEOUT = 0.01

class RobotPosition:

    def __init__(self, x, y, theta, speed, omega):
        self._x = x
        self._y = y 
        self._theta = theta
        self._speed = speed
        self._omega = omega
    
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
        return self._x 

    @property
    def y(self):
        return self._y

    @property
    def theta(self):
        return (self._theta / RADIAN_TO_MSG_FACTOR) - RADIAN_TO_MSG_FACTOR

    @property
    def speed(self):
        return self._speed

    @property
    def omega(self):
        return self._omega


    def serial_decode(self, data_bytes):
        s = bitstring.BitStream(data_bytes)
        self._x, self._y, self._theta, self._speed, self._omega = s.unpack(
            'uint16, uint16, uint16, uint16, uint16')

    def serial_encode(self):
        s = bitstring.BitStream('uint16, uint16, uint16, uint16, uint16',
        self._x, self._y, self._theta, self._speed, self._omega )


class TypeUp(Enum):
    POS_VEL =0
    BUTTONS = 1
    VOLTAGE = 2

class Communication:


    def __init__(self):
        self.ser = serial.Serial(SERIAL_PATH,SERIAL_BAUDRATE, timeout = TIMEOUT)

    def receive_message(self):
        waiting = 0

        
        if !waiting:
	        b_header1 = self.ser.read()
	        if b_header1 == '0xFF':
	            b_header2 = self.ser.read():
	                if b_header2 == '0xFF':
	                    b_length = self.ser.read()
	                    b_length = bitstring.Bitstream(b_length).unpack('uint8')
	                    if ser.inWaiting() >= b_length:
	                    	payload_extended = self.ser.read(b_length)

	                        waiting = 0
	                    else:
	                        waiting = 1
	    else : 
	    	if sr.inWaiting() >= b_length:
	    		payload_extended = self.ser.read(b_length)
	    		waiting = 0






