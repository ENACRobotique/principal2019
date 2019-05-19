
 
# -*-coding:Latin-1 -*
from collections import namedtuple
import serial.threaded
import traceback
import sys
import re
import numpy as np
import time

class UST(serial.threaded.LineReader):

    TERMINATOR = b'\n'

    Command = namedtuple('Command',['command', 'answer_expected', 'answer'])

    START_RANGING = Command('#GT15466', False, '')
    STOP_RANGING = Command('#ST5297', True, '#ST00A845')
    ID = Command('#IN0D54', True, '')
    ID2 = Command('#CLC2DD', True, '')

    PLOP = Command('#GR0EEE1', True, '')

    def __init__(self):
        super(UST, self).__init__()
        self.answer_expected = False
        self.answer = None
        self.scan_regex = re.compile(r"^#GT00:([0-9A-F]{12}):([0-9]{6}):([0-9A-F]{4332})$")
        self.measurement_regex = re.compile(r"(.{4})(.{4})")
    
        self.timestamp = 0
        self.distances = np.zeros(541,dtype="uint16")
        self.puissances = np.zeros(541,dtype="uint16")

    def connection_made(self, transport):
        super(UST, self).connection_made(transport)
        sys.stdout.write('port opened\n')
        #self.write_line('#ST5297\n')

    def handle_line(self, data):
        sys.stdout.write('line received:{}\n'.format(repr(data)))
        if self.answer_expected and self.answer in data:
            self.answer_expected = False
            #print('answer {} received !'.format(self.answer))
            self.answer = None
        m = self.scan_regex.match(data)
        if m is not None:
            timestamp = int(m.group(1), 16)
            mysterious_field = m.group(2)
            mapped_data = map(self.raw_data_to_data, re.findall(self.measurement_regex, m.group(3)))
            scan_data = list(mapped_data)

            self.timestamp = timestamp/10**6
            self.distances = np.fromiter((elt[0] for elt in scan_data), dtype=np.uint16)
            #sys.stdout.write(self.distances.__repr__())
            self.puissances = np.fromiter((elt[1] for elt in scan_data), dtype=np.uint16)

        else:
            #print("fail to match data !")
            sys.stdout.write('line received:{}\n'.format(repr(data)))

    def connection_lost(self, exc):
        if exc:
            traceback.print_exc(exc)
        sys.stdout.write('port closed\n')

    def raw_data_to_data(self, raw):
        raw_distance, raw_quality = raw
        distance = int(raw_distance, 16)
        quality = int(raw_quality, 16)
        return distance, quality

    def send_command(self, command):
        self.write_line(command.command+"\n")
        if command.answer_expected:
            self.answer_expected = True
            self.answer = command.answer

    def stop_ranging(self):
        self.send_command(self.STOP_RANGING)

    def start_ranging(self):
        self.stop_ranging()
        self.send_command(self.START_RANGING)

    def get_measures(self):
        #sys.stdout.write(str(self.distances)+"\n")
        return self.timestamp, self.distances, self.puissances

def main():
    ser = serial.Serial('/dev/ttyS4', baudrate=115200, timeout=1)
    #ser = serial.Serial('COM5', baudrate=115200, timeout=1)
    with serial.threaded.ReaderThread(ser, UST) as protocol:
        protocol.stop_ranging()
        time.sleep(1)
        protocol.start_ranging()
        print("1")
        while True :
            pass
    #protocol.write_line('#GT15466\n')
    #protocol.write_line('#GT15466\n')

if __name__=='__main__':
    with serial.Serial('/dev/ttyS4', 115200, timeout=1) as ser:
        ser.write(b'#ST5297\n')
        print("stop")
        ser.write(b'#GT15466\n')
        print("start")
        x = ser.read()          # read one byte
        s = ser.read(1000)        # read up to 1 thousand bytes (timeout)
        line = ser.readline()   # read a '\n' terminated line
        print(x,s,line)
