#!/usr/bin/python2.6
 
# coding: utf-8
import time
import numpy as np
import random
from ustNoQt import UST
from serial import Serial
from serial.threaded import ReaderThread
import threading
from testsignal import kill
import os
import signal
import sys

class UstReader(threading.Thread):


    def __init__(self,data,dataP,mutex, port, func):
        #func est la fonction a executer d=quand les donnees doivent etre emises

        threading.Thread.__init__(self)
        self.data = data
        self.dataP = dataP
        self.mutex = mutex
        self.ser = Serial(port = port, baudrate=115200, timeout=1)

        self.is_running = False
        self.start_now = True
        self.stop_now = False

        signal.signal(signal.SIGINT, func)

    #def __del__(self):
    #self.wait()

    def run(self):
        with ReaderThread(self.ser, UST) as protocol:
            while True :
                if self.start_now :
                    protocol.start_ranging()
                    self.start_now  = False
                    self.is_running = True

                if self.stop_now :
                    protocol.stop_ranging()
                    self.stop_now = False
                    self.is_running = False
                    
                if self.is_running :
                    self.mutex.acquire()
                    sys.stdout.write("Reader : mutex Acquire\n")
                    data = protocol.get_measures()
                    self.data[:] = data[1]
                    self.dataP[:] = data[2]
                    self.mutex.release()
                    sys.stdout.write("Reader : mutex Released\n")
                    sys.stdout.write("Data Received \n")
                    kill(os.getpid(), signal.SIGINT) #emet un signal pour executer la fonction func
                    time.sleep(0.001)
            
    def start_ranging(self) :
        self.start_now = True


    def stop_ranging(self) :
        self.stop_now = True
