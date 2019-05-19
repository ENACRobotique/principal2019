#!/usr/bin/python2.6
 
# -*-coding:Latin-1 -*
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

class UstReader(threading.Thread):
    def __init__(self,data,dataP,mutex, port, func):
        #func est la fonction à exécuter d=quand les données doivent être émises

        threading.Thread.__init__(self)
        self.data = data
        self.dataP = dataP
        self.mutex = mutex

        self.is_running = False
        self.start_now = True
        self.stop_now = False

        self.i=0

        signal.signal(signal.SIGINT, func)

    def run(self):
        while True :
            if self.start_now :
                #protocol.start_ranging()
                self.start_now  = False
                self.is_running = True

            if self.stop_now :
                #protocol.stop_ranging()
                self.stop_now = False
                self.is_running = False
                
            if self.is_running :
                self.mutex.acquire()
                #data = protocol.get_measures()
                self.data[:] = [self.i]
                self.dataP[:] = [self.i]
                self.i+=1
                self.mutex.release()
                #print("ok")
                kill(os.getpid(), signal.SIGINT) #emet un signal pour exécuter la fonction func

                time.sleep(1)

    def start_ranging(self) :
        self.start_now = True


    def stop_ranging(self) :
        self.stop_now = True
