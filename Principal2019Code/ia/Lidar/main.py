#!/usr/bin/python2.6
 
# coding: utf-8

from UstReaderNoQt import UstReader
from threading import Thread, Lock
import time
import sys
from util import point_inside_polygon,lidarDistance2xy


class LIDAR() :

    def __init__(self,xLidar=0,yLidar=0,orientationLidar=0) :
        self.data = []
        self.dataP = []
        self.distancexy = []
        self.xLidar = xLidar
        self.yLidar = yLidar
        self.orientationLidar = orientationLidar
        self.mutex = Lock()
        self.proximityAlert = 0
        self.Obstacles = []
        self.LimiteTerrain =[(0,0),(3000,0),(3000,1543),(0,1543)]

    def maFonction(self,a,b) :
        self.mutex.acquire()
        sys.stdout.write("majDistance : mutex Acquire\n")

        prox = 0
        self.distancexy = []
        self.Obstacles = []

        for i,d in enumerate(self.data) :
            self.distancexy.append(lidarDistance2xy((self.xLidar,self.yLidar),self.orientationLidar,d,i))

            if d < 400 :
                prox = 1

            if point_inside_polygon(self.distancexy[-1],self.LimiteTerrain) :
                self.Obstacles.append(self.distancexy[-1])

        self.proximityAlert = prox

        self.mutex.release()
        sys.stdout.write("majDistance : mutex Release\n")

    def plus1(self) :
        print("ta mere 2")

if __name__ == '__main__':
    port='/dev/ttyS4'
    Lidar = LIDAR()
    ReaderLidar = UstReader(Lidar.data,Lidar.dataP,Lidar.mutex, port , Lidar.maFonction)
    ReaderLidar.start()
    while True : #la boucle ici est necessaire pour que la fonction Lidar.maFonction dans le Reader soit toujours appele
        pass