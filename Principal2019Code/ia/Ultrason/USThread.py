from threading import Thread
import params as p

class USManager():
    
    def init(self,FL_range,FR_range,RL_range,RR_range,FL_data=0,FR_data=0,RL_data=0,RR_data=0):
        self.updateRanges(FL_range, FR_range, RL_range, RR_range)
        self.update(FL_data,FR_data,RL_data,RR_data)
        
    def update(self,FL_data,FR_data,RL_data,RR_data):
        self.FL_data = FL_data
        self.FR_data = FR_data
        self.RL_data = RL_data
        self.RR_data = RR_data
        
    def updateRanges(self,FL_range,FR_range,RL_range,RR_range):
        self.FL_range = FL_range
        self.FR_range = FR_range
        self.RL_range = RL_range
        self.RR_range = RR_range
        
    def obstacleDetected(self):
        return (self.FL_data < self.FL_range or self.FR_data < self.FR_range or self.RL_data < self.RL_range or self.RR_data < self.RR_range)



class USThread(Thread):
    
    def __init__(self):
        Thread.__init__(self)
        self.USManager = USManager(500,500,500,500)
        
    def run(self):
        while(True):
            #On récupère les données des ultrasons grâce à l'i2c
            FL_data = 0
            FR_data = 0
            RL_data = 0
            RR_data = 0
            self.USManager.update(FL_data,FR_data,RL_data,RR_data)
            
            if(self.USManager.obstacleDetected()):
                with p.VERROU:
                    p.OBSTACLE_DETECTED = True
            else:
                with p.VERROU:
                    p.OBSTACLE_DETECTED = False
