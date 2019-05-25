

class lidarZone():
    def __init__(self):
        self.zone1 = 0
        self.zone2 = 0
        self.zone3 = 0
    
    def __repr__(self):
        return "zone 1 : {}; zone 2 = {}; zone 3 = {}".format(self.zone1,self.zone2,self.zone3)
    
    def update(self,zone1,zone2,zone3):
        self.zone1 = zone1
        self.zone2 = zone2
        self.zone3 = zone3
        
    def activated_zone1(self):
        return self.zone1 == 1
    
    def activated_zone2(self):
        return self.zone2 == 1
        
    def activated_zone3(self):
        return self.zone3 == 1