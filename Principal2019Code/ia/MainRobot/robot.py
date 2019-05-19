from Capteurs.lidar_zones import lidarZone

class RobotPosition:

    def __init__(self, x=0, y=0, theta=0, speed=0, omega=0):
        self._x = x
        self._y = y 
        self._theta = theta
        self._speed = speed
        self._omega = omega
        
        self._lidarZone = lidarZone()

    def __repr__(self):
        return('x={} ; y={} ; theta={} ; speed={} ; omega={}'.format(self.x, self.y, self.theta, self.speed, self.omega))
    
    def update(self, x, y, theta, speed, omega):
        self._x = x
        self._y = y
        self._theta = theta
        self._speed = speed
        self._omega = omega

    def updateZones(self,zone1,zone2,zone3):
        self._lidarZone.update(zone1, zone2, zone3)

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