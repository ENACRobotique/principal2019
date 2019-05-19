import math
def line_intersection(line1, line2):
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1]) #Typo was here

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
       return None,None

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return x, y

class Point:
    def __init__(self,x,y):
        self.x = x
        self.y = y

def ccw(A,B,C):
    return (C.y-A.y)*(B.x-A.x) > (B.y-A.y)*(C.x-A.x)

def intersect(A,B,C,D):
    return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)


def drawTkline(canvas,rho,theta) :
    w = 400
    h = 400
    rho/=10
    # on rappelle rho = xcos(theta)+ysin(theta)
    # 1er cas : theta entre +315 et 360 ET 0 et 45 ET entre +135 et +205
    # la ligne coupe les bords haut et bas
    # 2eme cas : la ligne coupe les bords droit et gauche
    if math.radians(315)<=theta<=math.radians(360) or math.radians(0)<=theta<=math.radians(45) or math.radians(135)<=theta<=math.radians(205) :
        y0 = 0
        y1 = h
        x0 = rho/math.cos(theta)
        x1 = (rho-y1*math.sin(theta))/math.cos(theta)
    else : 
        x0 = 0
        x1 = w
        y0 = rho/math.sin(theta)
        y1 = (rho-x1*math.cos(theta))/math.sin(theta)
    #print(rho,theta)
    #print (x0,y0,x1,y1)
    return x0,y0,x1,y1

def lidarDistance2xy(position,orientationRad,distance,i) :
    # soit une mesure de lidar : i désignant la ième mesure faites 
    # (angle i*0,5 degré par rapport à l'angle 0 du lidar)
    # position et orientationRad sont la position et l'orientation supposé du lidar
    # renvoit la position en coordonné xy du point atteint par cette mesure
    d = distance
    x  = position[0] + d*math.cos(orientationRad+(-135+i*0.5)*math.pi/180)
    y  = position[1] - d*math.sin(orientationRad+(-135+i*0.5)*math.pi/180)
    return (x,y)

def lidarDetection2xy(position,orientationRad,data_distance) :
    detection_xy = []
    for i in range(len(data_distance)) :
        detection_xy.append(lidarDistance2xy(position,orientationRad,data_distance[i],i))
    return detection_xy


def polar2cartesian(rho,theta) :
    x = rho*math.cos(theta)
    y = rho*math.sin(theta)
    return x,y

class lineCode() :
    
    def __init__(self,rho,theta) : 
        self.rho = rho
        self.theta = theta
        self.x,self.y = polar2cartesian(rho,theta)

    def distance(self,other) :
        return math.sqrt((self.x-other.x)**2+(self.y-other.y)**2)

    def draw(self,canvas,h,w,color) :
        scale = 20
        midh = h//2
        midw = w//2
        x1 = midw + self.x/scale
        y1 = midh + self.y/scale
        x2 = x1 + 5
        y2 = y1 + 5
        canvas.create_oval(x1, y1, x2, y2, fill=color)

    def difTheta(self,other) : 
        dtheta = self.theta-other.theta
        if dtheta > math.pi :
            dtheta -= 2*math.pi
        return dtheta

    def difRho(self,other) :
        return self.rho-other.rho

    def difxy(self,other) :
        d = self.difRho(other)
        x = -d*math.cos(self.theta)
        y = -d*math.sin(self.theta)
        return (x,y)

    def __repr__(self) :
        return "rho = "+str(self.rho)+" ; theta = "+str(self.theta)

def distance2point(x1,y1,x2,y2) :
    return math.sqrt((x1-x2)**2+(y1-y2)**2)



# determine if a point is inside a given polygon or not
# Polygon is a list of (x,y) pairs.

def point_inside_polygon(Point,poly):
    x = Point[0]
    y = Point[1]

    n = len(poly)
    inside =False

    p1x,p1y = poly[0]
    for i in range(n+1):
        p2x,p2y = poly[i % n]
        if y > min(p1y,p2y):
            if y <= max(p1y,p2y):
                if x <= max(p1x,p2x):
                    if p1y != p2y:
                        xinters = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x,p1y = p2x,p2y

    return inside
