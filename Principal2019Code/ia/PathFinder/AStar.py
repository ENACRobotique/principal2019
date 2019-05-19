"""
A* grid based planning
author: Atsushi Sakai(@Atsushi_twi)
        Nikos Kanargias (nkana@tee.gr)
See Wikipedia article (https://en.wikipedia.org/wiki/A*_search_algorithm)
"""

import matplotlib.pyplot as plt
import math
import Bezier
import numpy as np
from shapely.geometry.polygon import Polygon
from shapely.geometry import Point
import sys
path = "../Communication"
sys.path.append(path)
path = "../PathFinder"
sys.path.append(path)
path = "../Ultrason"
sys.path.append(path)
path = "../../ia"
sys.path.append(path)
from Communication import communication

show_animation = True


class Node:

    def __init__(self, x, y, cost, pind):
        self.x = x
        self.y = y
        self.cost = cost
        self.pind = pind

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)


def calc_final_path(ngoal, closedset, reso):
    # generate final course
    rx, ry = [ngoal.x * reso], [ngoal.y * reso]
    pind = ngoal.pind
    while pind != -1:
        n = closedset[pind]
        rx.append(n.x * reso)
        ry.append(n.y * reso)
        pind = n.pind

    return rx, ry


def a_star_planning(sx, sy, gx, gy, ox, oy, reso, rr):
    """
    gx: goal x position [m]
    gx: goal x position [m]
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    reso: grid resolution [m]
    rr: robot radius[m]
    """

    nstart = Node(round(sx / reso), round(sy / reso), 0.0, -1)
    ngoal = Node(round(gx / reso), round(gy / reso), 0.0, -1)
    ox = [iox / reso for iox in ox]
    oy = [ioy / reso for ioy in oy]

    obmap, minx, miny, maxx, maxy, xw, yw = calc_obstacle_map(ox, oy, reso, rr)

    motion = get_motion_model()

    openset, closedset = dict(), dict()
    openset[calc_index(nstart, xw, minx, miny)] = nstart

    while 1:
        c_id = min(
            openset, key=lambda o: openset[o].cost + calc_heuristic(ngoal, openset[o]))
        current = openset[c_id]

        # show graph
        if show_animation:  # pragma: no cover
            plt.plot(current.x * reso, current.y * reso, "xc")
            if len(closedset.keys()) % 10 == 0:
                plt.pause(0.001)

        if current.x == ngoal.x and current.y == ngoal.y:
            print("Find goal")
            ngoal.pind = current.pind
            ngoal.cost = current.cost
            break

        # Remove the item from the open set
        del openset[c_id]
        # Add it to the closed set
        closedset[c_id] = current

        # expand search grid based on motion model
        for i, _ in enumerate(motion):
            node = Node(current.x + motion[i][0],
                        current.y + motion[i][1],
                        current.cost + motion[i][2], c_id)
            n_id = calc_index(node, xw, minx, miny)

            if n_id in closedset:
                continue

            if not verify_node(node, obmap, minx, miny, maxx, maxy):
                continue

            if n_id not in openset:
                openset[n_id] = node  # Discover a new node
            else:
                if openset[n_id].cost >= node.cost:
                    # This path is the best until now. record it!
                    openset[n_id] = node

    rx, ry = calc_final_path(ngoal, closedset, reso)

    return rx, ry


def calc_heuristic(n1, n2):
    w = 1.5  # weight of heuristic
    d = w * math.sqrt((n1.x - n2.x)**2 + (n1.y - n2.y)**2)
    return d


def verify_node(node, obmap, minx, miny, maxx, maxy):

    if node.x < minx:
        return False
    elif node.y < miny:
        return False
    elif node.x >= maxx:
        return False
    elif node.y >= maxy:
        return False

    if obmap[node.x][node.y]:
        return False

    return True


def calc_obstacle_map(ox, oy, reso, vr):

    minx = round(min(ox))
    miny = round(min(oy))
    maxx = round(max(ox))
    maxy = round(max(oy))
    #  print("minx:", minx)
    #  print("miny:", miny)
    #  print("maxx:", maxx)
    #  print("maxy:", maxy)

    xwidth = round(maxx - minx)
    ywidth = round(maxy - miny)
    #  print("xwidth:", xwidth)
    #  print("ywidth:", ywidth)

    # obstacle map generation
    obmap = [[False for i in range(ywidth)] for i in range(xwidth)]
    for ix in range(xwidth):
        x = ix + minx
        for iy in range(ywidth):
            y = iy + miny
            #  print(x, y)
            for iox, ioy in zip(ox, oy):
                d = math.sqrt((iox - x)**2 + (ioy - y)**2)
                if d <= vr / reso:
                    obmap[ix][iy] = True
                    break

    return obmap, minx, miny, maxx, maxy, xwidth, ywidth


def calc_index(node, xwidth, xmin, ymin):
    return (node.y - ymin) * xwidth + (node.x - xmin)


def create_map():
    """
    Map Edition 2019
    """
    ox, oy =[], []
    
    "Bords de la table"
    for i in range(0,2000,10) :
        ox.append(i)
        oy.append(0.0)
        
    for i in range(0,2000,10) :
        ox.append(i)
        oy.append(3000.0)    
        
    for i in range(0,3000,10) :
        ox.append(0.0)
        oy.append(i)
        
    for i in range(0,3000,10) :
        ox.append(2000.0)
        oy.append(i)
  
    "Portes atome x6"
    for i in range(450,1050,10) :
        ox.append(1540)
        oy.append(i)
        
    for i in range(1950,2550,10) :
        ox.append(1540)
        oy.append(i)
        
    "Montes Balance"
    for i in range(450,2550,10) :
        ox.append(1600)
        oy.append(i)

    "Fin de montée balance"
    for i in range(1600,2000,10) :
        ox.append(i)
        oy.append(1220)
        
    for i in range(1600,2000,10) :
        ox.append(i)
        oy.append(1770)
    
    "Barre séparation balance"
    for i in range(1340,2000,10) :
        ox.append(i)
        oy.append(1500)
    
    "Accelerateur de particule"
    for i in range(500,2500,10) :
        ox.append(20)
        oy.append(i)
    
    return ox,oy

def BordDeMap():
    """
    Polygone définissant une zone aux bords de la map pour le calcul des points de passage critique du A*
    (i.e. là où le robot est vraiment proche des bords)
    """
    Bords_15cm=[(0,0),(200,0),(200,122.8),(160,122.8),(160,45),(154,45),(154.3,148),(134.3,148),(134.3,152),(154.3,152),(154.3,255),
           (160,255),(160,177.2),(200,177.2),(200,300),(0,300),(0,1),
           (15,1),(15,285),(185,285),(185,192.2),(175,192.2),(175,270),(139.3,270),(139.3,167),(119.3,167),(119.3,133),
           (139.3,133),(139.3,30),(175,30),(175,107.8),(185,107.8),(185,15),(16,15),(16,0.5)
           ]
    return Bords_15cm

def InterieurDeMap():
    """
    Polygone définissant une zone aux à l'interieur de la map pour le calcul des points de passage critique du A*
    (i.e. là où le robot n'est pas proche des bords)
    """
    Interieur_20cm = [(200,200),(200,2800),(1343,2800),(1343,1720),(1143,1720),(1143,1280),(1343,1280),(1343,200)]
    return Interieur_20cm

def get_motion_model():
    # dx, dy, cost
    motion = [[1, 0, 1],
              [0, 1, 1],
              [-1, 0, 1],
              [0, -1, 1],
              [-1, -1, math.sqrt(2)],
              [-1, 1, math.sqrt(2)],
              [1, -1, math.sqrt(2)],
              [1, 1, math.sqrt(2)]]

    return motion


def add_obstacles(ox,oy,obstacle):
    """
    Ajoute l'obstacle désiré aux listes ox et oy
    obstacle : Liste de Point
    """
    for point in obstacle :
        ox.append(point.x)
        oy.append(point.y)
    
def calc_control_points(rx,ry,sx,sy,gx,gy):
    """
    Transforme les points calculé par l'algorithme A* en point utilisable par Bezier
    """
    control_points = [Point(rx[i],ry[i]) for i in range(len(rx)-2,0,-1)] # le premier et le dernier point sont enlevé, on met sx,sy et gx,gy a la place
    control_points.append(Point(gx,gy))
    control_points.insert(0,Point(sx,sy))
    return control_points
    

def main():
    print(__file__ + " start!!")

    # start and goal position
    sx = 300.0  # [mm]
    sy = 450.0  # [mm]
    gx = 1300.0  # [mm]
    gy = 450.0  # [mm]
    grid_size = 70  # [mm] #précision du A*
    robot_size = 150  # [mm] #distance de sécurité avec les obstacles i.e. taille du robot/2

    ox, oy = create_map()
    #obstacle = [Point(600,1500),Point(700,1500),Point(800,1500),Point(900,1500)]
    #add_obstacles(ox, oy, obstacle)
    
    print("map ok2")
    '''
    for i in range(60):
        ox.append(i)
        oy.append(0.0)
    for i in range(60):
        ox.append(60.0)
        oy.append(i)
    for i in range(61):
        ox.append(i)
        oy.append(60.0)
    for i in range(61):
        ox.append(0.0)
        oy.append(i)
    for i in range(40):
        ox.append(20.0)
        oy.append(i)
    for i in range(40):
        ox.append(40.0)
        oy.append(60.0 - i)
    '''
    
    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "xr")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")

    rx, ry = a_star_planning(sx, sy, gx, gy, ox, oy, grid_size, robot_size)


    print (rx,ry)
    control_points = [Point(rx[i],ry[i]) for i in range(len(rx)-1,-1,-1)]
    
    poly = Polygon(InterieurDeMap())    
    #on repère les points critique où il faudra passer nécessairement (points proche des murs)
    #puis on calcul les chemins avec Bezier entre ces différents points
    AllPath = []
    newPath = []
    
    for i,point in enumerate(control_points) :
        newPath.append(point)
        if (not point.within(poly) and len(newPath)>3) or i== len(control_points)-1:
            AllPath.append(newPath)
            newPath = [point]
            
    BezierPaths = [Bezier.calc_bezier_path(discrete_control_points, n_points=10*len(discrete_control_points)) for discrete_control_points in AllPath ]
    
    
    cx=[]
    cy=[]
    for path in BezierPaths :
        for point in path :
            cx.append(point[0])
            cy.append(point[1])
    
    
    if show_animation:  # pragma: no cover
        plt.plot(rx, ry, "-r")
        for path in BezierPaths :
            plt.plot(path.T[0], path.T[1], "-g")
        plt.show()
        

def TestPP():
    print(__file__ + " start!!")

    # start and goal position
    sx = 300.0  # [mm]
    sy = 450.0  # [mm]
    gx = 1300.0  # [mm]
    gy = 450.0  # [mm]
    grid_size = 70  # [mm] #précision du A*
    robot_size = 150  # [mm] #distance de sécurité avec les obstacles i.e. taille du robot/2

    ox, oy = create_map()
    obstacle = [Point(600,1500),Point(700,1500),Point(800,1500),Point(900,1500)]
    add_obstacles(ox, oy, obstacle)
    
    print("map ok1")
    '''
    for i in range(60):
        ox.append(i)
        oy.append(0.0)
    for i in range(60):
        ox.append(60.0)
        oy.append(i)
    for i in range(61):
        ox.append(i)
        oy.append(60.0)
    for i in range(61):
        ox.append(0.0)
        oy.append(i)
    for i in range(40):
        ox.append(20.0)
        oy.append(i)
    for i in range(40):
        ox.append(40.0)
        oy.append(60.0 - i)
    '''
    
    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "xr")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")

    rx, ry = a_star_planning(sx, sy, gx, gy, ox, oy, grid_size, robot_size)


    print (rx,ry)
    control_points = [Point(rx[i],ry[i]) for i in range(len(rx)-1,-1,-1)]
    
    poly = Polygon(InterieurDeMap())    
    #on repère les points critique où il faudra passer nécessairement (points proche des murs)
    #puis on calcul les chemins avec Bezier entre ces différents points
    AllPath = []
    newPath = []
    
    for i,point in enumerate(control_points) :
        newPath.append(point)
        if (not point.within(poly) and len(newPath)>3) or i== len(control_points)-1:
            AllPath.append(newPath)
            newPath = [point]
            
    BezierPaths = [Bezier.calc_bezier_path(discrete_control_points, n_points=100*len(discrete_control_points)) for discrete_control_points in AllPath ]

    
    return BezierPaths



if __name__ == '__main__':
	main()