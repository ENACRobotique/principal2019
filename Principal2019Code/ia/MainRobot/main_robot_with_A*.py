from time import time
import math

import sys
path = "../PathFinder"
sys.path.append(path)
path = "../Ultrason"
sys.path.append(path)
path = "../../ia"
sys.path.append(path)
path = "../Communication"
sys.path.append(path)
path = "../StateMachine"
sys.path.append(path)
path = "../PurePursuit"
sys.path.append(path)

from Communication import main_communication
from StateMachine import state_machine
import robot
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from PurePursuit import pure_pursuit as pp
from PurePursuit import path_factory as pf
import matplotlib.pyplot as plt


from PathFinder import AStar,Bezier,TestPP
from PurePursuit.path_manager import Path
import params as p

if __name__ == '__main__':
    robot = robot.RobotPosition(300, 450, -math.pi/2, 0, 0)
    print(robot)
    comm = main_communication.CommManager(robot)
    behaviour = state_machine.FSMMatch(robot)
    comm.sendPositionMessage()
    comm.sendLidarMessage(1, 0, 0, 0, 0)
    
   
    # start and goal position EN MILIMETRE (Meme unite que le robot) !!!!
    sx = 300.0  # [mm]
    sy = 450.0  # [mm]
    gx = 1300.0  # [mm]
    gy = 450.0  # [mm]
    grid_size = 70  # [mm] #précision du A*
    robot_size = 150  # [mm] #distance de sécurité avec les obstacles i.e. taille du robot/2
    
    print("A* BEGIN")
    ox, oy = AStar.create_map()
    #obstacle = [Point(600,1500),Point(700,1500),Point(800,1500),Point(900,1500)]
    #AStar.add_obstacles(ox, oy, obstacle) 
    rx, ry = AStar.a_star_planning(sx, sy, gx, gy, ox, oy, grid_size, robot_size)
    print("A* DONE")

    
    print("Bezier BEGIN")
    control_points = [Point(rx[i],ry[i]) for i in range(len(rx)-1,-1,-1)]
    poly = Polygon(AStar.BordDeMap())    
    #on repère les points critique où il faudra passer nécessairement (points proche des murs)
    #puis on calcul les chemins avec Bezier entre ces différents portions de trajectoire
    AllPath = []
    newPath = []
    for i,point in enumerate(control_points) :
        newPath.append(point)
        if (point.within(poly) and len(newPath)>3) or i== len(control_points)-1:
            AllPath.append(newPath)
            #print(len(newPath))
            newPath = [point]
            
    BezierPaths = [Bezier.calc_bezier_path(discrete_control_points, n_points=50*len(discrete_control_points)) for discrete_control_points in AllPath ]
    print("Bezier DONE")
    '''
    #Avec Pure-Poursuite d'Elie
    
    # On regroupe tout et on enlève les doublons :
    TotalPath = []
    PathToPrint = []
    for Bpath in BezierPaths :
        for point in Bpath :
            if len(TotalPath) == 0 or TotalPath[-1] != point : #la vérification de len == 0 est faites avant la comparaison donc pas de soucis
                TotalPath.append(Point(point[0]*10,point[1]))
                PathToPrint.append((point[0]*10,point[1]))
    """
    Nbpoints = 10000
    RealPath = pf.line(Nbpoints, Point(300,450), Point(1600,450))
    """
    RealPath = Path(TotalPath)
    #print(len(PathToPrint))
    print("Path ok")


    RealPath.compute_speed(0.2,0.85,p.SPEED_MAX)
    print(RealPath.speed)

    tracking = pp.PurePursuit(RealPath)
    
    x = []
    y = []
    
    t0 = time()
    '''
    
    #Avec Pure-Poursuite de maxime
    cx=[]
    cy=[]
    for path in BezierPaths :
        for point in path :
            cx.append(point[0]) 
            cy.append(point[1])
    print("Create Tracking")
    tracking = TestPP.PurePursuit(cx,cy,robot)

    while True:#time()-t0<35:
        
        comm.receive_message()
                

        #omega, speed = tracking.compute(robot, False)
        omega, speed = tracking.compute(robot)
        print("omega_cons = ", omega, "speed_cons = ", speed)
        #print("lidar : {}, {}, {}".format(robot._lidarZone.activated_zone1(),robot._lidarZone.activated_zone2(),robot._lidarZone.activated_zone3()))
            
        comm.sendVelocityMessage(speed, omega)
            
            
            
    #np.savez('outfile.npz', x=np.array(x), y=np.array(y))