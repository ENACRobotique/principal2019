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


from PurePursuit import pure_pursuit as pp
from PurePursuit import path_factory as pf
from shapely.geometry import Point
from PathFinder import AStar
from PathFinder import Bezier
from PathFinder import TestPP
from PathFinder import Map
import math
import matplotlib.pyplot as plt




"""
Nbpoints = 1000
path = pf.line(Nbpoints, Point(1,1), Point(3000,2))
#path.compute_speed(0.2,0.85,300)
print([(point.x,point.y)for point in path.points])
#print(path.speed)
"""

def main():
    #state initialisation
    
    #map creation
    # -Map
    # -add obstacle
    
    #goal and start point definition
    
    #path creation
    #- AStar planning (+ AStar parameters grid size and robot size)
    #- Bezier
    
    #Pure-Poursuite



    
    #Map
    MyMap = Map.Map()
    obstacle = []#rectangle x = 700 - 800 y = 300 - 600
    for i in range(300,600,10) :
        obstacle.append(Point(700,i))
    for i in range(300,600,10) :
        obstacle.append(Point(800,i))
    for i in range(700,800,10) :
        obstacle.append(Point(i,300))
    for i in range(700,800,10) :
        obstacle.append(Point(i,600))  
    MyMap.add_obstacles(obstacle)
    
    #goal and start
    sx = 300.0  # [mm]
    sy = 450.0  # [mm]
    gx = 1300.0  # [mm]
    gy = 450.0  # [mm]
    #MyMap.plot_map(sx, sy, gx, gy)
    
    # initial state
    state = TestPP.State(x=sx, y=sy, yaw=-math.pi/2, v=0.0)
    
    #path Creation
    
    #AStar parameters
    grid_size = 70  # [mm] #précision du A*
    robot_size = 150  # [mm] #distance de sécurité avec les obstacles i.e. taille du robot/2
    
    #AStar planning
    ox,oy = MyMap.get()
    rx, ry = AStar.a_star_planning(sx, sy, gx, gy, ox, oy, grid_size, robot_size)
    
    #calcul controlPoints used for Bezier
    controlPoints = AStar.calc_control_points(rx,ry,sx,sy,gx,gy)
    NPoints = 10 # number of points(for Bezier) between each control points calculated by Astar
    
    #Bezier
    Path = Bezier.calc_bezier_path(controlPoints, n_points=NPoints*len(controlPoints))
    
    #calcul cx,cy use for Pure Poursuit
    cx,cy = Bezier.calc_cxcy(Path)
    
    
    #parameter for Pure-Poursuite
    k = 0.1  # look forward gain
    Lfc = 2.0  # look-ahead distance
    Kp = 1.0  # speed proportional gain
    dt = 0.1  # [s] time increment for simulation
    L = 2.9  # [mm] wheel base of vehicle
    target_speed = 300.0 / 3.6  # [mm/s]
    
    T = 100 #simulation time
    
    #initialisation
    lastIndex = len(cx) - 1
    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    
    target_ind = TestPP.calc_target_index(state, cx, cy)
    
    speed_cons = 0
    while T >= time and lastIndex > target_ind:
        ai = TestPP.PIDControl(target_speed, state.v)
        di, target_ind = TestPP.pure_pursuit_control(state, cx, cy, target_ind)
        speed_cons += ai
        print(speed_cons,di*180/math.pi)
        state = TestPP.update(state, ai, di)

        time = time + dt

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)

        if True:  # pragma: no cover
            plt.cla()
            MyMap.plot_map(sx, sy, gx, gy)
            TestPP.plot_arrow(state.x, state.y, state.yaw)
            plt.plot(cx, cy, "-r", label="course")
            plt.plot(x, y, "-b", label="trajectory")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
            plt.pause(0.001)
    


if __name__ == '__main__':
    print("Test begin")
    main()