# -*- coding: utf-8 -*-

from path import Point, Path

from math import sin, cos, sqrt, pi, floor
import numpy as np
import matplotlib.pyplot as plt


def line(Nbpoints, A, B):
    t = np.linspace(0,1,Nbpoints)
    X = A.x*(1-t)+B.x*t
    Y = A.y*(1-t)+B.y*t
    path = Path([Point(X[i], Y[i]) for i in range(Nbpoints)])
    return path


def polyline(Nbpoints, *args):
    n_arg = len(args)
    N = floor(Nbpoints/(n_arg-1))
    path0 = line(N, args[0], args[1])
    for i in range(1, n_arg-1):
        print(i)
        path = line(N, args[i], args[i+1])
        path0.points = np.concatenate((path0.points, path.points))
    return path0


def plot_polyline(*args):
    for i in range(0, len(args)-1):
        plt.plot([args[i].x, args[i+1].x],[args[i].y, args[i+1].y], '--r')


#A, B, C, D = Point(0,0), Point(5,-5), Point(7,2), Point(4,15)
#plt.figure()
#plot_polyline(A, B, C, D)
#plt.show()
#path = polyline(1000, A, B, C, D)


def circle(Nbpoints, C, r):
    x_c, y_c = C.x, C.y
    t = np.linspace(0, 2*pi, Nbpoints)
    X = r*np.cos(t)+x_c
    Y = r*np.sin(t)+y_c
    path = Path([Point(X[i], Y[i]) for i in range(Nbpoints)])
    return path

def plot_circle(path):
    plt.plot([path.points[i].x for i in range(len(path.points))],
             [path.points[i].y for i in range(len(path.points))], '--r')


def parametric_curve(Nbpoints, x_equation, y_equation):
    """
    Param t between 0 and 1.
    :param Nbpoints: int, nb of points of the path
    :param x_equation: a function f(t) for t in [0,1]
    :param y_equation: a function f(t) for t in [0,1]
    :return: path, class Path
    """
    t = np.linspace(0, 1, Nbpoints)
    X = [x_equation(t[i]) for i in range(Nbpoints)]
    Y = [y_equation(t[i]) for i in range(Nbpoints)]
    path = Path([Point(X[i], Y[i]) for i in range(Nbpoints)])
    return X, Y

#def f(t): return cos(2*pi*t)
#def g(t): return sin(2*pi*t)
#path = parametric_curve(1000, f, g)




