'''
Created By:
    Baher Kherbek
'''
from sympy import Eq, solve_linear_system, Matrix
import math
import sympy as sp
import numpy as np
import matplotlib.pyplot as plt
from GetConstants import CalculateConstants

#Input (angle in radians)
current = [0, 1, 0]
target = [5, 0, 45 * math.pi / 180]
TotalTime = 30

def CalculateAngularVelocities(u1, u2, wheelRadius = 3, wheelsDistance = 3):
    thetaRdot, thetaLdot = sp.symbols('thetaRdot thetaLdot')
    row1 = [wheelRadius/2, wheelRadius/2, u1]
    row2 = [wheelRadius/wheelsDistance, -wheelRadius/wheelsDistance, u2]

    system = Matrix((row1, row2))
    solve = solve_linear_system(system, thetaRdot, thetaLdot)

    return solve[thetaRdot], solve[thetaLdot]


def Plot(TotalTime, current, goal):
    Constants = CalculateConstants(current=current, goal=goal, Time=TotalTime)
    epsilon = TotalTime / 3
    omega = 2 * math.pi / epsilon
    c2 = (goal[0] - current[0]) / epsilon
    c1, c3, Z1t1, Z2t1, Z3t1, Z1t2, Z2t2, Z3t2 = sp.symbols('c1 c3 Z1t1 Z2t1 Z3t1 Z1t2 Z2t2 Z3t2')


    print(Constants)

    V1 = []
    V2 = []
    x = []
    y = []
    theta = []
    dev_thetaR = []
    dev_thetaL = []
    dev_x = []
    dev_y = []
    dev_theta = []
    time = []
    time2 = list(range(0, TotalTime+1))

    TimeNow = 0.0
    temp = 0
    Sampling = 0.1

    while TimeNow < TotalTime :
        time.append(TimeNow)
        #i = 0 and i = 1
        if TimeNow < (TotalTime / 3)  or TimeNow > 2 * (TotalTime / 3):
            #Forward Velocities
            u1 = 0
            u2 = Constants[c1 if TimeNow < TotalTime / 3 else c3] * (1 - math.cos(omega * TimeNow))
            V1.append(u1)
            V2.append(u2)
            
            #Angular Velocities
            thetaRdot, thetaLdot = CalculateAngularVelocities(u1, u2)
            dev_thetaR.append(thetaRdot)
            dev_thetaL.append(thetaLdot)


            #Axis Velocities
            dev_x.append(0)
            dev_y.append(0)
            dev_theta.append(u2)

            #Center Displacement
            if temp % (1 / Sampling) == 0:
                try:
                    x.append((u1 * math.cos(Theta)) + x[len(x) - 1])
                    y.append((u1 * math.sin(Theta)) + y[len(y) - 1])
                    theta.append(u2 + theta[len(theta) - 1])
                except:
                    x.append(0)
                    y.append(0)
                    theta.append(0)
            
            temp += 1


        #j = 0
        else:
            temp += 1
            Theta = math.atan(Constants[Z2t2]) * math.pi / 180
            #Forward Velocities
            u1 = c2 * (1 - math.cos(omega * TimeNow))
            u2 = 0
            V1.append(u1)
            V2.append(u2)

            #Angular Velocities
            thetaRdot, thetaLdot = CalculateAngularVelocities(u1, u2)           
            dev_thetaR.append(thetaRdot)
            dev_thetaL.append(thetaLdot)

            #Center Displacement
            Theta = math.atan(Constants[Z2t2])
            if temp % (1 / Sampling) == 0:
                x.append((u1 * math.cos(Theta)) + x[len(x) - 1])
                y.append((u1 * math.sin(Theta)) + y[len(y) - 1])
                theta.append(Theta)
            
            #Axis Velocities
            Theta = math.atan(Constants[Z2t2])
            dev_x.append(math.cos(Theta) * u1)
            dev_y.append(math.sin(Theta) * u1)
            dev_theta.append(u2)
            
            
          
        TimeNow += Sampling #Simulation Step Size (Precision)
    

    #Plot Data
    fig, axes = plt.subplots(2, 2)

    axes[0, 0].plot(time, V1, label='V1', linestyle='--')
    axes[0, 0].plot(time, V2, label='V2')
    axes[0, 0].legend()

    axes[0, 1].plot(time, dev_thetaL, label='thetaLdot', linestyle='--')
    axes[0, 1].plot(time, dev_thetaR, label='thetaRdot', linestyle='-.')
    axes[0, 1].legend()

    axes[1, 0].plot(time, dev_x, label='Xdot')
    axes[1, 0].plot(time, dev_y, label='Ydot', linestyle='-.')
    axes[1, 0].plot(time, dev_theta, label='Thetadot')
    axes[1, 0].legend()

    axes[1, 1].plot(time2, x, label='X')
    axes[1, 1].plot(time2, y, label='Y')
    axes[1, 1].plot(time2, theta, label='Theta')
    axes[1, 1].legend()
    plt.show()

if __name__ == '__main__':
    Plot(current=current, goal=target, TotalTime=TotalTime)
