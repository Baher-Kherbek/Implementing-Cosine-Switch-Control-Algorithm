"""
Created by:
	Baher Kher Bek
"""


#!/usr/bin/env python3.8
import rospy
import math
import time
from geometry_msgs.msg import Point
from GetContants import CalculateConstants
import sympy as sp
from sympy import Eq, solve_linear_system, Matrix


def CalculateAngularVelocities(u1, u2, wheelRadius = 0.035, wheelsDistance = 0.2):
    thetaRdot, thetaLdot = sp.symbols('thetaRdot thetaLdot')
    row1 = [wheelRadius/2, wheelRadius/2, u1]
    row2 = [wheelRadius/wheelsDistance, -wheelRadius/wheelsDistance, u2]

    system = Matrix((row1, row2))
    solve = solve_linear_system(system, thetaRdot, thetaLdot)

    return solve[thetaRdot], solve[thetaLdot]

if __name__ == '__main__':

    #ROS
    rospy.init_node('CosineSwitchControl', anonymous=True)
    pub = rospy.Publisher('/Control', Point, queue_size=10)
    msg = Point()

    #INPUT
    n = 3
    current = [0, 0, 0]
    goal = [0.4, 0.4, 0 * math.pi / 180]
    TotalTime = 6
    epsilon = TotalTime / n
    omega = 2 * math.pi / epsilon

    #Get Constants
    Constants = CalculateConstants(current, goal, TotalTime)

    StartTime = time.time()
    TimeNow = time.time() - StartTime
    while TimeNow < TotalTime:

        #i=0 or i=1
        if TimeNow < TotalTime / 3 or TimeNow > (2 * TotalTime / 3):
            #Forward Velocities
            u1 = 0
            u2 = Constants[sp.symbols('c1') if TimeNow < TotalTime / 3 else sp.symbols('c3')] * (1 - math.cos(omega * TimeNow))
            
            #Motor's Angular Velocities
            thetaRdot, thetaLdot = CalculateAngularVelocities(u1, u2)
            msg.x = -int(thetaRdot * 9.549296585513721)
            msg.y = -int(thetaLdot * 9.549296585513721)

            print(f'{TimeNow}  :    {thetaRdot}')



        #j=0
        else:
            #Forward Velocities
            c2 = (goal[0] - current[0]) / epsilon
            u1 = c2 * (1 - math.cos(omega * TimeNow))
            u2 = 0

            #Angular Velocities
            thetaRdot, thetaLdot = CalculateAngularVelocities(u1, u2)
            msg.x = thetaRdot * 9.549296585513721
            msg.y = thetaLdot * 9.549296585513721

            #print(f'{TimeNow}  :    {thetaRdot}')

        pub.publish(msg)
        time.sleep(0.001)
        TimeNow = time.time() - StartTime
    
    msg.x = 0
    msg.y = 0
    pub.publish(msg)
    print('Done')

