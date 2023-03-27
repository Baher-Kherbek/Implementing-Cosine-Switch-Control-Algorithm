"""
Created by:
	Baher Kher Bek
"""


#!/usr/bin/env python3.8
import rospy
import os
import math
import time
from geometry_msgs.msg import Twist
from GetContants import CalculateConstants
import sympy as sp

print(time.time())

def readySimulation(current, goal, TotalTime):
    os.system('rosservice call /kill turtle1 && rosservice call /spawn ' + str(current[0]) + ' ' + str(
        current[1]) + ' ' + str(current[2]) + ' Sycarex')
    os.system('rosservice call /spawn ' + str(goal[0]) + ' ' + str(goal[1]) + ' ' + str(goal[2]) + ' target')
    time.sleep(2)
    os.system('rosservice call /kill target')

    consts = CalculateConstants(current, goal, TotalTime)
    return(consts)

def startSimulation(TotalTime, Constants):
    print('starting')
    StartTime = time.time()
    TimeNow = time.time() - StartTime
    epsilon = TotalTime / 3
    omega = 2 * math.pi / epsilon
    msg = Twist()

    while TimeNow < TotalTime:
        if TimeNow < TotalTime / 3 or TimeNow > (2 * TotalTime / 3):
            msg.angular.z = Constants[sp.symbols('c1') if TimeNow < TotalTime / 3 else sp.symbols('c3')] * (1 - math.cos(omega * TimeNow))
            msg.linear.x = 0

        else:
            msg.linear.x = Constants[sp.symbols('c2')] * (1 - math.cos(omega * TimeNow))
            msg.angular.z = 0

        pub.publish(msg)
        TimeNow = time.time() - StartTime

if __name__ == '__main__':
    rospy.init_node('CosineSwitchControl', anonymous=True)
    pub = rospy.Publisher('/Sycarex/cmd_vel', Twist, queue_size=10)
    current = [1, 1, 3.14]
    goal = [5, 5, 45 * math.pi / 180]
    Time = 30

    constants = readySimulation(current, goal, Time)
    print(constants)
    startSimulation(Time, constants)



