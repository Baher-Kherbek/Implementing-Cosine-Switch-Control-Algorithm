"""
Created by:
	Baher Kher Bek
"""

from sympy import Eq, solve_linear_system, Matrix
import math
import sympy as sp
import numpy as np


def CalculateConstants(current, goal, Time):
    epsilon = Time / 3
    c2 = (goal[0] - current[0]) / epsilon
    c1, c3, Z1t1, Z2t1, Z3t1, Z1t2, Z2t2, Z3t2 = sp.symbols('c1 c3 Z1t1 Z2t1 Z3t1 Z1t2 Z2t2 Z3t2')

    row1 = [0, 0, 1, 0, 0, 0, 0, 0, current[0]]
    row2 = [-1 * epsilon, 0, 0, 1, 0, 0, 0, 0, math.tan(current[2])]
    row3 = [0, 0, 0, 0, 1, 0, 0, 0, current[1]]

    row4 = [0, 0, -1, 0, 0, 1, 0, 0, epsilon * c2]
    row5 = [0, 0, 0, -1, 0, 0, 1, 0, 0]
    row6 = [0, 0, 0, -c2 * epsilon, -1, 0, 0, 1, 0]

    row7 = [0, 0, 0, 0, 0, 1, 0, 0, goal[0]]
    row8 = [0, epsilon, 0, 0, 0, 0, 1, 0, math.tan(goal[2])]
    row9 = [0, 0, 0, 0, 0, 0, 0, 1, goal[1]]

    system = Matrix((row1, row2, row3, row4, row5, row6, row7, row8, row9))
    Constants = solve_linear_system(system, c1, c3, Z1t1, Z2t1, Z3t1, Z1t2, Z2t2, Z3t2)

    c2 = sp.symbols('c2')
    Constants[c2] = (goal[0] - current[0]) / epsilon
    return Constants


if __name__ == '__main__':
    consts = CalculateConstants([0, 1, 0], [5, 0, 45 * math.pi / 180], 30)
    c1, c3, Z1t1, Z2t1, Z3t1, Z1t2, Z2t2, Z3t2 = sp.symbols('c1 c3 Z1t1 Z2t1 Z3t1 Z1t2 Z2t2 Z3t2')
    print(consts[c1])

