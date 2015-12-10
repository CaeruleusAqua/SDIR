#!/usr/bin/python2.7

import math
import numpy as np
from numpy.core.multiarray import int_asbuffer
from kinematics_base import Kinematics_base
from scipy.optimize import minimize

from numpy import sin, cos, arctan2, arccos
from numpy.core.umath import arccos


class Kinematics_numeric(Kinematics_base):
    def __init__(self):
         self.destination_point = []



    def resetZeroEquality(self, point):

        for i in range(0,len(point)):
            if np.abs(point[i]) < self.ALMOST_ZERO:
                point[i] = 0

        return point


    def cost_function(self, thetas):
        new_theta = [thetas[0], thetas[1], thetas[2], 0, 0]
        errors = self.direct_kin_to_wrist(new_theta) - self.destination_point
        return (errors ** 2).sum()



    def inverse_kin( self, point):
        """ inverse kinematic calculation

        @param [in] point <b><i><c> [point_type]: </c></i></b> point desc
        @param [in] condition_angle <b><i><c> [condition_angle_type]: </c></i></b> condition_angle desc
        @return <b><i><c> [return_type]: </c></i></b> return desc
        """

        self.destination_point = point
        x0 = [0.0, 0.0, 0.0]


        erg = minimize(self.cost_function, x0, method='SLSQP', options={'maxiter': 1e7, 'disp': False},
                           bounds=[(self.min_angles_[0], self.max_angles_[0]), (self.min_angles_[1],self.max_angles_[1]), (self.min_angles_[2], self.max_angles_[2])])




        return erg


    def isSolutionValid(self, solution):
        """ todo

        :param todo
        :type todo
        :returns: todo
        :rtype: todo
        """
        if len(solution) != 5:
            #print("length is not 5!")
            return False
        for i in range(0,len(solution)):
            if math.isnan(solution[i]):
                #print("index"), i, ("is nan!")
                return False
            elif solution[i] < self.min_angles_[i] or solution[i] > self.max_angles_[i]:
                #print("index:"), i, (" %.4f [%.4f; %.4f]") %(solution[i], self.min_angles_[i], self.max_angles_[i])
                return False

        return True