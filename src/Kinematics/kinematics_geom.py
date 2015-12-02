#!/usr/bin/python2.7

import math
import numpy as np
from numpy.core.multiarray import int_asbuffer
from kinematics_base import Kinematics_base

from numpy import sin, cos, arctan2, arccos
from numpy.core.umath import arccos


class Kinematics_geom(Kinematics_base):
    def __init__(self):
        pass



    def resetZeroEquality(self, point):

        for i in range(0,len(point)):
            if np.abs(point[i]) < self.ALMOST_ZERO:
                point[i] = 0

        return point


    def inverse_kin( self, point):
        """ inverse kinematic calculation

        @param [in] point <b><i><c> [point_type]: </c></i></b> point desc
        @param [in] condition_angle <b><i><c> [condition_angle_type]: </c></i></b> condition_angle desc
        @return <b><i><c> [return_type]: </c></i></b> return desc
        """



        theta_0 = np.empty([2])
        theta_0[0]=np.arctan2(point[1],point[0])                 # turn robot arm into wrist point plane

        if theta_0[0] < 0:
            theta_0[1] = theta_0[0] + np.pi
        else:
            theta_0[1] = theta_0[0] - np.pi

        shoulder_point = np.empty([2])                                  # point 3 in presentation
        shoulder_point[0]=self.direct_kin_to_shoulder([theta_0[0]]);
        shoulder_point[1]=self.direct_kin_to_shoulder([theta_0[1]]);
        wrist_point_0=0



        return []


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