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

        wp = point
        wp = self.direct_kin_to_wrist([0.0 , math.radians(45.0) , math.radians(-1.0) , 0.0])


        # --------------------calculate Theta0 -----------------

        theta_0 = np.empty([2])
        theta_0[0]=np.arctan2(wp[1], wp[0])                 # turn robot arm into wrist point plane

        if theta_0[0] < 0:
            theta_0[1] = theta_0[0] + np.pi
        else:
            theta_0[1] = theta_0[0] - np.pi

        print "theta0: ", np.round(theta_0,3)


        # --------------------calculate Theta1 -----------------

        shoulder=self.direct_kin_to_shoulder([theta_0[0]])

        X_zp=math.sqrt( (shoulder[0] - wp[0])**2  +  (shoulder[1] - wp[1])**2 )
        Z_zp=wp[2]-shoulder[2]

        beta1 = math.atan2(Z_zp,X_zp)

        R=np.linalg.norm(wp-shoulder)
        a=abs(self.dh[3]['a'])
        b=abs(self.dh[4]['d'])
        d=math.sqrt(a**2 + b**2)
        e=abs(self.dh[2]['a'])

        beta2=math.acos((d**2-e**2-R**2)/(-2*e*R))
        theta_1 = -math.pi/2+beta1+beta2

        print "theta1: ", math.degrees(theta_1)

        # --------------------calculate Theta2 -----------------

        beta1 = math.acos((-R**2+d**2+e**2)/(2*d*e))
        beta2 = math.asin(b/d)

        print "theta2"
        print math.degrees(beta1+beta2-math.pi)




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