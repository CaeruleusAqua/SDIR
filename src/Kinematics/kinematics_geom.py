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

        wp=point

        # --------------------calculate Theta0 -----------------

        theta_0 = np.empty([2])
        theta_0[0]=-np.arctan2(wp[1], wp[0])                 # turn robot arm into wrist point plane

        if theta_0[0] < 0:
            theta_0[1] = theta_0[0] + np.pi
        else:
            theta_0[1] = theta_0[0] - np.pi

        #print "theta0: ", np.round(theta_0,3)



        # --------------------calculate Theta1 and Theta2-----------------

        solutions=[]

        for theta0 in theta_0:

            shoulder=self.direct_kin_to_shoulder([theta0])


            distance_from_orign = math.sqrt( wp[0]**2  +  wp[1]**2 )
            if distance_from_orign < self.dh[1]['a']:
                X_zp=-math.sqrt( (shoulder[0] - wp[0])**2  +  (shoulder[1] - wp[1])**2 )
            else:
                X_zp=math.sqrt( (shoulder[0] - wp[0])**2  +  (shoulder[1] - wp[1])**2 )
            Z_zp=wp[2]-shoulder[2]

            print "SH:",np.round(shoulder,3)
            print "WP:",np.round(wp,3)

            beta1 = math.atan2(Z_zp,X_zp)

            #print "beta1: ", math.degrees(beta1)




            R=np.linalg.norm(wp-shoulder)
            print "R: ",R
            a=abs(self.dh[3]['a'])
            b=abs(self.dh[4]['d'])
            d=math.sqrt(a**2 + b**2)
            e=abs(self.dh[2]['a'])

            print 
            
            f = (d**2-e**2-R**2)/(-2*e*R)
            
            if f < -1.0 or f > 1.0:
                continue
            
            beta2=math.acos((d**2-e**2-R**2)/(-2*e*R))
            #print "beta2: ", math.degrees(beta2)


            #print "beta2: ",math.degrees(beta2)
            #beta2=math.acos(1)
            theta_1 = -math.pi/2+beta1+beta2

            theta_1_2 = beta1-beta2 -math.pi/2
            #print "theta1: ", math.degrees(theta_1)


            #print (-R**2+d**2+e**2)/(2*d*e)
            beta1 = math.acos((-R**2+d**2+e**2)/(2*d*e))
            #beta1 = math.acos(-1)
            beta2 = math.asin(b/d)
            theta_2 = beta1+beta2-math.pi
            theta_2_2 = math.pi-(beta1-beta2)
            #print "beta1: ", math.degrees(beta1)
            #print "beta2: ", math.degrees(beta2)


            solutions.append([theta0,theta_1,beta1+beta2-math.pi])
            solutions.append([theta0,theta_1_2,theta_2_2])




        return solutions


    def isSolutionValid(self, solution):
        """ todo

        :param todo
        :type todo
        :returns: todo
        :rtype: todo
        """
        if len(solution) != 3:
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