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

        theta_00=-np.arctan2(wp[1], wp[0])                 # turn robot arm into wrist point plane

        if theta_00 < 0:
            theta_01 = theta_00  + np.pi
        else:
            theta_01 = theta_00 - np.pi

        #print "theta0: ", np.round(theta_0,3)



        # --------------------calculate Theta1-----------------

        solutions=[]


        shoulder=self.direct_kin_to_shoulder([theta_00])


        distance_from_orign = math.sqrt( wp[0]**2  +  wp[1]**2 )
        if distance_from_orign < self.dh[1]['a']:
            X_zp=-math.sqrt( (shoulder[0] - wp[0])**2  +  (shoulder[1] - wp[1])**2 )
        else:
            X_zp=math.sqrt( (shoulder[0] - wp[0])**2  +  (shoulder[1] - wp[1])**2 )
        Z_zp=wp[2]-shoulder[2]


        beta1 = math.atan2(Z_zp,X_zp)
        R=np.linalg.norm(wp-shoulder)

        a=abs(self.dh[3]['a'])
        b=abs(self.dh[4]['d'])
        d=math.sqrt(a**2 + b**2)
        e=abs(self.dh[2]['a'])

        f = (d**2-e**2-R**2)/(-2*e*R)

        print "f: ",f
        if f < -1.0 :
            f=-1

        if f > 1.0:
            f=1

        beta2=math.acos(f)

        theta_1 = -math.pi/2+beta1+beta2

        theta_1_2 = beta1-beta2 -math.pi/2

         # --------------------calculate Theta2-----------------

        f2 = (-R**2+d**2+e**2)/(2*d*e)
        print "f2:", f2
        if f2 < -1.0 :
            f2=-1

        if f2 > 1.0:
            f2=1
        beta1 = math.acos(f2)
        #beta1 = math.acos(-1)
        beta2 = math.asin(b/d)
        theta_2 = beta1+beta2-math.pi
        theta_2_2 = math.pi-(beta1-beta2)
        #print "beta1: ", math.degrees(beta1)
        #print "beta2: ", math.degrees(beta2)


        solutions.append([theta_00,theta_1,beta1+beta2-math.pi])
        solutions.append([theta_00,theta_1_2,theta_2_2])



        # --------------------calculate Theta1-----------------

        shoulder=self.direct_kin_to_shoulder([theta_01])


        distance_from_orign = math.sqrt( wp[0]**2  +  wp[1]**2 )
        if distance_from_orign < self.dh[1]['a']:
            X_zp=-math.sqrt( (shoulder[0] - wp[0])**2  +  (shoulder[1] - wp[1])**2 )
        else:
            X_zp=-math.sqrt( (shoulder[0] - wp[0])**2  +  (shoulder[1] - wp[1])**2 )
        Z_zp=wp[2]-shoulder[2]


        beta1 = math.atan2(Z_zp,X_zp)
        R=np.linalg.norm(wp-shoulder)

        a=abs(self.dh[3]['a'])
        b=abs(self.dh[4]['d'])
        d=math.sqrt(a**2 + b**2)
        e=abs(self.dh[2]['a'])

        f = (d**2-e**2-R**2)/(-2*e*R)

        print "f: ",f
        if f < -1.0 :
            f=-1

        if f > 1.0:
            f=1

        beta2=math.acos(f)

        theta_1 = -math.pi/2+beta1+beta2

        theta_1_2 = beta1-beta2 -math.pi/2

         # --------------------calculate Theta2-----------------

        f2 = (-R**2+d**2+e**2)/(2*d*e)
        print "f2:", f2
        if f2 < -1.0 :
            f2=-1

        if f2 > 1.0:
            f2=1
        beta1 = math.acos(f2)
        #beta1 = math.acos(-1)
        beta2 = math.asin(b/d)
        theta_2 = beta1+beta2-math.pi
        theta_2_2 = math.pi-(beta1-beta2)
        #print "beta1: ", math.degrees(beta1)
        #print "beta2: ", math.degrees(beta2)


        solutions.append([theta_01,theta_1,beta1+beta2-math.pi])
        solutions.append([theta_01,theta_1_2,theta_2_2])



        T03 = self.get_dh_transform(self.dh[0],0.0) * self.get_dh_transform(self.dh[1],solutions[0][0])*\
              self.get_dh_transform(self.dh[2],solutions[0][1]) * self.get_dh_transform(self.dh[3],solutions[0][2])
        #T03 = IK.getTorigin() * IK.getT1(t1) * IK.getT2(t2) * IK.getT3(t3)
        iT03 = np.linalg.inv(T03)



        #T0G should be an Rotation Matrix (roll, pitch, yaw)
        T0G = self.get_dh_transform(self.dh[0],0.0) * self.get_dh_transform(self.dh[1],solutions[0][0])*\
              self.get_dh_transform(self.dh[2],solutions[0][1]) * self.get_dh_transform(self.dh[3],solutions[0][2])*\
              self.get_dh_transform(self.dh[4],0) * self.get_dh_transform(self.dh[5],0) * self.get_dh_transform(self.dh[6],0)



        c4s5 = -(iT03[0, 0:4] * T0G[0:4, 2])[0,0]
        s4s5 = -(iT03[1, 0:4] * T0G[0:4, 2])[0,0]


        theta4 = np.arctan2(s4s5, c4s5) #if s5 is 0 (singularity), atan2 returns 0.0 :-)
        #print "Theta4: ",math.degrees(theta4)

        c5 =   -(iT03[2, 0:4] * T0G[0:4, 2])[0,0]
        s5 =   ((iT03[0, 0:4] * T0G[0:4, 3])[0,0])/(self.dh[6]['d']*np.cos(theta4))
        theta5 = np.arctan2(s5, c5)
        #print "Theta5: ",math.degrees(theta5)

        c6 =   ((iT03[2, 0:4] * T0G[0:4, 0])[0,0])/(-np.sin(theta5))
        s6 =   ((iT03[2, 0:4] * T0G[0:4, 1])[0,0])/(-np.sin(theta5))
        theta6 = np.arctan2(s6, c6)
        #print "Theta6: ",math.degrees(theta6)

        #theta4dash = IK.smallerAngle(theta4 + np.pi)
        #theta5dash = -theta5


        return solutions


    def isSolutionValid(self, solution, wrist_point):
        """ todo

        :param todo
        :type todo
        :returns: todo
        :rtype: todo
        """

        valid=True

        for (i,sol) in enumerate(solution):
            if sol > self.max_angles_[i] or sol<self.min_angles_[i]:
                valid = False

        if np.linalg.norm(self.direct_kin_to_wrist([solution[0],solution[1],solution[2],0.0,0.0])-wrist_point) > 0.01:
            valid = False
        return valid

