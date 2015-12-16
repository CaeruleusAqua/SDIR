#!/usr/bin/python2.7

import math
import numpy as np

from numpy import sin, cos, arctan2, arccos
from numpy.core.umath import arccos

class Kinematics_base:
    def __init__(self):
        pass

    ALMOST_PLUS_ONE=0.9999999
    ALMOST_MINUS_ONE=-0.9999999
    ALMOST_ZERO=0.0000001

    min_angles_ = [math.radians(-185.0),math.radians(-135.0),math.radians(-120.0),math.radians(-350.0),math.radians(-130.0),math.radians(-350.0)]
    max_angles_ = [math.radians(185.0),math.radians(35.0),math.radians(158.0),math.radians(350.0),math.radians(130.0),math.radians(350.0)]

    dh=list(({'theta':0.0            ,'d':0.0        ,'a':0.0        ,'alpha':-np.pi        }, #rot by 180 degree
             {'theta':0.0            ,'d':-0.815     ,'a':0.350      ,'alpha':-np.pi/2      }, #from KS1 to KS2
             {'theta':-np.pi/2       ,'d':0.0        ,'a':-1.200     ,'alpha':0.0           }, #from KS2 to KS3
             {'theta':0.0            ,'d':0.0        ,'a':-0.145     ,'alpha':np.pi/2       }, #from KS3 to KS4
             {'theta':0.0            ,'d':-1.545     ,'a':0.0        ,'alpha':-np.pi/2      }, #from KS4 to KS5
             {'theta':0.0            ,'d':0.0        ,'a':0.0        ,'alpha':np.pi/2       }, #from KS5 to KS6
             {'theta':0.0            ,'d':-0.158     ,'a':0.0        ,'alpha':np.pi         })) #to tool


    def direct_kin(self, thetas):
        """ kinematic calculation
        @param [in] thetas <b><i><c> [vector-5]: </c></i></b> joint angles
        @return <b><i><c> [vector-3]: </c></i></b> destination position
        """
        trans = self.get_dh_transform(self.dh[0], 0) * \
                self.get_dh_transform(self.dh[1], thetas[0]) * self.get_dh_transform(self.dh[2], thetas[1]) * \
                self.get_dh_transform(self.dh[3], thetas[2]) * self.get_dh_transform(self.dh[4], thetas[3]) * \
                self.get_dh_transform(self.dh[5], thetas[4]) * self.get_dh_transform(self.dh[6], thetas[5])
        alpha = np.degrees(math.atan2(trans[1,0],trans[0,0]))
        beta = np.degrees(math.atan2(-trans[3,1],math.sqrt(trans[0,0]**2+trans[1,0]**2)))
        gamma = np.degrees(math.atan2(trans[2,1],trans[2,2]))
        return [np.array((trans * np.matrix((0, 0, 0, 1)).transpose()).transpose())[0][0:3],alpha,beta,gamma]


    def direct_kin_to_shoulder(self, thetas):
        """ kinematic calculation
        @param [in] thetas <b><i><c> [vector-5]: </c></i></b> joint angles
        @return <b><i><c> [vector-3]: </c></i></b> destination position
        """
        trans = self.get_dh_transform(self.dh[0], 0) * \
                self.get_dh_transform(self.dh[1], thetas[0])
        return np.array((trans * np.matrix((0, 0, 0, 1)).transpose()).transpose())[0][0:3]

    def direct_kin_to_wrist(self, thetas):
        """ kinematic calculation
        @param [in] thetas <b><i><c> [vector-5]: </c></i></b> joint angles
        @return <b><i><c> [vector-3]: </c></i></b> destination position
        """
        trans = self.get_dh_transform(self.dh[0], 0) * \
                self.get_dh_transform(self.dh[1], thetas[0]) * self.get_dh_transform(self.dh[2], thetas[1]) * \
                self.get_dh_transform(self.dh[3], thetas[2]) * self.get_dh_transform(self.dh[4], thetas[3])
        return np.array((trans * np.matrix((0, 0, 0, 1)).transpose()).transpose())[0][0:3]



    def get_dh_transform(self, dh, theta=0):
        """ Denavit-Hartenberg-Transformation
        @param [in] dh <b><i><c> [vector-4]: </c></i></b> dh parameter set for koordinate system
        @return <b><i><c> [matrix-4x4]: </c></i></b> Transformation Marix
        """

        trans = np.matrix(((cos(dh['theta'] + theta), -sin(dh['theta'] + theta) * cos(dh['alpha']),
                    sin(dh['theta'] + theta) * sin(dh['alpha']), dh['a'] * cos(dh['theta'] + theta)),
                   (sin(dh['theta'] + theta), cos(dh['theta'] + theta) * cos(dh['alpha']),
                    -cos(dh['theta'] + theta) * sin(dh['alpha']), dh['a'] * sin(dh['theta'] + theta)),
                   (0, sin(dh['alpha']), cos(dh['alpha']), dh['d']),
                   (0, 0, 0, 1)))
        return trans


    def getRotationZ(self,a):
        trans = np.matrix(( cos(a),-sin(a),0,0 ),
                          (sin(a),cos(a),0,0),
                          (0,0,1,0),
                          (0,0,0,1))
        return trans

    def getRotationXYZ(self,a,b,c,x,y,z):
        trans = np.matrix((( cos(a)*cos(b), -sin(a)*cos(c)+sin(c)*sin(b)*cos(a), sin(a)*sin(c)+cos(c)*sin(b)*cos(a),   x ),
                          ( sin(a)*cos(b),  cos(a)*cos(c)+sin(b)*sin(a)*sin(c),  cos(a)*-sin(c)+sin(b)*sin(a)*cos(c),  y ),
                          ( -sin(b),        sin(c)*cos(b),                       cos(b)*cos(c),                        z ),
                          ( 0,              0,                                   0,                                    1 )
                          ))
        return trans




    def get_inv_transform(self, dh, theta=0):
        """ inverse Denavit-Hartenberg-Transformation
        @param [in] dh <b><i><c> [vector-4]: </c></i></b> dh parameter set for koordinate system
        @return <b><i><c> [matrix-4x4]: </c></i></b> Transformation Marix
        """
        trans = np.matrix((  (cos(dh['theta'] + theta), sin(dh['theta'] + theta), 0, -dh['a']),
                             (-sin(dh['theta'] + theta) * cos(dh['alpha']), cos(dh['theta'] + theta) * cos(dh['alpha']),
                              sin(dh['alpha']), -dh['d'] * sin(dh['alpha'])),
                             (sin(dh['alpha']) * sin(dh['theta'] + theta), -cos(dh['theta'] + theta) * sin(dh['alpha']),
                              cos(dh['alpha']), -dh['d'] * cos(dh['alpha'])),
                             (0, 0, 0, 1)))
        return trans
