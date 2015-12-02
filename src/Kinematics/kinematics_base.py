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

    min_angles_ = [math.radians(-185.0),math.radians(-135.0),math.radians(-120.0)]
    max_angles_ = [math.radians(185.0),math.radians(35.0),math.radians(158.0)]

    # dh=list(({'theta':0.0           ,'d':-0.055     ,'a':0.35       ,'alpha':0.0               }, #from write plane to joint_1 (KS0)
    #         {'theta':0.0            ,'d':0.147      ,'a':0.033      ,'alpha':np.pi/2       } ,#from KS0 to joint 2 (KS1)
    #         {'theta':np.pi/2        ,'d':0.0        ,'a':0.155      ,'alpha':0.0               }, #from KS1 to joint 3 (KS2)
    #         {'theta':0.0            ,'d':0.0        ,'a':0.135      ,'alpha':0.0               }, #from KS2 to joint 4 (KS3)
    #         {'theta':np.pi/2        ,'d':0.0        ,'a':0.0        ,'alpha':np.pi/2       }, #from KS3 to joint 5 (KS4)
    #         {'theta':0.0            ,'d':0.2175     ,'a':0.0        ,'alpha':0.0               }, #from KS4 to tcp
    #         {'theta':0.0            ,'d':0.02       ,'a':0.0        ,'alpha':0.0               })) #from tcp to pencil

    #dh=list(({'theta':0.0            ,'d':0.815      ,'a':0.35      ,'alpha':np.pi/2       } ,#from KS0 to joint 2 (KS1)
    #        {'theta':np.pi/2        ,'d':0.0        ,'a':1.200      ,'alpha':0.0               }, #from KS1 to joint 3 (KS2)
    #        {'theta':0.0            ,'d':0.145        ,'a':1.545      ,'alpha':0.0               })) #from KS2 to joint 4 (KS3)

    dh=list(({'theta':0.0            ,'d':0.0        ,'a':0.0        ,'alpha':-np.pi        }, #from write plane to joint_1 (KS0)
             {'theta':0.0            ,'d':-0.815     ,'a':0.350      ,'alpha':-np.pi/2      }, #from KS1 to joint 3 (KS2)
             {'theta':-np.pi/2       ,'d':0.0        ,'a':-1.200     ,'alpha':0.0           }, #from KS2 to joint 4 (KS3)
             {'theta':0.0            ,'d':0.0        ,'a':-0.145     ,'alpha':np.pi/2       }, #from KS3 to joint 5 (KS4)
             {'theta':0.0            ,'d':-1.545     ,'a':0.0        ,'alpha':-np.pi/2      }, #from KS4 to tcp
             {'theta':0.0            ,'d':0.0        ,'a':0.0        ,'alpha':np.pi/2       }, #from KS4 to tcp
             {'theta':0.0            ,'d':-0.158     ,'a':0.0        ,'alpha':np.pi         })) #from tcp to pencil


    def direct_kin(self, thetas):
        """ kinematic calculation
        @param [in] thetas <b><i><c> [vector-5]: </c></i></b> joint angles
        @return <b><i><c> [vector-3]: </c></i></b> destination position
        """
        trans = self.get_dh_transform(self.dh[0], 0) * \
                self.get_dh_transform(self.dh[1], thetas[0]) * self.get_dh_transform(self.dh[2], thetas[1]) * \
                self.get_dh_transform(self.dh[3], thetas[2]) * self.get_dh_transform(self.dh[4], thetas[3]) * \
                self.get_dh_transform(self.dh[5], thetas[4]) * self.get_dh_transform(self.dh[6], thetas[5])
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
