import Kinematics as kin
import kinematics_geom as kin_base
import MotionFunctions as mf
import numpy as np
import math



kin=kin_base.Kinematics_geom()

print "Zero koordinates"
zero = kin.direct_kin_to_wrist([math.radians(0.0), math.radians(0.0), math.radians(0.0), 0.0, 0.0, 0.0])
print np.round(zero,3)

#wp = kin.direct_kin_to_wrist([math.radians(0.0) , math.radians(0.0) , math.radians(0.0) , 0.0])
#print "WP: ",wp
zero=[0.1, 1.1, 2.0]


angles = kin.inverse_kin(zero)
print "angles: "
print np.degrees(np.round(angles,3))
print np.round(kin.direct_kin_to_wrist([angles[0][0],angles[0][1],angles[0][2],0.0]),3)
print np.round(kin.direct_kin_to_wrist([angles[1][0],angles[1][1],angles[1][2],0.0]),3)
print np.round(kin.direct_kin_to_wrist([angles[2][0],angles[2][1],angles[2][2],0.0]),3)
print np.round(kin.direct_kin_to_wrist([angles[3][0],angles[3][1],angles[3][2],0.0]),3)