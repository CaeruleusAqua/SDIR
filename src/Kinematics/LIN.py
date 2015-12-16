import numpy as np
from MotionProfile import MotionProfileAsync

class LIN(object):
    def __init__(self, kin):
        self.kin = kin
    
    def calculate(self, start_cfg, target_cfg):
        #First we make a relative movement vom [0, 0, 0] to [x, y, z]
        diff = np.subtract(target_cfg, start_cfg)
        
        # Now we apply our motion profile over the length of that resulting vector
        length = np.sqrt(diff.dot(diff))
        
        profile = MotionProfileAsync(np.array([4]), np.array([0.8]))
        
        trajectory_length = profile.calculate(np.array([0]), np.array([length]), 0.01)
        
        trajectory_length = trajectory_length.reshape((trajectory_length.shape[0],))
        
        # And convert those back to absolute cartesian coordinates
        trajectory = np.add(start_cfg, np.multiply(
                                 diff,
                                 np.tile(
                                         np.divide(trajectory_length, 
                                                   np.tile([length], trajectory_length.shape[0]))
                                         .reshape((trajectory_length.shape[0], 1)),
                                          3)
        ))
        
        if trajectory.shape[0] != 0:
            trajectory[trajectory.shape[0] - 1] = target_cfg
        
        # No solve the inverse kinematic for every subpoint
        angles = np.empty([trajectory.shape[0], 6])

        for i, t in enumerate(trajectory):
            sols = self.kin.inverse_kin(t)
            sol = None
            min = 1000000.0
            
            for s in sols:
                if self.kin.isSolutionValid(s, t):
                    if i > 0:
                        a = np.subtract(angles[i - 1], s)
                        d = np.sqrt(a.dot(a))
                        
                        if d < min:
                            min = d
                            sol = s
                    else:
                        sol = s
                        break
                
            if sol is None:
                print "No solution found!"
                
                return None
            
            angles[i] = sol

        return angles
            