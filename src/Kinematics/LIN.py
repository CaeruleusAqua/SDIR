import numpy as np
from MotionProfile import MotionProfileAsync

class LIN(object):
    def __init__(self, kin):
        self.kin = kin
    
    def calculate(self, start_cfg, target_cfg):
        print "Start: ", start_cfg
        print "target: ", target_cfg
        #First we make a relative movement vom [0, 0, 0] to [x, y, z]
        diff = np.subtract(target_cfg[:3], start_cfg)
        
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
            trajectory[trajectory.shape[0] - 1] = target_cfg[:3]
        
        # Now solve the inverse kinematic for every subpoint
        angles = np.empty([trajectory.shape[0], 6])

        for i, t in enumerate(trajectory):
            sols = self.kin.inverse_kin(t,np.radians(target_cfg[3:]))
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
            
            
            #a = sol
            
            #if (i>0):
            #    a = np.subtract(angles[i - 1], sol)
            
            #max_vel = np.array([1.74532925, 1.3962634, 1.3962634, 4.01425728, 2.87979327, 4.34586984]) 
                        
            
            #if (np.divide(max_vel,np.tile(100.0, max_vel.shape[0])) < np.abs(a))[0:3].any():
            #    print a
            #    
            #    print np.divide(max_vel,np.tile(100.0, max_vel.shape[0]))
            #    
            #    print np.divide(max_vel,np.tile(100.0, max_vel.shape[0])) < np.abs(a)
            #    
            #    print "No solution found!"
            #    
            #    return None
            
            
            print sol
            
            angles[i] = sol

        return angles
            