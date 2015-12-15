import numpy as np
from MotionProfile import MotionProfileAsync

class LIN(object):
    def __init__(self, kin):
        self.kin = kin
    
    def calculate(self, start_cfg, target_cfg):
        print start_cfg
        print target_cfg
        
        diff = np.subtract(target_cfg, start_cfg)
        length = np.sqrt(diff.dot(diff))
        
        profile = MotionProfileAsync(np.array([4]), np.array([0.8]))
        
        trajectory_length = profile.calculate(np.array([0]), np.array([length]), 0.01)
        
        trajectory_length = trajectory_length.reshape((trajectory_length.shape[0],))
        
        print trajectory_length
        
        trajectory = np.multiply(
                                 diff,
                                 np.tile(
                                         np.divide(trajectory_length, 
                                                   np.tile([length], trajectory_length.shape[0]))
                                         .reshape((trajectory_length.shape[0], 1)),
                                          3)
        )
        
        trajectory = np.add(start_cfg, trajectory)
        
        print trajectory
                
        trajectory[trajectory.shape[0] - 1] = target_cfg
        
        angles = np.empty([trajectory.shape[0], 6])

        for i, t in enumerate(trajectory):
            sols = self.kin.inverse_kin(t)
            sol = None
            
            for s in sols:
                if self.kin.isSolutionValid(s):
                    sol = s
                    break
                
            if sol is None:
                print "No solution found!"
                
                return None
            
            angles[i] = np.concatenate((sol, np.array([0, 0, 0])))
        
        return angles
            