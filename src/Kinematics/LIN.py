import numpy as np
from MotionProfile import MotionProfileAsync
from Motion import Motion

class LIN(Motion):
    def __init__(self, kin):
        super(LIN, self).__init__(kin)
    
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
        
        return self.sample_trajectory(trajectory, target_cfg[3:])
            