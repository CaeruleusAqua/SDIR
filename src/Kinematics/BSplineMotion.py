from __future__ import division

import numpy as np
from SplineMotion import SplineMotion
from scipy.linalg import solve_banded

class BSplineMotion(SplineMotion):
    def __init__(self, kin):
        super(BSplineMotion, self).__init__(kin)
    
    def _forward_control_points(self, dps):
        num_cps = dps.shape[0] - 1;

        ab = np.matrix([
          np.tile(1, num_cps),
          np.tile(4, num_cps),
          np.tile(1, num_cps)
        ])
        
        # Fixup first and last row
        ab[0,0] = 0
        ab[1,0] = 2
        ab[1, num_cps - 1] = 7
        ab[2, num_cps - 1] = 0
        
        b = np.empty([num_cps, dps.shape[1]])
        
        for i in range(0, num_cps - 1):
            b[i] = 4 * dps[i] + 2 * dps[i + 1]
        
        # Fixup first and last element
        b[0] = dps[0] + 2 * dps[1]
        b[num_cps - 1] = 8 * dps[num_cps - 1] + dps[num_cps]
        
        return solve_banded((1, 1), ab, b)
    
    def _backward_control_points(self, dps, fcps):
        n = fcps.shape[0]
        bcps = np.empty(fcps.shape)

        for i in range(0, n):
            if i < n - 1:
                bcps[i] = 2 * dps[i + 1] - fcps[i + 1]
            else:
                bcps[i] = (dps[-1] + fcps[-1])/np.tile(2, fcps.shape[1])
        
        return bcps
    
    def _apply_weights(self, p, w):
        return np.append(p, np.tile(1, (p.shape[0], 1)), axis=1) * np.tile(w.reshape(p.shape[0], 1), p.shape[1] + 1)
    
    def _remove_weights(self, p):
        return p[0:-1] / np.tile(p[-1], p.shape[0] - 1)
    
    def _bezier(self, ps, w, n):
        result = np.empty([n, ps.shape[1]])
        ps = self._apply_weights(ps, w)

        for k in range(0, n):
            t = k / (n - 1)
            tmp = np.copy(ps)
            frac_a = np.tile((1 - t), tmp.shape[1])
            frac_b = np.tile(t, tmp.shape[1])
            
            for j in range(ps.shape[0] - 1, -1, -1):
                for i in range(0, j):
                    tmp[i] =  frac_a * tmp[i] + frac_b * tmp[i + 1] 
            
            result[k] = self._remove_weights(tmp[0])
        
        return result
    
    def _bezier_path(self, dps, fcps, bcps, n_each):
        n = dps.shape[0] - 1
        result = np.empty([(n_each) * n + 1, dps.shape[1]])
        
        for i in range(0, n):
            points = np.array([dps[i], fcps[i], bcps[i], dps[i+1]])
            
            b = self._bezier(points, np.tile(1, 4), n_each + 1)
            
            result[i * n_each:(i+1)*n_each] = b[:-1]
        
        result[-1] = dps[-1]
        
        return result
    
    def move(self, dps, dt, tool):
        base = np.mean(dps, axis=0)
        
        dps -= np.tile(base, (dps.shape[0], 1))
        
        fcps = self._forward_control_points(dps)
        bcps = self._backward_control_points(dps, fcps)
        path = self._bezier_path(dps, fcps, bcps, 200)
        
        path += np.tile(base, (path.shape[0], 1))
                
        return (path, self.sample_trajectory(path, tool))
    