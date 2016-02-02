import numpy as np

class Motion(object):
	def __init__(self, kin):
		self.kin = kin

	def _resample_movement(self, path, dt, vel):
		result = [path[0]]
		
		target_distance = vel * dt
		
		i = 1
		while i < path.shape[0]:
			curr = result[-1]
			n = path[i]
			
			distance = np.linalg.norm(n - curr)
			
			if target_distance > distance:
				i += 1
			elif target_distance < distance:
				frac = np.tile(target_distance / distance, path.shape[1])
				next_new = curr + (n - curr) * frac
				result.append(next_new)
			else:
				result.append(next)
				i += 1
		
		#result.append(path[-1])
		
		return np.array(result)
	
	def sample_trajectory(self, trajectory, tool):
		trajectory = self._resample_movement(trajectory, 0.01, 0.4)
		
		# Now solve the inverse kinematic for every subpoint
		angles = np.empty([trajectory.shape[0], 6])

		for i, t in enumerate(trajectory):
			sols = self.kin.inverse_kin(t,tool)
			sol = None
			min_sol = 1000000.0
			
			for s in sols:
				if self.kin.isSolutionValid(s, t):
					if i > 0:
						a = np.subtract(angles[i - 1], s)
						d = a.dot(a)
						
						if d < min_sol:
							min_sol = d
							sol = s
					else:
						sol = s
						break
				
			if sol is None:
				print "No solution found!"
				
				return None
			
			angles[i] = sol
		return angles