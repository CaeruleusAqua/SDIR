import numpy as np

class Motion(object):
	def __init__(self, kin):
		self.kin = kin
	
	def sample_trajectory(self, trajectory, tool):
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
						d = np.sqrt(a.dot(a))
						
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