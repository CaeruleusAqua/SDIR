import numpy as np
import math

class MotionProfileAsync(object):
	
	# Note: max_accel and max_vel may change during calculations
	def __init__(self, max_accel, max_vel):
		self.max_accel = max_accel
		self.max_vel = max_vel
	
	def calc_diff(self, start_cfg, target_cfg):
		return np.subtract(target_cfg, start_cfg)
	
	def calc(self, diff):
		# Acceleration time
		t_a = np.divide(self.max_vel, self.max_accel)
		# Acceleration way
		s_a = np.divide(np.multiply(self.max_vel, t_a), np.tile(2.0, self.max_vel.shape[0]))
		
		# We assume that acceleration = deacceleration
		t_d = t_a
		s_d = s_a
		
		t = np.empty([self.max_vel.size, 3])
		
		redo = False
		
		for i, s in enumerate(diff):
			if s >= s_a[i] + s_d[i]:
				# Time we use constant velocity
				t_c = (s - s_a[i] - s_d[i]) / self.max_vel[i]
				t[i] = np.array([t_a[i], t_c, t_d[i]])
			elif s <= 0.01:
				t[i] = np.array([0.0, 0.0, 0.0])
			else:
				# We don't have enough way to accelerate to maximum velocity
				# so we recalculate it and try again
				self.max_vel[i] = math.sqrt(2.0 * s / (1.0/self.max_accel[i] + 1.0/self.max_accel[i]))
				
				redo = True
		
		if redo:
			return self.calc(diff)
		
		return t
		
	def sample(self, diff, t, time_step):
		print t		
		print np.sum(t, axis=1)
		t_max = np.amax(np.sum(t, axis=1))
		print t_max

		steps = int(math.ceil(t_max / time_step))
		print steps
		
		if steps == 0:
			return np.empty([0, diff.size])
		
		samples = np.empty([steps, diff.size])
		
		for i in xrange(steps):
			time = i * time_step
			
			for j, t_values in enumerate(t):
				if time < t_values[0]:
					samples[i][j] = self.max_accel[j] / 2.0 * math.pow(time, 2)
				elif time < t_values[0] + t_values[1]:
					samples[i][j] = self.max_vel[j] * (time - t_values[0]) + self.max_accel[j] / 2.0 * math.pow(t_values[0], 2)
				elif time < t_values[0] + t_values[1] + t_values[2]:
					local_t = time - t_values[0] - t_values[1]
					samples[i][j] = self.max_accel[j] / 2.0 * math.pow(local_t, 2) + self.max_vel[j] * (local_t + t_values[1]) + self.max_accel[j] / 2.0 * math.pow(t_values[0], 2)
				else:
					samples[i][j] = diff[j]

		# Ensure that last values are always correct and don't have
		# sampling related errors		
		samples[steps - 1] = diff
		
		print samples
		return samples
	
	def calculate(self, start_cfg, target_cfg, time_step):
		diff = self.calc_diff(start_cfg, target_cfg)
		sign = np.sign(diff)
		diff = np.absolute(diff)
		
		t = self.calc(diff)
		samples = self.sample(diff, t, time_step)
		samples = np.multiply(np.reshape(np.tile(sign, samples.shape[0]), samples.shape), samples)
		return np.add(np.reshape(np.tile(start_cfg, samples.shape[0]), samples.shape), samples)
		

class MotionProfileSync(MotionProfileAsync):
	pass

class MotionProfileFullSync(MotionProfileAsync):
	pass