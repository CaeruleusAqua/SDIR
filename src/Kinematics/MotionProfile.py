import numpy as np
import math

ALMOST_ZERO=0.0000001

class MotionProfileAsync(object):
	
	# Note: max_accel and max_vel may change during calculations
	def __init__(self, max_accel, max_vel):
		self.max_accel = max_accel
		self.max_vel = max_vel
	
	def to_rel(self, start_cfg, target_cfg):
		rel = np.subtract(target_cfg, start_cfg)
		sign = np.sign(rel)
		return (np.absolute(rel), sign)
	
	def to_abs(self, start_cfg, samples, sign):
		samples = np.multiply(np.reshape(np.tile(sign, samples.shape[0]), samples.shape), samples)
		
		return np.add(np.reshape(np.tile(start_cfg, samples.shape[0]), samples.shape), samples)
	
	def calc(self, diff):
		# Acceleration time
		t_a = np.round(np.divide(self.max_vel, self.max_accel), 6)
		# Acceleration way
		s_a = np.round(np.divide(np.multiply(self.max_vel, t_a), np.tile(2.0, self.max_vel.shape[0])), 6)
		
		# We assume that acceleration = deacceleration
		t_d = t_a
		s_d = s_a
		
		t = np.empty([self.max_vel.shape[0], 3])
		
		redo = False
		
		for i, s in enumerate(diff):
			if s >= s_a[i] + s_d[i]:
				# Time we use constant velocity
				t_c = (s - s_a[i] - s_d[i]) / self.max_vel[i]
				t[i] = np.array([t_a[i], t_c, t_d[i]])
			elif s <= ALMOST_ZERO:
				t[i] = np.array([0.0, 0.0, 0.0])
			else:
				# We don't have enough way to accelerate to maximum velocity
				# so we recalculate it and try again
				n = math.sqrt(2.0 * s / (1.0/self.max_accel[i] + 1.0/self.max_accel[i]))
				print "%i: Change from %f to %f" % (i, self.max_vel[i], n)
				print s, s_a[i], s_d[i]
				self.max_vel[i] = n
				
				redo = True
		
		if redo:
			return self.calc(diff)
		
		print t
		
		return t
		
	def sample(self, diff, t, time_step):
		#print t		
		#print np.sum(t, axis=1)
		t_max = np.amax(np.sum(t, axis=1))
		#print t_max

		steps = int(math.ceil(t_max / time_step))
		#print steps
		
		if steps == 0:
			return np.empty([0, diff.size])
		
		samples = np.empty([steps, diff.size])
		
		for i in xrange(steps):
			time = i * time_step
			
			for j, t_values in enumerate(t):
				if time < t_values[0]:
					samples[i][j] = self.max_accel[j] / 2.0 * math.pow(time, 2)
				elif time < t_values[0] + t_values[1]:
					local_time = time - t_values[0]
					samples[i][j] = self.max_vel[j] * local_time + self.max_accel[j] / 2.0 * math.pow(t_values[0], 2)
				elif time < t_values[0] + t_values[1] + t_values[2]:
					local_t = time - t_values[0] - t_values[1]
					samples[i][j] = -self.max_accel[j] / 2.0 * math.pow(local_t, 2) + self.max_vel[j] * local_t + self.max_vel[j] * (t_values[1]) + self.max_accel[j] / 2.0 * math.pow(t_values[0], 2)
				else:
					samples[i][j] = diff[j]

		# Ensure that last values are always correct and don't have
		# sampling related errors		
		samples[steps - 1] = diff
		
		#print samples
		return samples
	
	def calculate(self, start_cfg, target_cfg, time_step):
		rel, sign = self.to_rel(start_cfg, target_cfg)

		t = self.calc(rel)
		samples = self.sample(rel, t, time_step)
		
		return self.to_abs(start_cfg, samples, sign)

class MotionProfileSync(MotionProfileAsync):
	pass

class MotionProfileFullSync(MotionProfileAsync):
	pass
