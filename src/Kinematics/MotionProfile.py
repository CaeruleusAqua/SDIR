import numpy as np
import math

ALMOST_ZERO = 0.0000001

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
	
	def _calc(self, diff, max_vel, max_accel):
		max_vel = np.copy(max_vel)
		max_accel = np.copy(max_accel)
		
		# Acceleration time
		t_a = np.round(np.divide(max_vel, max_accel), 6)
		# Acceleration way
		s_a = np.round(np.divide(np.multiply(max_vel, t_a), np.tile(2.0, max_vel.shape[0])), 6)
		
		# We assume that acceleration = deacceleration
		t_d = t_a
		s_d = s_a
		
		t = np.empty([max_vel.shape[0], 3])
		
		redo = False
		
		for i, s in enumerate(diff):
			if s >= s_a[i] + s_d[i]:
				# Time we use constant velocity
				t_c = (s - s_a[i] - s_d[i]) / max_vel[i]
				t[i] = np.array([t_a[i], t_c, t_d[i]])
			elif s <= ALMOST_ZERO:
				t[i] = np.array([0.0, 0.0, 0.0])
			else:
				# We don't have enough way to accelerate to maximum velocity
				# so we recalculate it and try again
				n = math.sqrt(2.0 * s / (1.0 / max_accel[i] + 1.0 / max_accel[i]))
				print "%i: Change from %f to %f" % (i, max_vel[i], n)
				print s, s_a[i], s_d[i]
				max_vel[i] = n
				
				redo = True
		
		if redo:
			return self._calc(diff, max_vel, max_accel)
		
		print t
		
		return t, max_vel, max_accel
	
	def calc(self, diff, max_vel, max_accel):
		return self._calc(diff, max_vel, max_accel)
		
	def sample(self, diff, t, time_step, max_vel, max_accel):
		# print t		
		# print np.sum(t, axis=1)
		t_max = np.amax(np.sum(t, axis=1))
		# print t_max

		steps = int(math.ceil(t_max / time_step))
		# print steps
		
		if steps == 0:
			return np.empty([0, diff.size])
		
		samples = np.empty([steps, diff.size])
		
		for i in xrange(steps):
			time = i * time_step
			
			for j, t_values in enumerate(t):
				if time < t_values[0]:
					samples[i][j] = max_accel[j] / 2.0 * math.pow(time, 2)
				elif time < t_values[0] + t_values[1]:
					local_time = time - t_values[0]
					samples[i][j] = max_vel[j] * local_time + max_accel[j] / 2.0 * math.pow(t_values[0], 2)
				elif time < t_values[0] + t_values[1] + t_values[2]:
					local_t = time - t_values[0] - t_values[1]
					samples[i][j] = -max_accel[j] / 2.0 * math.pow(local_t, 2) + max_vel[j] * local_t + max_vel[j] * (t_values[1]) + max_accel[j] / 2.0 * math.pow(t_values[0], 2)
				else:
					samples[i][j] = diff[j]

		# Ensure that last values are always correct and don't have
		# sampling related errors		
		samples[steps - 1] = diff
		
		# print samples
		return samples
	
	def calculate(self, start_cfg, target_cfg, time_step):
		rel, sign = self.to_rel(start_cfg, target_cfg)

		t, max_vel, max_accel = self.calc(rel, self.max_vel, self.max_accel)
		samples = self.sample(rel, t, time_step, max_vel, max_accel)
		
		return self.to_abs(start_cfg, samples, sign)

class MotionProfileSync(MotionProfileAsync):
	def calc(self, diff, max_vel, max_accel):
		# First calculate async solution
		async, max_vel, max_accel = super(MotionProfileSync, self).calc(diff, max_vel, max_accel)
		
		# Find axis taking the longes time
		t_max = np.tile(np.amax(np.sum(async, axis=1)), async.shape[0])
		
		max_accel_to_2 = np.multiply(max_accel, max_accel)
		
		tmp1 = np.sqrt(
					np.subtract(np.multiply(
							np.multiply(max_accel_to_2, max_accel_to_2),
							np.multiply(t_max, t_max)
						),
						np.add(
							np.multiply(
								np.multiply(
									np.tile(2.0, async.shape[0]),
									max_accel
								),
								np.multiply(
									max_accel_to_2,
									diff
								)
							),
							np.multiply(
								np.multiply(
									np.tile(2.0, async.shape[0]),
									max_accel
								),
								np.multiply(
									max_accel_to_2,
									diff
								)
							)
						)
					)
				)
		tmp2 = np.multiply(max_accel_to_2, t_max)
		tmp3 = np.add(max_accel, max_accel)
		
		max_vel_new = np.divide(np.subtract(tmp2, tmp1), tmp3)
		
		# For axis that don't move, we're getting a max_vel of 0 which is not possible
		# so we restore the old value
		for i, s in enumerate(diff):
			if s < ALMOST_ZERO:
				max_vel_new[i] = max_vel[i]
		
		sync, max_vel_new, max_accel = super(MotionProfileSync, self).calc(
			diff,
			max_vel_new,
			max_accel
		)
		
		return sync, max_vel_new, max_accel
	
