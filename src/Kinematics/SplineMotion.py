from Motion import Motion

class SplineMotion(Motion):
    def __init__(self, kin):
        super(SplineMotion, self).__init__(kin)
    
    # Returns (trajectory, angles)
    def move(self, points, dt, tool):
        raise NotImplemented()