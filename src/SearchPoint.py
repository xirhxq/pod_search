import numpy as np

class SearchPoint:
    def __init__(self, id, uavPosF, uavPosL, uavPosU, uavYaw, yawRange, timeout):
        self.id = id
        self.uavPosF = uavPosF
        self.uavPosL = uavPosL
        self.uavPosU = uavPosU
        self.uavYaw = uavYaw
        self.timeout = timeout
    
    @property
    def uavPos(self):
        return np.array([[self.uavPosF], [self.uavPosL], [self.uavPosU]])
    
    def __str__(self):
        return f'({self.uavPosF}, {self.uavPosL}, {self.uavPosU}), {self.uavYaw} Deg, {self.timeout} seconds'
    
    def __repr__(self):
        return f'({self.uavPosF}, {self.uavPosL}, {self.uavPosU}), {self.uavYaw} Deg, {self.timeout} seconds'
    
    def toList(self):
        return [self.id, self.uavPosF, self.uavPosL, self.uavPosU, self.uavYaw, self.timeout]