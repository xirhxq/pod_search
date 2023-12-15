import numpy as np

class SearchPoint:
    def __init__(self, uavPosF=300, uavPosL=0, uavPosU=40, uavYaw=0, timeout=10):
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