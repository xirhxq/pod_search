from PodParas import *

class PodAngles:
    def __init__(self, pitchDeg=0, yawDeg=0, hfovDeg=0, maxRateDeg=0, laserOn=False, laserRange=0):
        self.pitchDeg = pitchDeg
        while yawDeg <= -180:
            yawDeg += 360
        while yawDeg > 180:
            yawDeg -= 360
        self.yawDeg = yawDeg
        self.hfovDeg = clipHfov(hfovDeg)
        self.maxRateDeg = maxRateDeg
        self.laserOn = laserOn
        self.laserRange = laserRange
            
    def __repr__(self):
        return f'PodAngles(pitch={self.pitchDeg:.2f}, yaw={self.yawDeg:.2f}, hfov={self.hfovDeg:.2f}, maxRate={self.maxRateDeg:.2f}, laserOn={self.laserOn})'

    def __eq__(self, other):
        return (
            abs(self.pitchDeg - other.pitchDeg) < 0.001 and 
            abs(self.yawDeg - other.yawDeg) < 0.001 and 
            abs(self.hfovDeg - other.hfovDeg) < 0.001 and 
            # self.maxRateDeg == other.maxRateDeg and 
            self.laserOn == other.laserOn and 
            self.laserRange == other.laserRange
        )

