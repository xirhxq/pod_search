from pyquaternion import Quaternion
from collections import deque
import rospy

from Utils import *


class QuaternionBuffer:
    def __init__(self, name='Buffer', maxAge=0.4):
        self.buffer = deque()
        self.name = name
        self.maxAge = maxAge

        self.preT = None
        self.preQ = None

    @property
    def empty(self):
        return len(self.buffer) == 0

    def addMessage(self, msg):
        self.buffer.append((rospy.Time.now().to_sec(), Quaternion(msg.w, msg.x, msg.y, msg.z)))

    def getMessage(self):
        if self.empty:
            return None

        targetTime = rospy.Time.now().to_sec() - self.maxAge
        print(f'nowTime {rospy.Time.now().to_sec()} targetTime {targetTime}')
        while self.buffer[0][0] < targetTime:
            self.preT = self.buffer[0][0]
            self.preQ = self.buffer[0][1]
            self.buffer.popleft()
            print(f'pop {self.preT}')

        if self.preT is None or self.preQ is None:
            return None

        if not (self.preT <= targetTime <= self.buffer[0][0]):
            raise AssertionError(f'Buffer not right {self.preT.to_sec():.3f} -- {targetTime.to_sec():3f} -- {self.buffer[0][0]:.3f}')

        t1 = self.preT
        t2 = self.buffer[0][0]
        val1 = self.preQ
        val2 = self.buffer[0][1]
        t = targetTime
        q = Quaternion.slerp(val1, val2, (t - t1) / (t2 - t1))

        print(f'{t1} -- {t} -- {t2}\n{val1} -- {q} -- {val2}')
        print(f'uavQuatwithDelay: {[q.x, q.y, q.z, q.w]}')
        return [q.x, q.y, q.z, q.w]

    def getMessageNoDelay(self):
        if self.empty:
            return None
        q = [self.buffer[-1][1].x, self.buffer[-1][1].y, self.buffer[-1][1].z, self.buffer[-1][1].w]
        print(f'uavQuatNoDelay: {q}')
        return q 


if __name__ == '__main__':
    qb = QuaternionBuffer()
    rospy.init_node('QuaternionBuffer', anonymous=True)

