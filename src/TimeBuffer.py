from collections import deque
import rospy
from std_msgs.msg import Float32

import PodParas


class TimeBuffer:
    def __init__(self, name='Buffer'):
        self.buffer = deque()
        self.name = name
        self.maxAge = PodParas.podDelay

        self.preT = None
        self.preMsg = 0.0

    @property
    def empty(self):
        return not self.buffer

    def addMessage(self, msg):
        time = rospy.Time.now()
        self.buffer.append((time, msg))

    def getData(self, x):
        if hasattr(x, 'data'):
            return x.data
        else:
            return x

    def getMessage(self):
        if not self.buffer:
            return self.getData(self.preMsg)

        currentTime = rospy.Time.now()
        targetTime = currentTime - rospy.Duration.from_sec(self.maxAge)
        
        while self.buffer and self.buffer[0][0] < targetTime:
            self.preT = self.buffer[0][0]
            self.preMsg = self.buffer[0][1]
            self.buffer.popleft()

        if self.preT == None or self.preMsg == None:
            return self.getData(self.preMsg)
        
        t1 = self.preT.to_sec()
        t2 = self.buffer[0][0].to_sec()
        val1 = self.getData(self.preMsg)
        val2 = self.getData(self.buffer[0][1])

        ret = val1 + (val2 - val1) * (targetTime.to_sec() - t1) / (t2 - t1)
        return ret

        return self.buffer[0][1].data

    def getMessageNoDelay(self):
        if not self.buffer:
            return self.getData(self.preMsg)
        currentTime = rospy.Time.now()
        targetTime = currentTime - rospy.Duration.from_sec(self.maxAge)
        while self.buffer and self.buffer[0][0] < targetTime:
            self.preT = self.buffer[0][0]
            self.preMsg = self.buffer[0][1]
            self.buffer.popleft()

        if not self.buffer:
            return self.getData(self.preMsg)
        return self.buffer[-1][1].data

    def outputBuffer(self):
        print(self.name + ': [')
        t = rospy.Time.now()
        for b in self.buffer:
            print('-', (t - b[0]).to_sec(), b[1])
        print(']')