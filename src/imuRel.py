#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from time import sleep

from scipy.spatial.transform import Rotation as R
import scipy.spatial.transform as st

class IMUREL:
    def __init__(self):
        rospy.init_node('imuRel', anonymous=True)
        self.name = 'suav'
        self.djiName = '/dji_osdk_ros'
        self.q0 = None
        self.q = None
        self.qRel = None

        self.r0 = None
        self.r = None
        self.rRel = None

        self.ypr0 = None

        rospy.Subscriber(self.name + self.djiName + '/imu', Imu, self.imuCallback, queue_size=1)

        self.imuRelPub = rospy.Publisher(self.name + self.djiName + '/imuRel', Imu, queue_size=1)

    
    def imuCallback(self, msg):
        self.q = [
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
        ]

        if self.q0 is None:
            self.q0 = self.q
            print(f'q0 is {self.q0}')
            self.r0 = R.from_quat(self.q0)
            self.ypr0 = self.r0.as_euler('zyx', degrees=True)
        else: 
            self.r = R.from_quat(self.q)

            self.rRel = self.r0.inv() * self.r

            self.qRel = self.rRel.as_quat()
            # ypr = self.rRel.as_euler('zyx', degrees=True)
            # yprStr = 'ypr'
            # print('-' * 20)
            print('Euler 0: ', ' '.join([f'{"ypr"[i]}{self.ypr0[i]:6.3f}deg' for i in range(3)]))
            # print('Now Euler: ', ' '.join([f'{yprStr[i]}{ypr[i]:6.3f}deg' for i in range(3)]))
            # print(self.qRel)

            imuMsg = Imu()
            imuMsg.header.stamp = rospy.Time.now()
            imuMsg.orientation.x = self.qRel[0]
            imuMsg.orientation.y = self.qRel[1]
            imuMsg.orientation.z = self.qRel[2]
            imuMsg.orientation.w = self.qRel[3]

            self.imuRelPub.publish(imuMsg)



if __name__ == '__main__':
    ir = IMUREL()
    rospy.spin()
