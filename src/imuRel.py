#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from time import sleep

from scipy.spatial.transform import Rotation as R

class IMUREL:
    def __init__(self):
        rospy.init_node('imuRel', anonymous=True)
        self.name = 'suav'
        self.q0 = None
        self.q = None
        self.qRel = None

        rospy.Subscriber('/imu/data', Imu, self.imuCallback)

        self.imuRelPub = rospy.Publisher('/' + self.name + '/imuRel', Imu, queue_size=1)

    
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
        else: 
            r0 = R.from_quat(self.q0)
            r = R.from_quat(self.q)

            rRel = r0.inv() * r

            qRel = rRel.as_quat()
            print(rRel.as_euler('zyx', degrees=True))
            print(qRel)



if __name__ == '__main__':
    ir = IMUREL()
    rospy.spin()
