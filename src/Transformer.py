#! /usr/bin/env python3

# Class Transformer: transform pixel error to absolute position
# using the onboard pod pitch, yaw, zoom and UAV pitch, yaw, roll
# assume the UAV is at (0, 0, h)

# get UAV orientation by subscribe to uavName + '/imu' topic, sensor_msgs/Imu type
# and calculate by the quaternion to euler angles

# get pod pitch and yaw by subscribe to '/pod_comm/pitch' and '/pod_comm/yaw' topic, std_msgs/Float32 type

import argparse
import time
from collections import deque
from math import degrees, radians
from signal import signal, SIGINT

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Imu
from spirecv_msgs.msg import TargetsInFrame
from std_msgs.msg import Float32, Bool, Float64MultiArray, Int16

from Classifier import Classifier
from DataLogger import DataLogger
from ShowBar import ShowBar
from Utils import *


def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    exit(0)


signal(SIGINT, signal_handler)


class TimeBuffer:
    def __init__(self, name='Buffer'):
        self.buffer = deque()
        self.name = name

    @property
    def empty(self):
        return not self.buffer

    def addMessage(self, msg, maxAge=0.5):
        time = rospy.Time.now()
        self.buffer.append((time, msg))

        oldestTime = time - rospy.Duration.from_sec(maxAge)

        while self.buffer and self.buffer[0][0] < oldestTime:
            self.buffer.popleft()

    def getMessage(self, time):
        if not self.buffer:
            return None

        currentTime = rospy.Time.now()
        targetTime = currentTime - rospy.Duration.from_sec(time)

        return self.buffer[0][1]
        closestTimeDiff = float('inf')
        closestMsg = None

        for b in self.buffer:
            timeDiff = abs((b[0] - targetTime).to_sec())
            if timeDiff < closestTimeDiff:
                closestTimeDiff = timeDiff
                closestMsg = b[1]
        
        return closestMsg

    def getMessageNoDelay(self):
        if not self.buffer:
            return None
        return self.buffer[-1][1]

    def outputBuffer(self):
        print(self.name + ': [')
        t = rospy.Time.now()
        for b in self.buffer:
            print('-', (t - b[0]).to_sec(), b[1])
        print(']')


class Transformer:
    def __init__(self, logOn=False):
        self.TRANSFORM_DEBUG = False
        self.orderFromSearcher = False
        self.uavQuat = [0, 0, 0, 1]

        self.h = 1.6
        self.a = self.h / 100 * 3000
        self.selfPos = np.array([-self.h / 4, 0, self.h])

        self.podPitchBuffer = TimeBuffer('Pod Pitch Buffer')
        self.podYawBuffer = TimeBuffer('Pod Yaw Buffer')

        self.podHfovBuffer = TimeBuffer('Pod HFov Buffer')

        self.uavName = 'M300'
        self.podName = 'pod_comm'

        rospy.Subscriber('/' + self.uavName + '/imu', Imu, self.imuCallback)
        rospy.Subscriber('/' + self.uavName + '/pos', Odometry, self.posCallback)
        rospy.Subscriber('/' + self.uavName + '/height', Float32, self.hCallback)

        rospy.Subscriber('/' + self.podName + '/pitch', Float32, self.pitchCallback)
        rospy.Subscriber('/' + self.podName + '/yaw', Float32, self.yawCallback)
        rospy.Subscriber('/' + self.podName + '/hfov', Float32, self.hfovCallback)

        rospy.Subscriber('/uav1/spirecv/common_object_detection', TargetsInFrame, self.targetsCallback, queue_size=1)

        rospy.Subscriber('/' + self.podName + '/toTransformer', Bool, self.orderFromSearcherCallback)

        self.clsfy = Classifier()

        self.logOn = logOn
        if self.logOn:
            self.dtlg = DataLogger("data.csv")

        self.startTime = rospy.Time.now().to_sec()

        self.targetsAvailable = 30
        variable_info = [
                            ("rosTime", "double"),
                            ("podYaw", "double"),
                            ("podPitch", "double"),
                            ("podYawDelayed", "double"),
                            ("podPitchDelayed", "double"),
                            ("targetCnt", "int"),
                        ] + [
                            (f'target{i}[3]', "list") for i in range(self.targetsAvailable)
                        ]

        if self.logOn:
            self.dtlg.initialize(variable_info)

        self.aimPub = rospy.Publisher('/' + self.podName + '/aim', Float64MultiArray, queue_size=1)
        self.aimFailSub = rospy.Subscriber('/' + self.podName + '/aimFail', Int16, self.aimFailCallback, queue_size=1)

    def aimFailCallback(self, msg):
        self.clsfy.targetsCheck[msg.data] = True
        self.clsfy.targetsReal[msg.data] = False

    def orderFromSearcherCallback(self, msg):
        self.orderFromSearcher = msg.data

    def posCallback(self, msg):
        self.selfPos[0] = msg.pose.pose.position.x
        self.selfPos[1] = msg.pose.pose.position.y

    def imuCallback(self, msg):
        orientation = msg.orientation
        self.uavQuat = [orientation.x, orientation.y, orientation.z, orientation.w]

    def pitchCallback(self, msg):
        msg.data = 90 - msg.data
        self.podPitchBuffer.addMessage(msg)

    def yawCallback(self, msg):
        self.podYawBuffer.addMessage(msg)

    def hfovCallback(self, msg):
        self.podHfovBuffer.addMessage(msg)

    def hCallback(self, msg):
        self.selfPos[2] = msg.data

    def targetsCallback(self, msg):
        # tic = rospy.Time.now().to_sec()
        for target in msg.targets:
            if target.category == 'car':
                self.transform(target.cx, target.cy, score=target.score)
        # toc = rospy.Time.now().to_sec()
        # print(f'Callback time {toc - tic}')

    def transform(self, pixelX, pixelY, score=1):
        if not self.orderFromSearcher:
            return
        timeDiff = 0.4
        try:
            podHfov = self.podHfovBuffer.getMessage(timeDiff).data
            podVfov = degrees(2 * np.arctan(np.tan(radians(podHfov) / 2) * 9 / 16))
            podYaw = self.podYawBuffer.getMessage(timeDiff).data
            podPitch = self.podPitchBuffer.getMessage(timeDiff).data
        except Exception as e:
            print(e)
            return

        pixelX = (pixelX - 0.5) * 2
        pixelY = (pixelY - 0.5) * 2
        if self.TRANSFORM_DEBUG:
            ShowBar(-1, 1).show(pixelX, str='PixelX')
            ShowBar(-1, 1).show(pixelY, str='PixelY')
        cameraYaw = pixelX * podHfov / 2
        cameraPitch = pixelY * podVfov / 2
        if self.TRANSFORM_DEBUG:
            ShowBar(-podHfov / 2, podHfov / 2).show(cameraYaw, str='CameraYaw')
            ShowBar(-podVfov / 2, podVfov / 2).show(cameraPitch, str='CameraPitch')
        rCameraYaw = R.from_euler('z', -cameraYaw, degrees=True)
        rCameraPitch = R.from_euler('y', cameraPitch, degrees=True)
        rCamera = rCameraPitch * rCameraYaw
        rPodYaw = R.from_euler('z', -podYaw, degrees=True)
        rPodPitch = R.from_euler('y', podPitch, degrees=True)
        if self.TRANSFORM_DEBUG:
            ShowBar(-90, 90).show(podYaw, str='PodYaw')
            ShowBar(0, 90).show(podPitch, str='PodPitch')

        rUAV = R.from_quat(self.uavQuat)

        imgTarget = [1000, 0, 0]
        imgTargetRel = (rUAV * rPodYaw * rPodPitch * rCamera).apply(imgTarget)
        # print(f'imgTarget: {imgTargetRel}')

        realTargetRel = imgTargetRel * self.selfPos[2] / (-imgTargetRel[2])
        # print(f'realTargetRel: {realTargetRel}')

        realTargetAbs = realTargetRel + self.selfPos
        # print((
        #     f'XY: ({pixelX:.2f}, {pixelY:.2f}) '
        #     f'cYP: ({cameraYaw:.2f}, {cameraPitch:.2f}) '
        #     f'pYP: ({podYaw:.2f}, {podPitch:.2f}) '
        #     f'Target @ {realTargetAbs[0]:.2f}, {realTargetAbs[1]:.2f}, {realTargetAbs[2]:.2f} '
        #     f'Score {score:.2f}'
        # ))

        if self.TRANSFORM_DEBUG:
            print(f'-' * 20)

        if not self.outOfBound(*realTargetAbs):
            self.clsfy.newPos(*realTargetAbs)
            # self.clsfy.outputTargets()

    def untransform(self, pos):
        # from pos and self.pos and self.uavQuat
        # cal the pod pitch and yaw to aim at pos
        # return podPitch, podYaw
        rUAV = R.from_quat(self.uavQuat)
        rUAVInv = rUAV.inv()
        posRel = pos - self.selfPos
        targetBody = rUAVInv.apply(posRel)

        podPitch = np.arctan2(-targetBody[2], np.sqrt(targetBody[0] ** 2 + targetBody[1] ** 2))
        podYaw = -np.arctan2(targetBody[1], targetBody[0])

        podPitch = 90 - np.degrees(podPitch)
        podYaw = np.degrees(podYaw)

        return podPitch, podYaw

    def outOfBound(self, x, y, z):
        if x < 0 or x > self.a:
            return True
        if abs(y) > self.a / 2:
            return True
        return False

    def log(self):
        self.dtlg.log("rosTime", rospy.Time.now().to_sec() - self.startTime)
        self.dtlg.log("podYaw", self.podYawBuffer.getMessageNoDelay().data)
        self.dtlg.log("podPitch", self.podPitchBuffer.getMessageNoDelay().data)
        self.dtlg.log("podYawDelayed", self.podYawBuffer.getMessage(0.5).data)
        self.dtlg.log("podPitchDelayed", self.podPitchBuffer.getMessage(0.5).data)
        t = self.clsfy.targetsList()
        self.dtlg.log("targetCnt", len(t))
        for i in range(self.targetsAvailable):
            if i < len(t):
                self.dtlg.log(f'target{i}', t[i])
            else:
                self.dtlg.log(f'target{i}', [-1, -1, -1])
        self.dtlg.newline()

    def spin(self):
        while not rospy.is_shutdown():
            print('-' * 20)
            if self.logOn and not self.podYawBuffer.empty and not self.podPitchBuffer.empty:
                self.log()
            
            t = self.clsfy.targets
            tLen = len(t)
            tCnt = self.clsfy.targetsCnt
            tCheck = self.clsfy.targetsCheck
            tReal = self.clsfy.targetsReal
            for i in range(tLen):
                print(
                    f'Target[{i}] @ {", ".join([f"{x:.2f}" for x in t[i]])}, {tCnt[i]} Frames, ' 
                    f'{("" if tCheck[i] else (YELLOW + "Not Checked")) if tCnt[i] >= self.clsfy.checkThreshold else "Not enough"}' 
                    f'{((GREEN + "is a target") if tReal[i] else (RED + "not a target")) if tCheck[i] else ""}'
                    f'{RESET}'
                )


            tInd = self.clsfy.firstNotChecked()
            if tInd is not None:
                aimPitch, aimYaw = self.untransform(self.clsfy.targets[tInd])
                msg = Float64MultiArray(data=[1, aimPitch, aimYaw, tInd])
                print(f'{RED}==> Aiming @ Target [{tInd}] <== {RESET}p{aimPitch:.2f}, y{aimYaw:.2f}')
                self.aimPub.publish(msg)
            else:
                msg = Float64MultiArray(data=[-1, -1, -1, -1])
                self.aimPub.publish(msg)

            time.sleep(0.05)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--log', help='turn on log or not', action="store_true")
    args, unknown = parser.parse_known_args()

    rospy.init_node('Transformer', anonymous=True)
    # ipt = input('If debug? (y/n): ')
    t = Transformer(args.log)
    time.sleep(1)

    t.spin()
    # if ipt == 'y':
    #     t.TRANSFORM_DEBUG = True
