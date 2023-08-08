#! /usr/bin/env python3

# Class Transformer: transform pixel error to absolute position
# using the onboard pod pitch, yaw, zoom and UAV pitch, yaw, roll
# assume the UAV is at (0, 0, h)

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
from std_msgs.msg import Float32, Bool, Float64MultiArray, Int16, Empty
from geometry_msgs.msg import Vector3Stamped

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

    def addMessage(self, msg, maxAge=0.8):
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
    def __init__(self, logOn=False, debug=False, infBound=False):
        self.debug = debug
        self.infBound = infBound

        self.orderFromSearcher = False
        self.uavQuat = [0, 0, 0, 1]

        self.h = 0.6
        self.a = self.h / 100 * 3000
        self.selfPos = np.array([0, 0, self.h])

        self.podPitchBuffer = TimeBuffer('Pod Pitch Buffer')
        self.podYawBuffer = TimeBuffer('Pod Yaw Buffer')
        self.podHfovBuffer = TimeBuffer('Pod HFov Buffer')
        self.podDelay = 0.6

        self.uavName = 'suav'
        self.podName = 'pod'
        self.osdkName = 'dji_osdk_ros'
        self.uwbName = 'uwb'
        self.heightSensorName = 'height_sensor'

        rospy.Subscriber(self.uavName + '/' + self.osdkName + '/imuRel/noData', Imu, self.imuCallback)
        rospy.Subscriber(self.uavName + '/' + self.uwbName + '/filter/odom', Odometry, self.posCallback)
        rospy.Subscriber(self.uavName + '/' + self.heightSensorName + '/data', Vector3Stamped, self.hCallback)

        rospy.Subscriber(self.uavName + '/' + self.podName + '/pitch', Float32, self.pitchCallback)
        rospy.Subscriber(self.uavName + '/' + self.podName + '/yaw', Float32, self.yawCallback)
        rospy.Subscriber(self.uavName + '/' + self.podName + '/hfov', Float32, self.hfovCallback)

        rospy.Subscriber(self.uavName + '/' + self.podName + '/car_detection', TargetsInFrame, self.targetsCallback, queue_size=1)

        rospy.Subscriber(self.uavName + '/' + self.podName + '/toTransformer', Bool, self.orderFromSearcherCallback)

        self.clsfy = Classifier()

        self.logOn = logOn
        if self.logOn:
            import rospkg
            pre = rospkg.RosPack().get_path('pod_search')
            self.dtlg = DataLogger(pre, "data.csv")

        self.startTime = rospy.Time.now().to_sec()

        self.targetsAvailable = 30
        variable_info = [
            ("rosTime", "double"),
            ("podYaw", "double"),
            ("podYawDelayed", "double"),
            ("podPitch", "double"),
            ("podPitchDelayed", "double"),
            ("podHfov", "double"),
            ("podHfovDelayed", "double"),
            ("podVfov", "double"),
            ("podVfovDelayed", "double"),
            ('selfPos[3]', 'list')
        ] 
        for i in range(self.targetsAvailable):
            variable_info.append((f'target{i}[3]', "list"))
            variable_info.append((f'targetCnt{i}', 'int'))
            variable_info.append((f'targetCheck{i}', 'bool'))
            variable_info.append((f'targetReal{i}', 'bool'))

        if self.logOn:
            self.dtlg.initialize(variable_info)
            print(self.dtlg.variable_names)

        self.aimPub = rospy.Publisher(self.uavName + '/' + self.podName + '/aim', Float64MultiArray, queue_size=1)
        rospy.Subscriber(self.uavName + '/' + self.podName + '/aimFail', Int16, self.aimFailCallback, queue_size=1)
        rospy.Subscriber(self.uavName + '/' + self.podName + '/classifierClear', Empty, self.clear, queue_size=1)
        self.streamPub = rospy.Publisher(self.uavName + '/' + self.podName + '/stream', Float64MultiArray, queue_size=1) 

    def clear(self, msg):
        self.clsfy.clear()


    def aimFailCallback(self, msg):
        self.clsfy.targetsCheck[msg.data] = True
        self.clsfy.targetsReal[msg.data] = False

    def orderFromSearcherCallback(self, msg):
        self.orderFromSearcher = msg.data

    def posCallback(self, msg):
        self.selfPos[0] = msg.pose.pose.position.x + 0.3
        self.selfPos[1] = msg.pose.pose.position.y - 0.2

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
        self.selfPos[2] = msg.vector.y

    def targetsCallback(self, msg):
        # tic = rospy.Time.now().to_sec()
        for target in msg.targets:
            if target.category == 'car' and target.category_id != 100:
                self.transform(target.cx, target.cy, target.category_id, target.score)
        # toc = rospy.Time.now().to_sec()
        # print(f'Callback time {toc - tic}')

    def calTarget(self, pixelX, pixelY, category):
        timeDiff = self.podDelay
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
        cameraYaw = pixelX * podHfov / 2
        cameraPitch = pixelY * podVfov / 2
        rCameraYaw = R.from_euler('z', -cameraYaw, degrees=True)
        rCameraPitch = R.from_euler('y', cameraPitch, degrees=True)
        rCamera = rCameraPitch * rCameraYaw
        rPodYaw = R.from_euler('z', -podYaw, degrees=True)
        rPodPitch = R.from_euler('y', podPitch, degrees=True)

        rUAV = R.from_quat(self.uavQuat)

        imgTarget = [1000, 0, 0]
        imgTargetRel = (rUAV * rPodYaw * rPodPitch * rCamera).apply(imgTarget)
        # print(f'imgTarget: {imgTargetRel}')

        realTargetRel = imgTargetRel * self.selfPos[2] / (-imgTargetRel[2])
        # print(f'realTargetRel: {realTargetRel}')

        realTargetAbs = realTargetRel + self.selfPos
        
        if self.debug:
            print((
                f'XY: ({pixelX:.2f}, {pixelY:.2f}) '
                f'cYP: ({cameraYaw:.2f}, {cameraPitch:.2f}) '
                f'pYP: ({podYaw:.2f}, {podPitch:.2f}) '
                f'Target @ {realTargetAbs[0]:.2f}, {realTargetAbs[1]:.2f}, {realTargetAbs[2]:.2f} '
            ))
        
        return realTargetAbs

    def transform(self, pixelX, pixelY, category, score):
        if not self.orderFromSearcher and not self.debug:
            return
        realTargetAbs = self.calTarget(pixelX, pixelY, category)
        
        if realTargetAbs is None:
            return

        if not self.outOfBound(*realTargetAbs):
            # self.clsfy.newPos(*realTargetAbs)
            self.clsfy.updateTarget(category, list(realTargetAbs), score)
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
        if self.infBound:
            return False
        if x < 0 or x > self.a:
            return True
        if abs(y) > self.a / 2:
            return True
        return False

    def log(self):
        self.dtlg.log("rosTime", rospy.Time.now().to_sec() - self.startTime)
        self.dtlg.log("podYaw", self.podYawBuffer.getMessageNoDelay().data)
        self.dtlg.log("podYawDelayed", self.podYawBuffer.getMessage(self.podDelay).data)
        self.dtlg.log("podPitch", self.podPitchBuffer.getMessageNoDelay().data)
        self.dtlg.log("podPitchDelayed", self.podPitchBuffer.getMessage(self.podDelay).data)
        hFov = self.podHfovBuffer.getMessageNoDelay().data 
        hFovDelayed = self.podHfovBuffer.getMessage(self.podDelay).data 
        self.dtlg.log("podHfov", hFov)
        self.dtlg.log("podHfovDelayed", hFovDelayed)
        vFov = degrees(2 * np.arctan(np.tan(radians(hFov) / 2) * 9 / 16))
        vFovDelayed = degrees(2 * np.arctan(np.tan(radians(hFovDelayed) / 2) * 9 / 16))
        self.dtlg.log('podVfov', vFov)
        self.dtlg.log('podVfovDelayed', vFovDelayed)
        self.dtlg.log('selfPos', list(self.selfPos))

        t = self.clsfy.targets
        tLen = len(t)
        tCnt = self.clsfy.targetsCnt
        tCheck = self.clsfy.targetsCheck
        tReal = self.clsfy.targetsReal
        for i in range(self.targetsAvailable):
            if i < len(t):
                self.dtlg.log(f'target{i}', t[i])
                self.dtlg.log(f'targetCnt{i}', tCnt[i])
                self.dtlg.log(f'targetCheck{i}', tCheck[i])
                self.dtlg.log(f'targetReal{i}', tReal[i])
            else:
                self.dtlg.log(f'target{i}', [-1, -1, -1])
                self.dtlg.log(f'targetCnt{i}', 0)
                self.dtlg.log(f'targetCheck{i}', False)
                self.dtlg.log(f'targetReal{i}', False)

        self.dtlg.newline()

    def spin(self):
        while not rospy.is_shutdown():
            print('-' * 20)
            np.set_printoptions(precision=2)
            print(f'My pos: {self.selfPos}')
            print(f'RelImu: {R.from_quat(self.uavQuat).as_euler("zyx", degrees=True)}')


            if self.logOn and not self.podYawBuffer.empty and not self.podPitchBuffer.empty:
                self.log()

            t = self.clsfy.targets
            tLen = len(t)
            tScore = self.clsfy.targetsScore
            tCnt = self.clsfy.targetsCnt
            tCheck = self.clsfy.targetsCheck
            tReal = self.clsfy.targetsReal
            for i in range(tLen):
                print(
                    f'Target[{i}] @ {", ".join([f"{x:.2f}" for x in t[i]])}, {tCnt[i]} Frames, '
                    f'{("" if tCheck[i] else (YELLOW + "Not Checked")) if tCnt[i] >= self.clsfy.checkThreshold else "Not enough"}'
                    f'{((GREEN + "is a target") if tReal[i] else (RED + "not a target")) if tCheck[i] else ""}'
                    f', Score: {tScore[i]:.2f}'
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
           
            streamInd = self.clsfy.highestScoreIndex()
            if streamInd is not None and 0 <= streamInd < len(t):
                streamPitch, streamYaw = self.untransform(self.clsfy.targets[streamInd])
                msg = Float64MultiArray(data=[1, streamPitch, streamYaw, streamInd])
                self.streamPub.publish(msg)
            time.sleep(0.05)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--log', help='turn on log or not', action="store_true")
    parser.add_argument('--debug', help='debug or not', action='store_true')
    parser.add_argument('--infBound', help='infinite bound', action='store_true')
    args, unknown = parser.parse_known_args()

    rospy.init_node('Transformer', anonymous=True)
    t = Transformer(logOn=args.log, debug=args.debug, infBound=args.infBound)
    time.sleep(1)

    t.spin()
