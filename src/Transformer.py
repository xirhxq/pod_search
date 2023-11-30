#! /usr/bin/env python3

import argparse
import time
from math import degrees, radians
from signal import signal, SIGINT
import numpy as np

import rospy
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Imu
from spirecv_msgs.msg import TargetsInFrame
from std_msgs.msg import Int8, Float32, Float64MultiArray, MultiArrayDimension

from Classifier import Classifier
from DataLogger import DataLogger
from TimeBuffer import TimeBuffer
from QuaternionBuffer import QuaternionBuffer
from Utils import *
import PodParas
from LocatingEKF import LocatingEKF


def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    exit(0)


signal(SIGINT, signal_handler)


class Transformer:
    def __init__(self, args):
        self.args = args
        print(YELLOW + 'ARGS: ', self.args, RESET)

        if self.args.cali:
            import rospkg
            pre = rospkg.RosPack().get_path('pod_search')
            self.caliLog = DataLogger(pre, 'cali.csv')

            variable_info = [
                ('rosTime', 'double'),
                ('podRoll', 'double'),
                ('podRollDelayed', 'double'),
                ("podPitch", "double"),
                ("podPitchDelayed", "double"),
                ("podYaw", "double"),
                ("podYawDelayed", "double"),
                ("podHfov", "double"),
                ("podHfovDelayed", "double"),
                ("podVfov", "double"),
                ("podVfovDelayed", "double"),
                ('pixelX', 'double'),
                ('pixelY', 'double'),
                ('cameraYaw', 'double'),
                ('cameraPitch', 'double'),
                ('uavQuat[4]', 'list'),
                ('uavQuatDelayed[4]', 'list'),
                ('uavEuler[3]', 'list'),
                ('selfPos[3]', 'list'),
                ('targetPosAbs[3]', 'list'),
                ('targetId', 'int')
            ]
            self.caliLog.initialize(variable_info)
            print(self.caliLog.variable_names) 

        if self.args.log:
            import rospkg
            pre = rospkg.RosPack().get_path('pod_search')
            self.dtlg = DataLogger(pre, "data.csv")

            self.targetsAvailable = 10
            variable_info = [
                ("rosTime", "double"),
                ('podRoll', 'double'),
                ('podRollDelayed', 'double'),
                ("podPitch", "double"),
                ("podPitchDelayed", "double"),
                ("podYaw", "double"),
                ("podYawDelayed", "double"),
                ("podHfov", "double"),
                ("podHfovDelayed", "double"),
                ("podVfov", "double"),
                ("podVfovDelayed", "double"),
                ('selfPos[3]', 'list'),
                ('uavQuatDelayed[4]', 'list'),
                ('uavQuat[4]', 'list')
            ] 
            for i in range(self.targetsAvailable):
                variable_info.append((f'target{i}[3]', "list"))
                variable_info.append((f'targetCnt{i}', 'int'))

            self.dtlg.initialize(variable_info)
            print(self.dtlg.variable_names)

        self.h = 10
        self.a = self.h / 100 * 3000
        self.selfPos = np.array([0, 0, self.h])
        
        self.dockENU = np.array([20, 225, 0])
        self.dockENU = np.array([-20, -10, 0])
        self.usvENU = self.dockENU

        self.podRollBuffer = TimeBuffer('Pod Roll Buffer')
        self.podPitchBuffer = TimeBuffer('Pod Pitch Buffer')
        self.podYawBuffer = TimeBuffer('Pod Yaw Buffer')
        self.podHfovBuffer = TimeBuffer('Pod HFov Buffer')
        self.podLaserRangeBuffer = TimeBuffer('Laser Range Buffer')
        self.uavQuatBuffer = QuaternionBuffer('IMU Buffer')
        self.podDelay = 0.4

        self.uavName = 'suav'
        self.podName = 'pod'
        self.osdkName = 'dji_osdk_ros'
        self.uwbName = 'uwb'

        rospy.Subscriber(self.uavName + '/' + self.osdkName + '/imu' + ('/noData' if self.args.noImu else ''), Imu, self.imuCallback)
        rospy.Subscriber(self.uavName + '/' + self.uwbName + '/filter/odom', Odometry, self.posCallback)

        rospy.Subscriber(self.uavName + '/' + self.podName + '/roll', Float32, self.rollCallback)
        rospy.Subscriber(self.uavName + '/' + self.podName + '/pitch', Float32, self.pitchCallback)
        rospy.Subscriber(self.uavName + '/' + self.podName + '/yaw', Float32, self.yawCallback)
        rospy.Subscriber(self.uavName + '/' + self.podName + '/hfov', Float32, self.hfovCallback)

        rospy.Subscriber(self.uavName + '/' + self.podName + '/laserRange', Float32, self.laserRangeCallback)

        rospy.Subscriber(self.uavName + '/' + self.podName + '/vessel_det', TargetsInFrame, self.vesselDetectionCallback, queue_size=1)
        rospy.Subscriber(self.uavName + '/' + self.podName + '/usv_detection', TargetsInFrame, self.usvDetectionCallback, queue_size=1)

        self.searchState = -1
        rospy.Subscriber(self.uavName + '/' + self.podName + '/searchState', Int8, lambda msg: setattr(self, 'searchState', msg.data))

        self.clsfy = Classifier()

        self.startTime = rospy.Time.now().to_sec()

        self.streamPub = rospy.Publisher(self.uavName + '/' + self.podName + '/stream', Float64MultiArray, queue_size=1) 

        self.trackPub = rospy.Publisher(self.uavName + '/' + self.podName + '/track', Float64MultiArray, queue_size=1)
        
        self.usvTargetPub = rospy.Publisher(self.uavName + '/' + self.podName + '/target_nav_position', Pose2D, queue_size=1)
        self.dockPub = rospy.Publisher(self.uavName + '/' + self.podName + '/dock', Float64MultiArray, queue_size=1)

        if self.args.noB2GB:
            self.yawB2GB = 0
            self.pitchB2GB = 0
            self.rollB2GB = 0
            print(f'{RED}No rB2GB{RESET}')
        else:
            self.yawB2GB = 30.68
            self.pitchB2GB = 0
            self.rollB2GB = 0.22
            print(f'{GREEN}With rB2GB on{RESET}')
        self.rB2GB = R.from_euler('zyx', [self.yawB2GB, self.pitchB2GB, self.rollB2GB], degrees=True)

        self.ekfDict = {}
    
    def getTimeNow(self):
        return rospy.Time.now().to_sec() - self.startTime

    def posCallback(self, msg):
        # self.selfPos[0] = msg.pose.pose.position.x + 0.6
        # self.selfPos[1] = msg.pose.pose.position.y - 0.3
        self.selfPos[2] = msg.pose.pose.position.z + 100 - 91.6

    def imuCallback(self, msg):
        orientation = msg.orientation
        self.uavQuatBuffer.addMessage(orientation)

    def rollCallback(self, msg):
        self.podRollBuffer.addMessage(msg)

    def pitchCallback(self, msg):
        self.podPitchBuffer.addMessage(msg)

    def yawCallback(self, msg):
        self.podYawBuffer.addMessage(msg)

    def hfovCallback(self, msg):
        self.podHfovBuffer.addMessage(msg)

    def laserRangeCallback(self, msg):
        self.podLaserRangeBuffer.addMessage(msg)

    def vesselDetectionCallback(self, msg):
        for target in msg.targets:
            if target.category_id != 100:
                self.transform(target.cx, target.cy, target.category, target.category_id, target.score)

    def usvDetectionCallback(self, msg):
        for target in msg.targets:
            self.transform(target.cx, target.cy, target.category, target.category_id, target.score)

    def calTarget(self, pixelX, pixelY, category, categoryID):
        timeDiff = self.podDelay
        try:
            podHfov = self.podHfovBuffer.getMessage(timeDiff).data
            podYaw = self.podYawBuffer.getMessage(timeDiff).data
            podPitch = self.podPitchBuffer.getMessage(timeDiff).data
            podRoll = self.podRollBuffer.getMessage(timeDiff).data
            podLaserRange = self.podLaserRangeBuffer.getMessage(timeDiff).data
        except Exception as e:
            print(e)
            return

        try: 
            uavQuat = self.uavQuatBuffer.getMessage()
        except Exception as e:
            # print(e)
            uavQuat = np.array([0, 0, 0, 1])
        if uavQuat is None:
            uavQuat = np.array([0, 0, 0, 1])

        pixelX = (pixelX - 0.5) * 2
        pixelY = (pixelY - 0.5) * 2
        cameraAzimuth = -np.degrees(np.arctan(np.tan(np.radians(podHfov) / 2) * pixelX))
        podVfov = PodParas.getVFovFromHFov(podHfov)
        cameraElevation = np.degrees(np.arctan(np.tan(np.radians(podVfov) / 2) * pixelY))

        rPodRoll = R.from_euler('x', podRoll, degrees=True)
        rPodYaw = R.from_euler('z', podYaw, degrees=True)
        rPodPitch = R.from_euler('y', podPitch, degrees=True)
        rGB2P = rPodYaw * rPodRoll * rPodPitch
        rI2B = R.from_quat(uavQuat)
        rI2P = rI2B * self.rB2GB * rGB2P

        if podLaserRange < 10 or podLaserRange > 3000:
            return None
        
        if categoryID not in self.ekfDict:
            self.ekfDict[categoryID] = LocatingEKF(initialT=self.getTimeNow())
        
        Z = np.array([[podLaserRange], [cameraAzimuth], [cameraElevation], [self.selfPos[2]]])
        self.ekfDict[categoryID].newFrame(self.getTimeNow(), Z, self.selfPos.reshape(3, 1), rGB2P.as_matrix().T, rI2B.as_matrix().T)
        realTargetAbsI = self.ekfDict[categoryID].ekf.x[0:3]

        trackData = Float64MultiArray(data=[cameraElevation + podPitch, cameraAzimuth + podYaw, podHfov, podPitch])
        trackData.layout.dim = [MultiArrayDimension(label=category)]
        self.trackPub.publish(trackData)


        if self.args.debug:
            yprStr = {'y': f'{BLUE}y{RESET}', 'p': f'{YELLOW}p{RESET}', 'r': f'{RED}r{RESET}'}

            np.set_printoptions(precision=2)
            print((
                f'h{self.selfPos[2]:.2f}'
                f'px({pixelX:.2f}, {pixelY:.2f}) '
                f'c({yprStr["y"]}{cameraAzimuth:.2f}, {yprStr["p"]}{cameraElevation:.2f}) '
                f'p({yprStr["y"]}{podYaw:.2f}, {yprStr["p"]}{podPitch:.2f}, {yprStr["r"]}{podRoll:.2f}) '
                f'q{rI2B.as_euler("zyx", degrees=True)}'
                f'TtoD ENU{(realTargetAbsI - self.dockENU)[:2]}'
            ))

            if self.args.cali:
                self.caliLog.log("rosTime", self.getTimeNow())
                self.caliLog.log("podYaw", self.podYawBuffer.getMessageNoDelay().data)
                self.caliLog.log("podYawDelayed", podYaw)
                self.caliLog.log("podPitch", self.podPitchBuffer.getMessageNoDelay().data)
                self.caliLog.log("podPitchDelayed", podPitch)
                self.caliLog.log("podRoll", self.podRollBuffer.getMessageNoDelay().data)
                self.caliLog.log("podRollDelayed", podRoll)
                hFov = self.podHfovBuffer.getMessageNoDelay().data 
                self.caliLog.log("podHfov", hFov)
                self.caliLog.log("podHfovDelayed", podHfov)
                vFov = PodParas.getVFovFromHFov(hFov)
                self.caliLog.log('podVfov', vFov)
                self.caliLog.log('podVfovDelayed', podVfov)
                self.caliLog.log('pixelX', pixelX)
                self.caliLog.log('pixelY', pixelY)
                self.caliLog.log('cameraYaw', cameraAzimuth)
                self.caliLog.log('cameraPitch', cameraElevation)
                self.caliLog.log('uavQuat', self.uavQuatBuffer.getMessageNoDelay())
                self.caliLog.log('uavQuatDelayed', list(uavQuat))
                self.caliLog.log('uavEuler', list(rI2B.as_euler('zyx', degrees=True)))
                self.caliLog.log('selfPos', list(self.selfPos))
                self.caliLog.log('targetPosAbs', list(realTargetAbsI))
                self.caliLog.log('targetId', categoryID)
                self.caliLog.newline()
        
        return realTargetAbsI

    def transform(self, pixelX, pixelY, category, categoryID, score):
        if self.searchState <= 0 and not self.args.debug:
            return
        realTargetAbs = self.calTarget(pixelX, pixelY, category, categoryID)
        if realTargetAbs is None:
            return
        if not self.outOfBound(*realTargetAbs):
            # self.clsfy.newPos(*realTargetAbs)
            if category == 'boat' or category == 'usv':
                if self.searchState == 6 or self.searchState == 4:
                    self.clsfy.updateTarget(categoryID, list(realTargetAbs), score)
            elif category == 'usv':
                if self.searchState == 6:
                    self.usvENU = realTargetAbs

            # self.clsfy.outputTargets()

    def untransform(self, pos):
        if self.uavQuatBuffer.empty:
            rB2I = R.from_quat([0, 0, 0, 1])
        else:
            rB2I = R.from_quat(self.uavQuatBuffer.getMessageNoDelay()).inv()
        rGB2B = self.rB2GB.inv()
        posRel = pos - self.selfPos
        targetBody = (rGB2B * rB2I).apply(posRel)

        podPitch = np.arctan2(-targetBody[2], np.sqrt(targetBody[0] ** 2 + targetBody[1] ** 2))
        podYaw = np.arctan2(targetBody[1], targetBody[0])

        podPitch = np.degrees(podPitch)
        podYaw = np.degrees(podYaw)

        return podPitch, podYaw

    def outOfBound(self, x, y, z):
        if self.args.infBound:
            return False
        if x < 0 or x > self.a:
            return True
        if abs(y) > self.a / 2:
            return True
        return False

    def log(self):
        self.dtlg.log("rosTime", self.getTimeNow())
        self.dtlg.log("podYaw", self.podYawBuffer.getMessageNoDelay().data)
        self.dtlg.log("podYawDelayed", self.podYawBuffer.getMessage(self.podDelay).data)
        self.dtlg.log("podPitch", self.podPitchBuffer.getMessageNoDelay().data)
        self.dtlg.log("podPitchDelayed", self.podPitchBuffer.getMessage(self.podDelay).data)
        self.dtlg.log("podRoll", self.podRollBuffer.getMessageNoDelay().data)
        self.dtlg.log("podRollDelayed", self.podRollBuffer.getMessage(self.podDelay).data)
        hFov = self.podHfovBuffer.getMessageNoDelay().data 
        hFovDelayed = self.podHfovBuffer.getMessage(self.podDelay).data 
        self.dtlg.log("podHfov", hFov)
        self.dtlg.log("podHfovDelayed", hFovDelayed)
        vFov = PodParas.getVFovFromHFov(hFov)
        vFovDelayed = PodParas.getVFovFromHFov(hFovDelayed)
        self.dtlg.log('podVfov', vFov)
        self.dtlg.log('podVfovDelayed', vFovDelayed)
        self.dtlg.log('selfPos', list(self.selfPos))
        self.dtlg.log('uavQuatDelayed', list(self.uavQuatBuffer.getMessage()))
        self.dtlg.log('uavQuat', list(self.uavQuatBuffer.getMessageNoDelay()))

        t = self.clsfy.targets
        tLen = len(t)
        tCnt = self.clsfy.targetsCnt
        for i in range(self.targetsAvailable):
            if i < tLen:
                self.dtlg.log(f'target{i}', t[i])
                self.dtlg.log(f'targetCnt{i}', tCnt[i])
            else:
                self.dtlg.log(f'target{i}', [-1, -1, -1])
                self.dtlg.log(f'targetCnt{i}', 0)

        self.dtlg.newline()

    def ControlStateMachine(self):
        if self.searchState == 0 or self.searchState == 5:
            self.clsfy.clear()

    def spin(self):
        while not rospy.is_shutdown():
            self.ControlStateMachine()

            print('-' * 20)
            # np.set_printoptions(precision=2)
            # print(f'My pos: {self.selfPos}')
            # print(f'RelImu: {R.from_quat(self.uavQuat).as_euler("zyx", degrees=True)}')

            for key, ekf in self.ekfDict.items():
                if ekf.t < self.getTimeNow() - 0.1:
                    ekf.newFrame(self.getTimeNow(), np.array([0]), 0, 0, 0)
                    self.clsfy.updateTarget(key, list(ekf.ekf.x[0:3]))

            if self.args.log and \
               not self.podYawBuffer.empty and \
               not self.podPitchBuffer.empty and \
               not self.podRollBuffer.empty and \
               not self.uavQuatBuffer.empty:
                self.log()

            t = self.clsfy.targets
            tLen = len(t)
            tScore = self.clsfy.targetsScore
            tCnt = self.clsfy.targetsCnt
            for i in range(tLen):
                print(
                    f'Target[{i}] @ [{t[i][0][0]:.2f}, {t[i][1][0]:.2f}, {t[i][2][0]:.2f}], '
                    f'{YELLOW if tCnt[i] < 15 else GREEN}'
                    f'{tCnt[i]} Frames{RESET}, '
                    f'Score: {tScore[i]:.2f}'
                    f'{RESET}'
                )

            streamInd = self.clsfy.lowestScoreIndex(threshold=5)
            if streamInd is not None and 0 <= streamInd < len(t):
                streamPitch, streamYaw = self.untransform(self.clsfy.targets[streamInd])
                msg = Float64MultiArray(data=[streamPitch, streamYaw, streamInd])
                self.streamPub.publish(msg)

                if self.searchState == 6:
                    realTargetRelUSVI = self.clsfy.targets[streamInd] - self.usvENU 
                    realTargetRelUSVTheta = np.degrees(np.arctan2(*realTargetRelUSVI[[1, 0]]))
                    self.usvTargetPub.publish(Pose2D(x=realTargetRelUSVI[0], y=realTargetRelUSVI[1], theta=realTargetRelUSVTheta)) 
            
            dockPitch, dockYaw = self.untransform(self.dockENU)
            self.dockPub.publish(Float64MultiArray(data=[1, dockPitch, dockYaw, 0]))

            time.sleep(0.05)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--log', help='turn on log or not', action="store_true")
    parser.add_argument('--debug', help='debug or not', action='store_true')
    parser.add_argument('--infBound', help='infinite bound', action='store_true')
    parser.add_argument('--cali', help='calibration', action='store_true')
    parser.add_argument('--noB2GB', help='not adding rB2GB', action='store_true')
    parser.add_argument('--noImu', help='not using IMU', action='store_true')
    args, unknown = parser.parse_known_args()
    
    rospy.init_node('Transformer', anonymous=True)
    t = Transformer(args)
    time.sleep(1)

    t.spin()
