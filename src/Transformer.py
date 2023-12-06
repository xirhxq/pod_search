#! /usr/bin/env python3

import argparse
import time
import subprocess
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

class Transformer:
    def __init__(self, args):
        self.args = args
        print(YELLOW + 'ARGS: ', self.args, RESET)

        import rospkg
        pre = rospkg.RosPack().get_path('pod_search')
        self.ekfLog = DataLogger(pre, 'ekf.csv')
        self.ekfLog.initialize([("usvEKFx[6][1]", "matrix")])

        if self.args.log:
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

        self.cameraAzimuth = None
        self.cameraElevation = None

        self.uavRollDeg = 0
        self.uavPitchDeg = 0
        self.uavYawDeg = 180

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

        self.usvEKF = LocatingEKF(initialT=self.getTimeNow())
    
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

    @property
    def podRoll(self):
        return self.podRollBuffer.getMessage(self.podDelay)

    def pitchCallback(self, msg):
        self.podPitchBuffer.addMessage(msg)

    @property
    def podPitch(self):
        return self.podPitchBuffer.getMessage(self.podDelay)

    def yawCallback(self, msg):
        self.podYawBuffer.addMessage(msg)

    @property
    def podYaw(self):
        return self.podYawBuffer.getMessage(self.podDelay)

    def hfovCallback(self, msg):
        self.podHfovBuffer.addMessage(msg)

    @property
    def podHfov(self):
        return self.podHfovBuffer.getMessage(self.podDelay)

    @property
    def podVfov(self):
        return  PodParas.getVFovFromHFov(self.podHfov)

    def laserRangeCallback(self, msg):
        self.podLaserRangeBuffer.addMessage(msg)

    @property
    def podLaserRange(self):
        return self.podLaserRangeBuffer.getMessage(self.podDelay)

    @property
    def rB2P(self):
        try: 
            rPodRoll = R.from_euler('x', self.podRoll, degrees=True)
            rPodYaw = R.from_euler('z', self.podYaw, degrees=True)
            rPodPitch = R.from_euler('y', self.podPitch, degrees=True)
            rGB2P = rPodYaw * rPodRoll * rPodPitch
            return rGB2P.as_matrix()
        except:
            return None

    @property
    def rP2B(self):
        try:
            return self.rB2P.T
        except:
            return None

    @property
    def rI2B(self):
        try:
            rUavRoll = R.from_euler('x', self.uavRollDeg, degrees=True)
            rUavPitch = R.from_euler('y', self.uavPitchDeg, degrees=True)
            rUavYaw = R.from_euler('z', self.uavYawDeg, degrees=True)
            rI2B = rUavYaw * rUavRoll * rUavPitch
            return rI2B.as_matrix()
        except:
            return None

    @property
    def rB2I(self):
        try:
            return self.rI2B.T
        except:
            return None

    def vesselDetectionCallback(self, msg):
        for target in msg.targets:
            if target.category_id != 100:
                px = (target.cx - 0.5) * 2
                py = (target.cy - 0.5) * 2
                try:
                    self.cameraAzimuth = -np.degrees(np.arctan(np.tan(np.radians(self.podHfov) / 2) * px))
                    self.cameraElevation = np.degrees(np.arctan(np.tan(np.radians(self.podVfov) / 2) * py))
                    trackData = Float64MultiArray(data=[self.cameraElevation + self.podPitch, self.cameraAzimuth + self.podYaw, self.podHfov * target.w / 0.15, 20])
                    trackData.layout.dim = [MultiArrayDimension(label=target.category)]
                    self.trackPub.publish(trackData)
                except:
                    pass
                # self.transform(target.cx, target.cy, target.category, target.category_id, target.score)

    def usvDetectionCallback(self, msg):
        pass
        self.cameraAzimuth = None
        self.cameraElevation = None
        for target in msg.targets:
            px = (target.cx - 0.5) * 2
            py = (target.cy - 0.5) * 2
            try:
                self.cameraAzimuth = -np.degrees(np.arctan(np.tan(np.radians(self.podHfov) / 2) * px))
                self.cameraElevation = np.degrees(np.arctan(np.tan(np.radians(self.podVfov) / 2) * py))
                trackData = Float64MultiArray(data=[self.cameraElevation + self.podPitch, self.cameraAzimuth + self.podYaw, self.podHfov * target.w / 0.2, 20])
                trackData.layout.dim = [MultiArrayDimension(label=target.category)]
                self.trackPub.publish(trackData)
            except:
                pass

    def calTarget(self, pixelX, pixelY, category, categoryID):
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
        
        trackData = Float64MultiArray(data=[cameraElevation + podPitch, cameraAzimuth + podYaw, podHfov, 10])
        trackData.layout.dim = [MultiArrayDimension(label=category)]
        self.trackPub.publish(trackData)

        if podLaserRange < 10 or podLaserRange > 3000:
            # podLaserRange = 200.0
            return None
        
        if categoryID not in self.ekfDict:
            self.ekfDict[categoryID] = LocatingEKF(initialT=self.getTimeNow())
        
        Z = np.array([[podLaserRange], [cameraAzimuth], [cameraElevation], [self.selfPos[2]]])
        self.ekfDict[categoryID].newFrame(self.getTimeNow(), Z, self.selfPos.reshape(3, 1), rGB2P.as_matrix().T, rI2B.as_matrix().T)
        realTargetAbsI = self.ekfDict[categoryID].ekf.x[0:3]



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

    def ControlStateMachine(self):
        if self.searchState == 0 or self.searchState == 5:
            self.clsfy.clear()

    def spin(self):
        while not rospy.is_shutdown():
            self.ControlStateMachine()

            print('-' * 20)

            if self.cameraAzimuth is None or self.cameraElevation is None:
                newZ = np.array([[0.0], [0.0], [0.0], [self.selfPos[2]]])
            else:
                newZ = np.array([[self.podLaserRange], [self.cameraAzimuth], [self.cameraElevation], [self.selfPos[2]]])
            
            self.usvEKF.newFrame(self.getTimeNow(), newZ, self.selfPos.reshape(3, 1), self.rP2B, self.rB2I)

            print(f'{self.usvEKF.ekf.x = }')
            self.ekfLog.log('usvEKFx', self.usvEKF.ekf.x)
            self.ekfLog.newline()

            time.sleep(0.01)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--log', help='turn on log or not', action="store_true")
    parser.add_argument('--debug', help='debug or not', action='store_true')
    parser.add_argument('--infBound', help='infinite bound', action='store_true')
    parser.add_argument('--noB2GB', help='not adding rB2GB', action='store_true')
    parser.add_argument('--noImu', help='not using IMU', action='store_true')
    parser.add_argument('--bag', help='rosbag record', action='store_false')
    args, unknown = parser.parse_known_args()

    if args.bag:
        command = 'rosbag record -a -x "/suav/pod/main_camera_images.*" -o track'
        process = subprocess.Popen(command, shell=True)

    def signal_handler(sig, frame):
        print('You pressed Ctrl+C!')
        if args.bag:
            process.terminate()
        exit(0)

    signal(SIGINT, signal_handler)
    
    rospy.init_node('Transformer', anonymous=True)
    t = Transformer(args)
    time.sleep(3)

    t.spin()
