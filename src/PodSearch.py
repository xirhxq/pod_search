#! /usr/bin/env python3

import time
import yaml
import argparse
import subprocess
import numpy as np
from os import system
# from rich import print
from signal import signal, SIGINT

import rospy
import rospkg
from std_msgs.msg import Float32, Bool, Float64MultiArray, Int8, Int16, MultiArrayDimension, Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry

from spirecv_msgs.msg import TargetsInFrame
from scipy.spatial.transform import Rotation as R

import PodParas
from AutoTra import AutoTra
from Utils import *
from PodAngles import PodAngles
from SearchPoint import SearchPoint
from Classifier import Classifier
from DataLogger import DataLogger
from TimeBuffer import TimeBuffer
from LocatingEKF import LocatingEKF


class State:
    INIT = 0
    PREPARE = 1
    SEARCH = 2
    STREAM = 4
    END = 5
    TRACK = 6
    DOCK = 7
    REFIND = 8
    
def stepEntrance(method):
    def wrapper(self, *args, **kwargs):
        self.tic = self.getTimeNow()
        return method(self, *args, **kwargs)
    return wrapper

def delayStart(method):
    def wrapper(self, *args, **kwargs):
        startThreshold = 3
        if self.taskTime >= startThreshold:
            return method(self, *args, **kwargs)
    return wrapper


class PodSearch:
    def __init__(self, args):
        # arg parsing
        self.args = args
        print(BLUE + 'ARGS:', self.args, RESET)

        self.packagePath = rospkg.RosPack().get_path('pod_search')
        with open(self.packagePath + '/config/config.yaml', 'r') as config:
            self.config = yaml.safe_load(config)

        # ros node initialising
        rospy.init_node('pod_search', anonymous=True)
        self.rate = rospy.Rate(10)

        # namespaces
        self.uavName = 'suav'
        self.deviceName = 'pod'

        # From PodComm: pod pitch, yaw, hfov, laser on, laser range
        self.podPitchBuffer = TimeBuffer('Pod Pitch Buffer')
        rospy.Subscriber(self.uavName + '/' + self.deviceName + '/pitch', Float32, lambda msg: self.podPitchBuffer.addMessage(msg))
        self.podYawBuffer = TimeBuffer('Pod Yaw Buffer')
        rospy.Subscriber(self.uavName + '/' + self.deviceName + '/yaw', Float32, lambda msg: self.podYawBuffer.addMessage(msg))
        self.podRollBuffer = TimeBuffer('Pod Roll Buffer')
        rospy.Subscriber(self.uavName + '/' + self.deviceName + '/roll', Float32, lambda msg: self.podRollBuffer.addMessage(msg))
        self.podHfovBuffer = TimeBuffer('Pod HFov Buffer')
        rospy.Subscriber(self.uavName + '/' + self.deviceName + '/hfov', Float32, lambda msg: self.podHfovBuffer.addMessage(msg))
        self.podLaserOn = False
        rospy.Subscriber(self.uavName + '/' + self.deviceName + '/laserOn', Bool, lambda msg: setattr(self, 'podLaserOn', msg.data))
        self.podLaserRangeBuffer = TimeBuffer('Pod Laser Range Buffer')
        rospy.Subscriber(self.uavName + '/' + self.deviceName + '/laserRange', Float32, lambda msg: self.podLaserRangeBuffer.addMessage(msg))

        # From PodComm: pod pitch, yaw, hfov at target or not
        self.podPitchAtTarget = False
        rospy.Subscriber(self.uavName + '/' + self.deviceName + '/pAtTarget', Bool, lambda msg: setattr(self, 'podPitchAtTarget', msg.data))
        self.podYawAtTarget = False
        rospy.Subscriber(self.uavName + '/' + self.deviceName + '/yAtTarget', Bool, lambda msg: setattr(self, 'podYawAtTarget', msg.data))
        self.podHfovAtTarget = False
        rospy.Subscriber(self.uavName + '/' + self.deviceName + '/fAtTarget', Bool, lambda msg: setattr(self, 'podHfovAtTarget', msg.data))

        # From PodComm: pod pitch, yaw, hfov feedback
        self.podCommFeedback = PodAngles(pitchDeg=0.0, yawDeg=0.0, hfovDeg=0.0)
        rospy.Subscriber(self.uavName + '/' + self.deviceName + '/pFeedback', Float32, lambda msg: setattr(self.podCommFeedback, 'pitchDeg', msg.data))
        rospy.Subscriber(self.uavName + '/' + self.deviceName + '/yFeedback', Float32, lambda msg: setattr(self.podCommFeedback, 'yawDeg', msg.data))
        rospy.Subscriber(self.uavName + '/' + self.deviceName + '/fFeedback', Float32, lambda msg: setattr(self.podCommFeedback, 'hfovDeg', msg.data))

        # To PodComm: expected pod pitch, yaw, hfov, max spin rate, laser on
        self.expectedPodAngles = PodAngles()
        self.pitchPub = rospy.Publisher(self.uavName + '/' + self.deviceName + '/expectedPitch', Float32, queue_size=10)
        self.yawPub = rospy.Publisher(self.uavName + '/' + self.deviceName + '/expectedYaw', Float32, queue_size=10)
        self.hfovPub = rospy.Publisher(self.uavName + '/' + self.deviceName + '/expectedHfov', Float32, queue_size=10)
        self.expectedMaxRatePub = rospy.Publisher(self.uavName + '/' + self.deviceName + '/maxRate', Float32, queue_size=10)
        self.expectedLaserOnPub = rospy.Publisher(self.uavName + '/' + self.deviceName + '/expectedLaserOn', Bool, queue_size=10)

        # From spirecv-ros: usv detection
        self.usvCameraAzimuth = None
        self.usvCameraElevation = None
        self.vesselCameraAzimuth = None
        self.vesselCameraElevation = None
        rospy.Subscriber(self.uavName + '/' + self.deviceName + '/vessel_det', TargetsInFrame, self.vesselDetectionCallback, queue_size=1)
        rospy.Subscriber(self.uavName + '/' + self.deviceName + '/usv_detection', TargetsInFrame, self.usvDetectionCallback, queue_size=1)

        # From localisation: location
        self.uavPos = np.array(self.config['uavInitialPos'])

        # From suav: suav control state
        self.uavState = 0
        rospy.Subscriber(self.uavName + '/uavState', Int16, lambda msg: setattr(self, 'uavState', msg.data))

        # To others: my state
        self.state = State.INIT
        self.searchStatePub = rospy.Publisher(self.uavName + '/' + self.deviceName + '/searchState', Int16, queue_size=1)

        # timers
        self.startTime = self.getTimeNow()
        self.taskTime = 0
        self.tic = self.getTimeNow()
        self.toc = self.getTimeNow()

        # [StepPrepare] counters
        self.searchRoundCnt = 0
        self.searchViewCnt = -1

        # [StepPrepare] search points pub
        self.searchPoints = [SearchPoint(**item) for item in self.config['SearchPoints']]
        print(f'Search Points: {self.searchPoints}')
        self.searchPointPub = rospy.Publisher(self.uavName + '/searchPoint', Float64MultiArray, queue_size=1)
        
        # [StepSearch] trajectory setting

        self.searchAreaPoints = [
            (0, self.config['SearchArea']['leftLength']),
            (self.config['SearchArea']['frontLength'], self.config['SearchArea']['leftLength']),
            (self.config['SearchArea']['frontLength'], -self.config['SearchArea']['rightLength']),
            (0, -self.config['SearchArea']['rightLength']),
        ]
        self.autoTras = []
        for round in self.config['searchConfig']:
            tmpList = []
            for view in self.config['SearchPoints']:
                tmpList.append(
                    AutoTra(
                        pitchLevelOn=True,
                        overlapOn=True,
                        drawNum=-1,
                        fast=self.args.fast,
                        config={
                            'height': view['uavPosU'],
                            'areaPoints': self.searchAreaPoints,
                            'ray': (view['uavPosF'], view['uavPosL'], view['uavYaw']),
                            'xFLU': 0,
                            'hfovPitchRatio': round['hfovPitchRatio'],
                            'theTime': round['theTime']
                        }
                    )
                )
            self.autoTras.append(tmpList)
        print(self.autoTras)
        self.autoTra = self.autoTras[0][0]
        self.tra = self.autoTra.theList

        # [StepSearch] trajectory counter
        self.traCnt = 0

        # [StepSearch] target ids & its scores
        self.vesselDict = {}

        # [StepSearch] ignore uav or not
        self.uavReady = True if self.args.mode == 'test' else False

        # [StepSearch] selected target id
        self.targetId = None

        # [StepSearch] last capture time of targets
        self.lastVesselCaptureTime = {}

        # [StepDock] pod angles that UAV look back to find USV
        self.dockData = PodAngles(**self.config['dockData'])
        print(f'{self.dockData = }')

        # [StepTrack] pod angles used to track targets
        self.trackData = {}
        self.trackName = None

        # [StepTrack] land flag
        self.landFlag = -1
        rospy.Subscriber('/usv/suav_land_flag', Int8, lambda msg: setattr(self, 'landFlag', msg.data))

        # [StepTrack] guidance data
        self.usvTargetPub = rospy.Publisher(self.uavName + '/' + self.deviceName + '/target_nav_position', PoseStamped, queue_size=1)

        # [StepTrack] last capture time of usv
        self.lastUSVCaptureTime = self.getTimeNow()

        # [StepTrack] ekfs for locating
        self.ekfNames = ['usv', 'boat']
        self.ekfs = {n: LocatingEKF(initialT=self.getTimeNow()) for n in self.ekfNames}
        self.ekfLogs = {n: DataLogger(self.packagePath, n + 'ekf.csv') for n in self.ekfNames}
        self.ekfLogs['usv'].initialize([('usvEKFx[6][1]', 'matrix')])

        # [StepTrack] target vessel position
        self.targetPos = np.array(self.config['targetInitialPos'])

        # [StepTrack] For test: target vessel position subscribing
        rospy.Subscriber(self.uavName + '/' + self.deviceName + '/targetPos', Float64MultiArray, self.targetPosCallback)

        # [StepRefind] target name & related pod angles
        self.refindName = None
        self.refindPodAngles = None

        signal(SIGINT, self.signalHandler)

        # set initial state
        if args.trackUSV:
            self.toStepTrack(trackName='usv')
            print('<<<TRACK MODE: USV>>>')
        if args.trackVessel:
            self.toStepTrack(trackName='boat')
            print('<<<TRACK MODE: Vessel>>>')
        if args.dock:
            self.toStepDock()
            print('<<<DOCK MODE>>>')


        print('Initialising finished...')
        print(f'{len(self.autoTras) = }')
        print(BOLD, BLUE, self.autoTras, RESET)
        print(f'{len(self.searchPoints) = }')
        print(BOLD, BLUE, self.searchPoints, RESET)
        input('Type anything to continue...')

    def targetPosCallback(self, msg):
        self.targetPos = np.array([[msg.data[0]], [msg.data[1]], [0]])

    @property
    def podPitchDeg(self):
        return self.podPitchBuffer.getMessageNoDelay()

    @property
    def podYawDeg(self):
        return self.podYawBuffer.getMessageNoDelay()

    @property
    def podRollDeg(self):
        return self.podRollBuffer.getMessageNoDelay()

    @property
    def podHfovDeg(self):
        return self.podHfovBuffer.getMessageNoDelay()

    @property
    def podVfovDeg(self):
        return PodParas.getVFovFromHFov(self.podHfovDeg)
    
    @property
    def podLaserRange(self):
        return self.podLaserRangeBuffer.getMessageNoDelay()

    @property
    def podPitchDegDelayed(self):
        return self.podPitchBuffer.getMessage()

    @property
    def podYawDegDelayed(self):
        return self.podYawBuffer.getMessage()
    
    @property
    def podRollDegDelayed(self):
        return self.podRollBuffer.getMessage()
    
    @property
    def podHfovDegDelayed(self):
        return self.podHfovBuffer.getMessage()
    
    @property
    def podVfovDegDelayed(self):
        return PodParas.getVFovFromHFov(self.podHfovDegDelayed)
    
    @property
    def podLaserRangeDelayed(self):
        return self.podLaserRangeBuffer.getMessage()
    
    @property
    def rB2PDelayed(self):
        return R.from_euler('zyx', [self.podYawDegDelayed, self.podPitchDegDelayed, self.podRollDegDelayed], degrees=True).as_matrix()
    
    @property
    def rP2BDelayed(self):
        return self.rB2PDelayed.T

    def vesselDetectionCallback(self, msg):
        self.vesselCameraAzimuth = None
        self.vesselCameraElevation = None
        for target in msg.targets:
            px = (target.cx - 0.5) * 2
            py = (target.cy - 0.5) * 2
            try:
                self.vesselCameraAzimuth = -np.degrees(np.arctan(np.tan(np.radians(self.podHfovDegDelayed) / 2) * px))
                self.vesselCameraElevation = np.degrees(np.arctan(np.tan(np.radians(self.podVfovDegDelayed) / 2) * py))
                id = str(target.category_id)
                if id != '100':
                    self.trackData[id] = PodAngles(
                        pitchDeg=self.vesselCameraElevation + self.podPitchDegDelayed, 
                        yawDeg=self.vesselCameraAzimuth + self.podYawDegDelayed, 
                        hfovDeg=self.podHfovDegDelayed, #PodParas.clipHfov(self.podHfovDegDelayed * target.w / 0.3), 
                        maxRateDeg=3
                    )
                    score = target.score
                    print(f'{BOLD}{BLUE}{id = } {score = }{RESET}')
                    if self.state == State.SEARCH and target.category_id != 100:
                        if id in self.vesselDict.keys():
                            self.vesselDict[id] = min(self.vesselDict[id], score)
                        else:
                            self.vesselDict[id] = score
                        self.lastVesselCaptureTime[id] = self.getTimeNow()
                    
            except Exception as e:
                pass
                # print(e)

    def usvDetectionCallback(self, msg):
        self.usvCameraAzimuth = None
        self.usvCameraElevation = None
        for target in msg.targets:
            px = (target.cx - 0.5) * 2
            py = (target.cy - 0.5) * 2
            try:
                self.usvCameraAzimuth = -np.degrees(np.arctan(np.tan(np.radians(self.podHfovDegDelayed) / 2) * px))
                self.usvCameraElevation = np.degrees(np.arctan(np.tan(np.radians(self.podVfovDegDelayed) / 2) * py))
                self.trackData['usv'] = PodAngles(
                    pitchDeg=self.usvCameraElevation + self.podPitchDegDelayed, 
                    yawDeg=self.usvCameraAzimuth + self.podYawDegDelayed, 
                    hfovDeg=PodParas.clipHfov(self.podHfovDegDelayed * target.w / 0.2), 
                    maxRateDeg=3
                )
                self.lastUSVCaptureTime = self.getTimeNow()
            except Exception as e:
                pass
                # print(e)

    def getTimeNow(self):
        return rospy.Time.now().to_sec()

    def getPodAnglesNow(self):
        return PodAngles(
            pitchDeg=self.podPitchDeg,
            yawDeg=self.podYawDeg,
            hfovDeg=self.podHfovDeg,
            maxRateDeg=self.expectedPodAngles.maxRateDeg
        )

    def getHfovFromPitch(self, pitch):
        return min(PodParas.maxHfov, max(PodParas.minHfov, pitch * self.autoTra.hfovPitchRatio))

    def isAtTarget(self):
        return (
                self.podPitchAtTarget and
                self.podYawAtTarget and
                self.podHfovAtTarget and
                self.podCommFeedback == self.expectedPodAngles
        )

    @stepEntrance
    def toStepInit(self):
        self.state = State.INIT

    def stepInit(self):
        self.expectedPodAngles = PodAngles(
            **self.config['podInitialData']
        )
        self.pubPYZMaxRate()
        if self.isAtTarget() and self.uavReady:
            self.toStepPrepare()

    @stepEntrance
    def toStepPrepare(self):
        self.state = State.PREPARE
        self.searchViewCnt += 1
        if self.searchViewCnt == len(self.searchPoints):
            self.searchViewCnt = 0
            self.searchRoundCnt += 1
        if self.searchRoundCnt == len(self.autoTras):
            self.toStepEnd()
            return
        self.autoTra = self.autoTras[self.searchRoundCnt][self.searchViewCnt]
        self.tra = self.autoTra.theList

    def stepPrepare(self):
        print(
            f'{BOLD}{BLUE}==> '
            f'StepPrepare, '
            f'Round #{self.searchRoundCnt + 1}, View #{self.searchViewCnt + 1}'
            f' <=={RESET}'
        )
        self.expectedPodAngles = self.tra[0]
        self.pubPYZMaxRate()
        self.searchPointPub.publish(data=self.searchPoints[self.searchViewCnt].toList())
        if self.uavState % 10 == 1:
            self.toStepSearch()

    @stepEntrance
    def toStepSearch(self):
        self.state = State.SEARCH
        self.traCnt = 0

    def stepSearch(self):
        print(
            f'{BOLD}{BLUE}==> '
            f'StepSearch @ #{self.traCnt + 1}/{len(self.tra)}, '
            f'Round #{self.searchRoundCnt + 1}, View #{self.searchViewCnt + 1}'
            f' <=={RESET}'
        )
        self.expectedPodAngles = self.tra[self.traCnt]
        self.pubPYZMaxRate()
        if self.isAtTarget():
            self.traCnt += 1
        if self.traCnt == len(self.tra):
            if len(self.vesselDict) > 0:
                self.targetId, _ = min(self.vesselDict.items(), key=lambda x: x[1])
            self.toStepPrepare()
        if self.targetId is not None and self.getTimeNow() - self.lastVesselCaptureTime[self.targetId] < 0.1:
            self.toStepTrack(self.targetId)

    @stepEntrance
    def toStepTrack(self, trackName):
        self.state = State.TRACK
        self.trackName = trackName
        self.trackData[trackName] = []
        self.ekfs[trackName] = LocatingEKF(initialT=self.getTimeNow())
        if not self.podLaserOn:
            self.expectedLaserOn = True
            self.expectedLaserOnPub.publish(True)
        print(f'{self.expectedLaserOn = }')
    
    def stepTrack(self):
        print(
            f'{BOLD}{BLUE}==> '
            f'Step Track for {self.trackName}: {self.toc - self.tic:.2f} seconds'
            f' <=={RESET}'
        )
        if self.landFlag == 1:
            self.toStepEnd()
        if self.getTimeNow() - self.lastUSVCaptureTime >= 5.0:
            self.toStepRefind(self.trackName)
        print(f'{self.expectedLaserOn = }')
        print(f'{self.trackName = }, {self.trackData[self.trackName] = }')
        if not self.podLaserOn:
            self.expectedLaserOn = True
            self.expectedLaserOnPub.publish(True)
        if self.trackName in self.trackData:
            self.expectedPodAngles = self.trackData[self.trackName]
            self.pubPYZMaxRate()
        ekfZ = np.array([[self.podLaserRange], [self.usvCameraAzimuth], [self.usvCameraElevation], [self.uavPos[2][0]]])
        if not (0 < ekfZ[0][0] < 4000 and ekfZ[1][0] is not None and ekfZ[2][0] is not None):
            ekfZ = np.array([[0], [0], [0], [0]])
        self.ekfs[self.trackName].newFrame(
            self.getTimeNow(),
            ekfZ,
            self.uavPos,
            self.rP2BDelayed,
            R.from_euler('zyx', [180, 0, 0], degrees=True).as_matrix().T
        )
        print(f'{self.ekfs[self.trackName].ekf.x = }')
        if self.trackName == 'usv':
            if self.ekfs[self.trackName].ekf.x is not None:
                usvPos = self.ekfs[self.trackName].ekf.x[:3].reshape((3, 1))
                self.ekfLogs[self.trackName].log('usvEKFx', self.ekfs[self.trackName].ekf.x)
                self.ekfLogs[self.trackName].newline()
                print(f'{self.targetPos = }')
                usvToTargetENU = self.targetPos - usvPos
                usvToTargetTheta = np.degrees(np.arctan2(usvToTargetENU[1][0], usvToTargetENU[0][0]))
                self.usvTargetPub.publish(
                    header=Header(frame_id='target'),
                    pose=Pose(
                        position=Point(
                            x=usvToTargetENU[0][0], 
                            y=usvToTargetENU[1][0]
                        ), 
                        orientation=Quaternion(
                            w=usvToTargetTheta
                        )
                    )
                )
        else:
            if self.toc - self.tic >= 10:
                self.targetPos = self.ekfs[self.trackName].ekf.x[:3].reshape(3, 1)
                self.toStepDock()
        
    @stepEntrance
    def toStepRefind(self, refindName):
        self.state = State.REFIND
        self.refindPodAngles = self.getPodAnglesNow()
        self.refindName = refindName

    def stepRefind(self):
        print(
            f'{BOLD}{BLUE}==> '
            f'Refinding {self.refindName} '
            f'@ (p{self.refindPodAngles.pitchDeg:.2f}, y{self.refindPodAngles.yawDeg:.2f}, hfov{self.refindPodAngles.hfovDeg:.2f})'
            f' <=={RESET}'
        )
        self.expectedPodAngles = PodAngles(
            pitchDeg=self.refindPodAngles.pitchDeg + (self.toc - self.tic) / 3 * np.sin((self.toc - self.tic) / 15 * 2 * np.pi),
            yawDeg=self.refindPodAngles.yawDeg + (self.toc - self.tic) / 3 * np.cos((self.toc - self.tic) / 15 * 2 * np.pi),
            hfovDeg=PodParas.clipHfov(self.refindPodAngles.hfovDeg * 1.5),
            maxRateDeg=10
        )
        self.pubPYZMaxRate()
        if self.getTimeNow() - self.lastUSVCaptureTime < 0.1:
            self.toStepTrack(self.refindName)

    @stepEntrance
    def toStepDock(self):
        self.state = State.DOCK

    def stepDock(self):
        print(
            f'***'
            f'Dock {self.toc - self.tic:.2f}, '
            f'{(GREEN + "At Target" + RESET) if self.isAtTarget() else (RED + "Not At Target" + RESET)}'
            f'***'
        )
        self.expectedPodAngles = self.dockData
        self.pubPYZMaxRate()
        if self.toc - self.tic >= 10:
            self.toStepTrack('usv')

    @stepEntrance
    def toStepEnd(self):
        self.state = State.END

    def stepEnd(self):
        print(f'StepEnd with {self.toc - self.tic:.2f} seconds')
        if self.toc - self.tic >= 3.0:
            exit(0)

    def pubPYZMaxRate(self):
        self.pitchPub.publish(self.expectedPodAngles.pitchDeg)
        self.yawPub.publish(self.expectedPodAngles.yawDeg)
        self.hfovPub.publish(self.expectedPodAngles.hfovDeg)
        self.expectedMaxRatePub.publish(self.expectedPodAngles.maxRateDeg)
        self.expectedLaserOnPub.publish(self.expectedPodAngles.laserOn)

    def controlStateMachine(self):
        self.toc = self.getTimeNow()
        self.searchStatePub.publish(Int16(self.state))
        if self.uavState >= 4:
            self.uavReady = True
        if self.state == State.INIT:
            self.stepInit()
        elif self.state == State.PREPARE:
            self.stepPrepare()
        elif self.state == State.SEARCH:
            self.stepSearch()
        elif self.state == State.END:
            self.stepEnd()
        elif self.state == State.STREAM:
            self.stepStream()
        elif self.state == State.TRACK:
            self.stepTrack()
        elif self.state == State.DOCK:
            self.stepDock()
        elif self.state == State.REFIND:
            self.stepRefind()
        else:
            print("Invalid state")

    @delayStart
    def spinOnce(self):
        system('clear')
        print('-' * 20)
        print(f'### PodSearch ###')
        print(f'Me @ State #{self.state} sUAV @ State #{self.uavState}')
        print(
            f'Time {self.taskTime:.1f} / {self.autoTra.expectedTime:.2f}',
            (GREEN + "At Target" + RESET) if self.isAtTarget() else (RED + "Not at Target" + RESET)
        )
        print(GREEN if self.podPitchAtTarget else RED, end='')
        print(f'Pitch: {self.podPitchDeg:.2f} -> {self.expectedPodAngles.pitchDeg:.2f} == {self.podCommFeedback.pitchDeg:.2f}{RESET}')
        print(GREEN if self.podYawAtTarget else RED, end='')
        print(f'Yaw: {self.podYawDeg:.2f} -> {self.expectedPodAngles.yawDeg:.2f} == {self.podCommFeedback.yawDeg:.2f}{RESET}')
        print(GREEN if self.podHfovAtTarget else RED, end='')
        print(f'HFov: {self.podHfovDeg:.2f} -> {self.expectedPodAngles.hfovDeg:.2f} == {self.podCommFeedback.hfovDeg:.2f}{RESET}')
        self.controlStateMachine()
        print(f'{self.vesselDict = }')
        print(f'{self.targetId = }')
        print(f'{self.trackData = }')
        print(f'{self.targetPos = }')
    
    def signalHandler(self, sig, frame):
        print('You pressed Ctrl+C!')
        print('Turning Laser Off...')
        if self.podLaserOn:
            self.expectedLaserOnPub.publish(False)
        print('Laser is Off...')
        exit(0)

    def spin(self):
        while not rospy.is_shutdown():
            self.taskTime = self.getTimeNow() - self.startTime
            self.spinOnce()
            self.rate.sleep()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=['test', 'takeoff'], default='takeoff')
    parser.add_argument('--test', help='on ground test', action='store_true')
    parser.add_argument('--trackUSV', help='track usv', action='store_true')
    parser.add_argument('--trackVessel', help='track usv', action='store_true')
    parser.add_argument('--dock', help='look at dock', action='store_true')
    parser.add_argument('--fast', help='using default paras & skip confirmation', action='store_true')
    parser.add_argument('--bag', help='rosbag record', action='store_false')
    args, unknown = parser.parse_known_args()

    if args.bag:
        command = 'rosbag record -a -x "/suav/pod/main_camera_images.*" -o track'
        process = subprocess.Popen(command, shell=True)

    time.sleep(3)
    
    np.set_printoptions(
        precision=3, 
        threshold=10, 
        edgeitems=3, 
        linewidth=80,
        suppress=True
    )

    podSearch = PodSearch(args)

    podSearch.spin()
