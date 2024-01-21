#! /usr/bin/env python3

import re
import time
import yaml
import rich
import copy
import json
import heapq
import socket
import argparse
import datetime
import pyfiglet
import itertools
import subprocess
import numpy as np
from os import system
from enum import Enum
from signal import signal, SIGINT
from rich.console import Console

import rospy
import rospkg
from std_msgs.msg import Float32, Bool, Float64MultiArray, Int8, Int16, MultiArrayDimension, Header, Float32MultiArray, String
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, QuaternionStamped
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


class State(Enum):
    INIT = 0
    PREPARE = 1
    SEARCH = 2
    STREAM = 4
    END = 5
    TRACK = 6
    DOCK = 7
    REFIND = 8
    GUIDE = 9
    
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
        self.console = Console()

        # arg parsing
        self.args = args
        print(BLUE + 'ARGS:', self.args, RESET)

        self.packagePath = rospkg.RosPack().get_path('pod_search')
        with open(self.packagePath + '/config/config' + self.args.config + '.yaml', 'r') as config:
            self.config = yaml.safe_load(config)

        if not self.args.fast:
            self.config['SearchPoints'] = [self.config['SearchPoints'][0]]
            self.config['searchConfig'] = [self.config['searchConfig'][0]]

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

        # [StepTrack] Tracking ratio inside bbox, [-1, 1]
        self.trackX = 0
        self.trackY = -1
        
        rospy.Subscriber(self.uavName + '/' + self.deviceName + '/vessel_det', TargetsInFrame, self.vesselDetectionCallback, queue_size=1)
        rospy.Subscriber(self.uavName + '/' + self.deviceName + '/usv_detection', TargetsInFrame, self.usvDetectionCallback, queue_size=1)
        rospy.Subscriber(self.uavName + '/' + self.deviceName + '/tv_det', TargetsInFrame, self.vesselDetectionCallback, queue_size=1)

        # From localisation: location
        self.uavPos = np.array(self.config['uavInitialPos'])
        rospy.Subscriber(self.uavName + '/uwb/filter/odom', Odometry, self.uavPosCallback, queue_size=1)

        # From dji osdk: attitude
        self.uavEuler = [float(self.config['uavYawDeg'])]
        rospy.Subscriber(self.uavName + '/dji_osdk_ros/attitude', QuaternionStamped, self.djiAttitudeCallback, queue_size=1)

        # From suav: suav control state
        self.uavState = 0
        rospy.Subscriber(self.uavName + '/uavState', Int16, lambda msg: setattr(self, 'uavState', msg.data))

        # From suav: suav yaw
        self.uavYawDeg = 0
        rospy.Subscriber(self.uavName + '/xy_fcu/flight_data', Float32MultiArray, lambda msg: setattr(self, 'uavYawDeg', np.mod(450 - np.degrees(msg.data[15]), 360)))

        # From KSB: KSB state
        self.ksbState = 'None'
        rospy.Subscriber('/ksb/state', String, lambda msg: setattr(self, 'ksbState', msg.data)) 

        # To others: my state
        self.state = State.INIT
        self.searchStatePub = rospy.Publisher(self.uavName + '/' + self.deviceName + '/searchState', Int16, queue_size=1)

        # To others: search system state:
        self.systemState = 'COMM_TEST'
        self.systemStatePub = rospy.Publisher(self.uavName + '/state', String, queue_size=1)

        # From others: others state
        for name in self.config['others']:
            setattr(self, name + 'State', 'NONE')
            rospy.Subscriber(f'/{name}/state', String, lambda msg, name=name: setattr(self, f'{name}State', msg.data))
        # rospy.Subscriber('/tuav6/state', String, lambda msg: setattr(self, 'tuav6State', msg.data))
        # rospy.Subscriber('/tuav1/state', String, lambda msg: setattr(self, 'tuav1State', msg.data))
        

        # To streamer
        self.toStreamerPub = rospy.Publisher(self.uavName + '/toStreamer', Int8, queue_size=1)

        # [StepPrepare: pre-search] pre-search counter
        self.preGuideCnt = 0
        self.preGuideCntGen = itertools.cycle(range(len(self.config['preGuideData'])))

        # [StepPrepare] counters
        self.searchRoundCnt = 0
        self.searchViewCnt = -1

        # [StepPrepare] search points pub
        self.searchPoints = [SearchPoint(**item) for item in self.config['SearchPoints']]
        print(f'Search Points: {self.searchPoints}')
        self.backPoint = SearchPoint(**self.config['SearchPoints'][0])
        self.backPoint.id = 6
        self.dockPoint = SearchPoint(**self.config['SearchPoints'][0])
        self.dockPoint.id = 65
        self.dockPoint.uavYaw = 180
        self.trackPoint = copy.deepcopy(self.dockPoint)
        self.idGen = itertools.cycle(range(66, 89))
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
                            'areaPoints': self.searchAreaPoints,
                            'uavPos': (view['uavPosF'], view['uavPosL'], view['uavPosU']),
                            'yaw': view['uavYaw'],
                            'yawRange': view['yawRange'],
                            'theTime': round['theTime'],
                            'widthRatio': round['widthRatio']
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

        # [StepTrack] report target number
        self.reportNumber = 1

        # [StepTrack] land flag
        self.landFlag = -1
        rospy.Subscriber('/usv/suav_land_flag', Int8, lambda msg: setattr(self, 'landFlag', msg.data))

        # [StepGuide] distance from datalink
        self.datalinkR = 0

        # [StepGuide] guidance data
        self.usvTargetPub = rospy.Publisher(self.uavName + '/' + self.deviceName + '/target_nav_position', PoseStamped, queue_size=1)

        # [StepGuide] last capture time of usv
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

        # [StepGuide] fake target position
        self.fakeTargetPos = None
        self.guideFake = None

        signal(SIGINT, self.signalHandler)

        # set initial state
        if args.trackUSV:
            self.toStepGuide()
            print('<<<TRACK MODE: USV>>>')
        if args.trackVessel:
            self.toStepTrack(trackName=args.id)
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

        self.waitForStart()

        # timers
        self.startTime = self.getTimeNow()
        self.taskTime = 0
        self.tic = self.getTimeNow()
        self.toc = self.getTimeNow()

    def getDatalinkR(self):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                request_json = json.dumps({"get":"radioinfo"}).encode('utf-8')
                s.sendto(request_json,(self.config['myDatalinkIpAddr'], 9999))
                s.settimeout(1.0)
                
                try:
                    responce, _ = s.recvfrom(1024)
                    responce_str = responce.decode('utf-8')

                    match = re.search(r'{.*}',responce_str)

                    if match:
                        valid_json = match.group(0)
                        try:
                            responce_data = json.loads(valid_json)

                        except json.JSONDecodeError as e:
                            print(f"JSON decoding error: {e}")

                    for sender in responce_data["senders"]:
                        dist = sender["dist"]
                        ipAddr = sender["ipAddr"]
                        if ipAddr == self.config['usvDatalinkIpAddr'] and 0 < dist < 3000:
                            self.datalinkR = dist

                except socket.timeout:
                    print("Timed out waiting for a Datalink packet.")
        except:
            pass

    @property
    def othersAllReady(self):
        if not self.args.check:
            return True
        for name in self.config['others']:
            state = getattr(self, name + 'State', 'NONE')
            if state != 'READY' and state != 'STANDBY':
                return False
        return True
    
    @property
    def relatedAllReady(self):
        if not self.args.check:
            return True
        for name in self.config['related']:
            state = getattr(self, name + 'State', 'NONE')
            if state != 'COMM_TEST' and state != 'READY':
                return False
        return True

    def waitForStart(self):
        startTime = datetime.datetime.now()
        if self.args.start == 'minute':
            startTime = (startTime + datetime.timedelta(minutes=1)).replace(second=0, microsecond=0)
        elif self.args.start == 'hour':
            startTime = (startTime + datetime.timedelta(hours=1)).replace(minute=0, second=0, microsecond=0)
        else:
            startTime = (startTime + datetime.timedelta(seconds=1))

        while datetime.datetime.now() < startTime or self.systemState != 'COUNTDOWN':
            if self.relatedAllReady:
                self.systemState = 'READY'
            if self.systemState == 'READY' and self.othersAllReady:
                self.systemState = 'COUNTDOWN'
            self.systemStatePub.publish(self.systemState)
            system('clear')
            self.console.rule(
                f'[bold red]'
                f'Set start at {startTime}'
            )
            if startTime > datetime.datetime.now():
                remainingTime = startTime - datetime.datetime.now()
                countDownStr = str(remainingTime).split('.')[0]
                colour = 'green'
            else:
                countDownStr = str(datetime.datetime.now() - startTime).split('.')[0]
                colour = 'red'
            self.console.print(
                pyfiglet.figlet_format(countDownStr),
                justify='center',
                style=colour
            )
            self.console.rule(
                self.uavName + ': ' + self.systemState
            )
            for name in self.config['others']:
                self.console.print(
                    name + ': ' + getattr(self, name + 'State', 'NONE'),
                    justify='center',
                    style=('green' if getattr(self, name + 'State', 'NONE') == 'READY' else 'red3')
                )
            time.sleep(1)

        self.systemState = 'START'

    def targetPosCallback(self, msg):
        self.targetPos = np.array([[msg.data[0]], [msg.data[1]], [0]])

    def uavPosCallback(self, msg):
        self.uavPos = np.array([[msg.twist.twist.linear.x], [msg.twist.twist.linear.y], [msg.twist.twist.linear.z]])

    def djiAttitudeCallback(self, msg):
        quaternion = [
            msg.quaternion.x,
            msg.quaternion.y,
            msg.quaternion.z,
            msg.quaternion.w
        ]
        self.uavEuler = R.from_quat(quaternion).as_euler('zyx', degrees=True).tolist()
        self.uavYawDeg = self.uavEuler[0]

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
            px = (target.cx + self.trackX * (target.w / 2) - 0.5) * 2
            py = (target.cy + self.trackY * (target.h / 2) - 0.5) * 2
            try:
                self.vesselCameraAzimuth = -np.degrees(np.arctan(np.tan(np.radians(self.podHfovDegDelayed) / 2) * px))
                self.vesselCameraElevation = np.degrees(np.arctan(np.tan(np.radians(self.podVfovDegDelayed) / 2) * py))
                id = str(target.category_id)
                if id != '100':
                    if self.args.id == 'boat' and self.args.trackVessel:
                        id = 'boat'
                    expectedHfovDeg = self.podHfovDegDelayed
                    if target.w < self.config['trackWidth']['min']:
                        expectedHfovDeg = PodParas.clipHfov(self.podHfovDegDelayed * target.w / self.config['trackWidth']['min'])
                    if target.w > self.config['trackWidth']['max']:
                        expectedHfovDeg = PodParas.clipHfov(self.podHfovDegDelayed * target.w / self.config['trackWidth']['max'])
                    self.trackData[id] = PodAngles(
                        pitchDeg=self.vesselCameraElevation + self.podPitchDegDelayed, 
                        yawDeg=self.vesselCameraAzimuth + self.podYawDegDelayed, 
                        hfovDeg=expectedHfovDeg, 
                        maxRateDeg=self.config['trackMaxRateDeg'],
                        laserOn=self.config['laserOn']
                    )
                    score = target.score
                    self.lastVesselCaptureTime[id] = self.getTimeNow()
                    # print(f'{BOLD}{BLUE}{id = } {score = }{RESET}')
                    if self.state == State.SEARCH and target.category_id != 100:
                        if id in self.vesselDict.keys():
                            self.vesselDict[id] = min(self.vesselDict[id], score)
                        else:
                            self.vesselDict[id] = score
                    
            except Exception as e:
                pass
                print(e)

    def getMinScoreTargetIdAndScore(self):
        if len(self.vesselDict) == 0:
            return None
        return min(self.vesselDict.items(), key=lambda x: x[1])
    
    def getKthScoreTargetIdAndScore(self, k):
        if len(self.vesselDict) == 0:
            return None
        item = heapq.nsmallest(k, self.vesselDict.items(), key=lambda x: x[1])
        return item[-1] if len(item) == k else None

    def usvDetectionCallback(self, msg):
        self.usvCameraAzimuth = None
        self.usvCameraElevation = None
        for target in msg.targets:
            px = (target.cx + self.trackX * (target.w / 2) - 0.5) * 2
            py = (target.cy + self.trackY * (target.h / 2) - 0.5) * 2
            try:
                self.usvCameraAzimuth = -np.degrees(np.arctan(np.tan(np.radians(self.podHfovDegDelayed) / 2) * px))
                self.usvCameraElevation = np.degrees(np.arctan(np.tan(np.radians(self.podVfovDegDelayed) / 2) * py))
                if target.w < self.config['guideWidth']['min'] or target.w > self.config['guideWidth']['max']:
                    expectedHfovDeg = PodParas.clipHfov(self.podHfovDegDelayed * target.w / self.config['guideWidth']['avg'])
                elif self.trackData['usv'] is not None:
                    expectedHfovDeg = self.trackData['usv'].hfovDeg
                else:
                    expectedHfovDeg = self.podHfovDeg
                self.trackData['usv'] = PodAngles(
                    pitchDeg=self.usvCameraElevation + self.podPitchDegDelayed, 
                    yawDeg=self.usvCameraAzimuth + self.podYawDegDelayed, 
                    hfovDeg=expectedHfovDeg, 
                    maxRateDeg=self.config['guideMaxRateDeg'],
                    laserOn=self.config['laserOn']
                )
                self.lastUSVCaptureTime = self.getTimeNow()
            except Exception as e:
                print(e)

    def getTimeNow(self):
        return rospy.Time.now().to_sec()

    def getPodAnglesNow(self):
        return PodAngles(
            pitchDeg=self.podPitchDeg,
            yawDeg=self.podYawDeg,
            hfovDeg=self.podHfovDeg,
            maxRateDeg=self.expectedPodAngles.maxRateDeg
        )

    def isAtTarget(self):
        if self.args.head_only:
            return self.toc - self.tic >= 5.0
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
        if self.isAtTarget():
            self.toStepPrepare()

    @stepEntrance
    def toStepPrepare(self):
        self.state = State.PREPARE
        self.searchViewCnt += 1
        if self.searchViewCnt == len(self.searchPoints):
            self.searchViewCnt = 0
            self.searchRoundCnt += 1
        if self.searchRoundCnt == len(self.autoTras):
            self.searchRoundCnt = 0
            # self.toStepDock()
            # self.toStepEnd()
            return
        self.autoTra = self.autoTras[self.searchRoundCnt][self.searchViewCnt]
        self.tra = self.autoTra.theList
        self.expectedPodAngles = copy.deepcopy(self.tra[0])

    def stepPrepare(self):
        self.console.rule(
            f'[bold blue]'
            f'Preparing, '
            f'Round #{self.searchRoundCnt + 1}, View #{self.searchViewCnt + 1}, '
            f'waiting for {self.searchPoints[self.searchViewCnt].id:d}X1'
        )
        self.pubPYZMaxRate()
        self.searchPointPub.publish(data=self.searchPoints[self.searchViewCnt].toList())
        if self.uavState % 10 == 1 and self.uavState // 100 == self.searchPoints[self.searchViewCnt].id:
            self.toStepSearch()

    @stepEntrance
    def toStepSearch(self, reset=True):
        self.state = State.SEARCH
        if reset:
            self.traCnt = 0

    def stepSearch(self):
        self.console.rule(
            f'[bold blue]'
            f'Searching #{self.traCnt + 1}/{len(self.tra)}, '
            f'Round #{self.searchRoundCnt + 1}, View #{self.searchViewCnt + 1}, '
            f'Spend {self.autoTra.expectedTime:.1f}'
        )
        self.expectedPodAngles = copy.deepcopy(self.tra[self.traCnt])
        if self.uavState % 10 == 0:
            self.expectedPodAngles.maxRateDeg = 0
            self.console.rule(
                f'[red3]'
                f'FREEZING!!!!'
            )
        self.pubPYZMaxRate()
        idAndScore = self.getMinScoreTargetIdAndScore()
        if idAndScore is not None and idAndScore[1] < self.config['targetSimilarityThreshold']:
            self.targetId = idAndScore[0]
            self.toStepTrack(self.targetId)
        if self.isAtTarget() and self.toc - self.tic >= 3:
            self.traCnt += 1
        if self.traCnt == len(self.tra):
            if len(self.vesselDict) > 0:
                self.targetId = self.getMinScoreTargetIdAndScore()[0]
            self.toStepPrepare()
        if self.targetId is not None and self.getTimeNow() - self.lastVesselCaptureTime[self.targetId] < 0.1:
            self.toStepTrack(self.targetId)

    @stepEntrance
    def toStepTrack(self, trackName):
        self.state = State.TRACK
        self.trackName = trackName
        self.trackData[trackName] = self.getPodAnglesNow()
        self.ekfs[trackName] = LocatingEKF(initialT=self.getTimeNow())
        if not self.podLaserOn and self.config['laserOn']:
            self.expectedLaserOn = True
        else:
            self.expectedLaserOn = False
        self.expectedLaserOnPub.publish(self.expectedLaserOn)
    
    def stepTrack(self):
        self.console.rule(
            f'[bold blue]'
            f'Tracking for {self.trackName}... Laser '
            f"{'[bold red] on' if self.expectedLaserOn else '[bold green]off'}"
            f'Report #{self.reportNumber}'
        )
        self.console.rule(
            f'[bold blue]'
            f'Search Point #{self.trackPoint.id}: '
            f'{self.trackPoint}'
        )
        self.console.print(
            f'{self.trackData[self.trackName]}'
        )
        if self.trackName in self.lastVesselCaptureTime:
            self.console.print(
                f'{self.getTimeNow() - self.lastVesselCaptureTime[self.trackName]}'
            )
        self.trackX = np.sin((self.toc - self.tic) * 2 * np.pi / 10)
        self.trackY = -1 + np.sin((self.toc - self.tic) * 2 * np.pi / 10)
        self.console.rule(
            f'[magenta2]'
            f'track ({self.trackX:6.2f}, {self.trackY:6.2f})'
        )
        if self.getTimeNow() - self.lastVesselCaptureTime[self.trackName] >= 5.0:
            self.toStepRefind(self.trackName)
        if not self.podLaserOn and self.config['laserOn']:
            self.expectedLaserOn = True
            self.expectedLaserOnPub.publish(True)
        if self.trackName in self.trackData and self.trackData[self.trackName].hfovDeg > 0:
            self.expectedPodAngles = self.trackData[self.trackName]
            self.pubPYZMaxRate()
        ekfZ = np.array([[self.podLaserRange], [self.vesselCameraAzimuth], [self.vesselCameraElevation], [self.uavPos[2][0]]])
        self.console.print(f'{ekfZ = }')
        if not (0 < ekfZ[0][0] < 4000 and ekfZ[1][0] is not None and ekfZ[2][0] is not None):
            ekfZ = np.array([[0], [0], [0], [0]])
        self.ekfs[self.trackName].newFrame(
            self.getTimeNow(),
            ekfZ,
            self.uavPos,
            self.rP2BDelayed,
            R.from_euler('zyx', [self.uavYawDeg, 0, 0], degrees=True).as_matrix().T
        )
        print(f'{self.ekfs[self.trackName].ekf.x = }')
        if self.ksbState == 'TargetConfirmed' and self.ekfs[self.trackName].ekf.x is not None:
            self.targetPos = self.ekfs[self.trackName].ekf.x[:3].reshape(3, 1)
            self.toStepDock()
        if self.ksbState == 'TargetRejected' or self.toc - self.tic >= 300:
            self.reportNumber += 1
            if self.config['onReportFailure'] == 'next':
                idAndScore = self.getMinScoreTargetIdAndScore()
            elif self.config['onReportFailure'] == 'remove':
                self.vesselDict.pop(self.targetId)
                idAndScore = self.getMinScoreTargetIdAndScore()
            if idAndScore is None or self.reportNumber >= 3:
                self.vesselDict.clear()
                self.targetId = None
                self.toStepPrepare()
            else:
                self.targetId = idAndScore[0]
                self.toStepSearch(reset=False)
                
    @stepEntrance
    def toStepGuide(self):
        self.state = State.GUIDE
        self.guideFake = False
        self.trackName = 'usv'
        self.trackData[self.trackName] = self.getPodAnglesNow()
        if self.ekfs[self.trackName] is None or self.ekfs[self.trackName].ekf.x is None:
            self.ekfs[self.trackName] = LocatingEKF(initialT=self.getTimeNow())
        self.expectedLaserOn = False
        if self.guideFake:
            factor = -1 if self.targetPos[1][0] < self.uavPos[1][0] else 1
            self.fakeTargetPos = np.array([
                [self.uavPos[0][0]],
                [self.uavPos[1][0] + factor * self.config['usvGuideSideLength']],
                [0]
            ])
        if not self.podLaserOn and self.config['laserOn']:
            self.expectedLaserOn = True
        else:
            self.expectedLaserOn = False
        self.expectedLaserOnPub.publish(self.expectedLaserOn)

    def stepGuide(self):
        self.trackX = np.sin((self.toc - self.tic) * 2 * np.pi / 8)
        self.trackY = -1 + np.sin((self.toc - self.tic) * 2 * np.pi / 6)
        self.trackPoint.id = next(self.idGen)
        self.trackPoint.uavYaw = (self.toc - self.tic) * 3 + self.dockPoint.uavYaw
        self.searchPointPub.publish(data=self.trackPoint.toList())
        self.console.rule(
            f'[bold blue]'
            f'Tracking for {self.trackName}... Laser '
            f"{'[bold red] on' if self.expectedLaserOn else '[bold green]off'}"
        )
        self.console.rule(
            f'[bold blue]'
            f'Search Point #{self.trackPoint.id}: '
            f'{self.trackPoint}'
        )
        self.console.rule(
            f'[magenta2]'
            f'track ({self.trackX:6.2f}, {self.trackY:6.2f})'
        )
        self.console.print(
            f'{self.trackData[self.trackName]}'
        )
        if self.landFlag == 1:
            self.toStepEnd()
        if self.getTimeNow() - self.lastUSVCaptureTime >= 5.0:
            self.toStepRefind(self.trackName)
        if not self.podLaserOn and self.config['laserOn']:
            self.expectedLaserOn = True
            self.expectedLaserOnPub.publish(True)
        if self.trackName in self.trackData and self.trackData[self.trackName] is not None:
            self.expectedPodAngles = self.trackData[self.trackName]
            if not self.podLaserOn and self.config['laserOn']:
                self.expectedLaserOn = True
            self.pubPYZMaxRate()
        ekfZ = np.array([[self.podLaserRange], [self.usvCameraAzimuth], [self.usvCameraElevation], [self.uavPos[2][0]]])
        if (not (0 < self.podLaserRange < 3000)) and (0 < self.datalinkR < 3000):
            ekfZ[0][0] = self.datalinkR
        if not (0 < ekfZ[0][0] < 4000 and ekfZ[1][0] is not None and ekfZ[2][0] is not None):
            ekfZ = np.array([[0], [0], [0], [0]])
        self.ekfs[self.trackName].newFrame(
            self.getTimeNow(),
            ekfZ,
            self.uavPos,
            self.rP2BDelayed,
            R.from_euler('zyx', [self.uavYawDeg, 0, 0], degrees=True).as_matrix().T
        )
        print(f'{self.ekfs[self.trackName].ekf.x = }')
        # if self.toc - self.tic >= 60:
        #     self.toStepEnd()
        # return
        if self.ekfs[self.trackName].ekf.x is not None:
            usvPos = self.ekfs[self.trackName].ekf.x[:3].reshape((3, 1))
            self.ekfLogs[self.trackName].log('usvEKFx', self.ekfs[self.trackName].ekf.x)
            self.ekfLogs[self.trackName].newline()
            print(f'{self.targetPos = }')
            if self.guideFake:
                usvToTargetENU = self.fakeTargetPos - usvPos
                usvToTargetTheta = np.degrees(np.arctan2(usvToTargetENU[1][0], usvToTargetENU[0][0]))
            else:
                usvToTargetENU = self.targetPos - usvPos
                usvToTargetTheta = np.degrees(np.arctan2(usvToTargetENU[1][0], usvToTargetENU[0][0]))
            if self.guideFake:
                if usvPos[0][0] > self.uavPos[0][0]:
                    self.guideFake = False
            self.usvTargetPub.publish(
                header=Header(frame_id=('fake' if self.guideFake else 'target')),
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

    @stepEntrance
    def toStepRefind(self, refindName):
        self.state = State.REFIND
        self.refindPodAngles = self.getPodAnglesNow()
        self.refindName = refindName

    def stepRefind(self):
        self.console.rule(
            f'[bold blue]'
            f'Refinding {self.refindName} '
            f'@ (p{self.refindPodAngles.pitchDeg:.2f}, y{self.refindPodAngles.yawDeg:.2f}, hfov{self.refindPodAngles.hfovDeg:.2f})'
        )
        self.expectedPodAngles = PodAngles(
            pitchDeg=self.refindPodAngles.pitchDeg + (self.toc - self.tic) / 100 * np.sin((self.toc - self.tic) / 15 * 2 * np.pi),
            yawDeg=self.refindPodAngles.yawDeg + (self.toc - self.tic) / 30 * np.cos((self.toc - self.tic) / 15 * 2 * np.pi),
            hfovDeg=PodParas.clipHfov(self.refindPodAngles.hfovDeg * 1.5),
            maxRateDeg=10
        )
        self.pubPYZMaxRate()
        if self.getTimeNow() - self.lastUSVCaptureTime < 0.1:
            if self.refindName == 'usv':
                self.toStepGuide()
            else:
                self.toStepTrack(self.refindName)

    @stepEntrance
    def toStepDock(self):
        self.state = State.DOCK
        self.preGuideCnt = 0

    def stepDock(self):
        self.expectedPodAngles = PodAngles(**self.config['preGuideData'][self.preGuideCnt])
        if self.isAtTarget():
            self.preGuideCnt = next(self.preGuideCntGen)
        self.pubPYZMaxRate()
        self.searchPointPub.publish(data=self.dockPoint.toList())
        self.console.rule(
            f'[bold blue]'
            f'Search Point #{self.dockPoint.id}: '
            f'{self.dockPoint}'
        )
        if (
                self.toc - self.tic >= 10 and
                self.isAtTarget()
        ):
            self.toStepGuide()

    @stepEntrance
    def toStepEnd(self):
        self.state = State.END

    def stepEnd(self):
        self.searchPointPub.publish(data=self.backPoint.toList())
        if self.podLaserOn:
            self.expectedLaserOnPub.publish(False)
        if self.toc - self.tic >= 3.0:
            exit(0)

    def pubPYZMaxRate(self):
        self.pitchPub.publish(self.expectedPodAngles.pitchDeg)
        self.yawPub.publish(self.expectedPodAngles.yawDeg)
        self.hfovPub.publish(self.expectedPodAngles.hfovDeg)
        self.expectedMaxRatePub.publish(self.expectedPodAngles.maxRateDeg)
        self.expectedLaserOnPub.publish(self.expectedPodAngles.laserOn if self.config['laserOn'] else False)

    def controlStateMachine(self):
        self.toc = self.getTimeNow()
        self.searchStatePub.publish(Int16(self.state.value))
        if self.ksbState == 'GoHome' and self.state != State.END:
            self.toStepEnd()
        elif self.state == State.INIT:
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
        elif self.state == State.GUIDE:
            self.stepGuide()
        else:
            print("Invalid state")

    @delayStart
    def spinOnce(self):
        self.console.clear()
        self.console.rule(
            f'[magenta2]'
            f'PodSearch[/] @ '
            f'{self.taskTime:.1f} '
            f'[blue3]'
            f'Step{self.state.name}[/] @ '
            f'{self.toc - self.tic:.1f}',
            style=('green' if self.isAtTarget() else 'red')
        )
        self.console.rule(
            f'[red3]'
            f'suav in #{self.uavState} '
            f'@ ({", ".join([f"{self.uavPos[i][0]:.2f}" for i in range(3)])}), {self.uavYawDeg:.2f} Deg'
        )
        self.console.rule(
            f'[cyan3]'
            f'KSB state: {self.ksbState}'
        )
        self.console.rule(
            f'[red3]'
            f'Datalink range: {self.datalinkR:.2f}'
        )
        if (self.targetId is not None and self.searchRoundCnt > 0) or (self.searchRoundCnt == len(self.config['searchConfig']) - 1 and len(self.vesselDict) == 0):
            self.console.rule(
                f'[red3]'
                f'RESET ' * 50,
                style='red3'
            )
        if self.args.head_only:
            self.console.print(
                pyfiglet.figlet_format('Head Only'),
                justify='center'
            )
        else:
            self.console.print(
                f'Pitch: {self.podPitchDeg:.2f} -> {self.expectedPodAngles.pitchDeg:.2f} == {self.podCommFeedback.pitchDeg:.2f}',
                style='green' if self.podPitchAtTarget else 'red',
                justify='center'
            )
            self.console.print(
                f'Yaw: {self.podYawDeg:.2f} -> {self.expectedPodAngles.yawDeg:.2f} == {self.podCommFeedback.yawDeg:.2f}',
                style='green' if self.podYawAtTarget else 'red',
                justify='center'
            )
            self.console.print(
                f'HFov: {self.podHfovDeg:.2f} -> {self.expectedPodAngles.hfovDeg:.2f} == {self.podCommFeedback.hfovDeg:.2f}',
                style='green' if self.podHfovAtTarget else 'red',
                justify='center'
            )
        self.toStreamerPub.publish(Int8(data=(1 if self.state == State.TRACK else 0)))
        self.systemState = 'REPORT' if self.state.name == 'TRACK' else self.state.name
        self.systemStatePub.publish(self.systemState)
        self.controlStateMachine()
        self.console.print(f'{self.vesselDict = }')
        self.console.print(f'{self.targetId = }')
        # self.console.print(f'{self.trackData = }')
        self.console.print(f'{self.targetPos = }')
        self.getDatalinkR()
    
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
    parser.add_argument('--config', choices=['20', '300'], default='300')
    parser.add_argument('--trackUSV', help='track usv', action='store_true')
    parser.add_argument('--trackVessel', help='track usv', action='store_true')
    parser.add_argument('--id', default='boat')
    parser.add_argument('--dock', help='look at dock', action='store_true')
    parser.add_argument('--fast', help='using default paras & skip confirmation', action='store_false')
    parser.add_argument('--bag', help='rosbag record', action='store_true')
    parser.add_argument('--start', choices=['now', 'minute', 'hour'], default='minute')
    parser.add_argument('--init', help='initial state', default='None')
    parser.add_argument('--head-only', help='no pod', action='store_true')
    parser.add_argument('--check', help='check other system state', action='store_true')
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
