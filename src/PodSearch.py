#! /usr/bin/env python3

import argparse
from os import system
from signal import signal, SIGINT

import rospy
from std_msgs.msg import Float32, Bool, Float64MultiArray, Int8

from AutoTra import AutoTra
from Utils import *
import PodParas


def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    exit(0)


signal(SIGINT, signal_handler)


class State:
    INIT = 0
    SEARCH = 1
    STREAM = 4
    END = 5
    TRACK = 6
    DOCK = 7

class PodSearch:
    def __init__(self, args):
        # arg parsing
        self.args = args
        print(YELLOW + 'ARGS:', self.args, RESET)
        if not self.args.takeoff and not self.args.test:
            raise AssertionError("Please add --takeoff or --test arg")
        if self.args.takeoff and self.args.test:
            raise AssertionError("Not two args at the same time!")

        # ros node initialising
        rospy.init_node('pod_search', anonymous=True)
        self.rate = rospy.Rate(10)

        # namespaces
        self.uavName = 'suav'
        self.deviceName = 'pod'

        # From PodComm: pod pitch, yaw, hfov
        self.pitch = 0.0
        rospy.Subscriber(self.uavName + '/' + self.deviceName + '/pitch', Float32, lambda msg: setattr(self, 'pitch', msg.data))
        self.yaw = 0.0
        rospy.Subscriber(self.uavName + '/' + self.deviceName + '/yaw', Float32, lambda msg: setattr(self, 'yaw', msg.data))
        self.hfov = 0.0
        rospy.Subscriber(self.uavName + '/' + self.deviceName + '/hfov', Float32, lambda msg: setattr(self, 'hfov', msg.data))

        # From PodComm: pod pitch, yaw, hfov at target or not
        self.pAtTarget = False
        rospy.Subscriber(self.uavName + '/' + self.deviceName + '/pAtTarget', Bool, lambda msg: setattr(self, 'pAtTarget', msg.data))
        self.yAtTarget = False
        rospy.Subscriber(self.uavName + '/' + self.deviceName + '/yAtTarget', Bool, lambda msg: setattr(self, 'yAtTarget', msg.data))
        self.fAtTarget = False
        rospy.Subscriber(self.uavName + '/' + self.deviceName + '/fAtTarget', Bool, lambda msg: setattr(self, 'fAtTarget', msg.data))

        # From PodComm: pod pitch, yaw, hfov feedback
        self.pFeedback = 0.0
        rospy.Subscriber(self.uavName + '/' + self.deviceName + '/pFeedback', Float32, lambda msg: setattr(self, 'pFeedback', msg.data))
        self.yFeedback = 0.0
        rospy.Subscriber(self.uavName + '/' + self.deviceName + '/yFeedback', Float32, lambda msg: setattr(self, 'yFeedback', msg.data))
        self.fFeedback = 0.0
        rospy.Subscriber(self.uavName + '/' + self.deviceName + '/fFeedback', Float32, lambda msg: setattr(self, 'fFeedback', msg.data))

        # To PodComm: expected pod pitch, yaw, hfov, max spin rate
        self.expectedPitch = 0
        self.pitchPub = rospy.Publisher(self.uavName + '/' + self.deviceName + '/expectedPitch', Float32, queue_size=10)
        self.expectedYaw = 0
        self.yawPub = rospy.Publisher(self.uavName + '/' + self.deviceName + '/expectedYaw', Float32, queue_size=10)
        self.expectedHfov = 0
        self.hfovPub = rospy.Publisher(self.uavName + '/' + self.deviceName + '/expectedHfov', Float32, queue_size=10)
        self.maxRate = 0
        self.maxRatePub = rospy.Publisher(self.uavName + '/' + self.deviceName + '/maxRate', Float32, queue_size=10)

        # To others: my state
        self.state = State.INIT
        self.searchStatePub = rospy.Publisher(self.uavName + '/' + self.deviceName + '/searchState', Int8, queue_size=1)

        # From suav: suav control state
        self.uavState = 0
        rospy.Subscriber(self.uavName + '/uavState', Int8, lambda msg: setattr(self, 'uavState', msg.data))

        # From Transformer: stream data
        self.streamIndex = None
        self.streamPitch = 0
        self.streamYaw = 0
        self.streamFlag = False
        rospy.Subscriber(self.uavName + '/' + self.deviceName + '/stream', Float64MultiArray, self.streamCallback)

        # From Transformer: track data
        self.trackData = {'boat': [], 'usv': [], 'cup': []}
        rospy.Subscriber(self.uavName + '/' + self.deviceName + '/track', Float64MultiArray, self.trackCallback)

        # From Transformer: dock data
        self.dockData = []
        rospy.Subscriber(self.uavName + '/' + self.deviceName + '/dock', Float64MultiArray, lambda msg: setattr(self, 'dockData', msg.data))

        # From usv: land flag
        self.landFlag = -1
        rospy.Subscriber('/usv/suav_land_flag', Int8, lambda msg: setattr(self, 'landFlag', msg.data))
        
        # trajectory setting
        self.autoTra = AutoTra(pitchLevelOn=True, overlapOn=True, drawNum=-1, takeoff=self.args.takeoff, fast=self.args.fast)
        self.tra = self.autoTra.theList
        if not self.args.fast:
            input('Type anything to continue...')

        # trajectory counter
        self.traCnt = 0

        # timers
        self.startTime = self.getTimeNow()
        self.taskTime = 0
        self.endBeginTime = None

        # ignore uav or not
        self.uavReady = True if self.args.test else False

        # set initial state
        if args.track:
            self.toStepTrack()
            print('<<<TRACK MODE>>>')
        if args.dock:
            self.toStepDock()
            print('<<<DOCK MODE>>>')

    def streamCallback(self, msg):
        self.streamPitch = msg.data[0]
        self.streamYaw = msg.data[1]
        self.streamIndex = int(msg.data[2])

    def trackCallback(self, msg):
        self.trackData[msg.layout.dim[0].label] = msg.data

    def getTimeNow(self):
        return rospy.Time.now().to_sec()

    def isAtTarget(self):
        return (
                self.pAtTarget and
                self.yAtTarget and
                self.fAtTarget and
                abs(self.pFeedback - self.expectedPitch) < 0.001 and
                abs(self.yFeedback - self.expectedYaw) < 0.001 and
                abs(self.fFeedback - self.expectedHfov) < 0.001
        )

    def toStepInit(self):
        self.state = State.INIT

    def toStepSearch(self):
        self.state = State.SEARCH

    def toStepEnd(self):
        self.state = State.END
        self.endBeginTime = self.getTimeNow()

    def toStepStream(self):
        self.state = State.STREAM
        if self.streamIndex == None:
            print(f'{RED}No targets to stream{RESET}')
            self.toStepEnd()
        self.streamStartTime = self.getTimeNow()

    def toStepTrack(self, trackName='boat'):
        self.state = State.TRACK
        self.trackData[trackName] = []

    def toStepDock(self):
        self.state = State.DOCK
        self.dockTime = self.getTimeNow()

    def stepInit(self):
        self.expectedPitch = self.tra[0][0]
        self.expectedYaw = self.tra[0][1]
        self.expectedHfov = self.tra[0][2]
        self.maxRate = self.tra[0][3]
        self.pubPYZMaxRate()
        if self.isAtTarget() and self.uavReady:
            self.toStepSearch()

    def stepSearch(self):
        print(f'{GREEN}==> StepSearch @ #{self.traCnt + 1}/{len(self.tra)} <=={RESET}')
        self.expectedPitch = self.tra[self.traCnt][0]
        self.expectedYaw = self.tra[self.traCnt][1]
        self.expectedHfov = self.tra[self.traCnt][2]
        self.maxRate = self.tra[self.traCnt][3]
        self.pubPYZMaxRate()
        if self.isAtTarget():
            self.traCnt += 1
        if self.traCnt == len(self.tra):
            self.toStepStream()

    def stepStream(self):
        if not self.streamFlag and self.isAtTarget() and self.getTimeNow() - self.streamStartTime >= 3.0:
            self.streamFlag = True
            self.streamStartTime = self.getTimeNow()
        streamTime = self.getTimeNow() - self.streamStartTime
        print(f'{YELLOW}==> StepStream @ Target {self.streamIndex} <=={RESET}')
        print(f'Time: {streamTime:.2f}')
        print(f'StreamFlag: {self.streamFlag}')
        if not self.streamFlag:
            self.expectedPitch = self.streamPitch
            self.expectedYaw = self.streamYaw
            self.expectedHfov = self.getHfovFromPitch(self.streamPitch)
            self.maxRate = 20
        else:
            print('Tracking...')
            if len(self.trackData['boat']) != 4:
                return
            self.expectedPitch = self.trackData['boat'][0] - 0.5
            self.expectedYaw = self.trackData['boat'][1]
            self.expectedHfov = self.getHfovFromPitch(self.trackData['boat'][0], minHfov=4)
            self.maxRate = self.trackData['boat'][3]
        self.pubPYZMaxRate()
        if streamTime >= 10.0:
            self.toStepDock()

    def stepEnd(self):
        endTime = self.getTimeNow() - self.endBeginTime
        print(f'StepEnd with {endTime:.2f} seconds')
        if endTime >= 3.0:
            exit(0)

    def getHfovFromPitch(self, pitch):
        return min(PodParas.maxHfov, max(PodParas.minHfov, pitch * self.autoTra.hfovPitchRatio))

    def stepTrack(self):
        print('Step Track')
        if self.landFlag == 1:
            self.toStepEnd()
        trackName = 'boat'
        if len(self.trackData[trackName]) < 4:
            return
        self.expectedPitch = self.trackData[trackName][0] - 0.5
        self.expectedYaw = self.trackData[trackName][1]
        self.expectedHfov = self.getHfovFromPitch(self.trackData[trackName][0])
        self.maxRate = self.trackData[trackName][3]
        self.pubPYZMaxRate()

    def stepDock(self):
        print(
            f'Dock {self.getTimeNow() - self.dockTime:.2f}, '
            f'{(GREEN + "At Target" + RESET) if self.isAtTarget() else (RED + "Not At Target" + RESET)}'
        )
        if len(self.dockData) < 4:
            print('No dock data!!!')
            return
        self.expectedPitch = self.dockData[1]
        self.expectedYaw = self.dockData[2]
        self.expectedHfov = self.getHfovFromPitch(self.dockData[1])
        self.maxRate = 10
        self.pubPYZMaxRate()
        if self.getTimeNow() - self.dockTime >= 10:
            self.toStepTrack('usv')

    def pubPYZMaxRate(self):
        if self.expectedYaw < -90 or self.expectedYaw > 90:
            self.expectedYaw += 180
        self.pitchPub.publish(self.expectedPitch)
        self.yawPub.publish(self.expectedYaw)
        self.hfovPub.publish(self.expectedHfov)
        self.maxRatePub.publish(self.maxRate)

    def controlStateMachine(self):
        self.searchStatePub.publish(Int8(self.state))
        if self.uavState >= 4:
            self.uavReady = True
        if self.state == State.INIT:
            self.stepInit()
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
        else:
            print("Invalid state")

    def spin(self):
        while not rospy.is_shutdown():
            self.taskTime = self.getTimeNow() - self.startTime
            system('clear')
            print('-' * 20)
            print(f'### PodSearch ###')
            print(f'Me @ State #{self.state} sUAV @ State #{self.uavState}')
            print(
                f'Time {self.taskTime:.1f} / {self.autoTra.expectedTime:.2f}',
                (GREEN + "At Target" + RESET) if self.isAtTarget() else (RED + "Not at Target" + RESET)
            )
            print(GREEN if self.pAtTarget else RED, end='')
            print(f'Pitch: {self.pitch:.2f} -> {self.expectedPitch:.2f} == {self.pFeedback:.2f}{RESET}')
            print(GREEN if self.yAtTarget else RED, end='')
            print(f'Yaw: {self.yaw:.2f} -> {self.expectedYaw:.2f} == {self.yFeedback:.2f}{RESET}')
            print(GREEN if self.fAtTarget else RED, end='')
            print(f'HFov: {self.hfov:.2f} -> {self.expectedHfov:.2f} == {self.fFeedback:.2f} {RESET}')
            self.controlStateMachine()
            self.rate.sleep()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--takeoff', help='takeoff', action="store_true")
    parser.add_argument('--test', help='on ground test', action='store_true')
    parser.add_argument('--track', help='track mode', action='store_true')
    parser.add_argument('--dock', help='look at dock', action='store_true')
    parser.add_argument('--fast', help='using default paras & skip confirmation', action='store_true')
    args, unknown = parser.parse_known_args()
    
    podSearch = PodSearch(args)

    podSearch.spin()
