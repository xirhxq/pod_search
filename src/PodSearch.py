#! /usr/bin/env python3

import argparse
from os import system
from signal import signal, SIGINT

import pyfiglet
import rospy
from std_msgs.msg import Float32, Bool, Int16, Float64MultiArray, Empty, Int8

from Utils import *

from AutoTra import AutoTra

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    exit(0)


signal(SIGINT, signal_handler)


class State:
    INIT = 0
    SEARCH = 1
    BROWSE = 2
    AIM = 3
    STREAM = 4
    END = 5

class PodSearch:
    def __init__(self):
        self.state = State.INIT

        self.pitch = 0.0
        self.yaw = 0.0
        self.hfov = 0.0

        self.expectedPitch = 0
        self.expectedYaw = 0
        self.expectedHfov = 0
        self.maxRate = 0

        self.tra = [
            [90 - 20, -55, 50, 20], [90 - 20, 55, 50, 6],
            [90 - 11, 78, 20, 20], [90 - 11, -78, 20, 2],
            [90 - 4, -83, 6, 20], [90 - 4, 83, 6, 1.5],
            [90 - 1, 42, 3, 20], [90 - 1, -42, 3, 1],
            [90, 0, 60, 20]
        ]

        self.tra = [
            [90 - 30, -55, 50, 20], [90 - 30, 55, 50, 20],
            [90 - 11, 50, 20, 20], [90 - 11, -50, 20, 7],
            [90 - 4, -35, 6, 20], [90 - 4, 10, 6, 1],
            [90, 0, 60, 20]
        ]

        self.tra = [
            [90 - 4, -80, 10, 20], [90 - 4, 50, 10, 2],
            [90, 0, 60, 20]
        ]

        self.tra = [
            [90 - 8, -35, 20, 20], [90 - 8, 35, 20, 5],
            [90, 0, 60, 20]
        ]

        self.tra = [
            [61.52, -20, 60, 20], [61.52, 20, 60, 15],
            [82.85, 73, 22, 20], [82.85, -73, 22, 5],
            #[87.75, -57, 13, 20], [87.75, 57, 13, 3],
            [90, 0, 60, 20]
        ]

        self.tra = [
            [53.543243, -42.84, 33.52, 20], [53.543243, 42.84, 33.52, 11.17],
            [68.397683, 62.93, 22.70, 20], [68.397683, -62.93, 22.70, 7.57],
            [77.958248, -74.88, 13.74, 20], [77.958248, 74.88, 13.74, 4.58],
            [90, 0, 60, 20]
        ]

        #self.tra = [
        #    [87, -60, 20, 20], [87, 60, 20, 4]
        #]

        h = float(input('Input the search h: '))
        a = float(input('Input the length a: '))

        self.autoTra = AutoTra(h=h, a=a, pitchLevelOn=True, overlapOn=True, drawNum=-1, hfovPitchRatio=1.2)

        self.tra = self.autoTra.theList

        input('Type anything to continue...')

        self.traCnt = 0

        self.uavName = 'suav'
        self.deviceName = 'pod'

        rospy.init_node('pod_search', anonymous=True)
        self.startTime = self.getTimeNow()
        self.taskTime = 0
        self.rate = rospy.Rate(10)
        rospy.Subscriber(self.uavName + '/' + self.deviceName + '/pitch', Float32, lambda msg: setattr(self, 'pitch', msg.data))
        rospy.Subscriber(self.uavName + '/' + self.deviceName + '/yaw', Float32, lambda msg: setattr(self, 'yaw', msg.data))
        rospy.Subscriber(self.uavName + '/' + self.deviceName + '/hfov', Float32, lambda msg: setattr(self, 'hfov', msg.data))
        self.pitchPub = rospy.Publisher(
            self.uavName + '/' + self.deviceName + '/expectedPitch', Float32, queue_size=10)
        self.yawPub = rospy.Publisher(
            self.uavName + '/' + self.deviceName + '/expectedYaw', Float32, queue_size=10)
        self.hfovPub = rospy.Publisher(
            self.uavName + '/' + self.deviceName + '/expectedHfov', Float32, queue_size=10)
        self.maxRatePub = rospy.Publisher(
            self.uavName + '/' + self.deviceName + '/maxRate', Float32, queue_size=10)

        rospy.Subscriber(self.uavName + '/' + self.deviceName + '/pAtTarget', Bool, lambda msg: setattr(self, 'pAtTarget', msg.data))
        rospy.Subscriber(self.uavName + '/' + self.deviceName + '/yAtTarget', Bool, lambda msg: setattr(self, 'yAtTarget', msg.data))
        rospy.Subscriber(self.uavName + '/' + self.deviceName + '/fAtTarget', Bool, lambda msg: setattr(self, 'fAtTarget', msg.data))
        self.pAtTarget = False
        self.yAtTarget = False
        self.fAtTarget = False

        self.pFeedback = 0.0
        self.yFeedback = 0.0
        self.fFeedback = 0.0
        rospy.Subscriber(self.uavName + '/' + self.deviceName + '/pFeedback', Float32, lambda msg: setattr(self, 'pFeedback', msg.data))
        rospy.Subscriber(self.uavName + '/' + self.deviceName + '/yFeedback', Float32, lambda msg: setattr(self, 'yFeedback', msg.data))
        rospy.Subscriber(self.uavName + '/' + self.deviceName + '/fFeedback', Float32, lambda msg: setattr(self, 'fFeedback', msg.data))

        self.toTransformerPub = rospy.Publisher(
            self.uavName + '/' + self.deviceName + '/toTransformer', Bool, queue_size=10
        )

        rospy.Subscriber(self.uavName + '/' + self.deviceName + '/aim', Float64MultiArray, self.aimCallback)
        self.aimFailPub = rospy.Publisher(self.uavName + '/' + self.deviceName + '/aimFail', Int16, queue_size=10)
        rospy.Subscriber(self.uavName + '/' + self.deviceName + '/stream', Float64MultiArray, self.streamCallback)
        self.streamIndex = None
        self.streamPitch = 0
        self.streamYaw = 0
        self.streamFlag = False


        self.aimPitch = 0
        self.aimYaw = 0
        self.aimOn = False
        self.aimIndex = -1

        self.aimTimeThreshold = 10.0

        self.thisAimIndex = -1
        self.thisAimStartTime = 0

        self.classifierClearPub = rospy.Publisher(self.uavName + '/' + self.deviceName + '/classifierClear', Empty, queue_size=10)

        self.searchOverPub = rospy.Publisher(self.uavName + '/' + self.deviceName + '/searchOver', Empty, queue_size=10)
        self.uavReady = False

        self.endBeginTime = None

        self.browseTra = [
            [86, -10, 8],
            [86, -2, 8],
            [86, 6, 8],
            [86, 14, 8]
        ]
        self.browseCnt = 0
        self.browseBeginTime = None

        self.searchStatePub = rospy.Publisher(self.uavName + '/' + self.deviceName + '/searchState', Int8, queue_size=1)
        self.uavState = 0
        rospy.Subscriber(self.uavName + '/uavState', Int8, lambda msg: setattr(self, 'uavState', msg.data))

    def aimCallback(self, msg):
        if msg.data[0] > 0:
            self.aimOn = True
            self.aimPitch = msg.data[1]
            self.aimYaw = msg.data[2]
            self.aimIndex = int(msg.data[3])
        else:
            self.aimOn = False

    def streamCallback(self, msg):
        if msg.data[0] > 0:
            self.streamPitch = msg.data[1]
            self.streamYaw = msg.data[2]
            self.streamIndex = int(msg.data[3])

    def getTimeNow(self):
        return rospy.Time.now().to_sec()

    def isAtTarget(self):
        return (
                self.pAtTarget and
                self.yAtTarget and
                self.fAtTarget and
               # abs(self.pitch - self.expectedPitch) < 5 and
               # abs(self.yaw - self.expectedYaw) < 5 and
               # abs(self.hfov - self.expectedHfov) / self.expectedHfov < 0.25 and
                abs(self.pFeedback - self.expectedPitch) < 0.001 and
                abs(self.yFeedback - self.expectedYaw) < 0.001 and
                abs(self.fFeedback - self.expectedHfov) < 0.001
        )

    def toStepInit(self):
        self.state = State.INIT

    def toStepSearch(self):
        self.state = State.SEARCH

    def toStepAim(self):
        self.state = State.AIM
        self.thisAimIndex = self.aimIndex
        self.thisAimStartTime = self.getTimeNow()
        self.thisAimFinish = False

    def toStepEnd(self):
        self.state = State.END
        self.endBeginTime = self.getTimeNow()

    def toStepBrowse(self):
        self.state = State.BROWSE
        self.browseCnt = 0
        self.browseBeginTime = self.getTimeNow()
        self.browseFlag = False

    def toStepStream(self):
        self.state = State.STREAM
        if self.streamIndex == None:
            print(f'{RED}No targets to stream{RESET}')
            self.toStepEnd()
        self.streamStartTime = self.getTimeNow()

    def stepInit(self):
        self.expectedPitch = self.tra[0][0]
        self.expectedYaw = self.tra[0][1]
        self.expectedHfov = self.tra[0][2]
        self.maxRate = self.tra[0][3]
        self.pubPYZMaxRate()
        self.toTransformerPub.publish(Bool(False))
        self.classifierClearPub.publish(Empty())
        if self.isAtTarget() and self.uavReady:
            self.toStepSearch()

    def stepSearch(self):
        print(f'{GREEN}==> StepSearch @ {self.traCnt} <=={RESET}')
        self.expectedPitch = self.tra[self.traCnt][0]
        self.expectedYaw = self.tra[self.traCnt][1]
        self.expectedHfov = self.tra[self.traCnt][2]
        self.maxRate = self.tra[self.traCnt][3]
        self.pubPYZMaxRate()
        if self.isAtTarget():
            self.traCnt += 1
        self.toTransformerPub.publish(Bool(True))
        if self.traCnt == len(self.tra):
            self.toStepStream()
        # if self.aimOn:
        #     self.toStepAim()

    def stepAim(self):
        aimTime = self.getTimeNow() - self.thisAimStartTime
        if aimTime >= self.aimTimeThreshold and not self.thisAimFinish:
            self.aimFailPub.publish(Int16(self.thisAimIndex))
            self.thisAimFinish = True

        if not self.aimOn or self.aimIndex != self.thisAimIndex:
            self.thisAimFinish = True

        if self.thisAimFinish and self.isAtTarget():
            self.toStepSearch()
        
        self.expectedPitch = self.aimPitch
        self.expectedYaw = self.aimYaw
        self.expectedHfov = self.tra[self.traCnt][2] if self.thisAimFinish else self.tra[self.traCnt][2] / 2
        self.maxRate = 90 - self.aimPitch
        self.maxRate = 2
        self.pubPYZMaxRate()
        self.toTransformerPub.publish(Bool(True))
        
        print(f'{RED}==> StepAim @ Target {self.thisAimIndex} <=={RESET}')
        print(f'Time: {aimTime:.2f}',
              (RED + "Finished" + RESET) if self.thisAimFinish else "",
              (GREEN + "At Target" + RESET) if self.isAtTarget() else ""
        )
    
    def stepStream(self):
        if self.isAtTarget():
            self.streamFlag = True
        if not self.streamFlag:
            self.streamStartTime = self.getTimeNow()
        streamTime = self.getTimeNow() - self.streamStartTime
        print(f'{YELLOW}==> StepStream @ Target {self.streamIndex} <=={RESET}')
        print(f'Time: {streamTime:.2f}')
        print(f'StreamFlag: {self.streamFlag}')
        self.expectedPitch = self.streamPitch
        self.expectedYaw = self.streamYaw
        self.expectedHfov = 10
        self.maxRate = 20
        self.pubPYZMaxRate()
        if streamTime >= 10.0:
            self.toStepEnd()

    def stepEnd(self):
        self.searchOverPub.publish(Empty())
        endTime = self.getTimeNow() - self.endBeginTime
        print(f'StepEnd with {endTime:.2f} seconds')
        if endTime >= 3.0:
            exit(0)

    def stepBrowse(self):
        self.expectedPitch = self.browseTra[self.browseCnt][0]
        self.expectedYaw = self.browseTra[self.browseCnt][1]
        self.expectedHfov = self.browseTra[self.browseCnt][2]
        self.maxRate = 20
        print(f'{YELLOW}==> StepBrowse @ {self.browseCnt} <=={RESET}')
        print((f'{GREEN}Flag True{RESET}' if self.browseFlag else f'{RED}Flag False{RESET}'))
        print(f'Browse Time: {self.getTimeNow() - self.browseBeginTime:.2f}')
        self.pubPYZMaxRate()
        if self.isAtTarget():
            self.browseFlag = True
        if not self.browseFlag:
            self.browseBeginTime = self.getTimeNow()
        if self.getTimeNow() - self.browseBeginTime >= 3.0:
            self.browseCnt += 1
            self.browseFlag = False
        if self.browseCnt == len(self.browseTra):
            self.toStepEnd()

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
        elif self.state == State.AIM:
            self.stepAim()
        elif self.state == State.END:
            self.stepEnd()
        elif self.state == State.BROWSE:
            self.stepBrowse()
        elif self.state == State.STREAM:
            self.stepStream()
        else:
            print("Invalid state")

    def spin(self):
        while not rospy.is_shutdown():
            self.taskTime = self.getTimeNow() - self.startTime
            system('clear')
            print('-' * 20)
            print(f'### PodSearch ###')
            print(f'Me @ State #{self.state} sUAV @ State #{self.uavState}')
            #print(pyfiglet.figlet_format('PodSearch', font='slant'))
            print(
                f'Time {self.taskTime:.1f}',
                f'State: {self.state}',
                f'UAV Ready: {self.uavReady}',
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
    args, unknown = parser.parse_known_args()
    
    podSearch = PodSearch()

    print(f'--takeoff: {args.takeoff} --test: {args.test}')

    if not args.takeoff and not args.test:
        raise AssertionError("Please add --takeoff or --test arg")
    if args.takeoff and args.test:
        raise AssertionError("Not two args at the same time!")
    if args.test:
        podSearch.uavReady = True
        print('Set uavReady to True')
    elif args.takeoff:
        print('WILL TAKE OFF!!!')
    podSearch.spin()

