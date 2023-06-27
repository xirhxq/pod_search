#! /usr/bin/env python3

from os import system
from signal import signal, SIGINT

import pyfiglet
import rospy
from std_msgs.msg import Float32, Bool, Int16, Float64MultiArray

from Utils import *

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    exit(0)


signal(SIGINT, signal_handler)


class State:
    INIT = 0
    SEARCH = 1
    AIM = 2
    END = 3


class PodSearch:
    def __init__(self):
        self.state = State.INIT

        self.pitch = 0.0
        self.yaw = 0.0
        self.hfov = 0.0

        self.expectedPitch = 90
        self.expectedYaw = 0
        self.expectedHfov = 60
        self.maxRate = 20

        self.tra = [
            [90 - 30, -55, 60, 10], [90 - 30, 55, 60, 10],
            [90 - 11, 78, 20, 7], [90 - 11, -78, 20, 7],
            [90 - 4, -83, 10, 3], [90 - 4, 83, 10, 3],
            [90, 0, 60, 20]
        ]

        # test tra
        self.tra = [
            [90 - 20, -55, 50, 20], [90 - 20, 55, 50, 6],
            [90 - 11, 78, 20, 20], [90 - 11, -78, 20, 4],
            [90 - 4, -83, 6, 20], [90 - 4, 83, 6, 1.5],
            [90 - 1, 42, 3, 20], [90 - 1, -42, 3, 1],
            [90, 0, 60, 20]
        ]
        self.traCnt = 0

        rospy.init_node('pod_search', anonymous=True)
        self.startTime = self.getTimeNow()
        self.taskTime = 0
        self.rate = rospy.Rate(10)
        rospy.Subscriber('/pod_comm/pitch', Float32, lambda msg: setattr(self, 'pitch', msg.data))
        rospy.Subscriber('/pod_comm/yaw', Float32, lambda msg: setattr(self, 'yaw', msg.data))
        rospy.Subscriber('/pod_comm/hfov', Float32, lambda msg: setattr(self, 'hfov', msg.data)) 
        self.pitchPub = rospy.Publisher(
            '/pod_comm/expectedPitch', Float32, queue_size=10)
        self.yawPub = rospy.Publisher(
            '/pod_comm/expectedYaw', Float32, queue_size=10)
        self.hfovPub = rospy.Publisher(
            '/pod_comm/expectedHfov', Float32, queue_size=10)
        self.maxRatePub = rospy.Publisher(
            '/pod_comm/maxRate', Float32, queue_size=10)

        rospy.Subscriber('/pod_comm/pAtTarget', Bool, lambda msg: setattr(self, 'pAtTarget', msg.data))
        rospy.Subscriber('/pod_comm/yAtTarget', Bool, lambda msg: setattr(self, 'yAtTarget', msg.data))
        rospy.Subscriber('/pod_comm/fAtTarget', Bool, lambda msg: setattr(self, 'fAtTarget', msg.data))
        self.pAtTarget = False
        self.yAtTarget = False
        self.fAtTarget = False

        self.pFeedback = 0.0
        self.yFeedback = 0.0
        self.fFeedback = 0.0
        rospy.Subscriber('/pod_comm/pFeedback', Float32, lambda msg: setattr(self, 'pFeedback', msg.data))
        rospy.Subscriber('/pod_comm/yFeedback', Float32, lambda msg: setattr(self, 'yFeedback', msg.data))
        rospy.Subscriber('/pod_comm/fFeedback', Float32, lambda msg: setattr(self, 'fFeedback', msg.data))

        self.toTransformerPub = rospy.Publisher(
            '/pod_comm/toTransformer', Bool, queue_size=10
        )

        rospy.Subscriber('/pod_comm/aim', Float64MultiArray, self.aimCallback)
        self.aimFailPub = rospy.Publisher('/pod_comm/aimFail', Int16, queue_size=10)
        self.aimPitch = 0
        self.aimYaw = 0
        self.aimOn = False
        self.aimIndex = -1

        self.thisAimIndex = -1
        self.thisAimStartTime = 0


    def aimCallback(self, msg):
        if msg.data[0] > 0:
            self.aimOn = True
            self.aimPitch = msg.data[1]
            self.aimYaw = msg.data[2]
            self.aimIndex = int(msg.data[3])
        else:
            self.aimOn = False

    def getTimeNow(self):
        return rospy.Time.now().to_sec()

    def isAtTarget(self):
        return (
            self.pAtTarget and 
            self.yAtTarget and 
            self.fAtTarget and 
            self.pFeedback == self.expectedPitch and
            self.yFeedback == self.expectedYaw and
            self.fFeedback == self.expectedHfov
        )

    def toStepInit(self):
        self.state = State.INIT

    def toStepSearch(self):
        self.state = State.SEARCH

    def toStepAim(self):
        self.state = State.AIM
        self.thisAimIndex = self.aimIndex
        self.thisAimStartTime = self.getTimeNow()

    def toStepEnd(self):
        self.state = State.END

    def stepInit(self):
        self.expectedPitch = 90
        self.expectedYaw = 0
        self.expectedHfov = 60
        self.maxRate = 20
        self.pubPYZMaxRate()
        self.toTransformerPub.publish(Bool(False))
        if self.isAtTarget():
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
            self.toStepEnd()
        if self.aimOn:
            self.toStepAim()

    def stepAim(self):
        print(f'{RED}==> StepAim @ Target {self.thisAimIndex} <=={RESET}')
        print(f'Time: {self.getTimeNow() - self.thisAimStartTime}')
        #if not self.isAtTarget():
        #    self.thisAimStartTime = self.getTimeNow()
        self.expectedPitch = self.aimPitch
        self.expectedYaw = self.aimYaw
        self.expectedHfov = 90 - self.aimPitch
        self.maxRate = 20
        self.pubPYZMaxRate()
        self.toTransformerPub.publish(Bool(True))
        if self.getTimeNow() - self.thisAimStartTime >= 10.0:
            self.aimFailPub.publish(Int16(self.thisAimIndex))
            self.toStepSearch()
        if not self.aimOn:
            self.toStepSearch()
        if self.aimIndex != self.thisAimIndex:
            self.toStepSearch()

    def pubPYZMaxRate(self):
        self.pitchPub.publish(self.expectedPitch)
        self.yawPub.publish(self.expectedYaw)
        self.hfovPub.publish(self.expectedHfov)
        self.maxRatePub.publish(self.maxRate)

    def controlStateMachine(self):
        if self.state == State.INIT:
            self.stepInit()
        elif self.state == State.SEARCH:
            self.stepSearch()
        elif self.state == State.AIM:
            self.stepAim()
        elif self.state == State.END:
            exit(0)
        else:
            print("Invalid state")

    def spin(self):
        while not rospy.is_shutdown():
            self.taskTime = self.getTimeNow() - self.startTime
            system('clear')
            print('-' * 20)
            print(pyfiglet.figlet_format('PodSearch', font='slant'))
            print(f'Time {self.taskTime:.1f} State: {self.state}')
            print(GREEN if self.pAtTarget else RED, end='')
            print(f'Pitch: {self.pitch:.2f} -> {self.expectedPitch:.2f} == {self.pFeedback:.2f}{RESET}')
            print(GREEN if self.yAtTarget else RED, end='')
            print(f'Yaw: {self.yaw:.2f} -> {self.expectedYaw:.2f} == {self.yFeedback:.2f}{RESET}')
            print(GREEN if self.fAtTarget else RED, end='')
            print(f'HFov: {self.hfov:.2f} -> {self.expectedHfov:.2f} == {self.fFeedback:.2f} {RESET}')
            print(f'Is at target: {self.isAtTarget()}')
            print(f'Aim on: {self.aimOn}')
            print(f'Aim index: {self.aimIndex}')
            self.controlStateMachine()
            self.rate.sleep()


if __name__ == '__main__':
    podSearch = PodSearch()
    podSearch.spin()
