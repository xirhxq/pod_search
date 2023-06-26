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


# a state enum class
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

        self.expected_pitch = 90
        self.expected_yaw = 0
        self.expected_hfov = 60
        self.max_rate = 20

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
        self.start_time = self.getTimeNow()
        self.task_time = 0
        self.rate = rospy.Rate(10)
        rospy.Subscriber('/pod_comm/pitch', Float32, self.pitch_callback)
        rospy.Subscriber('/pod_comm/yaw', Float32, self.yaw_callback)
        rospy.Subscriber('/pod_comm/hfov', Float32, self.hfov_callback)
        self.pitch_pub = rospy.Publisher(
            '/pod_comm/expected_pitch', Float32, queue_size=10)
        self.yaw_pub = rospy.Publisher(
            '/pod_comm/expected_yaw', Float32, queue_size=10)
        self.hfov_pub = rospy.Publisher(
            '/pod_comm/expected_hfov', Float32, queue_size=10)
        self.max_rate_pub = rospy.Publisher(
            '/pod_comm/max_rate', Float32, queue_size=10)

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

        self.to_transformer_pub = rospy.Publisher(
            '/pod_comm/toTransformer', Bool, queue_size=10
        )

        rospy.Subscriber('/pod_comm/aim', Float64MultiArray, self.aim_callback)
        self.aimFailPub = rospy.Publisher('/pod_comm/aimFail', Int16, queue_size=10)
        self.aimPitch = 0
        self.aimYaw = 0
        self.aimOn = False
        self.aimIndex = -1

        self.thisAimIndex = -1
        self.thisAimStartTime = 0


    def aim_callback(self, data):
        if data.data[0] > 0:
            self.aimOn = True
            self.aimPitch = data.data[1]
            self.aimYaw = data.data[2]
            self.aimIndex = int(data.data[3])
        else:
            self.aimOn = False

    def pitch_callback(self, data):
        self.pitch = data.data

    def yaw_callback(self, data):
        self.yaw = data.data

    def hfov_callback(self, data):
        self.hfov = data.data

    def getTimeNow(self):
        return rospy.Time.now().to_sec()

    def is_at_target(self):
        return (
            self.pAtTarget and 
            self.yAtTarget and 
            self.fAtTarget and 
            self.pFeedback == self.expected_pitch and
            self.yFeedback == self.expected_yaw and
            self.fFeedback == self.expected_hfov
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
        self.expected_pitch = 90
        self.expected_yaw = 0
        self.expected_hfov = 60
        self.max_rate = 20
        self.pubPYZMaxRate()
        self.to_transformer_pub.publish(Bool(False))
        if self.is_at_target():
            self.toStepSearch()

    def stepSearch(self):
        print(f'{GREEN}==> StepSearch @ {self.traCnt} <=={RESET}')
        self.expected_pitch = self.tra[self.traCnt][0]
        self.expected_yaw = self.tra[self.traCnt][1]
        self.expected_hfov = self.tra[self.traCnt][2]
        self.max_rate = self.tra[self.traCnt][3]
        self.pubPYZMaxRate()
        if self.is_at_target():
            self.traCnt += 1
        self.to_transformer_pub.publish(Bool(True))
        if self.traCnt == len(self.tra):
            self.toStepEnd()
        if self.aimOn:
            self.toStepAim()

    def stepAim(self):
        print(f'{RED}==> StepAim @ Target {self.thisAimIndex} <=={RESET}')
        print(f'Time: {self.getTimeNow() - self.thisAimStartTime}')
        #if not self.is_at_target():
        #    self.thisAimStartTime = self.getTimeNow()
        self.expected_pitch = self.aimPitch
        self.expected_yaw = self.aimYaw
        self.expected_hfov = 20
        self.max_rate = 20
        self.pubPYZMaxRate()
        self.to_transformer_pub.publish(Bool(True))
        if self.getTimeNow() - self.thisAimStartTime >= 10.0:
            self.aimFailPub.publish(Int16(self.thisAimIndex))
            self.toStepSearch()
        if not self.aimOn:
            self.toStepSearch()
        if self.aimIndex != self.thisAimIndex:
            self.toStepSearch()

    def pubPYZMaxRate(self):
        self.pitch_pub.publish(self.expected_pitch)
        self.yaw_pub.publish(self.expected_yaw)
        self.hfov_pub.publish(self.expected_hfov)
        self.max_rate_pub.publish(self.max_rate)

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
            self.task_time = self.getTimeNow() - self.start_time
            system('clear')
            print('-' * 20)
            print(pyfiglet.figlet_format('PodSearch', font='slant'))
            print(f'Time {self.task_time:.1f} State: {self.state}')
            print(GREEN if self.pAtTarget else RED, end='')
            print(f'Pitch: {self.pitch:.2f} -> {self.expected_pitch:.2f} == {self.pFeedback}{RESET}')
            print(GREEN if self.yAtTarget else RED, end='')
            print(f'Yaw: {self.yaw:.2f} -> {self.expected_yaw:.2f} == {self.yFeedback}{RESET}')
            print(GREEN if self.fAtTarget else RED, end='')
            print(f'HFov: {self.hfov:.2f} -> {self.expected_hfov:.2f} == {self.fFeedback} {RESET}')
            print(f'Is at target: {self.is_at_target()}')
            print(f'Aim on: {self.aimOn}')
            print(f'Aim index: {self.aimIndex}')
            self.controlStateMachine()
            self.rate.sleep()


if __name__ == '__main__':
    pod_search = PodSearch()
    pod_search.spin()
