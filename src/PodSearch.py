#! /usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Bool
from time import time
from signal import signal, SIGINT


def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    exit(0)


signal(SIGINT, signal_handler)


# a state enum class
class State:
    INIT = 0
    SEARCH = 1
    AIM = 2


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
            [90 - 20, -55, 50, 20], [90 - 20, 55, 50, 8],
            [90 - 11, 78, 20, 20], [90 - 11, -78, 20, 5],
            [90 - 4, -83, 6, 20], [90 - 4, 83, 6, 1.5],
            [90 - 1, 42, 3, 20], [90 - 1, -42, 3, 1],
            [90, 0, 60, 20]
        ]
        self.traCnt = 0

        rospy.init_node('pod_search', anonymous=True)
        self.start_time = rospy.Time.now().to_sec()
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

        self.to_transformer_pub = rospy.Publisher(
            '/pod_comm/toTransformer', Bool, queue_size=10 
        )

    def pitch_callback(self, data):
        self.pitch = data.data

    def yaw_callback(self, data):
        self.yaw = data.data

    def hfov_callback(self, data):
        self.hfov = data.data

    def is_at_target(self):
        tol = 0.5
        hfov_tol = 3
        if abs(self.pitch - self.expected_pitch) < tol and \
                abs(self.yaw - self.expected_yaw) < tol and \
                abs(self.hfov - self.expected_hfov) < hfov_tol:
            return True
        else:
            return False

    def toStepInit(self):
        self.state = State.INIT

    def toStepSearch(self):
        self.state = State.SEARCH

    def toStepAim(self):
        self.state = State.AIM

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
        if self.traCnt < len(self.tra):
            self.expected_pitch = self.tra[self.traCnt][0]
            self.expected_yaw = self.tra[self.traCnt][1]
            self.expected_hfov = self.tra[self.traCnt][2]
            self.max_rate = self.tra[self.traCnt][3]
            self.pubPYZMaxRate()
            if self.is_at_target():
                self.traCnt += 1
            self.to_transformer_pub.publish(Bool(True))
        else:
            self.toStepAim()

    def stepAim(self):
        self.expected_pitch = 90
        self.expected_yaw = 0
        self.expected_hfov = 60
        self.max_rate = 20
        self.pubPYZMaxRate()
        self.to_transformer_pub.publish(Bool(False))
        pass

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
        else:
            print("Invalid state")

    def spin(self):
        while not rospy.is_shutdown():
            self.task_time = rospy.Time.now().to_sec() - self.start_time
            print('---------------------')
            print(f'Time {self.task_time:.1f} State: {self.state}')
            print(f'Pitch: {self.pitch:.2f} -> {self.expected_pitch:.2f}')
            print(f'Yaw: {self.yaw:.2f} -> {self.expected_yaw:.2f}')
            print(f'HFov: {self.hfov:.2f} -> {self.expected_hfov:.2f}')
            print(f'Is at target: {self.is_at_target()}')
            self.controlStateMachine()
            self.rate.sleep()


if __name__ == '__main__':
    pod_search = PodSearch()
    pod_search.spin()
