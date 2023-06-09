#! /usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from time import time

from signal import signal, SIGINT

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    exit(0)

signal(SIGINT, signal_handler)

class PodSearch:
    def __init__(self):
        self.pitch = 0.0
        self.yaw = 0.0
        self.zoom = 0.0
        self.tra = [
            [90 - 30, -55, 60, 20], [90 - 30, 55, 60, 20], 
            [90 - 11, 78, 20, 7], [90 - 11, -78, 20, 7],
            [90 - 4, -83, 10, 3], [90 - 4, 83, 10, 3],
            [90, 0, 60, 20]
        ]
        self.start_time = time()
        rospy.init_node('pod_search', anonymous=True)
        self.rate = rospy.Rate(10)
        rospy.Subscriber('/pod_comm/pitch', Float32, self.pitch_callback)
        rospy.Subscriber('/pod_comm/yaw', Float32, self.yaw_callback)
        rospy.Subscriber('/pod_comm/zoom', Float32, self.zoom_callback)
        self.pitch_pub = rospy.Publisher(
            '/pod_comm/expected_pitch', Float32, queue_size=10)
        self.yaw_pub = rospy.Publisher(
            '/pod_comm/expected_yaw', Float32, queue_size=10)
        self.zoom_pub = rospy.Publisher(
            '/pod_comm/expected_zoom', Float32, queue_size=10)
        self.max_rate_pub = rospy.Publisher(
            '/pod_comm/max_rate', Float32, queue_size=10)

    def pitch_callback(self, data):
        self.pitch = data.data

    def yaw_callback(self, data):
        self.yaw = data.data

    def zoom_callback(self, data):
        self.zoom = data.data

    def is_at_target(self):
        tol = 0.5
        ztol = 1
        if abs(self.pitch - self.expected_pitch) < tol and \
            abs(self.yaw - self.expected_yaw) < tol and \
            abs(self.zoom - self.expected_zoom) < ztol:
            return True

    def spin(self):
        while not rospy.is_shutdown():
            for target in self.tra:
                self.expected_pitch = target[0]
                self.expected_yaw = target[1]
                self.expected_zoom = target[2]
                self.max_rate = target[3]
                while not self.is_at_target():
                    print('---------------------')
                    print(f'Time: {time() - self.start_time:.2f}')
                    print(f'Pitch: {self.pitch:.2f} -> {self.expected_pitch:.2f}')
                    print(f'Yaw: {self.yaw:.2f} -> {self.expected_yaw:.2f}')
                    print(f'Zoom: {self.zoom:.2f} -> {self.expected_zoom:.2f}')
                    # print(f'Target: {self.expected_pitch}, {self.expected_yaw}, {self.expected_zoom}')
                    self.pitch_pub.publish(self.expected_pitch)
                    self.yaw_pub.publish(self.expected_yaw)
                    self.zoom_pub.publish(self.expected_zoom)
                    self.max_rate_pub.publish(self.max_rate)
                    self.rate.sleep()
            break


if __name__ == '__main__':
    pod_search = PodSearch()
    pod_search.spin()
