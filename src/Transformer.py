#! /usr/bin/env python3

# Class Transformer: transform pixel error to absolute position
# using the onboard pod pitch, yaw, zoom and UAV pitch, yaw, roll
# assume the UAV is at (0, 0, h)

# get UAV orientation by subscribe to uav_name + '/imu' topic, sensor_msgs/Imu type
# and calculate by the quaternion to euler angles

# get pod pitch and yaw by subscribe to '/pod_comm/pitch' and '/pod_comm/yaw' topic, std_msgs/Float32 type

import argparse
import time
from collections import deque
from math import degrees, radians
from signal import signal, SIGINT

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Imu
from spirecv_msgs.msg import TargetsInFrame
from std_msgs.msg import Float32, Bool, Float64MultiArray, Int16

from Classifier import Classifier
from DataLogger import DataLogger
from ShowBar import ShowBar


def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    exit(0)


signal(SIGINT, signal_handler)


class TimeBuffer:
    def __init__(self, name='Buffer'):
        self.buffer = deque()
        self.name = name

    @property
    def empty(self):
        return not self.buffer

    def add_message(self, msg, max_age=0.5):
        time = rospy.Time.now()
        self.buffer.append((time, msg))

        oldest_time = time - rospy.Duration.from_sec(max_age)

        while self.buffer and self.buffer[0][0] < oldest_time:
            self.buffer.popleft()

    def get_message(self, time):
        if not self.buffer:
            return None

        current_time = rospy.Time.now()
        target_time = current_time - rospy.Duration.from_sec(time)

        return self.buffer[0][1]
        closest_time_diff = float('inf')
        closest_msg = None

        for b in self.buffer:
            time_diff = abs((b[0] - target_time).to_sec())
            if time_diff < closest_time_diff:
                closest_time_diff = time_diff
                closest_msg = b[1]

        closest_msg = None

        for b in self.buffer:
            time_diff = abs((b[0] - target_time).to_sec())
            if time_diff < closest_time_diff:
                closest_time_diff = time_dif
        return closest_msg

    def get_message_no_delay(self):
        if not self.buffer:
            return None
        return self.buffer[-1][1]

    def output_buffer(self):
        print(self.name + ': [')
        t = rospy.Time.now()
        for b in self.buffer:
            print('-', (t - b[0]).to_sec(), b[1])
        print(']')


class Transformer:
    def __init__(self, log_on=False):
        self.TRANSFORM_DEBUG = False
        self.order_from_searcher = False
        self.uav_quat = [0, 0, 0, 1]

        self.h = 1.6
        self.a = self.h / 100 * 3000
        self.self_pos = np.array([-self.h / 4, 0, self.h])

        self.pod_pitch_buffer = TimeBuffer('Pod Pitch Buffer')
        self.pod_yaw_buffer = TimeBuffer('Pod Yaw Buffer')

        self.pod_hfov_buffer = TimeBuffer('Pod HFov Buffer')

        self.uav_name = 'M300'
        self.pod_name = 'pod_comm'

        rospy.Subscriber('/' + self.uav_name + '/imu', Imu, self.imu_callback)
        rospy.Subscriber('/' + self.uav_name + '/pos', Odometry, self.pos_callback)
        rospy.Subscriber('/' + self.uav_name + '/height', Float32, self.h_callback)

        rospy.Subscriber('/' + self.pod_name + '/pitch', Float32, self.pitch_callback)
        rospy.Subscriber('/' + self.pod_name + '/yaw', Float32, self.yaw_callback)
        rospy.Subscriber('/' + self.pod_name + '/hfov', Float32, self.hfov_callback)

        rospy.Subscriber('/uav1/spirecv/common_object_detection', TargetsInFrame, self.targets_callback, queue_size=1)

        rospy.Subscriber('/' + self.pod_name + '/toTransformer', Bool, self.order_from_searcher_callback)

        self.clsfy = Classifier()

        self.log_on = log_on
        if self.log_on:
            self.dtlg = DataLogger("data.csv")

        self.start_time = rospy.Time.now().to_sec()

        self.targets_available = 30
        variable_info = [
                            ("rosTime", "double"),
                            ("podYaw", "double"),
                            ("podPitch", "double"),
                            ("podYawDelayed", "double"),
                            ("podPitchDelayed", "double"),
                            ("targetCnt", "int"),
                        ] + [
                            (f'target{i}[3]', "list") for i in range(self.targets_available)
                        ]

        if self.log_on:
            self.dtlg.initialize(variable_info)

        self.aim_pub = rospy.Publisher('/' + self.pod_name + '/aim', Float64MultiArray, queue_size=1)
        self.aimFailSub = rospy.Subscriber('/' + self.pod_name + '/aimFail', Int16, self.aim_fail_callback, queue_size=1)

    def aim_fail_callback(self, msg):
        self.clsfy.targetsCheck[msg.data] = True
        self.clsfy.targetsReal[msg.data] = False

    def order_from_searcher_callback(self, msg):
        self.order_from_searcher = msg.data

    def pos_callback(self, msg):
        self.self_pos[0] = msg.pose.pose.position.x
        self.self_pos[1] = msg.pose.pose.position.y

    def imu_callback(self, msg):
        orientation = msg.orientation
        self.uav_quat = [orientation.x, orientation.y, orientation.z, orientation.w]

    def pitch_callback(self, msg):
        msg.data = 90 - msg.data
        self.pod_pitch_buffer.add_message(msg)

    def yaw_callback(self, msg):
        self.pod_yaw_buffer.add_message(msg)

    def hfov_callback(self, msg):
        self.pod_hfov_buffer.add_message(msg)

    def h_callback(self, msg):
        self.self_pos[2] = msg.data

    def targets_callback(self, msg):
        # tic = rospy.Time.now().to_sec()
        for target in msg.targets:
            if target.category == 'car':
                self.transform(target.cx, target.cy, score=target.score)
        # toc = rospy.Time.now().to_sec()
        # print(f'Callback time {toc - tic}')

    def transform(self, pixel_x, pixel_y, score=1):
        if not self.order_from_searcher:
            return
        time_diff = 0.4
        try:
            pod_hfov = self.pod_hfov_buffer.get_message(time_diff).data
            pod_vfov = degrees(2 * np.arctan(np.tan(radians(pod_hfov) / 2) * 9 / 16))
            pod_yaw = self.pod_yaw_buffer.get_message(time_diff).data
            pod_pitch = self.pod_pitch_buffer.get_message(time_diff).data
        except Exception as e:
            print(e)
            return

        pixel_x = (pixel_x - 0.5) * 2
        pixel_y = (pixel_y - 0.5) * 2
        if self.TRANSFORM_DEBUG:
            ShowBar(-1, 1).show(pixel_x, str='PixelX')
            ShowBar(-1, 1).show(pixel_y, str='PixelY')
        cameraYaw = pixel_x * pod_hfov / 2
        cameraPitch = pixel_y * pod_vfov / 2
        if self.TRANSFORM_DEBUG:
            ShowBar(-pod_hfov / 2, pod_hfov / 2).show(cameraYaw, str='CameraYaw')
            ShowBar(-pod_vfov / 2, pod_vfov / 2).show(cameraPitch, str='CameraPitch')
        rCameraYaw = R.from_euler('z', -cameraYaw, degrees=True)
        rCameraPitch = R.from_euler('y', cameraPitch, degrees=True)
        rCamera = rCameraPitch * rCameraYaw
        rPodYaw = R.from_euler('z', -pod_yaw, degrees=True)
        rPodPitch = R.from_euler('y', pod_pitch, degrees=True)
        if self.TRANSFORM_DEBUG:
            ShowBar(-90, 90).show(pod_yaw, str='PodYaw')
            ShowBar(0, 90).show(pod_pitch, str='PodPitch')

        rUAV = R.from_quat(self.uav_quat)

        img_target = [1000, 0, 0]
        img_target_rel = (rUAV * rPodYaw * rPodPitch * rCamera).apply(img_target)
        # print(f'img_target: {img_target_rel}')

        real_target_rel = img_target_rel * self.self_pos[2] / (-img_target_rel[2])
        # print(f'real_target_rel: {real_target_rel}')

        real_target_abs = real_target_rel + self.self_pos
        # print((
        #     f'XY: ({pixel_x:.2f}, {pixel_y:.2f}) '
        #     f'cYP: ({cameraYaw:.2f}, {cameraPitch:.2f}) '
        #     f'pYP: ({pod_yaw:.2f}, {pod_pitch:.2f}) '
        #     f'Target @ {real_target_abs[0]:.2f}, {real_target_abs[1]:.2f}, {real_target_abs[2]:.2f} '
        #     f'Score {score:.2f}'
        # ))

        if self.TRANSFORM_DEBUG:
            print(f'-' * 20)

        if not self.out_of_bound(*real_target_abs):
            self.clsfy.newPos(*real_target_abs)
            # self.clsfy.outputTargets()

    def untransform(self, pos):
        # from pos and self.pos and self.uav_quat
        # cal the pod pitch and yaw to aim at pos
        # return pod_pitch, pod_yaw
        rUAV = R.from_quat(self.uav_quat)
        rUAV_inv = rUAV.inv()
        pos_rel = pos - self.self_pos
        target_body = rUAV_inv.apply(pos_rel)

        podPitch = np.arctan2(-target_body[2], np.sqrt(target_body[0] ** 2 + target_body[1] ** 2))
        podYaw = -np.arctan2(target_body[1], target_body[0])

        podPitch = 90 - np.degrees(podPitch)
        podYaw = np.degrees(podYaw)

        return podPitch, podYaw

    def out_of_bound(self, x, y, z):
        if x < 0 or x > self.a:
            return True
        if abs(y) > self.a / 2:
            return True
        return False

    def log(self):
        self.dtlg.log("rosTime", rospy.Time.now().to_sec() - self.start_time)
        self.dtlg.log("podYaw", self.pod_yaw_buffer.get_message_no_delay().data)
        self.dtlg.log("podPitch", self.pod_pitch_buffer.get_message_no_delay().data)
        self.dtlg.log("podYawDelayed", self.pod_yaw_buffer.get_message(0.5).data)
        self.dtlg.log("podPitchDelayed", self.pod_pitch_buffer.get_message(0.5).data)
        t = self.clsfy.targetsList()
        self.dtlg.log("targetCnt", len(t))
        for i in range(self.targets_available):
            if i < len(t):
                self.dtlg.log(f'target{i}', t[i])
            else:
                self.dtlg.log(f'target{i}', [-1, -1, -1])
        self.dtlg.newline()

    def spin(self):
        while not rospy.is_shutdown():
            if self.log_on and not self.pod_yaw_buffer.empty and not self.pod_pitch_buffer.empty:
                self.log()

            t = self.clsfy.firstNotChecked()
            if t is not None:
                aimPitch, aimYaw = self.untransform(self.clsfy.targets[t])
                msg = Float64MultiArray(data=[1, aimPitch, aimYaw, t])
                print(f'Aim @ Target [{t}] Pitch {aimPitch:.2f}, Yaw {aimYaw:.2f}')
                self.aim_pub.publish(msg)
            else:
                msg = Float64MultiArray(data=[-1, -1, -1, -1])
                self.aim_pub.publish(msg)

            print('-' * 20)
            t = self.clsfy.targets
            tLen = len(t)
            tCnt = self.clsfy.targetsCnt
            tCheck = self.clsfy.targetsCheck
            tReal = self.clsfy.targetsReal
            for i in range(tLen):
                print(f'Target[{i}] @ {[f"{x:.2f}" for x in t[i]]}, {tCnt[i]}, {tCheck[i]} {tReal[i]}')

            time.sleep(0.05)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--log', help='turn on log or not', action="store_true")
    args, unknown = parser.parse_known_args()

    rospy.init_node('Transformer', anonymous=True)
    # ipt = input('If debug? (y/n): ')
    t = Transformer(args.log)
    time.sleep(1)

    t.spin()
    # if ipt == 'y':
    #     t.TRANSFORM_DEBUG = True
