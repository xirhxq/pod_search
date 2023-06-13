#! /usr/bin/env python3

# Class Transformer: transform pixel error to absolute position
# using the onboard pod pitch, yaw, zoom and UAV pitch, yaw, roll
# assume the UAV is at (0, 0, h)

# get UAV orientation by subscribe to uav_name + '/imu' topic, sensor_msgs/Imu type
# and calculate by the quaternion to euler angles

# get pod pitch and yaw by subscribe to '/pod_comm/pitch' and '/pod_comm/yaw' topic, std_msgs/Float32 type

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from scipy.spatial.transform import Rotation as R
import numpy as np
from math import atan, tan, degrees, radians
from nav_msgs.msg import Odometry
from ShowBar import ShowBar
from spirecv_msgs.msg import TargetsInFrame, Target, ROI
from collections import deque


class TimeBuffer:
    def __init__(self, name='Buffer'):
        self.buffer = deque()
        self.name = name

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
        closet_time_diff = float('inf')
        closet_msg = None

        for b in self.buffer:
            time_diff = abs((b[0] - target_time).to_sec())
            if time_diff < closet_time_diff:
                closet_time_diff = time_diff
                closet_msg = b[1]

        closet_msg = None

        for b in self.buffer:
            time_diff = abs((b[0] - target_time).to_sec())
            if time_diff < closet_time_diff:
                closet_time_diff = time_dif
        return closet_msg

    def output_buffer(self):
        print(self.name + ': [')
        t = rospy.Time.now()
        for b in self.buffer:
            print('-', (t - b[0]).to_sec(), b[1])
        print(']')
            

class Transformer:
    def __init__(self):
        self.TRANSFORM_DEBUG = False
        self.uav_quat = [0, 0, 0, 1]

        self.self_pos = [0, 0, 1.6]

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
                self.transform(target.cx, target.cy)
        # toc = rospy.Time.now().to_sec()
        # print(f'Callback time {toc - tic}')

    def transform(self, pixel_x, pixel_y):
        time_diff = 0.4
        try:
            pod_hfov = self.pod_hfov_buffer.get_message(time_diff).data
            pod_vfov = degrees(2 * np.arctan(np.tan(radians(pod_hfov) / 2) * 9 / 16))
            pod_yaw, pod_pitch = self.pod_yaw_buffer.get_message(time_diff).data, self.pod_pitch_buffer.get_message(time_diff).data
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
        print(f'pixelxy: ({pixel_x:.2f}, {pixel_y:.2f}) camera: ({cameraYaw:.2f}, {cameraPitch:.2f}) pod: ({pod_yaw:.2f}, {pod_pitch:.2f}) target @ {real_target_abs}')

        if self.TRANSFORM_DEBUG:
            print(f'-' * 20)


if __name__ == '__main__':
    rospy.init_node('Transformer', anonymous=True)
    ipt = input('If debug? (y/n): ')
    t = Transformer()
    if ipt == 'y':
        t.TRANSFORM_DEBUG = True
    while not rospy.is_shutdown():
        try: 
            # t.pod_pitch_buffer.output_buffer()
            # t.pod_yaw_buffer.output_buffer()
            # t.pod_hfov_buffer.output_buffer()
            #x, y = map(float, input('input pixel x and y: ').split())
            #t.transform(x, y)
            pass
        except Exception as e:
            print(e)
