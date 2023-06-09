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
import tf

class Transformer:
    def __init__(self):
        self.uav_pitch = 0.0
        self.uav_yaw = 0.0
        self.uav_roll = 0.0

        self.pod_pitch = 0.0
        self.pod_yaw = 0.0
        self.pod_zoom = 0.0

        self.uav_name = 'M300'
        self.pod_name = 'pod_comm'

        rospy.Subscriber('/' + self.uav_name + '/imu', Imu, self.imu_callback)
        rospy.Subscriber('/' + self.pod_name + '/pitch', Float32, self.pitch_callback)
        rospy.Subscriber('/' + self.pod_name + '/yaw', Float32, self.yaw_callback)
        rospy.Subscriber('/' + self.pod_name + '/zoom', Float32, self.zoom_callback)

    def imu_callback(self, msg):
        orientation = msg.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.uav_roll = euler[0]
        self.uav_pitch = euler[1]
        self.uav_yaw = euler[2]

    def pitch_callback(self, msg):
        self.pod_pitch = msg.data

    def yaw_callback(self, msg):
        self.pod_yaw = msg.data

    def zoom_callback(self, msg):
        self.pod_zoom = msg.data

    def transform(self, pixel_x, pixel_y):
        # assume the image of pod is 16:9, and self.pod_zoom is horizontal field of view
        # first transfer to line of sight angle
        # then transfer to absolute angle
        # then transfer to absolute position

        pass


