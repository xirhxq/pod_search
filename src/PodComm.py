#! /usr/bin/env python3

import serial
import threading
from time import time, sleep
from struct import pack, unpack
from sys import platform, exit
from os import system
import rospy
from std_msgs.msg import String, Float32
from math import tan, degrees, radians, atan
from collections import deque
from datetime import datetime, timedelta
import pyfiglet

HZ = 50

DOWN_FRAME_HEAD_1 = b'\xAA'
DOWN_FRAME_HEAD_2 = b'\x55'
DOWN_FRAME_HEAD = DOWN_FRAME_HEAD_1 + DOWN_FRAME_HEAD_2
UP_FRAME_HEAD = b'\xEB\x90'
FRAME_LEN = 48

B_ANY, B_S8, B_U8, B_S16, B_U16, B_S32, B_U32, B_F = 'x', 'b', 'B', 'h', 'H', 'i', 'I', 'f' 
DOWN_PROTO = '<' + B_U8 * 2 + B_U16 + B_S16 + B_U16 + B_ANY * 29 + B_U8 + B_ANY * 2 + B_U16 + B_U8 + B_ANY * 2 + B_U8 + B_U8 * 2

WAITING_DOWN_FRAME_HEAD_1 = 1
WAITING_DOWN_FRAME_HEAD_2 = 2
READING_DATA = 3

from glob import glob
PORT = glob('/dev/ttyUSB[0-9]*')[0]

from signal import signal, SIGINT

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    exit(0)

signal(SIGINT, signal_handler)

SECRET_DICT = {
    1: 140, 2: 160, 3: 180, 5: 220, 7: 250, 10: 290, 20: 355
}

def secret_interp(x, data=SECRET_DICT): 
    sorted_data = sorted(data.items())
    x_val, y_val = zip(*sorted_data)
    if x <= x_val[0]:
        return y_val[0]
    if x >= x_val[-1]:
        return y_val[-1]
    
    index = 0
    while x > x_val[index]:
        index += 1
    
    x1, y1 = x_val[index - 1], y_val[index - 1]
    x2, y2 = x_val[index], y_val[index]

    y = y1 + (x - x1) * (y2 - y1) / (x2 - x1)
    return y

class UP_MSG:
    def __init__(self):
        self.order_A = UP_FRAME_HEAD
        self.order_B = b'\xFF'
        self.order_C = b'\x00\x00'
        self.order_D = b'\x00\x00\x00\x00'
        self.order_E = b'\x00' * 45
    
    def msg(self):
        msg = self.order_A + self.order_B + self.order_C + self.order_D + self.order_E
        assert len(msg) == 54
        check_sum = sum(msg) & 0xFF
        return msg + pack('<B', check_sum)
    
    def zoom_down(self):
        self.order_B = b'\x48'
        return self.msg()
    
    def zoom_up(self):
        self.order_B = b'\x44'
        return self.msg()

    def text_onoff(self):
        self.order_B = b'\x93'
        return self.msg()

    def change_zoom_level(self, level):
        self.order_B = b'\xDC'
        self.order_C = pack('<h', int(level))
        print(f'change to {level}')
        return self.msg()

    def manual_py_rate(self, prate, yrate):
        self.order_B = b'\x25'
        self.order_C = pack('<h', int(prate))
        self.order_D = pack('<h', int(yrate)) + b'\x00\x00'
        return self.msg()


def timer(tol=1):
    def decorator(func):
        def wrapper(*args, **kwargs):
            start_time = time()
            result = func(*args, **kwargs)
            end_time = time()
            # print(f'Time elap: {end_time - start_time:.2f}')
            if end_time - start_time < tol:
                sleep(tol - end_time + start_time)
            # print(f'Ended')
            return result
        return wrapper
    return decorator

class POD_COMM:
    def __init__(self):
        self.init = False
        self.state = WAITING_DOWN_FRAME_HEAD_1
        self.expected_pitch = 90.0
        self.expected_yaw = 0.0
        self.expected_zoom = 4.3
        self.expected_zoom_level = 1
        self.start_time = time()
        self.lazy_tag = 0
        self.SENSOR_WIDTH = tan(radians(2.3) / 2) * 2 * 129

        self.pod_state_0 = 0x00
        self.pod_state_1 = 0x00
        self.pod_f = 4.3
        self.pod_zoom_level = 1
        self.pod_pitch = 0.0
        self.pod_yaw = 0.0
        self.pod_camera_state = 0x10
        self.pod_laser_res = 0
        self.pod_elec_zoom = 0
        self.pod_order = 0

        self.pod_yaw_deque = deque()
        self.pod_pitch_deque = deque()
        self.time_interval = timedelta(seconds=1)
        self.pod_yaw_v = 0
        self.pod_pitch_v = 0

        self.check_sum_right_cnt = 0
        self.check_sum_wrong_cnt = 0

        self.max_rate = 300
        self.zoom_unit = 4.3

        self.down_ser = serial.Serial(
            port=PORT,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0.5
        )
        self.data_buf = bytearray()

        # subscribe to rostopic '/pod_comm/expected_pitch_yaw'
        rospy.init_node('pod_comm', anonymous=True)
        rospy.Rate(10)
        rospy.Subscriber('/pod_comm/expected_pitch', Float32, self.p_callback)
        rospy.Subscriber('/pod_comm/expected_yaw', Float32, self.y_callback)
        rospy.Subscriber('/pod_comm/expected_hfov', Float32, self.hfov_callback)
        rospy.Subscriber('/pod_comm/expected_pitch_yaw', String, self.py_callback)
        rospy.Subscriber('/pod_comm/max_rate', Float32, self.max_rate_callback)
        self.pitch_pub = rospy.Publisher('/pod_comm/pitch', Float32, queue_size=10)
        self.yaw_pub = rospy.Publisher('/pod_comm/yaw', Float32, queue_size=10)
        self.hfov_pub = rospy.Publisher('/pod_comm/hfov', Float32, queue_size=10)

    def loose_zoom_level(self, z):
        return z

    def get_time_now(self):
        return time()

    def get_f(self, half_angle):
        return self.SENSOR_WIDTH / 2 / tan(radians(half_angle / 2))

    def get_hfov(self, f):
        return degrees(atan(self.SENSOR_WIDTH / 2 / f)) * 2

    def round(self, val, base):
        if val > base:
            return val - base * 2
        elif val < -base:
            return val + base * 2
        else:
            return val
    
    def gen_up_msg(self):
        up = UP_MSG()

        if self.init == False:
            up.text_onoff()
            self.init = True
        elif self.lazy_tag <= 15:
            pitch_diff = self.expected_pitch - self.pod_pitch
            yaw_diff = self.round(self.expected_yaw - self.pod_yaw, 180)
            abs_zoom_diff = (self.expected_zoom - self.pod_f)
            rel_zoom_diff = abs_zoom_diff / self.expected_zoom
            py_tol = 0.5
            z_tol = 0.05


            if self.expected_zoom_level != self.pod_zoom_level and self.lazy_tag == 0 and abs(rel_zoom_diff) > 0.1:
                # print(f'change zoom level {self.pod_zoom_level} to {self.expected_zoom_level}')
                up.change_zoom_level(self.expected_zoom_level)
                self.lazy_tag = 150

            elif rel_zoom_diff < -z_tol:
                # print(f'up decrease zoom a bit')
                up.zoom_down()
                if self.lazy_tag == 0:
                    self.lazy_tag = 20
                
            elif rel_zoom_diff > z_tol:
                # print(f'up increase zoom a bit')
                up.zoom_up()
                if self.lazy_tag == 0:
                    self.lazy_tag = 20

            elif  pitch_diff < -py_tol or pitch_diff > py_tol or yaw_diff < -py_tol or yaw_diff > py_tol:
                pr_max, yr_max = 300, self.max_rate
                prate = max(-pr_max, min(pr_max, pitch_diff * 200))
                yrate = max(-yr_max, min(yr_max, yaw_diff * 300))

                up.manual_py_rate(prate, yrate)



                # print(f'up pitch {self.pod_pitch:.2f} -> {self.expected_pitch:.2f} diff: {pitch_diff:.2f} rate: {prate:.2f}')
                # print(f'up yaw {self.pod_yaw:.2f} -> {self.expected_yaw:.2f} diff: {yaw_diff:.2f} rate: {yrate:.2f}')
        if self.lazy_tag > 0:
            self.lazy_tag -= 1
        

        return up.msg()



    def read_data(self):
        while True:
            data = self.down_ser.read(1)
            if data:
                if self.state == WAITING_DOWN_FRAME_HEAD_1:
                    if data == DOWN_FRAME_HEAD_1:
                        self.state = WAITING_DOWN_FRAME_HEAD_2
                elif self.state == WAITING_DOWN_FRAME_HEAD_2:
                    if data == DOWN_FRAME_HEAD_2:
                        self.state = READING_DATA
                        data_buf = bytearray()
                elif self.state == READING_DATA:
                    # print(f'reading data buffer len {len(data_buf)}')
                    data_buf.append(data[0])
                    if len(data_buf) == FRAME_LEN:
                        # print('down: ', DOWN_FRAME_HEAD.hex(), data_buf[:-2].hex(), data_buf[-2:].hex())
                        down_data = unpack(DOWN_PROTO, data_buf)
                        check_sum = down_data[-1] * 0x100 + down_data[-2]
                        if check_sum != sum(data_buf[:-2]):
                            # print('Checksum error')
                            # print('Checksum: ', f'{check_sum:02x}', 'Realsum: ', f'{sum(data_buf[:-2]):02x}')
                            self.check_sum_wrong_cnt += 1
                            # raise AssertionError
                        else:
                            self.check_sum_right_cnt += 1
                            self.pod_state_0, self.pod_state_1, pod_f_x10, pod_pitch_x100, pod_yaw_x100, self.pod_camera_state, self.pod_laser_res, self.pod_elec_zoom, self.pod_order = down_data[:-2]
                            self.pod_f = pod_f_x10 / 10
                            self.pod_pitch = pod_pitch_x100 / 100
                            self.pod_yaw = self.round(pod_yaw_x100 / 100, 180)
                            self.pod_zoom_level = self.loose_zoom_level(round(self.pod_f / self.zoom_unit))

                            current_time = datetime.now()
                            self.pod_yaw_deque.append((current_time, self.pod_yaw))
                            self.pod_pitch_deque.append((current_time, self.pod_pitch))

                            while len(self.pod_yaw_deque) > 0 and self.pod_yaw_deque[0][0] < current_time - self.time_interval:
                                # print(f'pop {self.pod_yaw_deque[0]}')
                                self.pod_yaw_deque.popleft()

                            while len(self.pod_pitch_deque) > 0 and self.pod_pitch_deque[0][0] < current_time - self.time_interval:
                                self.pod_pitch_deque.popleft()

                            if len(self.pod_yaw_deque) > 1:
                                self.pod_yaw_v = self.round(self.pod_yaw_deque[-1][1] - self.pod_yaw_deque[0][1], 180) / (self.pod_yaw_deque[-1][0] - self.pod_yaw_deque[0][0]).total_seconds()

                            if len(self.pod_pitch_deque) > 1:
                                self.pod_pitch_v = (self.pod_pitch_deque[-1][1] - self.pod_pitch_deque[0][1])/ (self.pod_pitch_deque[-1][0] - self.pod_pitch_deque[0][0]).total_seconds()


                            # print(f'Received state {self.pod_state_0}, camera state {self.pod_camera_state}, zoom {self.pod_f}, pitch {self.pod_pitch}, yaw {self.pod_yaw}')
                        # print(f'down zoom: {self.pod_f:.2f} pitch: {self.pod_pitch:.2f} yaw: {self.pod_yaw:.2f}')
                        self.state = WAITING_DOWN_FRAME_HEAD_1

            if self.state == WAITING_DOWN_FRAME_HEAD_1:
                data_buf = bytearray()
    
    def start_read(self):
        t_read = threading.Thread(target=self.read_data)
        t_read.start()

    @timer(tol=1 / HZ)
    def write_once(self):
        up_msg = self.gen_up_msg()
        # print('------------------------------')
        self.down_ser.write(up_msg)
    
    def write_data(self):
        while True:
            self.write_once()
            
    def start_write(self):
        t_write = threading.Thread(target=self.write_data)
        t_write.start()

    def py_callback(self, msg):
        self.expected_pitch, self.expected_yaw = list(map(float, msg.data.split()))
        # print('Received expected pitch and yaw: ', self.expected_pitch, self.expected_yaw)

    def p_callback(self, msg):
        self.expected_pitch = msg.data
        # print('Received expected pitch: ', self.expected_pitch)

    def y_callback(self, msg):
        while msg.data < 0:
            msg.data += 360
        while msg.data > 360:
            msg.data -= 360
        self.expected_yaw = msg.data
        # print('Received expected yaw: ', self.expected_yaw)

    def hfov_callback(self, msg):
        self.expected_zoom = self.get_f(msg.data)
        self.expected_zoom_level = self.loose_zoom_level(round(self.expected_zoom / self.zoom_unit))
        # print('Received expected zoom: ', self.expected_zoom)
    
    def max_rate_callback(self, msg):
        self.max_rate = secret_interp(msg.data)
        # print('Received max rate: ', self.max_rate)

    def print_state(self):
        system('clear')
        print('-' * 20)
        print(pyfiglet.figlet_format('PodComm', font='slant'))
        print(f'Pod state: {self.pod_state_0} {self.pod_state_1} Camera state: {self.pod_camera_state}')
        print(f'Pitch {self.pod_pitch:.1f} -> {self.expected_pitch:.1f}')
        print(f'Yaw {self.pod_yaw:.1f} -> {self.expected_yaw:.1f}')
        print(f'Zoom {self.pod_f:.1f}({self.pod_zoom_level}) -> {self.expected_zoom:.1f}({self.expected_zoom_level})')
        print(f'LazyTag {self.lazy_tag}')
        # print(f'Yaw deque: {self.pod_yaw_deque}')"
        # if len(self.pod_yaw_deque) > 1:
        #     print(f'Yaw history data: from {self.pod_yaw_deque[0][0].strftime("%H:%M:%S.%f")} to {self.pod_yaw_deque[-1][0].strftime("%H:%M:%S.%f")}')
        print(f'CHECKSUM right/wrong: {self.check_sum_right_cnt}/{self.check_sum_wrong_cnt}')

    def ros_pub(self):
        self.pitch_pub.publish(self.pod_pitch)
        self.yaw_pub.publish(self.pod_yaw)
        self.hfov_pub.publish(self.get_hfov(self.pod_f))

    @timer(tol=5 / HZ)
    def spin_once(self):
        self.print_state()
        self.ros_pub()
    
    def spin(self):
        self.start_read()
        self.start_write()
        while not rospy.is_shutdown():
            self.spin_once()


if __name__ == '__main__':
    print(f'PORT is {PORT}')
    pod_comm = POD_COMM()
    pod_comm.spin()
