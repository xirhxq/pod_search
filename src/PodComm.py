#! /usr/bin/env python3

import serial
import threading
from time import time, sleep
from struct import pack, unpack
from sys import platform, exit
from os import system
import rospy
from std_msgs.msg import String, Float32, Bool
from math import tan, degrees, radians, atan
from collections import deque
from datetime import datetime, timedelta
import pyfiglet
from Utils import *

HZ = 50

DOWN_FRAME_HEAD_1 = b'\xAA'
DOWN_FRAME_HEAD_2 = b'\x55'
DOWN_FRAME_HEAD = DOWN_FRAME_HEAD_1 + DOWN_FRAME_HEAD_2
UP_FRAME_HEAD = b'\xEB\x90'
FRAME_LEN = 48

bAny, bS8, bU8, bS16, bU16, bS32, bU32, bF = 'x', 'b', 'B', 'h', 'H', 'i', 'I', 'f' 
DOWN_PROTO = '<' + bU8 * 2 + bU16 + bS16 + bU16 + bAny * 29 + bU8 + bAny * 2 + bU16 + bU8 + bAny * 2 + bU8 + bU8 * 2

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

def secretInterp(x, data=SECRET_DICT): 
    sortedData = sorted(data.items())
    xVal, yVal = zip(*sortedData)
    if x <= xVal[0]:
        return yVal[0]
    if x >= xVal[-1]:
        return yVal[-1]
    
    index = 0
    while x > xVal[index]:
        index += 1
    
    x1, y1 = xVal[index - 1], yVal[index - 1]
    x2, y2 = xVal[index], yVal[index]

    y = y1 + (x - x1) * (y2 - y1) / (x2 - x1)
    return y

class UP_MSG:
    def __init__(self):
        self.orderA = UP_FRAME_HEAD
        self.orderB = b'\xFF'
        self.orderC = b'\x00\x00'
        self.orderD = b'\x00\x00\x00\x00'
        self.orderE = b'\x00' * 45
    
    def msg(self):
        msg = self.orderA + self.orderB + self.orderC + self.orderD + self.orderE
        assert len(msg) == 54
        checkSum = sum(msg) & 0xFF
        return msg + pack('<B', checkSum)
    
    def zoomDown(self):
        self.orderB = b'\x48'
        return self.msg()
    
    def zoomUp(self):
        self.orderB = b'\x44'
        return self.msg()

    def textOnOff(self):
        self.orderB = b'\x93'
        return self.msg()

    def changeZoomLevel(self, level):
        self.orderB = b'\xDC'
        self.orderC = pack('<h', int(level))
        print(f'change to {level}')
        return self.msg()

    def manualPYRate(self, prate, yrate):
        self.orderB = b'\x25'
        self.orderC = pack('<h', int(prate))
        self.orderD = pack('<h', int(yrate)) + b'\x00\x00'
        return self.msg()


def timer(tol=1):
    def decorator(func):
        def wrapper(*args, **kwargs):
            startTime = time()
            result = func(*args, **kwargs)
            endTime = time()
            # print(f'Time elap: {endTime - startTime:.2f}')
            if endTime - startTime < tol:
                sleep(tol - endTime + startTime)
            # print(f'Ended')
            return result
        return wrapper
    return decorator

class POD_COMM:
    def __init__(self):
        self.init = False
        self.state = WAITING_DOWN_FRAME_HEAD_1
        self.expectedPitch = 90.0
        self.expectedYaw = 0.0
        self.expectedZoom = 4.3
        self.expectedZoomLevel = 1
        self.startTime = time()
        self.lazyTag = 0
        self.SENSOR_WIDTH = tan(radians(2.3) / 2) * 2 * 129

        self.podState0 = 0x00
        self.podState1 = 0x00
        self.podF = 4.3
        self.podZoomLevel = 1
        self.podPitch = 0.0
        self.podYaw = 0.0
        self.podCameraState = 0x10
        self.podLaserRes = 0
        self.podElecZoom = 0
        self.podOrder = 0

        self.podYawDeque = deque()
        self.podPitchDeque = deque()
        self.timeInterval = timedelta(seconds=1)
        self.podYawV = 0
        self.podPitchV = 0

        self.checkSumRightCnt = 0
        self.checkSumWrongCnt = 0

        self.maxRate = 300
        self.zoomUnit = 4.3
        self.pyTol = 0.5
        self.zTol = 0.05

        self.downSer = serial.Serial(
            port=PORT,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0.5
        )
        self.dataBuf = bytearray()

        rospy.init_node('pod_comm', anonymous=True)
        rospy.Rate(10)
        rospy.Subscriber('/pod_comm/expectedPitch', Float32, lambda msg: ( 
            setattr(self, 'expectedPitch', msg.data)
        ))
        rospy.Subscriber('/pod_comm/expectedYaw', Float32, lambda msg: ( 
            setattr(self, 'expectedYaw', self.round(msg.data, 180))
        ))
        rospy.Subscriber('/pod_comm/expectedHfov', Float32, lambda msg: ( 
            setattr(self, 'expectedZoom', self.getF(msg.data)),
            setattr(
                self, 
                'expectedZoomLevel', 
                self.looseZoomLevel(
                    round(self.expectedZoom / self.zoomUnit)
                )
            )
        ))
        rospy.Subscriber('/pod_comm/maxRate', Float32, lambda msg: (
            setattr(self, 'maxRate', secretInterp(msg.data))
        ))
        self.pitchPub = rospy.Publisher('/pod_comm/pitch', Float32, queue_size=10)
        self.yawPub = rospy.Publisher('/pod_comm/yaw', Float32, queue_size=10)
        self.hfovPub = rospy.Publisher('/pod_comm/hfov', Float32, queue_size=10)
        self.pAtTargetPub = rospy.Publisher('/pod_comm/pAtTarget', Bool, queue_size=1)
        self.yAtTargetPub = rospy.Publisher('/pod_comm/yAtTarget', Bool, queue_size=1)
        self.fAtTargetPub = rospy.Publisher('/pod_comm/fAtTarget', Bool, queue_size=1)
        self.pFeedbackPub = rospy.Publisher('/pod_comm/pFeedback', Float32, queue_size=1)
        self.yFeedbackPub = rospy.Publisher('/pod_comm/yFeedback', Float32, queue_size=1)
        self.fFeedbackPub = rospy.Publisher('/pod_comm/fFeedback', Float32, queue_size=1)

    def looseZoomLevel(self, z):
        return z

    def getTime_now(self):
        return time()

    def getF(self, halfAngle):
        return self.SENSOR_WIDTH / 2 / tan(radians(halfAngle / 2))

    def getHfov(self, f):
        return degrees(atan(self.SENSOR_WIDTH / 2 / f)) * 2

    def round(self, val, base):
        if val > base:
            return val - base * 2
        elif val < -base:
            return val + base * 2
        else:
            return val
    
    def genUpMsg(self):
        up = UP_MSG()

        if self.init == False:
            up.textOnOff()
            self.init = True
        elif self.lazyTag <= 15:
            pitchDiff = self.expectedPitch - self.podPitch
            yawDiff = self.round(self.expectedYaw - self.podYaw, 180)
            absZoomDiff = (self.expectedZoom - self.podF)
            relZoomDiff = absZoomDiff / self.expectedZoom


            if self.expectedZoomLevel != self.podZoomLevel and self.lazyTag == 0 and abs(relZoomDiff) > 0.1:
                # print(f'change zoom level {self.podZoomLevel} to {self.expectedZoomLevel}')
                up.changeZoomLevel(self.expectedZoomLevel)
                self.lazyTag = 150

            elif relZoomDiff < -self.zTol:
                # print(f'up decrease zoom a bit')
                up.zoomDown()
                if self.lazyTag == 0:
                    self.lazyTag = 12
                
            elif relZoomDiff > self.zTol:
                # print(f'up increase zoom a bit')
                up.zoomUp()
                if self.lazyTag == 0:
                    self.lazyTag = 12

            elif abs(pitchDiff) > self.pyTol or abs(yawDiff) > self.pyTol:
                prMax, yrMax = 300, self.maxRate
                prate = max(-prMax, min(prMax, pitchDiff * 200))
                yrate = max(-yrMax, min(yrMax, yawDiff * 300))

                up.manualPYRate(prate, yrate)



                # print(f'up pitch {self.podPitch:.2f} -> {self.expectedPitch:.2f} diff: {pitchDiff:.2f} rate: {prate:.2f}')
                # print(f'up yaw {self.podYaw:.2f} -> {self.expectedYaw:.2f} diff: {yawDiff:.2f} rate: {yrate:.2f}')
        if self.lazyTag > 0:
            self.lazyTag -= 1
        

        return up.msg()



    def readData(self):
        while True:
            data = self.downSer.read(1)
            if data:
                if self.state == WAITING_DOWN_FRAME_HEAD_1:
                    if data == DOWN_FRAME_HEAD_1:
                        self.state = WAITING_DOWN_FRAME_HEAD_2
                elif self.state == WAITING_DOWN_FRAME_HEAD_2:
                    if data == DOWN_FRAME_HEAD_2:
                        self.state = READING_DATA
                        dataBuf = bytearray()
                elif self.state == READING_DATA:
                    # print(f'reading data buffer len {len(dataBuf)}')
                    dataBuf.append(data[0])
                    if len(dataBuf) == FRAME_LEN:
                        # print('down: ', DOWN_FRAME_HEAD.hex(), dataBuf[:-2].hex(), dataBuf[-2:].hex())
                        downData = unpack(DOWN_PROTO, dataBuf)
                        checkSum = downData[-1] * 0x100 + downData[-2]
                        if checkSum != sum(dataBuf[:-2]):
                            # print('Checksum error')
                            # print('Checksum: ', f'{checkSum:02x}', 'Realsum: ', f'{sum(dataBuf[:-2]):02x}')
                            self.checkSumWrongCnt += 1
                            # raise AssertionError
                        else:
                            self.checkSumRightCnt += 1
                            self.podState0, self.podState1, podFx10, podPitchx100, podYawx100, self.podCameraState, self.podLaserRes, self.podElecZoom, self.podOrder = downData[:-2]
                            self.podF = podFx10 / 10
                            self.podPitch = podPitchx100 / 100
                            self.podYaw = self.round(podYawx100 / 100, 180)
                            self.podZoomLevel = self.looseZoomLevel(round(self.podF / self.zoomUnit))

                            currentTime = datetime.now()
                            self.podYawDeque.append((currentTime, self.podYaw))
                            self.podPitchDeque.append((currentTime, self.podPitch))

                            while len(self.podYawDeque) > 0 and self.podYawDeque[0][0] < currentTime - self.timeInterval:
                                # print(f'pop {self.podYawDeque[0]}')
                                self.podYawDeque.popleft()

                            while len(self.podPitchDeque) > 0 and self.podPitchDeque[0][0] < currentTime - self.timeInterval:
                                self.podPitchDeque.popleft()

                            if len(self.podYawDeque) > 1:
                                self.podYawV = self.round(self.podYawDeque[-1][1] - self.podYawDeque[0][1], 180) / (self.podYawDeque[-1][0] - self.podYawDeque[0][0]).total_seconds()

                            if len(self.podPitchDeque) > 1:
                                self.podPitchV = (self.podPitchDeque[-1][1] - self.podPitchDeque[0][1])/ (self.podPitchDeque[-1][0] - self.podPitchDeque[0][0]).total_seconds()


                            # print(f'Received state {self.podState0}, camera state {self.podCameraState}, zoom {self.podF}, pitch {self.podPitch}, yaw {self.podYaw}')
                        # print(f'down zoom: {self.podF:.2f} pitch: {self.podPitch:.2f} yaw: {self.podYaw:.2f}')
                        self.state = WAITING_DOWN_FRAME_HEAD_1

            if self.state == WAITING_DOWN_FRAME_HEAD_1:
                dataBuf = bytearray()
    
    def startRead(self):
        tRead = threading.Thread(target=self.readData)
        tRead.start()

    @timer(tol=1 / HZ)
    def writeOnce(self):
        upMsg = self.genUpMsg()
        # print('------------------------------')
        self.downSer.write(upMsg)
    
    def writeData(self):
        while True:
            self.writeOnce()
            
    def startWrite(self):
        tWrite = threading.Thread(target=self.writeData)
        tWrite.start()

    def printState(self):
        system('clear')
        print('-' * 20)
        print(pyfiglet.figlet_format('PodComm', font='slant'))
        print(f'Pod state: {self.podState0} {self.podState1} Camera state: {self.podCameraState}')
        print(RED if abs(self.podPitch - self.expectedPitch) > self.pyTol else GREEN, end='')
        print(f'Pitch {self.podPitch:.1f} -> {self.expectedPitch:.1f}{RESET}')
        print(RED if abs(self.podYaw - self.expectedYaw) > self.pyTol else GREEN, end='')
        print(f'Yaw {self.podYaw:.1f} -> {self.expectedYaw:.1f}{RESET}')
        print(RED if abs(self.podF - self.expectedZoom) / self.expectedZoom > self.zTol else GREEN, end='')
        print(f'Zoom {self.podF:.1f}({self.podZoomLevel}) -> {self.expectedZoom:.1f}({self.expectedZoomLevel}){RESET}')
        print(f'Hfov {self.getHfov(self.podF):.2f} -> {self.getHfov(self.expectedZoom):2f}')
        print(f'LazyTag {self.lazyTag}')
        # print(f'Yaw deque: {self.podYawDeque}')"
        # if len(self.podYawDeque) > 1:
        #     print(f'Yaw history data: from {self.podYawDeque[0][0].strftime("%H:%M:%S.%f")} to {self.podYawDeque[-1][0].strftime("%H:%M:%S.%f")}')
        print(f'CHECKSUM right/wrong: {self.checkSumRightCnt}/{self.checkSumWrongCnt}')

    def rosPub(self):
        self.pitchPub.publish(self.podPitch)
        self.yawPub.publish(self.podYaw)
        self.hfovPub.publish(self.getHfov(self.podF))

        self.pAtTargetPub.publish(Bool(abs(self.podPitch - self.expectedPitch) < self.pyTol))
        self.yAtTargetPub.publish(Bool(abs(self.round(self.podYaw - self.expectedYaw, 180)) < self.pyTol))
        self.fAtTargetPub.publish(Bool(abs(self.podF - self.expectedZoom) / self.expectedZoom < self.zTol))
        
        self.pFeedbackPub.publish(self.expectedPitch)
        self.yFeedbackPub.publish(self.round(self.expectedYaw, 180))
        self.fFeedbackPub.publish(self.getHfov(self.expectedZoom))

    @timer(tol=5 / HZ)
    def spinOnce(self):
        self.printState()
        self.rosPub()
    
    def spin(self):
        self.startRead()
        self.startWrite()
        while not rospy.is_shutdown():
            self.spinOnce()


if __name__ == '__main__':
    print(f'PORT is {PORT}')
    pod_comm = POD_COMM()
    pod_comm.spin()
