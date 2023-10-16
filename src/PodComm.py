#! /usr/bin/env python3

import threading
import argparse
from collections import deque
from datetime import datetime, timedelta
from math import tan, degrees, radians, atan
from os import system, path, readlink
from struct import pack, unpack
from sys import exit
from time import time, sleep

import rospy
import serial
from std_msgs.msg import Float32, Bool

from Utils import *

HZ = 50

DOWN_FRAME_HEAD_1 = b'\xEE'
DOWN_FRAME_HEAD_2 = b'\x16'
DOWN_FRAME_HEAD = DOWN_FRAME_HEAD_1 + DOWN_FRAME_HEAD_2
UP_FRAME_HEAD = b'\xEB\x90'
FRAME_LEN = 30

bAny, bS8, bU8, bS16, bU16, bS32, bU32, bF = 'x', 'b', 'B', 'h', 'H', 'i', 'I', 'f'
DOWN_PROTO = '<' + bU8 * 4 + bS16 * 5 + bU8 * 2 + bAny * 2 + bS16 * 3 + bU16 + bU8 + bAny * 2 + bU8

def splitByte(byte, bitSizes=[1]*8):
    result = []
    position = 0
    for size in bitSizes[::-1]:
        mask = (1 << size) - 1
        value = (byte >> position) & mask
        result.append(value)
        position += size
    return result[::-1]

WAITING_DOWN_FRAME_HEAD_1 = 1
WAITING_DOWN_FRAME_HEAD_2 = 2
READING_DATA = 3

from glob import glob

PORT = '/dev/ttyUSB0'

from signal import signal, SIGINT

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    exit(0)

signal(SIGINT, signal_handler)

class UP_MSG:
    def __init__(self):
        self.orderA = UP_FRAME_HEAD
        self.orderB = b'\x00'
        self.orderX = b'\x00\x00'
        self.orderY = b'\x00\x00'
        self.order3 = b'\x00'
        self.orderZ = b'\x00'
        self.order4 = b'\x00' * 6

    def msg(self):
        msg = self.orderA + self.orderB + self.orderX + self.orderY + self.order3 + self.orderZ + self.order4
        assert len(msg) == 15
        checkSum = sum(msg) & 0xFF
        return msg + pack('<B', checkSum)

    def textOff(self):
        self.orderB = b'\x52'
        return self.msg()

    def antiFogOn(self):
        self.orderB = b'\x05'
        return self.msg()

    def antiFogOff(self):
        self.orderB = b'\x06'
        return self.msg()

    def followOff(self):
        self.orderB = b'\x0E'
        return self.msg()

    def changeZoomLevel(self, level):
        self.orderB = b'\x5a'
        self.orderX = pack('<h', int(level * 10))
        print(f'change to {level:.1f}')
        return self.msg()

    def changeViewType(self):
        self.orderB = b'\x01'
        self.order3 = b'\x00'
        return self.msg()

    def manualPYRate(self, prate, yrate):
        self.orderB = b'\x24'
        self.orderX = pack('<h', int(-10 * yrate))
        self.orderY = pack('<h', int(-10 * prate))
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
    def __init__(self, args):
        self.args = args
        self.startTime = time()

        # serial related paras:
        self.state = WAITING_DOWN_FRAME_HEAD_1
        self.checkSumRightCnt = 0
        self.checkSumWrongCnt = 0
        self.dataBuf = bytearray()
        self.downSer = serial.Serial(
            port=PORT,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0.5
        )

        # pod constants:
        self.SENSOR_WIDTH = tan(radians(2.3) / 2) * 2 * 135
        self.zoomUnit = 4.5
        self.pyTol = 0.5
        self.zTol = 0.05

        # pod states: bool or bit
        self.podImageEnhanceOn = None
        self.podRollControlMode = None
        self.podFollowModeOn = None
        self.podLockModeOn = None
        self.podLaserOn = None
        self.podBigViewType = None
        self.podSmallViewType = None

        # pod states: float
        self.podF = 0
        self.podZoomLevel = 0
        self.podRoll = 0.0
        self.podPitch = 0.0
        self.podYaw = 0.0
        self.podLaserRange = 0.0
        self.podRollRate = 0.0
        self.podPitchRate = 0.0
        self.podYawRate = 0.0

        self.expectedPitch = 0.0
        self.expectedYaw = 0.0
        self.expectedF = 9
        self.expectedZoomLevel = 2
        self.maxRate = 300

        # control paras:
        self.initTextOff = False
        self.initAntiFog = False
        self.lazyTag = 0
        self.lockCnt = 0

        # ros related:
        self.uavName = 'suav'
        self.deviceName = 'pod'

        rospy.init_node('pod_comm', anonymous=True)
        rospy.Rate(10)
        rospy.Subscriber(self.uavName + '/' + self.deviceName + '/expectedPitch', Float32, lambda msg: (
            setattr(self, 'expectedPitch', msg.data)
        ))
        rospy.Subscriber(self.uavName + '/' + self.deviceName + '/expectedYaw', Float32, lambda msg: (
            setattr(self, 'expectedYaw', self.round(msg.data, 180))
        ))
        rospy.Subscriber(self.uavName + '/' + self.deviceName + '/expectedHfov', Float32, lambda msg: (
            setattr(self, 'expectedF', self.getF(msg.data)),
            setattr(
                self,
                'expectedZoomLevel',
                self.looseZoomLevel(
                    round(self.expectedF / self.zoomUnit)
                )
            )
        ))
        rospy.Subscriber(self.uavName + '/' + self.deviceName + '/maxRate', Float32, lambda msg: (
            setattr(self, 'maxRate', msg.data)
        ))
        self.pitchPub = rospy.Publisher(self.uavName + '/' + self.deviceName + '/pitch', Float32, queue_size=1)
        self.yawPub = rospy.Publisher(self.uavName + '/' + self.deviceName + '/yaw', Float32, queue_size=1)
        self.hfovPub = rospy.Publisher(self.uavName + '/' + self.deviceName + '/hfov', Float32, queue_size=1)
        
        self.pNotAtTargetTime = self.getTimeNow()
        self.yNotAtTargetTime = self.getTimeNow()
        self.fNotAtTargetTime = self.getTimeNow()
        
        self.pAtTargetPub = rospy.Publisher(self.uavName + '/' + self.deviceName + '/pAtTarget', Bool, queue_size=1)
        self.yAtTargetPub = rospy.Publisher(self.uavName + '/' + self.deviceName + '/yAtTarget', Bool, queue_size=1)
        self.fAtTargetPub = rospy.Publisher(self.uavName + '/' + self.deviceName + '/fAtTarget', Bool, queue_size=1)
        self.pFeedbackPub = rospy.Publisher(self.uavName + '/' + self.deviceName + '/pFeedback', Float32, queue_size=1)
        self.yFeedbackPub = rospy.Publisher(self.uavName + '/' + self.deviceName + '/yFeedback', Float32, queue_size=1)
        self.fFeedbackPub = rospy.Publisher(self.uavName + '/' + self.deviceName + '/fFeedback', Float32, queue_size=1)

    def looseZoomLevel(self, z):
        return z

    def getTimeNow(self):
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

    def deadZone(self, x, z=110):
        if x == 0:
            return 0
        elif -z <= x and x <= z:
            return z * abs(x) / x
        else:
            return x

    def genUpMsg(self):
        up = UP_MSG()

        if self.podBigViewType != 0x00 or self.podSmallViewType != 0x02:
            up.changeViewType()
        elif not self.initTextOff:
            up.textOff()
            self.lazyTag = 10
            self.initTextOff = True
            print('Text off')
        # elif self.podFollowModeOn == 1:
        #     up.followOff()
        #     print('Follow Off')
        elif not self.podImageEnhanceOn:
            up.antiFogOn()
            self.lazyTag = 10
            print('AntiFog')
        elif self.lazyTag <= 15:
            pitchDiff = self.expectedPitch - self.podPitch
            yawDiff = self.round(self.expectedYaw - self.podYaw, 180)
            absZoomDiff = (self.expectedF - self.podF)
            relZoomDiff = absZoomDiff / self.expectedF

            if (not self.pAtTarget or not self.yAtTarget) and self.lazyTag == 0:
                prMax, yrMax = self.maxRate, self.maxRate
                prate = max(-prMax, min(prMax, pitchDiff))
                yrate = max(-yrMax, min(yrMax, yawDiff))

                up.manualPYRate(prate, yrate)

                print(f'up pitch {self.podPitch:.2f} -> {self.expectedPitch:.2f} diff: {pitchDiff:.2f} rate: {prate:.2f}')
                print(f'up yaw {self.podYaw:.2f} -> {self.expectedYaw:.2f} diff: {yawDiff:.2f} rate: {yrate:.2f}')
            elif self.pAtTargetStrict and self.yAtTargetStrict:
                self.lockCnt += 1

            elif not self.fAtTarget:
                if self.lazyTag == 0:
                    print(f'change zoom level {self.podZoomLevel} to {self.expectedZoomLevel}')
                    up.changeZoomLevel(self.expectedZoomLevel)
                    self.lazyTag = max(abs(self.expectedZoomLevel - self.podZoomLevel) * 10, 50)

            # elif relZoomDiff < -self.zTol:
            #     # print(f'up decrease zoom a bit')
            #     up.zoomDown()
            #     if self.lazyTag == 0:
            #         self.lazyTag = 12
            #
            # elif relZoomDiff > self.zTol:
            #     # print(f'up increase zoom a bit')
            #     up.zoomUp()
            #     if self.lazyTag == 0:
            #         self.lazyTag = 12

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
                        # print('down: ', DOWN_FRAME_HEAD.hex(), dataBuf[:-1].hex(), dataBuf[-1:].hex())
                        downData = unpack(DOWN_PROTO, dataBuf)
                        checkSum = downData[-1]
                        realSum = sum(dataBuf[:-1]) + 0xEE + 0x16
                        if checkSum != realSum & 0xFF:
                            # print('Checksum: ', f'{checkSum:02x}', 'Realsum: ', f'{realSum:02x}')
                            self.checkSumWrongCnt += 1
                            # raise AssertionError
                        else:
                            # print('Right!!!')
                            self.checkSumRightCnt += 1
                            (podState1, podState2, zoomx10Low8, podState3, 
                             xOffsetDegreex20, yOffsetDegreex20, 
                             podRollx100, podPitchx100, podYawx100,
                             waveHor, waveVer, 
                             podRollRatex100, podPitchRatex100, podYawRatex100,
                             laserRangex10, selfCheckRes) = downData[:-1]
                           
                            podState2Datas = splitByte(podState2)
                            
                            self.podImageEnhanceOn = podState2Datas[0]
                            self.podRollControlMode = podState2Datas[4]
                            self.podFollowModeOn = podState2Datas[5]
                            self.podLockModeOn = podState2Datas[6]
                            self.podLaserOn = podState2Datas[7]

                            podState3Datas = splitByte(podState3, [2, 2, 4])
                            
                            self.podBigViewType = podState3Datas[0]
                            self.podSmallViewType = podState3Datas[1]
                            zoomx10High4 = podState3Datas[2]

                            if self.args.read:
                                print(
                                    f'state1: {podState1:2x}\n'
                                    f'state2: {podState2:2x}\n'
                                    f'zoomx10Low8: {zoomx10Low8}\n'
                                    f'state3: {podState3:2x}\n'
                                    f'xOffsetDegx20: {xOffsetDegreex20}\n'
                                    f'yOffsetDegx20: {yOffsetDegreex20}\n'
                                    f'podRPYx100: R{podRollx100}, P{podPitchx100}, Y{podYawx100}\n'
                                    f'waveHor: {waveHor}, waveVer: {waveVer}\n'
                                    f'podRPYRatex100: R{podRollRatex100}, P{podPitchRatex100}, Y{podYawRatex100}\n'
                                    f'laserRangex10: {laserRangex10}\n'
                                    f'selfCheckRes: {selfCheckRes}\n'
                                )

                            self.podZoomLevel = (zoomx10Low8 + (zoomx10High4 << 8)) / 10
                            self.podF = self.podZoomLevel * self.zoomUnit
                            self.podPitch = -podPitchx100 / 100
                            self.podYaw = -self.round(podYawx100 / 100, 180)
                            self.podRoll = podRollx100 / 100
                            self.podLaserRange = laserRangex10 / 10
                            self.podRollRate = podRollRatex100 / 100
                            self.podPitchRate = podPitchRatex100 / 100
                            self.podYawRate = podYawRatex100 / 100
                            
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
        print('### PodComm ###')
        print((GREEN if self.podImageEnhanceOn else RED) + 'ImageEnhance' + RESET, end='')
        print((GREEN if self.podFollowModeOn else RED) + 'FollowMode' + RESET, end='')
        print((GREEN if self.podLockModeOn else RED) + 'LockMode' + RESET, end='')
        print('Laser: ' + (f'{self.podLaserRange:.1f}' if self.podLaserOn else 'Off'))
        print(f'BigView: {self.podBigViewType} SmallView: {self.podSmallViewType}')
        print(GREEN if self.pAtTarget else RED, end='')
        print(f'Pitch {self.podPitch:.2f} -> {self.expectedPitch:.2f}{RESET}')
        print(GREEN if self.yAtTarget else RED, end='')
        print(f'Yaw {self.podYaw:.2f} -> {self.expectedYaw:.2f}{RESET}')
        print(GREEN if self.fAtTarget else RED, end='')
        print(f'Zoom {self.podF:.1f}({self.podZoomLevel}) -> {self.expectedF:.1f}({self.expectedZoomLevel}){RESET}')
        print(f'Hfov {self.getHfov(self.podF):.2f} -> {self.getHfov(self.expectedF):.2f}')
        print('LazyTag: ', self.lazyTag, ' LockCnt: ', self.lockCnt)
        print(f'CHECKSUM right/wrong: {self.checkSumRightCnt}/{self.checkSumWrongCnt}')

    def rosPub(self):
        self.pitchPub.publish(self.podPitch)
        self.yawPub.publish(self.podYaw)
        self.hfovPub.publish(self.getHfov(self.podF))

        self.pAtTargetPub.publish(self.pAtTarget)
        self.yAtTargetPub.publish(self.yAtTarget)
        self.fAtTargetPub.publish(self.fAtTarget)

        self.pFeedbackPub.publish(self.expectedPitch)
        self.yFeedbackPub.publish(self.round(self.expectedYaw, 180))
        self.fFeedbackPub.publish(self.getHfov(self.expectedF))

    @property
    def pAtTargetStrict(self):
        return (abs(self.podPitch - self.expectedPitch) < self.pyTol * 0.8)

    @property
    def yAtTargetStrict(self):
        return (abs(self.round(self.podYaw - self.expectedYaw, 180)) < self.pyTol * 0.8)

    @property
    def pAtTarget(self):
        if not (abs(self.podPitch - self.expectedPitch) < self.pyTol):
            self.pNotAtTargetTime = self.getTimeNow()
        return self.getTimeNow() - self.pNotAtTargetTime > 0.1

    @property
    def yAtTarget(self):
        if not (abs(self.round(self.podYaw - self.expectedYaw, 180)) < self.pyTol):
            self.yNotAtTargetTime = self.getTimeNow()
        return self.getTimeNow() - self.yNotAtTargetTime > 0.1

    @property
    def fAtTarget(self):
        if not (self.expectedZoomLevel == self.podZoomLevel):
            self.fNotAtTargetTime = self.getTimeNow()
        return self.getTimeNow() - self.fNotAtTargetTime > 0.1

    @timer(tol=1 / HZ)
    def spinOnce(self):
        self.printState()
        self.rosPub()
        self.writeOnce()

    def spin(self):
        self.startRead()
        while not rospy.is_shutdown() and not self.args.read:
            self.spinOnce()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--read', help='read only', action='store_true')
    args, unknown = parser.parse_known_args()
    print(f'PORT is {PORT}')
    pod_comm = POD_COMM(args)
    pod_comm.spin()
