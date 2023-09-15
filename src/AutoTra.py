#!/usr/bin/env python3

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from math import degrees, atan, tan, radians, sin, cos, sqrt
import numpy as np


class AutoTra:
    def getVal(self, str='', default=None):
        str = input(f'Input {str} (Default: {default}): ')
        if str == '':
            return default
        else:
            return float(str)

    def __init__(self,
                 overlapOn=True,
                 pitchLevelOn=True,
                 drawNum=-1
        ):
        self.h = self.getVal(str='h', default=11.7)
        self.a = self.getVal(str='a', default=500)
        self.b = self.getVal(str='b', default=150)
        x = self.getVal(str='x', default=-200)
        self.hfovPitchRatio = self.getVal(str='hfov/pitch', default=3)
        self.theTime = self.getVal(str='THE Time', default=3)

        self.pos2d = np.array([x, 0])

        self.ratio = 16 / 9
        self.SENSOR_WIDTH = tan(radians(2.3) / 2) * 2 * 129

        print(f'a: {self.a:.2f} b:  {self.b:.2f} pos: {self.pos2d} h: {self.h:.2f}')

        self.cmap = plt.get_cmap('tab10')

        self.overlapD = 10 * self.h / 100 if overlapOn else 0

        self.pitchLevelOn = pitchLevelOn
        self.drawNum = drawNum

        def overlapM(m):
            if m == 90:
                return m
            r = self.getRFromPitch(m)
            rr = r * (self.h - self.overlapD) / self.h
            return self.getPitchFromR(rr)

        def overlap(p, m):
            hFov = self.getHFovFromPitch(p)
            vFov = 2 * degrees(atan(tan(radians(hFov / 2)) * 9 / 16))
            overlapRatio = 0.1
            gap = overlapM(m) - p
            # gap = m - p
            return gap < vFov / 2

        def getNewP(m):
            l, r = 0, m
            while abs(r - l) > 1e-3:
                mid = (l + r) / 2
                if overlap(mid, m):
                    r = mid
                else:
                    l = mid
            return l

        self.pRange = [90, 0]
        if self.pos2d[0] < 0:
            self.pRange[0] = degrees(atan(self.h / (-self.pos2d[0])))
            self.pRange[1] = degrees(atan(self.h / sqrt((self.a - self.pos2d[0]) ** 2 + self.b ** 2)))

        # self.pRange[0] = 50

        print('pRange:', self.pRange)
        nowMinP = self.pRange[0]
        nowMaxP = self.pRange[0]
        self.pitches = []

        while nowMinP > self.pRange[1]:
            self.pitches.append(getNewP(nowMinP))
            nowP = self.pitches[-1]
            nowVFov = self.getVFovFromPitch(nowP)
            nowMinP = min(nowMinP, nowP - nowVFov / 2)
            nowMaxP = min(nowMaxP, nowP + nowVFov / 2)
            # print(f'podP: {self.pitches[-1]:.12f} [{nowMinP:.2f}, {nowMaxP:.2f}]')

        print('-' * 10 + 'Pod P Results' + '-' * 10)
        for ind, podP in enumerate(self.pitches):
            print(f'### Round {ind + 1}')
            print(f'p: {podP:.12f} -> hfov: {self.getHFovFromPitch(podP):.2f}')
            print(f'yawRange: {self.getYawRangeFromPitch(podP)[1] - self.getHFovFromPitch(podP) / 2:.2f}')
            print(f'p Range: {podP - self.getVFovFromPitch(podP) / 2:.2f} ~ {podP + self.getVFovFromPitch(podP) / 2:.2f}')
            print(f'R Range: {self.getRFromPitch(podP - self.getVFovFromPitch(podP) / 2):.2f} ~ {self.getRFromPitch(podP + self.getVFovFromPitch(podP) / 2):.2f}')
            pass
        print('-' * 30)

        self.theList = []
        self.expectedTime = 0

        for ind, podP in enumerate(self.pitches):
            hfov = self.getHFovFromPitch(podP)
            yRange = self.getYawRangeFromPitch(podP)[1] - self.getHFovFromPitch(podP) / 2
            if ind % 2 == 0:
                yRange = -yRange
            print(f'[{podP:.6f}, {yRange:.2f}, {hfov:.2f}, 20], [{podP:.6f}, {-yRange:.2f}, {hfov:.2f}, {hfov/self.theTime:.2f}]')
            self.theList.append([podP, yRange, hfov, 20])
            self.theList.append([podP, -yRange, hfov, hfov / self.theTime])
            self.expectedTime += abs(yRange) * 2 / hfov * 4
            print(f'After this round time is {self.expectedTime:.2f}')

        #self.theList.append([0, 0, 20, 20])
        print('### The List ###')
        print(self.theList)
        print('################')
        print(f'Expected Total Time: {self.expectedTime:.2f}')

    def getRFromPitch(self, pitch):
        pitchCliped = min(self.pRange[0], max(self.pRange[1], pitch))
        # pitchCliped = pitch
        r = self.h / tan(radians(pitchCliped))
        return r

    def getPitchFromR(self, r):
        return degrees(atan(self.h / r))

    def getPosFromPitchYaw(self, pitch, yaw):
        r = self.getRFromPitch(pitch)
        return np.array([r * cos(radians(yaw)), r * sin(radians(yaw))]) + self.pos2d

    def clipPitch(self, pitch):
        return min(60, max(2.3, pitch))

    def getFFromHfov(self, hfov):
        return self.SENSOR_WIDTH / 2 / tan(radians(hfov / 2))

    def getHFovFromF(self, f):
        return degrees(2 * atan(self.SENSOR_WIDTH / 2 / f))

    def getHFovFromPitch(self, pitch):
        if not self.pitchLevelOn:
            return pitch
        hfovExact = self.clipPitch(self.hfovPitchRatio * pitch)
        # print(f'Pitch: {pitch:.2f} -> Hfov: {hfovExact:.2f}')
        fExact = self.getFFromHfov(hfovExact)
        # print(f'F: {fExact:.2f}')
        fUnit = 4.3
        fLevel = round(fExact / fUnit) * fUnit
        hfovLevel = self.getHFovFromF(fLevel)
        # print(f'fLevel: {fLevel:.2f} -> hfovLevel: {hfovLevel:.2f}')
        return min(60.0, max(hfovLevel, 2.3))

    def getVFovFromHFov(self, hFov):
        return 2 * degrees(atan(tan(radians(hFov / 2)) / self.ratio))

    def getVFovFromPitch(self, pitch):
        return self.getVFovFromHFov(self.getHFovFromPitch(pitch))

    def getYawRangeFromPitch(self, pitch):
        vFov = self.getVFovFromPitch(pitch)
        biggerPitch = pitch + vFov / 2
        rWithBiggerPitch = self.getRFromPitch(biggerPitch)
        smallerPitch = pitch - vFov / 2
        rWithSmallerPitch = self.getRFromPitch(smallerPitch)

        rToCorner = sqrt(self.pos2d[0] ** 2 + self.b ** 2)

        r = rToCorner
        if rWithBiggerPitch < rWithSmallerPitch <= rToCorner:
            r = rWithSmallerPitch
        elif rToCorner <= rWithBiggerPitch < rWithSmallerPitch:
            r = rWithBiggerPitch

        mostFar = np.array([0, sqrt(r ** 2 - self.pos2d[0] ** 2)])
        if mostFar[1] > self.b:
            mostFar[0] = sqrt(r ** 2 - self.b ** 2) + self.pos2d[0]
            mostFar[1] = self.b

        rel = mostFar - self.pos2d
        if rel[0] == 0:
            return -90, 90
        else:
            yaw = degrees(atan(rel[1] / rel[0]))
            return -yaw, yaw

    def drawSectorRing(self, ax, inner_radius, outer_radius, start_angle, end_angle, color='blue', alpha=0.2):
        sector = patches.Wedge(
            self.pos2d,
            outer_radius,
            start_angle,
            end_angle,
            width=outer_radius - inner_radius,
            facecolor=color,
            alpha=alpha,
            edgecolor='none'
        )
        ax.add_artist(sector)

    def drawRingTra(self, ax, radius, start_angle, end_angle, color='blue', alpha=1):
        ring = patches.Wedge(
            self.pos2d,
            radius,
            start_angle,
            end_angle,
            width=0.1,
            facecolor=color,
            alpha=alpha,
            edgecolor='none'
        )
        ax.add_artist(ring)

    def drawOneGlance(self, ax, pitch, yaw):
        hFov = self.getHFovFromPitch(pitch)
        vFov = self.getVFovFromPitch(pitch)
        ax.add_artist(
            patches.Wedge(
                self.pos2d,
                self.getRFromPitch(pitch - vFov / 2),
                yaw - hFov / 2,
                yaw + hFov / 2,
                width=self.getRFromPitch(pitch - vFov / 2) - self.getRFromPitch(pitch + vFov / 2),
                edgecolor='blue',
                facecolor='none'
            )
        )
        pos = self.getPosFromPitchYaw(pitch, yaw)
        ax.plot(*pos, '*', color='blue', alpha=0.5)

    def drawBetweenLine(self, ax, prePitch, nxtPitch, color='blue', alpha=1.0, rev=False):
        preHFov = self.getHFovFromPitch(prePitch)
        preYaw = self.getYawRangeFromPitch(prePitch)[1]
        prePos = self.getPosFromPitchYaw(prePitch, (preYaw - preHFov / 2) * (-1 if rev else 1))

        nxtHFov = self.getHFovFromPitch(nxtPitch)
        nxtYaw = self.getYawRangeFromPitch(nxtPitch)[1]
        nxtPos = self.getPosFromPitchYaw(nxtPitch, (nxtYaw - nxtHFov / 2) * (-1 if rev else 1))

        print(f'Between Line [{prePitch:.2f}, {preYaw - preHFov / 2:.2f}] --> [{nxtPitch:.2f}, {nxtYaw - nxtHFov / 2:.2f}]')
        ax.plot([prePos[0], nxtPos[0]], [prePos[1], nxtPos[1]], color=color, alpha=alpha, linewidth=2)

    def drawSearchAnnulus(self, ax, pitch, color='blue', alpha=0.5, rev=False):
        hFov = self.getHFovFromPitch(pitch)
        vFov = self.getVFovFromPitch(pitch)
        yawRange = self.getYawRangeFromPitch(pitch)
        print(f'This round P: {pitch:.2f} Yaw Range: {yawRange} Hfov: {self.getHFovFromPitch(pitch):.2f}')
        self.drawSectorRing(
            ax=ax,
            inner_radius=self.getRFromPitch(pitch + vFov / 2),
            outer_radius=self.getRFromPitch(pitch - vFov / 2),
            start_angle=yawRange[0],
            end_angle=yawRange[1],
            color=color,
            alpha=alpha
        )
        print(f'Ring: [{pitch:.2f}, {yawRange[0]:.2f}] --> [{pitch:.2f}, {yawRange[1]:.2f}]')
        self.drawRingTra(
            ax=ax,
            radius=self.getRFromPitch(pitch),
            start_angle=yawRange[0] + hFov / 2,
            end_angle=yawRange[1] - hFov / 2,
            color='red'
        )
        print(f'Pod Tra: [{pitch:.2f}, {yawRange[0] + hFov / 2:.2f}] --> [{pitch:.2f}, {yawRange[1] - hFov / 2:.2f}]')
        # self.drawOneGlance(
        #     ax=ax,
        #     pitch=pitch,
        #     yaw=(yawRange[0] + hFov / 2) * (-1 if rev else 1)
        # )

    def inflateInterval(self, interval=None, ratio=0.1):
        if interval is None:
            interval = [0, 1]
        interval = np.array(interval)
        return np.array([
            interval[0] - (interval[1] - interval[0]) * ratio,
            interval[1] + (interval[1] - interval[0]) * ratio
        ])

    def draw(self):
        self.fig = plt.figure(figsize=(20, 20))
        self.ax = self.fig.add_subplot(111)
        self.ax.set_aspect('equal')
        self.ax.set_xlim(self.inflateInterval([0, self.a], ratio=0.3))
        self.ax.set_ylim(self.inflateInterval([-self.b, self.b], ratio=0.3))

        self.ax.add_patch(
            patches.Rectangle(
                (0, -self.b),
                self.a,
                self.b * 2,
                facecolor='gray',
                alpha=0.5,
                edgecolor='black'
            )
        )

        for ind, p in enumerate(self.pitches):
            if 0 < self.drawNum <= ind:
                break
            print(f'--- Round #{ind + 1} ---')
            if ind > 0:
                self.drawBetweenLine(self.ax, self.pitches[ind - 1], p, rev=(ind % 2 == 0), color='red', alpha=1.0)
            color = self.cmap(ind)
            self.drawSearchAnnulus(self.ax, p, color=color, alpha=0.3, rev=(ind % 2 == 0))

        plt.show()


if __name__ == '__main__':
    autoTra = AutoTra(
        pitchLevelOn=True,
        overlapOn=True,
        drawNum=-1
    )
    autoTra.draw()
