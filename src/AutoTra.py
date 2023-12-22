#!/usr/bin/env python3

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from math import degrees, atan, tan, radians, sin, cos, sqrt, asin
import numpy as np
from rich.prompt import FloatPrompt

import PodParas
from PodAngles import PodAngles

def intersectLength(polygonPoints, ray):
    rayX, rayY, phi = ray
    phiRad = np.radians(phi)
    dirX, dirY = np.cos(phiRad), np.sin(phiRad)
    intersects = []

    for i in range(len(polygonPoints)):
        x1, y1 = polygonPoints[i]
        x2, y2 = polygonPoints[(i + 1) % len(polygonPoints)]
        edgeDirX, edgeDirY = x2 - x1, y2 - y1

        det = dirX * edgeDirY - dirY * edgeDirX
        if det != 0:
            t = -((rayX - x1) * edgeDirY - (rayY - y1) * edgeDirX) / det
            u = -((rayX - x1) * dirY - (rayY - y1) * dirX) / det
            if t >= 0 and u >= 0 and u <= 1:
                intersectX, intersectY = rayX + t * dirX, rayY + t * dirY
                intersects.append((intersectX, intersectY))

    minLength = 2000
    closetPoint = None
    for point in intersects:
        length = np.sqrt((point[0] - rayX) ** 2 + (point[1] - rayY) ** 2)
        if length < minLength:
            minLength = length
            closetPoint = point
    return minLength


class AutoTra:
    def __repr__(self):
        return (
            f'(h={self.height:.0f}, '
            f'f={self.frontLength:.0f}, '
            f'l={self.leftLength:.0f}, '
            f'r={self.rightLength:.0f}, '
            f'T={self.theTime:.1f}, '
            f'w={self.widthRatio:.1f})'
        )

    def __init__(self,
                 overlapOn=True,
                 pitchLevelOn=True,
                 drawNum=-1,
                 fast=False,
                 config=None
        ):
        self.height = float(config['height'])
        self.frontLength = intersectLength(
            config['areaPoints'],
            (config['ray'][0], config['ray'][1], config['ray'][2])
        )
        self.leftLength = intersectLength(
            config['areaPoints'],
            (config['ray'][0], config['ray'][1], config['ray'][2] + 90)
        )
        self.rightLength = intersectLength(
            config['areaPoints'],
            (config['ray'][0], config['ray'][1], config['ray'][2] - 90)
        )
        self.xFLU = float(config['xFLU'])
        self.theTime = float(config['theTime'])
        self.widthRatio = float(config['widthRatio'])
        if not fast:
            self.height = FloatPrompt.ask('height', default=self.height, show_default=True)
            self.frontLength = FloatPrompt.ask('front length', default=self.frontLength)
            self.leftLength = FloatPrompt.ask('left length', default=self.leftLength)
            self.rightLength = FloatPrompt.ask('right length', default=self.rightLength)
            self.xFLU = FloatPrompt.ask('x', default=self.xFLU)
            self.widthRatio = FloatPrompt.ask('width ratio', default=self.widthRatio)
            self.theTime = FloatPrompt.ask('THE Time', default=self.theTime)
        self.xyFLU = np.array([self.xFLU, 0])
        self.vesselLength = 10
        self.horizonLength = self.vesselLength / self.widthRatio

        print(
            f'Front length: {self.frontLength:.2f} / m, '
            f'Left length: {self.leftLength:.2f} / m, '
            f'Right length: {self.rightLength:.2f} / m, '
            f'xy FLU: {self.xyFLU}, '
            f'height: {self.height:.2f}'
        )

        self.cmap = plt.get_cmap('tab10')

        self.overlapD = 10 * self.height / 100 if overlapOn else 0

        self.pitchLevelOn = pitchLevelOn
        self.drawNum = drawNum

        def overlapM(m):
            if m == 90:
                return m
            r = self.getRFromPitch(m)
            rr = r * (self.height - self.overlapD) / self.height
            return self.getPitchFromR(rr)

        def overlap(p, m):
            hFov = self.getHFovFromTopPitch(p)
            vFov = PodParas.getVFovFromHFov(hFov)
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
        if self.xyFLU[0] < 0:
            self.pRange[0] = degrees(atan(self.height / (-self.xyFLU[0])))

        self.pRange[1] = degrees(atan(
            self.height /
            sqrt(
                (self.frontLength - self.xyFLU[0]) ** 2 +
                max(self.leftLength, self.rightLength) ** 2
            )
        ))

        # self.pRange[0] = 50

        print('pRange:', self.pRange)
        nowMinP = self.pRange[0]
        nowMaxP = self.pRange[0]
        self.pitches = []

        while nowMinP > self.pRange[1]:
            self.pitches.append(getNewP(nowMinP))
            nowP = self.pitches[-1]
            nowVFov = self.getVFovFromTopPitch(nowP)
            nowMinP = min(nowMinP, nowP - nowVFov / 2)
            nowMaxP = min(nowMaxP, nowP + nowVFov / 2)
            # print(f'podP: {self.pitches[-1]:.12f} [{nowMinP:.2f}, {nowMaxP:.2f}]')

        print('-' * 10 + 'Pod P Results' + '-' * 10)
        for ind, podP in enumerate(self.pitches):
            print(f'### Round {ind + 1}')
            print(f'p: {podP:.12f} -> hfov: {self.getHFovFromTopPitch(podP):.2f}')
            print(f'yawRange: {self.getYawRangeFromPitch(podP, addHfov=False)}')
            print(f'p Range: {podP - self.getVFovFromTopPitch(podP) / 2:.2f} ~ {podP + self.getVFovFromTopPitch(podP) / 2:.2f}')
            print(f'R Range: {self.getRFromPitch(podP - self.getVFovFromTopPitch(podP) / 2):.2f} ~ {self.getRFromPitch(podP + self.getVFovFromTopPitch(podP) / 2):.2f}')
            pass
        print('-' * 30)

        self.theList = []
        self.expectedTime = 0

        for ind, podP in enumerate(self.pitches):
            hfov = self.getHFovFromTopPitch(podP)
            yRange = self.getYawRangeFromPitch(podP, addHfov=False)
            if ind % 2 == 0:
                yRange = yRange[::-1]
            print(f'[{podP:.6f}, {yRange[0]:.2f}, {hfov:.2f}, 20], [{podP:.6f}, {yRange[1]:.2f}, {hfov:.2f}, {hfov/self.theTime:.2f}]')
            self.theList.append(PodAngles(pitchDeg=podP, yawDeg=yRange[0], hfovDeg=hfov, maxRateDeg=20, laserOn=False))
            self.theList.append(PodAngles(pitchDeg=podP, yawDeg=yRange[1], hfovDeg=hfov, maxRateDeg=hfov / self.theTime, laserOn=False))
            self.expectedTime += abs(yRange[1] - yRange[0]) / hfov * self.theTime
            print(f'After this round time is {self.expectedTime:.2f}')

        print('### The List ###')
        print(self.theList)
        print('################')
        print(f'Expected Total Time: {self.expectedTime:.2f}')

    def clipPitch(self, pitch):
        return min(self.pRange[0], max(self.pRange[1], pitch))

    def getRFromPitch(self, pitch):
        pitchCliped = self.clipPitch(pitch)
        r = self.height / tan(radians(pitchCliped))
        return r

    def getPitchFromR(self, r):
        return degrees(atan(self.height / r))

    def getPosFromPitchYaw(self, pitch, yaw):
        r = self.getRFromPitch(pitch)
        return np.array([r * cos(radians(yaw)), r * sin(radians(yaw))]) + self.xyFLU

    def getHfovFromCenterPitch(self, pitch):
        if not self.pitchLevelOn:
            return pitch
        r = self.getRFromPitch(pitch)
        halfL = min(self.horizonLength / 2, r)
        hfovExact = PodParas.clipHfov(
            2 * degrees(asin(
                halfL / r
            ))
        )
        # print(f'{pitch = :.2f} -> {hfovExact = :.2f}')
        fExact = PodParas.getFFromHfov(hfovExact)
        # print(f'F: {fExact:.2f}')
        fLevel = PodParas.getZoomLevelFromF(fExact)
        hfovLevel = PodParas.getHfovFromF(fLevel * PodParas.zoomUnit)
        # print(f'fLevel: {fLevel:.2f} -> hfovLevel: {hfovLevel:.2f}')
        return PodParas.clipHfov(hfovLevel)

    def getHFovFromTopPitch(self, pitch):
        if not self.pitchLevelOn:
            return pitch
        hfovExact = PodParas.clipHfov(PodParas.getHfovFromVFov(
            degrees(atan(
                sin(radians(pitch)) /
                (2 * PodParas.imageRatio * self.height / self.horizonLength + cos(radians(pitch)))
            )) * 2
        ))
        # print(f'{pitch = :.2f} -> {hfovExact = :.2f}')
        fExact = PodParas.getFFromHfov(hfovExact)
        # print(f'F: {fExact:.2f}')
        fLevel = PodParas.getZoomLevelFromF(fExact)
        hfovLevel = PodParas.getHfovFromF(fLevel * PodParas.zoomUnit)
        # print(f'fLevel: {fLevel:.2f} -> hfovLevel: {hfovLevel:.2f}')
        return PodParas.clipHfov(hfovLevel)

    def getVFovFromTopPitch(self, pitch):
        return PodParas.getVFovFromHFov(self.getHFovFromTopPitch(pitch))

    def getVfovFromCenterPitch(self, pitch):
        return PodParas.getVFovFromHFov(self.getHfovFromCenterPitch(pitch))

    def getHorizonLengthFromPitchHfov(self, pitch, hfov):
        r = self.getRFromPitch(pitch)
        return 2 * r * sin(radians(hfov / 2))

    def getYawRangeFromPitch(self, pitch, addHfov):
        if addHfov:
            return (
                -self.getOneSideMaxYawFromPitch(pitch, self.rightLength),
                self.getOneSideMaxYawFromPitch(pitch, self.leftLength)
            )
        else:
            return (
                -self.getOneSideMaxYawFromPitch(pitch, self.rightLength) + self.getHFovFromTopPitch(pitch) / 2,
                self.getOneSideMaxYawFromPitch(pitch, self.leftLength) - self.getHFovFromTopPitch(pitch) / 2
            )

    def getOneSideMaxYawFromPitch(self, pitch, sideLength):
        vFov = self.getVFovFromTopPitch(pitch)
        biggerPitch = pitch + vFov / 2
        rWithBiggerPitch = self.getRFromPitch(biggerPitch)
        smallerPitch = pitch - vFov / 2
        rWithSmallerPitch = self.getRFromPitch(smallerPitch)

        rToCorner = sqrt(self.xyFLU[0] ** 2 + sideLength ** 2)

        r = rToCorner
        if rWithBiggerPitch < rWithSmallerPitch <= rToCorner:
            r = rWithSmallerPitch
        elif rToCorner <= rWithBiggerPitch < rWithSmallerPitch:
            r = rWithBiggerPitch

        mostFar = np.array([0, sqrt(r ** 2 - self.xyFLU[0] ** 2)])
        if mostFar[1] > sideLength:
            mostFar[0] = sqrt(r ** 2 - sideLength ** 2) + self.xyFLU[0]
            mostFar[1] = sideLength

        rel = mostFar - self.xyFLU
        if rel[0] == 0:
            return 90
        else:
            yaw = degrees(atan(rel[1] / rel[0]))
            return yaw

    def drawSectorRing(self, ax, inner_radius, outer_radius, start_angle, end_angle, color='blue', alpha=0.2):
        sector = patches.Wedge(
            self.xyFLU,
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
        arc = patches.Arc(
            self.xyFLU,
            2*radius,
            2*radius,
            angle=0,
            theta1=start_angle,
            theta2=end_angle,
            color=color,
            alpha=alpha,
            linewidth=2
        )
        ax.add_artist(arc)

    def drawOneGlance(self, ax, pitch, yaw):
        hFov = self.getHFovFromTopPitch(pitch)
        vFov = self.getVFovFromTopPitch(pitch)
        ax.add_artist(
            patches.Wedge(
                self.xyFLU,
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
        preYaw = self.getYawRangeFromPitch(prePitch, addHfov=False)[0 if rev else 1]
        prePos = self.getPosFromPitchYaw(prePitch, preYaw)

        nxtYaw = self.getYawRangeFromPitch(nxtPitch, addHfov=False)[0 if rev else 1]
        nxtPos = self.getPosFromPitchYaw(nxtPitch, nxtYaw)

        print(f'Between Line [{prePitch:.2f}, {preYaw:.2f}] --> [{nxtPitch:.2f}, {nxtYaw:.2f}]')
        ax.plot([prePos[0], nxtPos[0]], [prePos[1], nxtPos[1]], color=color, alpha=alpha, linewidth=2)

    def drawSearchAnnulus(self, ax, pitch, color='blue', alpha=0.5, rev=False):
        hFov = self.getHFovFromTopPitch(pitch)
        vFov = self.getVFovFromTopPitch(pitch)
        yawRange = self.getYawRangeFromPitch(pitch, addHfov=True)
        print(f'This round P: {pitch:.2f} Yaw Range: {yawRange} Hfov: {self.getHFovFromTopPitch(pitch):.2f}')
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
        podYawRange = self.getYawRangeFromPitch(pitch, addHfov=False)
        self.drawRingTra(
            ax=ax,
            radius=self.getRFromPitch(pitch),
            start_angle=podYawRange[0],
            end_angle=podYawRange[1],
            color='red'
        )
        print(f'Pod Tra: [{pitch:.2f}, {podYawRange[0]:.2f}] --> [{pitch:.2f}, {podYawRange[1]:.2f}]')
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
        self.fig = plt.figure(figsize=(10, 10))
        self.ax = self.fig.add_subplot(111)
        self.ax.set_aspect('equal')
        self.ax.set_xlim(self.inflateInterval([0, self.frontLength], ratio=0.3))
        self.ax.set_ylim(self.inflateInterval([-self.leftLength, self.rightLength], ratio=0.3))

        self.ax.add_patch(
            patches.Rectangle(
                (0, -self.rightLength),
                self.frontLength,
                self.leftLength + self.rightLength,
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

        self.ax.set_title(f'Auto Tra {self}, Total Time: {self.expectedTime:.2f} seconds')
        plt.show()

    def drawHorizonLength(self):
        self.fig = plt.figure(figsize=(14,  6))
        self.axes = self.fig.subplots(1, 2)
        for ind, p in enumerate(self.pitches):
            hfov = self.getHFovFromTopPitch(p)
            pList = np.linspace(p - self.getVFovFromTopPitch(p) / 2, p + self.getVFovFromTopPitch(p) / 2, 100)
            rList = [self.getRFromPitch(p) for p in pList]
            hlList = [self.getHorizonLengthFromPitchHfov(p, hfov) for p in pList]
            self.axes[0].plot(rList, hlList, color=self.cmap(ind))
            self.axes[0].plot(self.getRFromPitch(p), self.getHorizonLengthFromPitchHfov(p, hfov), '*', color=self.cmap(ind))
            self.axes[1].plot(pList, hlList, color=self.cmap(ind))
            self.axes[1].plot(p, self.getHorizonLengthFromPitchHfov(p, hfov), '*', color=self.cmap(ind))

        self.axes[0].set_xlabel('R / m')
        self.axes[0].set_ylabel('Horizon Length / m')
        self.axes[1].set_xlabel('Pitch / deg')
        self.axes[1].set_ylabel('Horizon Length / m')
        plt.show()

    def drawHfovFromP(self):
        pList = np.linspace(self.pRange[1], self.pRange[0], 100)
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)
        self.ax.plot(pList, [self.getHFovFromTopPitch(p) for p in pList], label='Consider Top Horizon')
        self.ax.plot(pList, [self.getHfovFromCenterPitch(p) for p in pList], label='Consider Centre Horizon')
        self.ax.set_xlabel('Pitch / deg')
        self.ax.set_ylabel('Hfov / deg')
        self.ax.legend()
        plt.show()


if __name__ == '__main__':
    autoTra = AutoTra(
        pitchLevelOn=True,
        overlapOn=False,
        drawNum=-1,
        config={
            'areaPoints': [
                (0, 1700),
                (2000, 1700),
                (2000, -1500),
                (0, -1500)
            ],
            'ray': (1000, 0, 90),
            'xFLU': 0,
            'height': 40,
            'theTime': 3,
            'widthRatio': 0.06
        },
        fast=True
    )
    # autoTra.drawHfovFromP()
    autoTra.draw()
    autoTra.drawHorizonLength()
