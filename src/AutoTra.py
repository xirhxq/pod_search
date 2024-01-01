#!/usr/bin/env python3

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from math import degrees, atan, tan, radians, sin, cos, sqrt, asin, atan2
import numpy as np
from rich.prompt import FloatPrompt
from shapely.geometry import Point, Polygon

import PodParas
from PodAngles import PodAngles


def inflateInterval(interval, ratio):
    return [interval[0] - (interval[1] - interval[0]) * ratio / 2, interval[1] + (interval[1] - interval[0]) * ratio / 2]


class AutoTra:
    def __repr__(self):
        return (
            f'AutoTra('
            f'uavPos=({self.uavPos.x:.1f}, {self.uavPos.y:.1f}), '
            f'yaw={self.yaw:.1f}, '
            f'yawRange={self.config["yawRange"]:.1f}, '
            f'T={self.theTime:.1f}, '
            f'w={self.widthRatio:.2f})'
        )

    def __init__(self,
                 overlapOn=True,
                 pitchLevelOn=True,
                 drawNum=-1,
                 fast=False,
                 config=None
        ):
        self.config = config
        self.area = Polygon(self.config['areaPoints'])
        self.uavPos = Point(self.config['uavPos'][0], self.config['uavPos'][1])
        self.height = self.config['uavPos'][2]
        self.yaw = self.config['yaw']
        self.yaws = [self.yaw - self.config['yawRange'] / 2, self.yaw + self.config['yawRange'] / 2]
        self.yawLinspaceDeg = np.linspace(self.yaws[0], self.yaws[1], 100)
        self.theTime = float(config['theTime'])
        self.widthRatio = float(config['widthRatio'])
        if not fast:
            self.widthRatio = FloatPrompt.ask('width ratio', default=self.widthRatio)
            self.theTime = FloatPrompt.ask('THE Time', default=self.theTime)

        self.vesselLength = 10
        self.horizonLength = self.vesselLength / self.widthRatio

        self.range2DMax = 0
        for point in self.area.exterior.coords:
            self.range2DMax = max(
                self.range2DMax,
                sqrt((point[0] - self.uavPos.x) ** 2 + (point[1] - self.uavPos.y) ** 2)
            ) * 2

        self.raysPolygonPointList = [self.uavPos]
        for yawDeg in self.yawLinspaceDeg:
            yaw = radians(yawDeg)
            self.raysPolygonPointList.append(
                Point(
                    self.uavPos.x + self.range2DMax * cos(yaw),
                    self.uavPos.y + self.range2DMax * sin(yaw)
                )
            )
        self.raysPolygon = Polygon(self.raysPolygonPointList)

        self.searchPolygon = self.raysPolygon.intersection(self.area)

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

        self.distToSearchPolygon = self.uavPos.distance(self.searchPolygon)
        self.pMax = self.getPitchFromR(self.distToSearchPolygon)
        self.pRange = [self.pMax, self.pMax]
        for point in self.searchPolygon.exterior.coords:
            range2D = sqrt((point[0] - self.uavPos.x) ** 2 + (point[1] - self.uavPos.y) ** 2)
            p = self.getPitchFromR(range2D)
            self.pRange[1] = min(self.pRange[1], p)

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

        self.theList = []
        self.expectedTime = 0

        for ind, podP in enumerate(self.pitches):
            hfov = self.getHFovFromTopPitch(podP)
            yRange = self.getYawRangeFromPitch(podP, addHfov=False)
            pRange = (podP - PodParas.getVFovFromHFov(hfov) / 2, podP + PodParas.getVFovFromHFov(hfov) / 2)
            rRange = [self.getRFromPitch(p) for p in pRange]

            print(f'### Round {ind + 1}')
            print(f'{podP=:.2f} -> {hfov=:.2f}')
            print(f'{yRange=}')
            print(f'{pRange=}')
            print(f'{rRange=}')

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

    def adjustAndShow(self, ax):
        ax.set_xlim(min(self.config['areaPoints'], key=lambda x: x[0])[0], max(self.config['areaPoints'], key=lambda x: x[0])[0])
        ax.set_ylim(min(self.config['areaPoints'], key=lambda x: x[1])[1], max(self.config['areaPoints'], key=lambda x: x[1])[1])
        # make sure that xlim & ylim include self.uavPos
        ax.set_xlim(min(ax.get_xlim()[0], self.uavPos.x), max(ax.get_xlim()[1], self.uavPos.x))
        ax.set_ylim(min(ax.get_ylim()[0], self.uavPos.y), max(ax.get_ylim()[1], self.uavPos.y))
        ax.set_xlim(inflateInterval(ax.get_xlim(), 0.1))
        ax.set_ylim(inflateInterval(ax.get_ylim(), 0.1))
        ax.set_aspect('equal')
        plt.show()

    def drawBasic(self, ax):
        ax.add_patch(patches.Polygon(self.area.exterior.coords, color='k', alpha=0.2))

    def drawPoly(self, ax):
        # self.ax.add_patch(patches.Polygon(self.raysPolygon.exterior.coords, color='r', alpha=0.2))
        ax.add_patch(patches.Polygon(self.searchPolygon.exterior.coords, color='b', alpha=0.2))

    def clipPitch(self, pitch):
        return min(self.pRange[0], max(self.pRange[1], pitch))

    def getRFromPitch(self, pitch):
        pitchCliped = self.clipPitch(pitch)
        r = self.height / tan(radians(pitchCliped))
        return r

    def getPitchFromR(self, r):
        if r < 1e-3:
            return 90
        return degrees(atan(self.height / r))

    def getPosFromPitchYaw(self, pitch, yaw):
        r = self.getRFromPitch(pitch)
        return np.array([
            r * cos(radians(yaw + self.yaw)) + self.uavPos.x,
            r * sin(radians(yaw + self.yaw)) + self.uavPos.y
        ])

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
        return PodParas.getHfovFromExactHfov(hfovExact)

    def getHFovFromTopPitch(self, pitch):
        if not self.pitchLevelOn:
            return pitch
        hfovExact = PodParas.clipHfov(PodParas.getHfovFromVFov(
            degrees(atan(
                sin(radians(pitch)) /
                (2 * PodParas.imageRatio * self.height / self.horizonLength + cos(radians(pitch)))
            )) * 2
        ))
        return PodParas.getHfovFromExactHfov(hfovExact)

    def getVFovFromTopPitch(self, pitch):
        return PodParas.getVFovFromHFov(self.getHFovFromTopPitch(pitch))

    def getVfovFromCenterPitch(self, pitch):
        return PodParas.getVFovFromHFov(self.getHfovFromCenterPitch(pitch))

    def getHorizonLengthFromPitchHfov(self, pitch, hfov):
        r = self.getRFromPitch(pitch)
        return 2 * r * sin(radians(hfov / 2))

    def getAbsYawFromPos(self, pos: Point):
        return degrees(atan2(pos.y - self.uavPos.y, pos.x - self.uavPos.x))

    def getRelYawFromPos(self, pos: Point):
        return np.mod(self.getAbsYawFromPos(pos) - self.yaw + 180, 360) - 180

    def getYawRangeFromPitch(self, pitch, addHfov):
        # if addHfov:
        #     return (
        #         -self.config['yawRange'] / 2,
        #         self.config['yawRange'] / 2
        #     )
        # else:
        #     return (
        #         -self.config['yawRange'] / 2 + self.getHFovFromTopPitch(pitch) / 2,
        #         self.config['yawRange'] / 2 - self.getHFovFromTopPitch(pitch) / 2
        #     )
        # r = self.getRFromPitch(pitch)
        # print(f'{pitch=:.2f} {r=:.2f}')
        rs = [
            self.getRFromPitch(pitch),
            self.getRFromPitch(pitch - self.getVFovFromTopPitch(pitch) / 2),
            self.getRFromPitch(pitch + self.getVFovFromTopPitch(pitch) / 2)
        ]

        edgePoints = [
            Point(
                self.uavPos.x + r * cos(radians(yaw)),
                self.uavPos.y + r * sin(radians(yaw))
            )
            for yaw in self.yawLinspaceDeg for r in rs
        ]
        # print(f'{edgePoints=}')

        edgePoints = [
            edgePoint
            for edgePoint in edgePoints
            if self.searchPolygon.buffer(10).contains(edgePoint)
        ]
        # print(f'{edgePoints=}')
        edgeYaws = [
            self.getRelYawFromPos(edgePoint)
            for edgePoint in edgePoints
        ]
        # print(f'Rel {edgeYaws=}')
        print(f'min: {min(edgeYaws):.2f} max: {max(edgeYaws):.2f}')

        if addHfov:
            return (
                min(edgeYaws),
                max(edgeYaws)
            )
        else:
            hFov = self.getHFovFromTopPitch(pitch)
            return (
                min(edgeYaws) + hFov / 2,
                max(edgeYaws) - hFov / 2
            )

    def drawSectorRing(self, ax, inner_radius, outer_radius, start_angle, end_angle, color='blue', alpha=0.2):
        sector = patches.Wedge(
            (self.uavPos.x, self.uavPos.y),
            outer_radius,
            start_angle + self.yaw,
            end_angle + self.yaw,
            width=outer_radius - inner_radius,
            facecolor=color,
            alpha=alpha,
            edgecolor='none'
        )
        ax.add_artist(sector)

    def drawRingTra(self, ax, radius, start_angle, end_angle, color='blue', alpha=1):
        arc = patches.Arc(
            (self.uavPos.x, self.uavPos.y),
            2*radius,
            2*radius,
            angle=0,
            theta1=start_angle + self.yaw,
            theta2=end_angle + self.yaw,
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
                (self.uavPos.x, self.uavPos.y),
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

    def draw(self, ax):
        ax.set_aspect('equal')

        # self.drawBasic(ax=ax)
        self.drawPoly(ax=ax)

        for ind, p in enumerate(self.pitches):
            if 0 < self.drawNum <= ind:
                break
            print(f'--- Round #{ind + 1} ---')
            if ind > 0:
                self.drawBetweenLine(ax, self.pitches[ind - 1], p, rev=(ind % 2 == 0), color='red', alpha=1.0)
            color = self.cmap(ind)
            self.drawSearchAnnulus(ax, p, color=color, alpha=0.3, rev=(ind % 2 == 0))

        if ax.get_title() != '':
            ax.set_title(f'{ax.get_title()}\n{self}, Total Time:{self.expectedTime:.2f} seconds')
        else:
            ax.set_title(f'{self}, Total Time: {self.expectedTime:.2f} seconds')

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


def testMultipleAutoTras(n, baseYawDeg=0):
    yawUnit = 360 / n
    yaws = [yawUnit * i + baseYawDeg for i in range(n)]
    autoTras = [AutoTra(
        pitchLevelOn=True,
        overlapOn=True,
        drawNum=-1,
        config={
            'areaPoints': [
                (0, 1700),
                (2000, 1700),
                (2000, -1500),
                (0, -1500)
            ],
            'uavPos': [1000, 0, 40],
            'yaw': yaw,
            'yawRange': yawUnit,
            'theTime': 3,
            'widthRatio': 0.06
        },
        fast=True
    )
    for yaw in yaws]

    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111)
    autoTras[0].drawBasic(ax)
    [at.draw(ax) for at in autoTras]
    plt.show()


def testSingleAutoTra():
    autoTra = AutoTra(
        pitchLevelOn=True,
        overlapOn=True,
        drawNum=-1,
        config={
            'areaPoints': [
                (0, 1700),
                (2000, 1700),
                (2000, -1500),
                (0, -1500)
            ],
            'uavPos': [1000, 0, 40],
            'yaw': 330,
            'yawRange': 120,
            'theTime': 3,
            'widthRatio': 0.06
        },
        fast=True
    )

    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111)
    autoTra.drawBasic(ax)
    autoTra.draw(ax)
    plt.show()
    # autoTra.draw(ax)
    # autoTra.drawHorizonLength()
    # print(autoTra.theList)


if __name__ == '__main__':
    testSingleAutoTra()
    # testMultipleAutoTras(3, baseYawDeg=90)