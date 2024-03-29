#! /usr/bin/env python3

import math

import numpy as np


class Classifier:
    def __init__(self):
        self.targets = []
        self.targetsScore = []
        self.targetsCnt = []
        self.targetsCheck = []
        self.targetsReal = []
        self.threshold = 2

        self.checkThreshold = 1
        self.targetThreshold = 2

    def newTarget(self, pos=[0, 0, 0], score=0):
        self.targets.append(np.array(pos))
        self.targetsScore.append(score)
        self.targetsCnt.append(1)
        self.targetsCheck.append(False)
        self.targetsReal.append(False)

    def updateTarget(self, ind, pos, score=None):
        while len(self.targets) < ind + 1:
            self.newTarget()
        self.targets[ind] = np.array(pos)
        if score is not None:
            self.targetsScore[ind] = min(self.targetsScore[ind], score)
            self.targetsCnt[ind] += 1
        if self.targetsCnt[ind] > self.targetThreshold and not self.targetsCheck[ind]:
            self.targetsCheck[ind] = True
            self.targetsReal[ind] = True

    def firstNotChecked(self):
        tC = self.targetsCheck
        for ind, value in enumerate(tC):
            if not value and self.targetsCnt[ind] >= self.checkThreshold:
                return ind
        return None

        if False not in self.targetsCheck:
            return None
        return self.targetsCheck.index(False)

    def lowestScoreIndex(self, threshold=0):
        tS, tC = self.targetsScore, self.targetsCnt
        res = 1.01
        resInd = -1
        for ind, value in enumerate(tS):
            if tC[ind] >= threshold and value <= res:
                res = value
                resInd = ind
        if resInd == -1:
            return None
        return resInd

    def newPos(self, x, y, z):
        if len(self.targets) == 0:
            self.newTarget([x, y, z])
        else:
            distances = []
            for target in self.targets:
                distances.append(math.sqrt((x - target[0]) ** 2 + (y - target[1]) ** 2 + (z - target[2]) ** 2))
            distances = np.array(distances)
            if np.any(distances < self.threshold):
                index = np.argmin(distances)
                self.updateTarget(index, [x, y, z])
            else:
                self.newTarget([x, y, z])

    def targetsList(self):
        return self.targets

    def outputTargets(self):
        outputT = self.targets
        for ind, t in enumerate(outputT):
            print(f'[{ind + 1}] [{self.targetsCnt[ind]}] @ {t[0]:.2f}, {t[1]:.2f}, {t[2]:.2f}')

    def clear(self):
        self.targets.clear()
        self.targetsScore.clear()
        self.targetsCnt.clear()
        self.targetsCheck.clear()
        self.targetsReal.clear()
