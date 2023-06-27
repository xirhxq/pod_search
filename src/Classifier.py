#! /usr/bin/env python3

import math
import numpy as np


class Classifier:
    def __init__(self):
        self.targets = []
        self.targetsCnt = []
        self.targetsCheck = []
        self.targetsReal = []
        self.threshold = 1

    def newTarget(self, pos):
        self.targets.append(pos)
        self.targetsCnt.append(1)
        self.targetsCheck.append(False)
        self.targetsReal.append(False)

    def updateTarget(self, ind, pos):
        self.targets[ind] = pos
        self.targetsCnt[ind] += 1
        if self.targetsCnt[ind] > 50:
            self.targetsCheck[ind] = True
            self.targetsReal[ind] = True

    def firstNotChecked(self):
        if False not in self.targetsCheck:
            return None
        return self.targetsCheck.index(False)

    def newPos(self, x, y, z):
        if len(self.targets) == 0:
            self.newTarget([x, y, z])
        else:
            distances = []
            for target in self.targets:
                distances.append(math.sqrt((x - target[0])**2 + (y - target[1])**2 + (z - target[2])**2))
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
