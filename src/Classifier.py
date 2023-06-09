#! /usr/bin/env python3

import math
import numpy as np


class Classifier:
    def __init__(self):
        self.targets = []
        self.threshold = 100

    def newPos(self, x, y):
        if len(self.targets) == 0:
            self.targets.append([x, y])
        else:
            # calculate the distance between the new position and all the known targets
            distances = []
            for target in self.targets:
                distances.append(math.sqrt((x - target[0])**2 + (y - target[1])**2))
            distances = np.array(distances)
            # if the distance is less than 100m of any known target, then classify it as that target
            if np.any(distances < self.threshold):
                # find the index of the minimum distance
                index = np.argmin(distances)
                # update the target position
                self.targets[index] = [x, y]
            # if the distance is greater than 100m of all known targets, then classify it as a new type of target
            else:
                self.targets.append([x, y])

    def targetsList(self):
        return self.targets