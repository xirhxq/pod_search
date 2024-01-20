from filterpy.kalman import ExtendedKalmanFilter as EKF
import numpy as np
import time


class LocatingEKF:
    def __init__(self, initialT=0):
        self.ekf = EKF(dim_x=6, dim_z=4)
        self.ekf.x = None
        self.dt = None

        self.ekf.R = np.diag([1., 0.5, 0.5, 5])

        self.t = initialT
        self.updateTimes = []

    def setFQ(self):
        self.ekf.F = np.eye(6)

        sigmaA = 1.
        self.ekf.Q = np.zeros((6, 6))
        self.ekf.Q[0:3, 0:3] = np.eye(3) * sigmaA * self.dt ** 4 / 3
        self.ekf.Q[0:3, 3:6] = np.eye(3) * sigmaA * self.dt ** 3 / 2
        self.ekf.Q[3:6, 0:3] = np.eye(3) * sigmaA * self.dt ** 3 / 2
        self.ekf.Q[3:6, 3:6] = np.eye(3) * self.dt

    def firstFrame(self, Z, uavPos, rP2B, rB2I):
        r, alpha, epsilon, h = Z[0][0], Z[1][0], Z[2][0], Z[3][0]
        if r == 0 or r >= 3000:
            return None
        pCamera = np.array([
            [1],
            [np.tan(np.radians(alpha))],
            [np.tan(np.radians(epsilon))]
        ]) * r / np.linalg.norm(np.array([1, np.tan(np.radians(alpha)), np.tan(np.radians(epsilon))]))
        pI = rB2I.T @ rP2B.T @ pCamera
        return np.array([
            [uavPos[0][0] + pI[0][0]],
            [uavPos[1][0] + pI[1][0]],
            [uavPos[2][0] + pI[2][0]],
            [0],
            [0],
            [0]
        ])

    def newFrame(self, t, Z, uavPos, rP2B, rB2I):
        self.dt = t - self.t
        self.t = t

        if self.ekf.x is None:
            self.ekf.x = self.firstFrame(Z, uavPos, rP2B, rB2I)

        else:
            self.setFQ()
            if Z[0] < 10 or Z[0] >= 3000:
                self.ekf.predict()
                return self.ekf.x

            def HJacobianAt(x):
                pCamera = rP2B @ rB2I @ (x[0:3] - uavPos)
                pCx, pCy, pCz = pCamera[0][0], pCamera[1][0], pCamera[2][0]
                r = np.linalg.norm(pCamera)
                rCP = np.array([
                    [pCx / r, pCy / r, pCz / r],
                    [180. / np.pi * pCy / (pCx ** 2 + pCy ** 2), -180. / np.pi * pCx / (pCx ** 2 + pCy ** 2), 0],
                    [180. / np.pi * pCz / (pCx ** 2 + pCy ** 2 + pCz ** 2), 0, -180. / np.pi * pCx / (pCx ** 2 + pCy ** 2)]
                ]).reshape((3, 3))
                ret = np.zeros((4, 6))
                ret[0:3, 0:3] = rCP @ rP2B @ rB2I
                ret[3, 2] = 1
                return ret

            def hx(x):
                pCamera = rP2B @ rB2I @ (x[0:3] - uavPos)
                pCx, pCy, pCz = pCamera[0][0], pCamera[1][0], pCamera[2][0]
                r = np.linalg.norm(pCamera)
                return np.array([
                    [r],
                    [-np.degrees(np.arctan2(pCy, pCx))],
                    [-np.degrees(np.arctan2(pCz, pCx))],
                    [x[2][0] + uavPos[2][0]]
                ])

            self.updateTimes.append(self.t)

            self.ekf.update(Z, HJacobianAt, hx)
            self.ekf.predict()

        return self.ekf.x

