from math import tan, atan, radians, degrees

zoomUnit = 4.5
maxHfov = 63.49
minHfov = 2.36

sensorWidth = 1920 * 0.0029
viewTypeDict = {0: 'EO1', 1: 'EO2', 2: 'IR1', 3: 'IR2'}

imageRatio = 16 / 9

podDelay = 0.6


def getFFromHfov(hFov):
    return sensorWidth / 2 / tan(radians(hFov / 2))


def getHfovFromF(f):
    return degrees(atan(sensorWidth / 2 / f)) * 2


def getZoomLevelFromF(zoom):
    zoomLevel = zoom / zoomUnit
    res = (int(zoomLevel * 10)) / 10
    return res


def clipHfov(hfov):
    return min(maxHfov, max(minHfov, hfov))


def getVFovFromHFov(hFov):
    return 2 * degrees(atan(tan(radians(hFov / 2)) / imageRatio))


def getHfovFromVFov(vFov):
    return 2 * degrees(atan(tan(radians(vFov / 2)) * imageRatio))


def getHfovFromExactHfov(hfovDeg):
    return clipHfov(
        getHfovFromF(
            getZoomLevelFromF(
                getFFromHfov(hfovDeg)
            ) * zoomUnit
        )
    )


if __name__ == '__main__':
    import numpy as np
    import matplotlib.pyplot as plt
    hfovExactList = np.linspace(2.4, 60, 100)
    hfovList = [
        getHfovFromExactHfov(yaw)
        for yaw in hfovExactList
    ]
    plt.plot(hfovExactList, hfovList)
    plt.xlabel('exact hfov / deg')
    plt.ylabel('hfov / deg')
    plt.show()
