from math import tan, atan, radians, degrees

zoomUnit = 4.5
maxHfov = 63.49
minHfov = 2.36

sensorWidth = 1920 * 0.0029
viewTypeDict = {0: 'EO1', 1: 'EO2', 2: 'IR1', 3: 'IR2'}


def getFFromHfov(hFov):
    return sensorWidth / 2 / tan(radians(hFov / 2))

def getHfovFromF(f):
    return degrees(atan(sensorWidth / 2 / f)) * 2

def getZoomLevelFromF(zoom):
    zoomLevel = zoom / zoomUnit
    res = (int(zoomLevel * 10)) / 10
    print(f'zoom: {zoom} res: {res}')
    return res