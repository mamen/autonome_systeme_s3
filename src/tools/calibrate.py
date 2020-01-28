import pylab
import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt


def selectPoints(imgPath, nrPoints):
    fig = pylab.figure(1)
    fig.add_subplot(111).imshow(cv.imread(imgPath))
    pts = np.asanyarray(fig.ginput(nrPoints))
    fig.show()
    return pts




pts1 = selectPoints("/home/markus/Desktop/image.png", 25)

pts2 = np.asarray([
    [-20.0, 40.0],
    [-10.0, 40.0],
    [0.0, 40.0],
    [10.0, 40.0],
    [20.0, 40.0],

    [-20.0, 50.0],
    [-10.0, 50.0],
    [0.0, 50.0],
    [10.0, 50.0],
    [20.0, 50.0],

    [-20.0, 60.0],
    [-10.0, 60.0],
    [0.0, 60.0],
    [10.0, 60.0],
    [20.0, 60.0],

    [-20.0, 70.0],
    [-10.0, 70.0],
    [0.0, 70.0],
    [10.0, 70.0],
    [20.0, 70.0],

    [-20.0, 80.0],
    [-10.0, 80.0],
    [0.0, 80.0],
    [10.0, 80.0],
    [20.0, 80.0]
])

H = cv.findHomography(pts1, pts2)

print("H = np.array(" + str(H[0].tolist()) + ")")

# H = np.array([[-0.09288180869054626, 0.0054416583613956645, 55.428253772214454],
#               [0.0025116067633560767, 0.0029759558170538375, -97.45956201150065],
#               [0.00010195975796577268, -0.006257619108708003, 1.0]])

# selectionPoint = selectPoints("/home/markus/Desktop/bla.png", 1)
# homoPoint = np.asarray([selectionPoint[0][0], selectionPoint[0][1], 1])
# ptsResult = H.dot(homoPoint)
#
#
# x = ptsResult[1] / ptsResult[2]
# y = ptsResult[0] / ptsResult[2]
#
# print(x, y)

# plt.imshow(cv.imread("/home/markus/Desktop/test.png"))
#
# plt.scatter(x, y)
#
# plt.show()
