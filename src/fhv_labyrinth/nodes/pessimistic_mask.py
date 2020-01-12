from PIL import Image,ImageDraw
import numpy as np

pi = np.pi
dot = np.dot
sin = np.sin
cos = np.cos
ar = np.array
arange = np.arange
rad = lambda ang: ang*pi/180

def Rotate2D(pts,cnt,ang=pi/4):
    return dot(pts-cnt,ar([[cos(ang),sin(ang)],[-sin(ang),cos(ang)]]))+cnt

def createPessimisticMask(dimension, sight_distance, sight_width, pos, yaw):
    # isosceles triangle facing up
    p1x, p1y = p1 = pos
    p2 = p1x - sight_width / 2, p1y - sight_distance
    p3 = p1x + sight_width / 2, p1y - sight_distance

    pts = ar((p1, p2, p3))
    cnt = p1
    ang = yaw + pi / 2

    q1, q2, q3 = tuple(map(tuple, Rotate2D(pts,cnt,ang=ang).astype(int)))

    # draw it
    im = Image.new('L', dimension, 0)
    ImageDraw.Draw(im).polygon([q1, q2, q3], fill=1, outline=1)

    mask = ar(im)
    return mask

if __name__ == "__main__":
    dimension = (10, 10)
    pos = (4, 4)
    sight_distance = 7
    sight_width = 4
    yaw = pi * 1 / 8

    mask = createPessimisticMask(dimension, sight_distance, sight_width, pos, yaw)

    print(mask)
