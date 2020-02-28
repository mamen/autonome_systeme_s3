from PIL import Image,ImageDraw
import numpy as np

pi = np.pi
inf = np.inf
dot = np.dot
sin = np.sin
cos = np.cos
ar = np.array
arange = np.arange

def Rotate2D(pts,cnt,ang=pi/4):
    """
    rotate many 2D points around a centered point by angle
    """
    return dot(pts-cnt,ar([[cos(ang),sin(ang)],[-sin(ang),cos(ang)]]))+cnt

def trapezeMask(center, view):
    """
    creates an emulated perspective of the camera's sight field in form of a trapeze
    """
    (d_min, d_max), (w_min, w_max) = view

    # isosceles trapeze facing up
    cx, cy = center

    # top left
    p1 = cx + d_max, cy + w_max
    # top right
    p2 = cx + d_max, cy - w_max
    # bottom right
    p3 = cx + d_min, cy - w_min
    # bottom left
    p4 = cx + d_min, cy + w_min

    pts = ar((p1, p2, p3, p4))
    return pts

def triangleMask(center, view):
    """
    creates an emulated perspective of the camera's sight field in form of an isosceles triangle
    this was replaced by the trapeze due to unreal behavior.
    """
    view_distance, view_width = view

    # isosceles triangle facing up
    p1x, p1y = p1 = center
    p2 = p1x - view_width / 2, p1y - view_distance
    p3 = p1x + view_width / 2, p1y - view_distance

    pts = ar((p1, p2, p3))
    return pts

def createPessimisticMask(map_msg, scan_msg, view, pos_xy, yaw):
    # use map_msg
    _info = map_msg.info
    origin_xy = ar((_info.origin.position.x, _info.origin.position.y))
    height = _info.height
    width = _info.width
    resolution = _info.resolution

    # use scan_msg
    _scan = scan_msg
    ranges = ar(_scan.ranges)
    angles = arange(_scan.angle_min, _scan.angle_max + _scan.angle_increment / 2, _scan.angle_increment)
    
    # define helper
    dimension = (width, height)
    center = ((pos_xy - origin_xy) / resolution).astype(int)

    # create view_mask
    pts = trapezeMask(center, view)
    pts_rot = tuple(map(tuple, Rotate2D(pts,center,ang=yaw).astype(int)))
    im = Image.new('L', dimension, 0)
    ImageDraw.Draw(im).polygon(pts_rot, fill=1, outline=1)
    view_mask = ar(im)

    # create map_mask
    ranges[ranges==inf] = 0 # substitute infinity with 0
    xs = ranges * cos(angles)
    ys = ranges * sin(angles)
    pts = (ar((xs, ys)).T / resolution + center).astype(int)
    pts_rot = tuple(map(tuple, Rotate2D(pts,center,ang=yaw).astype(int)))
    im = Image.new('L', dimension, 0)
    ImageDraw.Draw(im).polygon(pts_rot, fill=1, outline=1)
    map_mask = ar(im)

    # prevent that the emulated view mask sees through walls
    mask = view_mask * map_mask
    return mask == 1
