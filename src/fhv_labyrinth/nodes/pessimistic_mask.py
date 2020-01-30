from PIL import Image,ImageDraw
import numpy as np
import tf

pi = np.pi
inf = np.inf
dot = np.dot
sin = np.sin
cos = np.cos
ar = np.array
arange = np.arange

def Rotate2D(pts,cnt,ang=pi/4):
    return dot(pts-cnt,ar([[cos(ang),sin(ang)],[-sin(ang),cos(ang)]]))+cnt

def trapezeMask(center, view):
    (d_min, d_max), (w_min, w_max) = view

    # isosceles triangle facing up
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
    view_distance, view_width = view

    # isosceles triangle facing up
    p1x, p1y = p1 = center
    p2 = p1x - view_width / 2, p1y - view_distance
    p3 = p1x + view_width / 2, p1y - view_distance

    pts = ar((p1, p2, p3))
    return pts

def createPessimisticMask(map_msg, odom_msg, scan_msg, view):
    # use odom_msg
    _pose = odom_msg.pose
    pos_xy = ar((_pose.position.x, _pose.position.y))
    roll, pitch, yaw = tf.transformations.euler_from_quaternion([_pose.orientation.x, _pose.orientation.y, _pose.orientation.z, _pose.orientation.w])

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
    
    # use view
    # min_distance, max_distance, min_width, max_width = view
    # view_distance, view_width = view
    
    #**************************************************************************
    
    dimension = (width, height)
    center = ((pos_xy - origin_xy) / resolution).astype(int)
    ang = yaw

    # create view_mask
    pts = trapezeMask(center, view)
    pts_rot = tuple(map(tuple, Rotate2D(pts,center,ang=ang).astype(int)))
    im = Image.new('L', dimension, 0)
    ImageDraw.Draw(im).polygon(pts_rot, fill=1, outline=1)
    view_mask = ar(im)

    # create map_mask
    ranges[ranges==inf] = 0 # substitute infinity with 0
    xs = ranges * cos(angles)
    ys = ranges * sin(angles)
    pts = (ar((xs, ys)).T / resolution + center).astype(int)
    pts_rot = tuple(map(tuple, Rotate2D(pts,center,ang=ang).astype(int)))
    im = Image.new('L', dimension, 0)
    ImageDraw.Draw(im).polygon(pts_rot, fill=1, outline=1)
    map_mask = ar(im)

    mask = view_mask * map_mask
    return mask == 1
