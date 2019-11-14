#!/usr/bin/env python

import rospy
from numpy import array, reshape, where
from nav_msgs.msg import OccupancyGrid
import cv2 as cv
import numpy as np
from cv2 import imshow, waitKey
from time import sleep

def processMap(grid):
    # grid.data (tuple of 147456 = 384**2 numbers)
    data = reshape(array(grid.data), (grid.info.width, grid.info.height))
    # data = ndarray (-1=undiscovered, 0=free, 100=occupied)
    # find free fields
    # imshow('foo', data)
    # waitKey(1)
    # (X, Y) = where(data == 0)
    # rospy.loginfo(X[0])
    # rospy.loginfo(Y[0])

def run():
    map_topic = rospy.get_param("~map_topic")
    map_subscription = rospy.Subscriber(map_topic, OccupancyGrid, processMap)
    # pub = rospy.Publisher('detectTags', TagDetected, queue_size=10)
    #IMAGE_TOPIC = rospy.get_param("~img_topic")
    while not rospy.is_shutdown():
        # rospy.loginfo(text)
        sleep(1)

def main():
    rospy.init_node('room_cleaner')

    try:
        run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
