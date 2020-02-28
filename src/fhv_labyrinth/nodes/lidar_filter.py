#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import time
from os.path import expanduser

class Scan:
    """
    This Node filters out holes in the LIDAR-scan, caused by small spaces between the walls of the labyrinth 
    """

    def __init__(self):
        """
        Node initialization
        """
        self.listenForNewScan()

    def listenForNewScan(self):
        """
        Subscribes the node to the LIDAR-scan topic
        """

        try:
            print("waiting")
            while not rospy.is_shutdown():
                rospy.Subscriber("/scan", LaserScan, self.processScan)
                rospy.spin()

        finally:
            rospy.loginfo('Shutting down node...')

    def filter(self, data, min_num_zero_to_keep):
        """
        Filters the LIDAR-data and removes holes, that are bigger than min_num_zero_to_keep
        A hole is represented as a 0 in the LIDAR-data
        """
        i = 0

        data = list(data)

        while i < len(data):

            if data[i] == 0:
                i += 1
                continue

            if data[(i + min_num_zero_to_keep + 1) % len(data)] == 0:
                i += 1
                continue

            count = 0

            for j in range(1, min_num_zero_to_keep + 1):
                if data[(i + j) % len(data)] == 0:
                    count += 1

            for j in range(1, min_num_zero_to_keep + 1):
                if data[(i + j) % len(data)] == 0:
                    if j <= count / 2:
                        data[(i + j) % len(data)] = data[i]
                    else:
                        data[(i + j) % len(data)] = data[(i + count + 1) % len(data)]

            i += 1

        return tuple(data)

    def processScan(self, data):
        """
        Publish the fixed scan-data
        """

        data.ranges = self.filter(data.ranges, 12)

        print("published new data")
        pub = rospy.Publisher('/scan/filtered', LaserScan, queue_size=100)
        pub.publish(data)


def main():
    rospy.init_node('lidar_filter')

    print("starting node")

    try:
        Scan()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
