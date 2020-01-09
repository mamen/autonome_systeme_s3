#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import time
from os.path import expanduser


class Scan:

    def __init__(self):
        self.listenForNewScan()

    def listenForNewScan(self):

        try:
            print("waiting")
            while not rospy.is_shutdown():
                rospy.Subscriber("/scan", LaserScan, self.processScan)
                rospy.spin()

        finally:
            rospy.loginfo('Shutting down node...')

    def filter(self, data, min_num_zero_to_keep):
        i = 0

        data = list(data)

        # cur_time = int(time.time())
        #
        # # Filename to write
        # filename = "{}/lidar/{}.csv".format(expanduser("~"), cur_time)
        #
        # # Open the file with writing permission
        # myfile = open(filename, 'w')
        #
        # # Write a line to the file
        # myfile.write(data.__str__().replace("(","").replace(")",""))
        #
        # # Close the file
        # myfile.close()

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

        # # Filename to write
        # filename = "{}/lidar/{}_fixed.csv".format(expanduser("~"), cur_time)
        #
        # # Open the file with writing permission
        # myfile = open(filename, 'w')
        #
        # # Write a line to the file
        # myfile.write(tuple(data).__str__().replace("(","").replace(")",""))
        #
        # # Close the file
        # myfile.close()

        return tuple(data)

    def processScan(self, data):

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
