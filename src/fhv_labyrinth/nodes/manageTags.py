#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from enum import Enum


class Manage:
    """
    This Node manages all found Tags.
    It keeps a list of tags and adds only found tags, that are not yet in that list.
    """

    TAGS = []

    def __init__(self):
        """
        Initialize the node
        """
        self.listenForTags()

    def isNewTag(self, newTag):
        """
        Checks, if the tag is already in the list by comparing the locations of each tag.
        """

        if len(self.TAGS) == 0:
            return True

        for t in self.TAGS:
            if self.calcDistance(t, newTag) < 0.05:
                return False

        return True

    def calcDistance(self, t1, t2):
        """
        Returns the euclidean distance between two tags
        """
        return np.sqrt((t1.x - t2.x) ** 2 + (t1.y - t2.y) ** 2)

    def processTag(self, data):

        if self.isNewTag(data):
            rospy.loginfo("NEW TAG WAS FOUND")
            self.TAGS.append(data)
        else:
            rospy.loginfo("TAG ALREADY FOUND")

    def listenForTags(self):
        """
        Initializes the subscriber for found tags
        """

        try:
            while not rospy.is_shutdown():
                rospy.Subscriber("tags_found", Point, self.processTag)
                rospy.spin()

        finally:
            rospy.loginfo('Shutting down node...')


def main():
    rospy.init_node('manageTags')

    try:
        Manage()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
