#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from enum import Enum


class Color(Enum):
    RED = 1,
    BLUE = 2


class Detection():

    current_pose =  Point(0, 0, 0)

    def __init__(self):
        self.detectTags()

    def setCurrentPose(self, data):

        try:
            self.current_pose = data.pose.pose.position
        except:
            rospy.logerr("AN ERROR OCCURED WHILE SETTING POSE")

    def processImage(self, data):
        COLOR = Color.RED

        LOWER_LIMIT = UPPER_LIMIT = np.array([0, 0, 0])

        if COLOR == Color.RED:
            LOWER_LIMIT = np.array([0, 50, 50])
            UPPER_LIMIT = np.array([10, 255, 255])
        elif COLOR == Color.BLUE:
            LOWER_LIMIT = np.array([30, 150, 50])
            UPPER_LIMIT = np.array([255, 255, 180])

        bridge = CvBridge()

        try:
            frame = bridge.imgmsg_to_cv2(data, "passthrough")

            hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
            mask = cv.inRange(hsv, LOWER_LIMIT, UPPER_LIMIT)
            res = cv.bitwise_and(frame, frame, mask=mask)

            if cv.mean(res) > (4, 4, 4):
                rospy.loginfo("TAG FOUND @ x: {} y: {} z: {}".format(self.current_pose.x, self.current_pose.y, self.current_pose.z))

                # TODO:
                # publish to token-found-topic

            # cv.imshow('res', res)
            # cv.waitKey(1)

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def detectTags(self):

        IMAGE_TOPIC = rospy.get_param("~image_topic")
        ODOMETRY_TOPIC = rospy.get_param("~odometry_topic")

        try:
            while not rospy.is_shutdown():
                rospy.Subscriber(IMAGE_TOPIC, Image, self.processImage)
                rospy.Subscriber(ODOMETRY_TOPIC, Odometry, self.setCurrentPose)
                rospy.spin()

        finally:
            rospy.loginfo('Shutting down node...')


def main():
    rospy.init_node('detectTags')

    try:
        Detection()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
