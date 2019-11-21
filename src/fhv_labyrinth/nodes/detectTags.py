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


class Detection:

    current_pose = Point(0, 0, 0)
    max_num_tags = -1
    tags_found = []
    show_cam = False

    def __init__(self, num_tags, show_cam):
        self.max_num_tags = num_tags
        self.show_cam = show_cam
        self.detectTags()

    def setCurrentPose(self, data):

        try:
            self.current_pose = data.pose.pose.position
        except:
            rospy.logerr("AN ERROR OCCURED WHILE SETTING POSE")

    def isNewTag(self, newTag):

        if len(self.tags_found) == 0:
            return True

        for t in self.tags_found:
            distance = self.calcDistance(t, newTag)
            if distance < 0.66:
                return False
            else:
                rospy.loginfo(distance)


        return True

    def calcDistance(self, t1, t2):
        return np.sqrt((t1.x - t2.x) ** 2 + (t1.y - t2.y) ** 2)

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

            if self.show_cam:
                cv.imshow('mask', frame)
                cv.waitKey(1)

            if cv.mean(res) > (4, 4, 4) and self.isNewTag(self.current_pose):
                rospy.loginfo("NEW TAG FOUND @ x: {} y: {} z: {}".format(self.current_pose.x, self.current_pose.y, self.current_pose.z))
                self.tags_found.append(self.current_pose)
                rospy.loginfo("I NOW HAVE {} UNIQUE TAGS IN MY LIST".format(len(self.tags_found)))

                if len(self.tags_found) == self.max_num_tags:
                    rospy.loginfo("I (SHOULD) HAVE FOUND ALL TAGS")

                pub = rospy.Publisher('tags_found', Point, queue_size=10)
                pub.publish(self.current_pose)

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def detectTags(self):

        IMAGE_TOPIC = rospy.get_param("~image_topic")
        ODOMETRY_TOPIC = rospy.get_param("~odometry_topic")

        try:
            rospy.Subscriber(IMAGE_TOPIC, Image, self.processImage)
            rospy.Subscriber(ODOMETRY_TOPIC, Odometry, self.setCurrentPose)
            rospy.spin()

        finally:
            rospy.loginfo('Shutting down node...')


def main():
    rospy.init_node('detectTags')

    NUM_TAGS = rospy.get_param("~num_tags_in_maze")
    SHOW_CAMERA =  rospy.get_param("~show_camera")

    try:
        Detection(NUM_TAGS, SHOW_CAMERA)
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
