#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from enum import Enum
from os.path import expanduser
import time


class Color(Enum):
    RED = 1,
    BLUE = 2,
    GREEN = 3


class Detection:

    current_pose = Point(0, 0, 0)
    max_num_tags = -1
    tags_found = []
    show_cam = False
    tags_color = None

    def __init__(self, num_tags, show_cam, tags_color):
        self.max_num_tags = num_tags
        self.show_cam = show_cam
        self.tags_color = tags_color
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

        LOWER_LIMIT = UPPER_LIMIT = np.array([0, 0, 0])

        if self.tags_color == Color.RED:
            LOWER_LIMIT = np.array([0, 50, 50])
            UPPER_LIMIT = np.array([10, 255, 255])
        elif self.tags_color == Color.BLUE:
            LOWER_LIMIT = np.array([100,150,0])
            UPPER_LIMIT = np.array([140,255,255])
        elif self.tags_color == Color.GREEN:
            LOWER_LIMIT = np.array([60, 10, 10])
            UPPER_LIMIT = np.array([80, 255, 255])

        bridge = CvBridge()

        try:
            frame = bridge.imgmsg_to_cv2(data, "passthrough")

            hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
            mask = cv.inRange(hsv, LOWER_LIMIT, UPPER_LIMIT)
            res = cv.bitwise_and(frame, frame, mask=mask)

            if self.show_cam:
                cv.imshow('original', frame)
                cv.imshow('mask', mask)
                cv.imshow('res', res)
                cv.waitKey(1)

            if cv.mean(res) > (4, 4, 4) and self.isNewTag(self.current_pose):
                rospy.loginfo("NEW TAG FOUND @ x: {} y: {} z: {}".format(self.current_pose.x, self.current_pose.y, self.current_pose.z))
                self.tags_found.append(self.current_pose)
                rospy.loginfo("I NOW HAVE {} UNIQUE TAGS IN MY LIST".format(len(self.tags_found)))

                if len(self.tags_found) == self.max_num_tags:
                    rospy.loginfo("I (SHOULD) HAVE FOUND ALL TAGS")
                    self.saveTagsToDisk()

                pub = rospy.Publisher('tags_found', Point, queue_size=10)
                pub.publish(self.current_pose)

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def saveTagsToDisk(self):
        filename = "{}/tags_{}.txt".format(expanduser("~"), int(time.time()))

        with open(filename, 'w') as f:
            f.write("id;x;y;z\n")
            i = 1
            for tag in self.tags_found:
                f.write("{};{};{};{}\n".format(i, tag.x, tag.y, tag.z))
                i += 1

        rospy.loginfo("STORED TAGS TO {}".format(filename))

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

    try:

        rospy.init_node('detectTags')

        NUM_TAGS = rospy.get_param("~num_tags_in_maze")
        SHOW_CAMERA = rospy.get_param("~show_camera")

        color = rospy.get_param("~tags_color")

        TAGS_COLOR = None

        if color == "RED":
            TAGS_COLOR = Color.RED
        elif color == "BLUE":
            TAGS_COLOR = Color.BLUE
        elif color == "GREEN":
            TAGS_COLOR = Color.GREEN

        if TAGS_COLOR is None:
            raise Exception("Invalid argument tags_color.")

        rospy.loginfo('############ PARAMS ############')
        rospy.loginfo('# NUM_TAGS: \t{0}'.format(NUM_TAGS))
        rospy.loginfo('# SHOW_CAMERA: \t{0}'.format(SHOW_CAMERA))
        rospy.loginfo('# TAGS_COLOR: \t{0}'.format(TAGS_COLOR))



        Detection(NUM_TAGS, SHOW_CAMERA, TAGS_COLOR)
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
