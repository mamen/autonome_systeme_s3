#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage
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
    tags_found = []

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

    def maskImage(self, image):
        LOWER_LIMIT = UPPER_LIMIT = np.array([0, 0, 0])

        if self.tags_color == Color.RED:
            LOWER_LIMIT = np.array([0, 50, 50])
            UPPER_LIMIT = np.array([10, 255, 255])
        elif self.tags_color == Color.BLUE:
            LOWER_LIMIT = np.array([100, 150, 0])
            UPPER_LIMIT = np.array([140, 255, 255])
        elif self.tags_color == Color.GREEN:
            ## vormittag:
            # LOWER_LIMIT = np.array([80, 0, 0])
            # UPPER_LIMIT = np.array([110, 200, 130])
            ## nachmittag:
            LOWER_LIMIT = np.array([60, 10, 10])
            UPPER_LIMIT = np.array([80, 255, 255])

        try:
            hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
            return cv.inRange(hsv, LOWER_LIMIT, UPPER_LIMIT)
        except Exception as e:
            rospy.logerr(e)

        return None

    def processImageRaspi(self, data):

        try:
            np_arr = np.fromstring(data.data, np.uint8)
            frame = cv.imdecode(np_arr, cv.IMREAD_COLOR)

            res = self.maskImage(frame)

            params = cv.SimpleBlobDetector_Params()

            params.filterByColor = True
            params.blobColor = 0

            # Filter by Area
            params.filterByArea = True
            params.minArea = 800
            params.maxArea = 10000

            params.filterByCircularity = False
            params.filterByConvexity = False
            params.filterByInertia = False

            detector = cv.SimpleBlobDetector_create(params)

            ret, mask = cv.threshold(res, 127, 255, cv.THRESH_BINARY_INV)

            # Detect blobs.
            keypoints = detector.detect(mask)

            if len(keypoints) > 0:
                biggest = sorted(keypoints, key=lambda x: x.size, reverse=True)[0]

                THRESHOLD = 20

                offset = biggest.pt[0] - (res.shape[1] / 2)

                print("robot is off by {}".format(offset))

                if offset > THRESHOLD:
                    print("move robot to the RIGHT")
                elif offset < -THRESHOLD:
                    print("move robot to the LEFT")
                else:
                    print("GO STRAIGT AHEAD")

            im_with_keypoints = cv.drawKeypoints(mask, keypoints, np.array([]), (0, 0, 255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

            if self.show_cam:
                cv.imshow('raspi orig', frame)
                cv.imshow("Keypoints", im_with_keypoints)
                cv.waitKey(1)

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def processImagePixy(self, data):
        print("Pixy")
        try:
            np_arr = np.fromstring(data.data, np.uint8)
            frame = cv.imdecode(np_arr, cv.IMREAD_COLOR)

            res = self.maskImage(frame)

            if self.show_cam:
                cv.imshow('pixy orig', frame)
                cv.imshow('pixy res', res)
                cv.waitKey(1)

            if cv.mean(res) > (4, 4, 4) and self.isNewTag(self.current_pose):
                rospy.loginfo("NEW TAG FOUND @ x: {} y: {} z: {}".format(self.current_pose.x, self.current_pose.y,
                                                                         self.current_pose.z))
                self.tags_found.append(self.current_pose)
                rospy.loginfo("I NOW HAVE {} UNIQUE TAGS IN MY LIST".format(len(self.tags_found)))

                if len(self.tags_found) == self.max_num_tags:
                    rospy.loginfo("I (SHOULD) HAVE FOUND ALL TAGS")
                    self.saveTagsToDisk()

                pub = rospy.Publisher('tags_found', Point, queue_size=10)
                pub.publish(self.current_pose)

        except Exception as e:
            rospy.logerr(e)

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

        RASPI_CAM__TOPIC = rospy.get_param("~image_topic_raspi_cam")
        PIXY_CAM_TOPIC = rospy.get_param("~image_topic_pixy_cam")
        ODOMETRY_TOPIC = rospy.get_param("~odometry_topic")

        try:
            rospy.loginfo("waiting for camera image")
            rospy.Subscriber(RASPI_CAM__TOPIC, CompressedImage, self.processImageRaspi)
            # rospy.Subscriber(PIXY_CAM_TOPIC, CompressedImage, self.processImagePixy)
            rospy.Subscriber(ODOMETRY_TOPIC, Odometry, self.setCurrentPose)

            rospy.spin()
        except Exception as e:
            rospy.logerr(e)
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
