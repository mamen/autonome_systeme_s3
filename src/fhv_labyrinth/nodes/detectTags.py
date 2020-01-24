#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, PointStamped
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from enum import Enum
from os.path import expanduser
import time
from geometry_msgs.msg import Twist
import tf


class State(Enum):
    SEARCHING = 1,
    CENTERING = 2,
    MOVING_TO_TAG = 3,
    TAG_REACHED = 4


class Color(Enum):
    RED = 1,
    BLUE = 2,
    GREEN = 3


class Detection:
    LIN_VEL_STEP_SIZE = 0.4
    ANG_VEL_STEP_SIZE = 0.3
    time_movement_started = 0
    last_blob_y_position = 0

    TAGS = []

    tag_publisher = rospy.Publisher('tags_found', Point, queue_size=10)
    marker_publisher = rospy.Publisher('clicked_point', PointStamped, queue_size=1)

    debug = False

    current_pose = Point(0, 0, 0)
    tags_found = []
    state = State.SEARCHING

    def __init__(self, num_tags, show_cam, tags_color, debug):
        self.max_num_tags = num_tags
        self.show_cam = show_cam
        self.tags_color = tags_color
        self.debug = debug
        self.detectTags()

    def log(self, string):
        if self.debug:
            print(string)

    def isNewTag(self, newTag):
        if len(self.TAGS) == 0:
            return True

        for t in self.TAGS:
            if self.calcDistance(t, newTag) < 0.5:
                return False

        return True

    def constrain(self, input, low, high):
        if input < low:
            input = low
        elif input > high:
            input = high

        return input

    def checkLinearLimitVelocity(self, vel):
        return self.constrain(vel, -0.22, 0.22)

    def checkAngularLimitVelocity(self, vel):
        return self.constrain(vel, -2.84, 2.84)

    def makeSimpleProfile(self, output, input, slop):
        if input > output:
            return min(input, output + slop)
        elif input < output:
            return max(input, output - slop)
        else:
            return input

    def setCurrentPose(self, data):
        try:
            self.current_pose = data.pose.pose
        except:
            rospy.logerr("AN ERROR OCCURED WHILE SETTING POSE")

    # def isNewTag(self, newTag):
    #     if len(self.tags_found) == 0:
    #         return True
    #
    #     for t in self.tags_found:
    #         distance = self.calcDistance(t, newTag)
    #         if distance < 0.66:
    #             return False
    #         else:
    #             rospy.loginfo(distance)
    #
    #     return True

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
            # LOWER_LIMIT = np.array([50, 10, 10])
            # UPPER_LIMIT = np.array([80, 255, 255])

            LOWER_LIMIT = np.array([20, 50, 50])
            UPPER_LIMIT = np.array([100, 255, 255])

        try:
            hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
            return cv.inRange(hsv, LOWER_LIMIT, UPPER_LIMIT)
        except Exception as e:
            rospy.logerr(e)

        return None

    def centerRobotToTag(self, offset, threshold):

        target_linear_vel = 0.0
        target_angular_vel = 0.0

        if offset > threshold:
            # Tag is on the right
            target_linear_vel = 0.0
            target_angular_vel = self.checkAngularLimitVelocity(target_angular_vel - self.ANG_VEL_STEP_SIZE)
        elif offset < -threshold:
            # Tag is on the left
            target_linear_vel = 0.0
            target_angular_vel = self.checkAngularLimitVelocity(target_angular_vel + self.ANG_VEL_STEP_SIZE)
        else:
            target_angular_vel = 0.0
            target_linear_vel = self.checkLinearLimitVelocity(target_linear_vel + self.LIN_VEL_STEP_SIZE)

        self.setMotorValues(target_linear_vel, target_angular_vel)

    def setMotorValues(self, linearVel, angelVel):
        control_linear_vel = 0.0
        control_angular_vel = 0.0

        twist = Twist()

        control_linear_vel = self.makeSimpleProfile(control_linear_vel, linearVel, (self.LIN_VEL_STEP_SIZE / 2.0))
        twist.linear.x = control_linear_vel
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        control_angular_vel = self.makeSimpleProfile(control_angular_vel, angelVel, (self.ANG_VEL_STEP_SIZE / 2.0))
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = control_angular_vel

        pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        pub.publish(twist)

    def detectBlobs(self, maskedImage):
        params = cv.SimpleBlobDetector_Params()

        params.filterByColor = True
        params.blobColor = 0

        params.minDistBetweenBlobs = 40

        # Filter by Area
        params.filterByArea = True
        params.minArea = 300
        params.maxArea = 1000000

        params.filterByCircularity = False
        params.filterByConvexity = False
        params.filterByInertia = False

        detector = cv.SimpleBlobDetector_create(params)

        ret, mask = cv.threshold(maskedImage, 127, 255, cv.THRESH_BINARY_INV)

        # Detect blobs.
        return detector.detect(mask), mask

    def getBiggestBlob(self, keypoints):
        return sorted(keypoints, key=lambda x: x.size, reverse=True)[0]

    def getOffset(self, tag_x, image_width):
        return tag_x - (image_width / 2)

    def isCurrentTagCentered(self, current_tag, image_width, threshold):
        return -threshold < self.getOffset(current_tag.pt[0], image_width) < threshold

    def calculatePositionOfTag(self, robot_pose, x, y):
        H = np.array([[-0.09288180869054626, 0.0054416583613956645, 55.428253772214454],
                      [0.0025116067633560767, 0.0029759558170538375, -97.45956201150065],
                      [0.00010195975796577268, -0.006257619108708003, 1.0]])

        result = H.dot(np.asarray([x, y, 1]))

        x = result[1] / result[2]
        y = result[0] / result[2]

        _, _, phi = tf.transformations.euler_from_quaternion(
            [robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w])

        T = np.asarray(
            [
                [np.cos(phi), -np.sin(phi), robot_pose.position.x],
                [np.sin(phi), np.cos(phi), robot_pose.position.y],
                [0, 0, 1]
            ]
        )

        x, y, z = np.matmul(T, np.asarray([(x/100), (y/100), 1]))

        pt = PointStamped()
        pt.header.frame_id = "base_link"
        pt.point.x = x
        pt.point.y = y
        pt.point.z = 1

        self.marker_publisher.publish(pt)

        return [x, y]

    def processImage(self, data):

        try:
            np_arr = np.fromstring(data.data, np.uint8)
            frame = cv.imdecode(np_arr, cv.IMREAD_COLOR)

            maskedImage = self.maskImage(frame)
            keypoints, mask = self.detectBlobs(maskedImage)

            THRESHOLD = 50

            SECONDS_TO_TAG = 1.4

            current_tag = None

            # draw image --------------------------------
            im_with_keypoints = cv.drawKeypoints(mask, keypoints, np.array([]), (0, 0, 255),
                                                 cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

            if self.show_cam:
                # cv.imshow('raspi orig', frame)
                cv.imshow("Keypoints", im_with_keypoints)
                cv.waitKey(1)
            # -------------------------------------------

            # get biggest blob
            if len(keypoints) > 0:
                current_tag = self.getBiggestBlob(keypoints)

                position = self.calculatePositionOfTag(self.current_pose, current_tag.pt[0], current_tag.pt[1])

                tag = Point(x=position[0], y=position[1], z=1)

                if self.isNewTag(tag):
                    print("========\r\nx: {}\r\ny: {}".format(tag.x, tag.y))
                    rospy.loginfo("NEW TAG WAS FOUND")
                    self.TAGS.append(tag)
                    print("I now have found {} tags".format(len(self.TAGS)))


                # # print("###############################################################")
                # print("Robot position: {}; {}".format(self.current_pose.position.x, self.current_pose.position.y))
                # print("Tag position: {}; {}".format(position[0], position[1]))

            # # blob detected
            # if self.state == State.SEARCHING and len(keypoints) > 0:
            #     # TODO: stop current navigation goal
            #     self.state = State.CENTERING
            #     print("now in state {}".format(self.state))
            #     return
            #
            # # blob detected and currently centering
            # if self.state == State.CENTERING and len(keypoints) > 0:
            #     if self.isCurrentTagCentered(current_tag, maskedImage.shape[1], THRESHOLD):
            #         # tag is centered
            #         self.state = State.MOVING_TO_TAG
            #         print("now in state {}".format(self.state))
            #     else:
            #         self.centerRobotToTag(self.getOffset(current_tag.pt[0], maskedImage.shape[1]), THRESHOLD)
            #
            #     return
            #
            # # no blob detected but currently centering some tag
            # if self.state == State.CENTERING and len(keypoints) == 0:
            #     self.state = State.SEARCHING
            #     # TODO: start navigation again
            #     print("now in state {}".format(self.state))
            #     return
            #
            # # lost vision of tag while moving to it
            # if self.state == State.MOVING_TO_TAG and len(keypoints) == 0:
            #     # did i loose it on the bottom of the image?
            #     self.log(self.last_blob_y_position)
            #     if self.last_blob_y_position > 550:
            #         self.log("###")
            #         self.log("# TAG LEFT BOTTOM")
            #         self.log("###")
            #         self.state = State.TAG_REACHED
            #         print("now in state {}".format(self.state))
            #     else:
            #         self.state = State.SEARCHING
            #         # stop the motors
            #         self.setMotorValues(0.0, 0.0)
            #         # TODO: start navigation again
            #         print("now in state {}".format(self.state))
            #
            #     return
            #
            # # tag still detected while moving to it
            # if self.state == State.MOVING_TO_TAG and len(keypoints) > 0:
            #     if not self.isCurrentTagCentered(current_tag, maskedImage.shape[1], THRESHOLD):
            #         self.state = State.CENTERING
            #         print("now in state {}".format(self.state))
            #         return
            #
            #     self.last_blob_y_position = current_tag.pt[1]
            #
            #     # keep moving straight
            #     self.setMotorValues(self.LIN_VEL_STEP_SIZE, 0.0)
            #     return
            #
            # # tag reached, drive last x seconds straight
            # if self.state == State.TAG_REACHED:
            #     if self.time_movement_started == 0:
            #         self.time_movement_started = time.time()
            #         self.log("SET TIME")
            #
            #     if self.time_movement_started > 0:
            #         self.log("{} seconds passed".format(time.time() - self.time_movement_started))
            #
            #     if time.time() - self.time_movement_started >= SECONDS_TO_TAG:
            #         # stop
            #         self.log(" TAG REACHED ")
            #         self.setMotorValues(0.0, 0.0)
            #         self.time_movement_started = 0
            #         self.state = State.SEARCHING
            #
            #         # publish to topic tags_found
            #         self.tag_publisher.publish(self.current_pose.position)
            #     return

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

        RASPI_CAM__TOPIC = rospy.get_param("~image_topic")
        ODOMETRY_TOPIC = rospy.get_param("~odometry_topic")

        try:
            rospy.loginfo("waiting for camera image")
            rospy.Subscriber(RASPI_CAM__TOPIC, CompressedImage, self.processImage)
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
        DEBUG = rospy.get_param("~debug")

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

        Detection(NUM_TAGS, SHOW_CAMERA, TAGS_COLOR, DEBUG)
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
