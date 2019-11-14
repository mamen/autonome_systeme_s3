#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError

class Detection():

    def __init__(self):
        self.detectTags()

    def processImage(self, data):
        # rospy.loginfo(data.header)

        bridge = CvBridge()

        try:
            cv_image = bridge.imgmsg_to_cv2(data, "passthrough")

            cv.imwrite('/home/markus/catkin_ws/test.jpeg', cv_image)

            rospy.loginfo(cv_image)

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def detectTags(self):

        IMAGE_TOPIC = rospy.get_param("~img_topic")

        try:
            while not rospy.is_shutdown():
                # pub = rospy.Publisher('detectTags', TagDetected, queue_size=10)

                # print("asdf")
                # get camera image

                sub_image = rospy.Subscriber(IMAGE_TOPIC, Image, self.processImage)

                # rospy.loginfo(text)
                # filter out blobs

                # check if tag is in image

                # tag = TagDetected()
                #
                # tag.test = 3
                #
                # pub.publish(tag)
                rospy.spin()

        finally:
            rospy.loginfo('a')
            # twist = Twist()
            # twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
            # twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
            # pub.publish(twist)

def main():
    rospy.init_node('detectTags')

    try:
        Detection()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
