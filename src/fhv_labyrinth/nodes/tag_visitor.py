#!/usr/bin/env python
import rospy
import tf
import numpy as np
import cv2 as cv
import cv_bridge as cvb
import actionlib

from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point, PointStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# framerate (hz)
hz = 1

# PINK
LOWER_LIMIT = np.array([150, 100, 100])
UPPER_LIMIT = np.array([170, 240, 255])

# Blob detector
params = cv.SimpleBlobDetector_Params()
params.filterByColor = True
params.blobColor = 0
params.minDistBetweenBlobs = 40
params.filterByArea = True
params.minArea = 800
params.maxArea = 1000000
params.filterByCircularity = False
params.filterByConvexity = False
params.filterByInertia = False
detector = cv.SimpleBlobDetector_create(params)

# Homography matrix
img_pts = np.array((
    (566.16085207, 633.06493435),
    (574.13351993, 530.97218097),
    (458.1943503,  530.73832472),
    (415.68878989, 633.27593301),
))

real_pts = np.array((
    (0.40, 0.025), # 1'
    (0.50, 0.025), # 2'
    (0.50, 0.075), # 3'
    (0.40, 0.075), # 4'
))

H = cv.findHomography(img_pts, real_pts)[0]

# tag config
MIN_DISTANCE_BETWEEN_TAGS = 0.6

"""
This Node detects Tags and positions the roboter in top of them.
"""
class TagVisitor(object):
    """
    Initialization
    """
    def __init__(self, topic_image, filename, frames):
        self.frame_map, self.frame_base_link = frames
        self.image_msg = None
        self.has_new_image = False
        self.tl = tf.TransformListener()
        self.sub_image = rospy.Subscriber(topic_image, CompressedImage, self.onImage)

    """
    This gets triggered when a new image is received.
    """
    def onImage(self, msg):
        self.image_msg = msg
        self.has_new_image = True

    """
    Checks, if the found tag has a big enough distance to all other already found tags.
    This should prevent tags from being detected twice.
    """
    def distanceBigEnoughGenerator(self, tag):
        for t in self.tags:
            d = ((t.x - tag.x) ** 2 + (t.y - tag.y) ** 2) ** 0.5
            yield d >= MIN_DISTANCE_BETWEEN_TAGS

    """
    The received image is masked and if a tag is found in that image, its position is being calculated
    """
    def performCalculation(self):
        try:
            self.has_new_image = False
            image_msg = self.image_msg

            _, mask = cv.threshold(
                cv.inRange(
                    cv.cvtColor(
                        cv.imdecode(
                            np.fromstring(
                                image_msg.data,
                                np.uint8
                            ),
                            cv.IMREAD_COLOR
                        ),
                        cv.COLOR_BGR2HSV
                    ),
                    LOWER_LIMIT,
                    UPPER_LIMIT
                ),
                127,
                255,
                cv.THRESH_BINARY_INV
            )

            keypoints = detector.detect(mask)

            # nothing to do if no keypoints available
            if not len(keypoints):
                rospy.loginfo('[TAG_DETECTOR] no keypoints found')
                return
            
            # sort by size
            keypoints.sort(key=lambda x: x.size, reverse=True)
            
            # current one is the biggest
            current_tag = keypoints[0]

            # do transformation
            result = H.dot(np.asarray((current_tag.pt[0], current_tag.pt[1], 1)))
            result /= result[2]

            _x, _y, _ = result # in meter

            ps = PointStamped()
            ps.header.frame_id = self.frame_base_link
            # ps.header.stamp = rospy.Time(0) 
            ps.header.stamp = image_msg.header.stamp
            ps.point = Point(x=_x, y=_y)
            
            ps_map = self.tl.transformPoint(self.frame_map, ps)

            tag = ps_map.point
        
            is_new_tag = all(self.distanceBigEnoughGenerator(tag))

            if not is_new_tag:
                return
            
            self.id += 1
            self.tags.append(tag)
            self.writeTag(tag)

            if show_tags:
                marker = Marker()
                marker.header.frame_id = self.frame_map
                marker.type = marker.SPHERE
                marker.action = marker.ADD
                marker.scale.x = 0.2
                marker.scale.y = 0.2
                marker.scale.z = 0.2
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.pose.position = tag
                marker.pose.orientation.w = 1.0
                marker.id = self.id - 1

                self.tag_markers.markers.append(marker)
                self.pub_tags.publish(self.tag_markers)

        except cvb.CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Transformation failed, oopsie")

    def spin(self):
        rate = rospy.Rate(hz)

        while not rospy.is_shutdown():
            # if self.has_new_image and self.robot_pose_msg:
            if self.has_new_image:
                self.performCalculation()
            rate.sleep()

"""
In case the node crashes, the wheels should stop spinning.
This does not yet work reliably.
"""
def stopWheels():
    twist = Twist()

    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0

    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = 0.0

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    pub.publish(twist)


def main():
    try:
        rospy.init_node('tag_detector', anonymous=True)
        
        rospy.on_shutdown(stopWheels)

        topic_image = rospy.get_param('topic_image', default='raspicam_node/image/compressed')
        filename = rospy.get_param('~tag_file', default='tags.csv')

        frames = (
            rospy.get_param('~frame_map', default='map'),
            rospy.get_param('~frame_base_link', default='base_link')
        )

        td = TagVisitor(topic_image, filename, frames)
        td.spin()
    except rospy.ROSInterruptException:
        stopWheels()
        pass


if __name__ == '__main__':
    main()
