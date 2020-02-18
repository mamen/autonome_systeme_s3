#!/usr/bin/env python
import rospy
import tf
import numpy as np
import cv2 as cv
import cv_bridge as cvb
import csv
import actionlib

from playsound import playsound

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import Header, String
from actionlib_msgs.msg import GoalStatus
from sensor_msgs.msg import CompressedImage

from topic_tools.srv import MuxSelect, MuxSelectRequest
from std_srvs.srv import Empty, EmptyRequest

coop_data_class = PointStamped

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

class CoopTagFinder(object):
    def __init__(self, tag_positions, topics, frames, tolerance, sound):
        # tag_positions
        self.search_list = tag_positions
        self.searching_list = []
        self.found_list = []

        # topics
        topic_search, topic_found, topic_image, self.topic_vel = topics

        # frames
        self.frame_map, self.frame_base_link = frames

        # tolerance
        self.tolerance = tolerance

        # sound
        self.sound = sound

        # messages
        self.image_msg = None
        self.has_new_image = False

        # mux selected
        self.mux_selected = None

        # transforms
        self.tl = tf.TransformListener()

        # subscribers
        self.sub_search = rospy.Subscriber(topic_search, coop_data_class, self.onSearch)
        self.sub_found = rospy.Subscriber(topic_found, coop_data_class, self.onFound)
        self.sub_image = rospy.Subscriber(topic_image, CompressedImage, self.onImage)
        self.sub_muxSelected = rospy.Subscriber('mux/selected', String, self.onMuxSelected)

        # publishers
        self.pub_search = rospy.Publisher(topic_search, coop_data_class, queue_size=2)
        self.pub_found = rospy.Publisher(topic_found, coop_data_class, queue_size=2)
        self.pub_vel = rospy.Publisher(self.topic_vel, Twist, queue_size=1)

        # service clients
        # mux
        self.mux_select = rospy.ServiceProxy('/denmen/mux/select', MuxSelect)
        self.mux_select.wait_for_service()

        # move base
        self.client = actionlib.SimpleActionClient('/denmen/move_base', MoveBaseAction)
        self.client.wait_for_server()

    def done(self):
        return (len(self.search_list) + len(self.searching_list)) == 0

    def next_target(self):
        next_search_list = self.search_list if len(self.search_list) else self.searching_list
        return next(iter(next_search_list))

    def onSearch(self, msg):
        # convert coop message into private format
        t_xy = msg.point.x, msg.point.y

        # find matching point in search list
        first_or_default = next(
            (xy for xy in self.search_list if np.linalg.norm(np.array(xy)-t_xy) <= self.tolerance),
            None
        )

        if first_or_default:
            self.search_list.remove(first_or_default)
            self.searching_list.append(first_or_default)
        
        # other robot found a tag we didn't find... who cares?
        # TODO add cancel behavior

    def onFound(self, msg):
        # convert coop message into private format
        t_xy = msg.point.x, msg.point.y

        # find matching point in searching list
        first_or_default = next(
            (xy for xy in self.searching_list if np.linalg.norm(np.array(xy)-t_xy) <= self.tolerance),
            None
        )

        if first_or_default:
            self.searching_list.remove(first_or_default)
            self.found_list.append(first_or_default)
        
        # other robot found a tag we didn't find... who cares?
        # TODO add cancel behavior

    def onImage(self, msg):
        self.image_msg = msg
        self.has_new_image = True

    def onMuxSelected(self, msg):
        self.mux_selected = msg.data

    def localize(self):
        rospy.wait_for_service('/denmen/global_localization')
        amcl_global_localization = rospy.ServiceProxy('/denmen/global_localization', Empty)
        amcl_global_localization(EmptyRequest())

    def getLatestKeypoints(self):
        try:
            _, mask = cv.threshold(
                cv.inRange(
                    cv.cvtColor(
                        cv.imdecode(
                            np.fromstring(
                                self.image_msg.data,
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
            self.image_width = mask.shape[1]

            keypoints = detector.detect(mask)
            keypoints.sort(key=lambda x: x.size, reverse=True)
        except cvb.CvBridgeError as e:
            keypoints = []
        return keypoints

    def spotTarget(self, target):
        if not self.has_new_image:
            return
        
        self.has_new_image = False
        try:
            keypoints = self.getLatestKeypoints()

            if not len(keypoints):
                return
            
            current_tag = keypoints[0]
            
            result = H.dot(np.asarray((current_tag.pt[0], current_tag.pt[1], 1)))
            result /= result[2]
            
            _x, _y, _ = result # in meter
            ps = PointStamped()
            ps.header.frame_id = self.frame_base_link
            # ps.header.stamp = rospy.Time(0) 
            ps.header.stamp = self.image_msg.header.stamp
            ps.point.x=_x
            ps.point.y=_y

            p_map = self.tl.transformPoint(self.frame_map, ps).point
            
            spotted = p_map.x, p_map.y

            return np.linalg.norm(np.array(spotted)-target) <= self.tolerance
        except cvb.CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Transformation failed, oopsie")

    def driveOnTag(self):

        keypoints = self.getLatestKeypoints()
        while len(keypoints):
            twist = Twist()
            kp = keypoints[0]
                
            offset = kp.pt[0] - self.image_width / 2
                
            if offset > 50:
                twist.angular.z = -0.15
            elif offset < -50:
                twist.angular.z = 0.15
            else:
                twist.linear.x = 0.11
                
            self.pub_vel.publish(twist)
            rospy.sleep(0.1)
            keypoints = self.getLatestKeypoints()

        twist = Twist()
        # forward 2 sec
        twist.linear.x = 0.22    
        self.pub_vel.publish(twist)
        rospy.sleep(1.75)

        # stop it
        twist = Twist()
        self.pub_vel.publish(twist)
        rospy.sleep(1)
                

    def findAll(self):
        if self.done():
            return
        # activate global localization (we can be anywhere)
        self.localize()

        rate = rospy.Rate(hz)

        while not rospy.is_shutdown() and not self.done():
            # get next goal in personal data format (x y map coordinates)
            (next_x, next_y) = self.next_target()

            target = PointStamped()
            target.header.frame_id = self.frame_map
            target.header.stamp = rospy.Time.now()
            target.point.x = next_x
            target.point.y = next_y

            # communicate to others that you look after this point
            self.pub_search.publish(target)

            # Creates a new goal with the MoveBaseGoal constructor      
            goal = MoveBaseGoal()
            
            # set header
            goal.target_pose.header.frame_id = self.frame_map
            goal.target_pose.header.stamp = rospy.Time.now()
            
            # set position
            goal.target_pose.pose.position = target.point

            # set orientation
            goal.target_pose.pose.orientation.w = 1.0

            # Sends the goal to the action server.
            self.client.send_goal(goal)

            while True:
                rate.sleep()

                state = self.client.get_state()
                spotted = self.spotTarget((next_x, next_y))

                if state != GoalStatus.ACTIVE:
                    break

                if spotted:
                    break
            
            if state == GoalStatus.ACTIVE and spotted:
                # target spotted
                    
                # i want to drive now!
                self.client.cancel_goal()

                mux_select_req = MuxSelectRequest()
                mux_select_req.topic = self.topic_vel

                prev_topic = self.mux_select(mux_select_req).prev_topic
            else:
                # houston, we have a problem!
                continue

            # wait for changes to apply
            while self.mux_selected != self.topic_vel:
                rate.sleep()
            
            # drive
            self.driveOnTag()

            # play sound
            playsound(self.sound)

            # target reached
            self.pub_found.publish(target)

            # revert mux
            mux_select_req = MuxSelectRequest()
            mux_select_req.topic = prev_topic

            self.mux_select(mux_select_req)

            while self.mux_selected != prev_topic:
                rate.sleep()

            # done, ready for next target    

def main():
    try:
        rospy.init_node('coop_tag_finder', anonymous=True)

        topics = (
            rospy.get_param('~topic_search', default='/coop_tag/searching'),
            rospy.get_param('~topic_found', default='/coop_tag/reached'),
            rospy.get_param('~topic_image', default='raspicam_node/image/compressed'),
            rospy.get_param('~topic_vel', default='finder_vel')
        )

        frames = (
            rospy.get_param('~frame_map', default='denmen/map'),
            rospy.get_param('~frame_base_link', default='denmen/base_link')
        )

        tolerance = rospy.get_param('~tolerance', default=0.4)

        filename = rospy.get_param('~filename', default='/home/ros/catkin_ws/src/fhv_labyrinth/tags/tags.csv')
        sound = rospy.get_param('~sound', default='../sounds/sound.mp3')

        with open(filename) as f:
            r = csv.reader(f, delimiter=';')
            # skip header
            r.next()
            tag_positions = [(float(x), float(y)) for _id, x, y, z in r]
        
        ctf = CoopTagFinder(tag_positions, topics, frames, tolerance, sound)
        ctf.findAll()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
