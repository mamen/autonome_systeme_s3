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
from sensor_msgs.msg import CompressedImage, LaserScan

from topic_tools.srv import MuxSelect, MuxSelectRequest
from std_srvs.srv import Empty, EmptyRequest

# this is the message type we all agreed on
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
    """
    This node manages the cooperation between two or more robots.
    """

    def __init__(self, tag_positions, topics, frames, tolerance, sound):
        """
        creates a new instance for finding tags
        """
        # tag_positions
        self.search_list = tag_positions
        self.searching_list = []
        self.found_list = []

        # topics
        topic_search, topic_found, topic_image, topic_scan, self.topic_vel = topics

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
        self.sub_scan = rospy.Subscriber(topic_scan, LaserScan, self.onScan)
        self.sub_muxSelected = rospy.Subscriber('mux/selected', String, self.onMuxSelected)

        # publishers
        self.pub_search = rospy.Publisher(topic_search, coop_data_class, queue_size=1, latch=True)
        self.pub_found = rospy.Publisher(topic_found, coop_data_class, queue_size=1, latch=True)
        self.pub_vel = rospy.Publisher(self.topic_vel, Twist, queue_size=1, latch=True)

        # service clients
        # mux
        self.mux_select = rospy.ServiceProxy('/denmen/mux/select', MuxSelect)
        self.mux_select.wait_for_service()

        # move base
        self.client = actionlib.SimpleActionClient('/denmen/move_base', MoveBaseAction)
        self.client.wait_for_server()

    def done(self):
        """
        Checks, if all tags have already been found.
        """
        return (len(self.search_list) + len(self.searching_list)) == 0

    def next_target(self):
        """
        Selects a new tag as the next target, either from search list or from searching if there is no more in search list
        """
        next_search_list = self.search_list if len(self.search_list) else self.searching_list
        return next(iter(next_search_list))

    def onSearch(self, msg):
        """
        Callback upon receiving searching message (either by ourselves or from external).
        """
        rospy.loginfo('search message with x: {} and y: {}'.format(msg.point.x, msg.point.y))
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
        rospy.loginfo('current state: search: {}, searching: {}, found: {}'.format(self.search_list, self.searching_list, self.found_list))

    def onFound(self, msg):
        """
        Callback upon receiving found message (either by ourselves or from external).
        """
        rospy.loginfo('found message with x: {} and y: {}'.format(msg.point.x, msg.point.y))
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
        rospy.loginfo('current state: search: {}, searching: {}, found: {}'.format(self.search_list, self.searching_list, self.found_list))

    def onImage(self, msg):
        """
        Callback upon receiving a new image
        """
        self.image_msg = msg
        self.has_new_image = True

    def onScan(self, msg):
        """
        Callback upon receiving a new LIDAR scan
        """
        self.scan_msg = msg

    def onMuxSelected(self, msg):
        """
        Callback upon receiving a multiplexer update
        """
        self.mux_selected = msg.data

    def driveRandomly(self, duration):
        """
        Moves the robot randomly. This is needed for the initial localisation.
        """

        rate = rospy.Rate(5)

        # change to manual drive if necessary
        change_mux = self.mux_selected != self.topic_vel
        if change_mux:
            mux_select_req = MuxSelectRequest()
            mux_select_req.topic = self.topic_vel
            prev_topic = self.mux_select(mux_select_req).prev_topic
            while self.mux_selected != self.topic_vel:
                rate.sleep()

        # drive randomly without touching things for x seconds
        then = rospy.Time.now() + rospy.Duration(duration)
        turn = True
        
        while rospy.Time.now() < then:
            twist = Twist()
            distances = np.array(self.scan_msg.ranges)

            distances[distances > 8.0] = 8.0

            front = np.concatenate((distances[:45], distances[315:]), axis=None)
            right = distances[45:135]
            down = distances[135:225]
            left = distances[225:315]

            if min(front.mean(), down.mean()) < min(left.mean(), right.mean()) or turn:
                # turn
                twist.angular.z = 0.5
            else:
                # go straight
                if front.mean() > down.mean():
                    # forward
                    twist.linear.x = 0.22
                else:
                    twist.linear.x = -0.22
            
            turn = not turn
            
            self.pub_vel.publish(twist)
            rate.sleep()

        # revert manual drive if necessary
        if change_mux:
            mux_select_req = MuxSelectRequest()
            mux_select_req.topic = prev_topic
            self.mux_select(mux_select_req)
            while self.mux_selected != prev_topic:
                rate.sleep()

    def localize(self):
        """
        Starts the robots initial localisation.
        """
        rospy.wait_for_service('/denmen/global_localization')
        amcl_global_localization = rospy.ServiceProxy('/denmen/global_localization', Empty)
        amcl_global_localization(EmptyRequest())

        self.driveRandomly(20)

    def getLatestKeypoints(self):
        """
        Detects tags in the image
        """
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
        """
        Checks, if the current target is visible in the camera
        """
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
        """
        Moves the robot on top of the desired tag
        """

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
        """
        Node entrypoint, starts the whole tag-finding-process.
        """
        # stop if there is nothing to do
        if self.done():
            return
        
        self.rate = rospy.Rate(hz)
        
        # activate global localization (we can be anywhere)
        self.localize()

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
                # wait for move base state to update properly
                self.rate.sleep()

                state = self.client.get_state()
                spotted = self.spotTarget((next_x, next_y))

                # this is rather bad. means robot is stuck
                if state != GoalStatus.ACTIVE:
                    break

                # excellent! now we can move manually the last centimeters
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
                self.rate.sleep()
            
            # drive
            self.driveOnTag()

            # play sound
            playsound(self.sound)

            # communicate to others that you reached the sought point
            self.pub_found.publish(target)

            # unwedge if somewhere close to wall
            # this prevents planning failures when too close to walls
            self.driveRandomly(7)

            # revert manual drive mode
            mux_select_req = MuxSelectRequest()
            mux_select_req.topic = prev_topic

            self.mux_select(mux_select_req)

            # wait for changes to apply
            while self.mux_selected != prev_topic:
                self.rate.sleep()

            # done, ready for next target    

def main():
    try:
        rospy.init_node('coop_tag_finder', anonymous=True)

        topics = (
            rospy.get_param('~topic_search', default='/coop_tag/searching'),
            rospy.get_param('~topic_found', default='/coop_tag/reached'),
            rospy.get_param('~topic_image', default='raspicam_node/image/compressed'),
            rospy.get_param('~topic_scan', default='scan'),
            rospy.get_param('~topic_vel', default='finder_vel')
        )

        frames = (
            rospy.get_param('~frame_map', default='denmen/map'),
            rospy.get_param('~frame_base_link', default='denmen/base_link')
        )

        tolerance = rospy.get_param('~tolerance', default=0.4)

        filename = rospy.get_param('~filename', default='/home/ros/catkin_ws/src/fhv_labyrinth/tags/tags.csv')
        sound = rospy.get_param('~sound', default='/home/ros/catkin_ws/src/fhv_labyrinth/sounds/sound.mp3')

        # parse csv tag file
        with open(filename) as f:
            r = csv.reader(f, delimiter=';')
            # skip header
            r.next()
            # we only need x and y
            tag_positions = [(float(x), float(y)) for _id, x, y, z in r]
        
        ctf = CoopTagFinder(tag_positions, topics, frames, tolerance, sound)
        ctf.findAll()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
