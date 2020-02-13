#!/usr/bin/env python
import rospy
import actionlib
import csv
import numpy as np

# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header

from std_srvs.srv import Empty, EmptyRequest

coop_data_class = PointStamped

class CoopTagFinder(object):
    def __init__(self, tag_positions, search_topic, found_topic, tolerance, frame_map):
        self.search_list = [tag_positions]
        self.searching_list = []
        self.found_list = []
        self.tolerance = tolerance

        self.frame_map = frame_map

        self.pub_search = rospy.Publisher(search_topic, coop_data_class, queue_size=2)
        self.pub_found = rospy.Publisher(found_topic, coop_data_class, queue_size=2)

        self.sub_search = rospy.Subscriber(search_topic, coop_data_class, self.onSearch)
        self.sub_found = rospy.Subscriber(found_topic, coop_data_class, self.onFound)

        # Create an action client called "move_base" with action definition file "MoveBaseAction"
        self.client = actionlib.SimpleActionClient('denmen/move_base', MoveBaseAction)

        # Waits until the action server has started up and started listening for goals.
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
        else:
            # other robot found a tag we didn't find... who cares?
            # TODO add cancel behavior
            pass

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
        else:
            # other robot found a tag we didn't find... who cares?
            # TODO add cancel behavior
            pass

    def localize(self):
        rospy.wait_for_service('/global_localization')
        amcl_global_localization = rospy.ServiceProxy('/global_localization', Empty)
        amcl_global_localization(EmptyRequest())

    def findAll(self):

        rospy.loginfo('fooo {}'.format(self.done()))

        if self.done():
            return
        # activate global localization (we can be anywhere)
        self.localize()

        rospy.loginfo('baaar')

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
            # Waits for the server to finish performing the action.
            wait = self.client.wait_for_result()
            # If the result doesn't arrive, assume the Server is not available
            if not wait:
                rospy.logerr("Action server not available!")
                rospy.signal_shutdown("Action server not available!")
            else:
                # Result of executing the action
                result = self.client.get_result()

                rospy.loginfo(result)

                # TODO analyze result to determine whether goal was found or not
                self.pub_found.publish(target)

def main():
    try:
        rospy.init_node('coop_tag_finder', anonymous=True)

        search_topic = rospy.get_param('~search_topic', default='/coop_tag/searching')
        found_topic = rospy.get_param('~found_topic', default='/coop_tag/reached')
        filename = rospy.get_param('~filename', default='tags.csv')
        tolerance = rospy.get_param('~tolerance', default=0.4)
        frame_map = rospy.get_param('~frame_map', default='map')

        with open(filename) as f:
            r = csv.reader(f, delimiter=';')
        
            # skip header
            r.next()

            tag_positions = [(float(x), float(y)) for _id, x, y, z in r]
        
        ctf = CoopTagFinder(tag_positions, search_topic, found_topic, tolerance, frame_map)
        ctf.findAll()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
