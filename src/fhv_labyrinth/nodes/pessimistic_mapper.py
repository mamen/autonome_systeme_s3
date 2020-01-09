#!/usr/bin/env python

'''
The gmapping package is a pretty good way for a robot to generate a 2D map of its environment by moving through it.
The explore_lite package is a neat way for a robot to discover unknown places by using the map generated from gmapping.

In our task, the goal is to manouver in an unknown area visiting a lot of its places, not just passing by and scan it via lidar.
Therefore, this mapper node basically undoes the work of the gmapping node by re-covering place it has not really visited yet.

The gmapping node does the mapping rather quickly, but we want it to slow it down so explore_lite actually does explore the area by visiting.
'''

import rospy

from nav_msgs.msg import OccupancyGrid, Odometry, MapMetaData
from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance

import numpy as np


class PessimisticMapper(object):
    '''
    Don't use mapped areas which you haven't seen on your own.
    '''

    def __init__(self, size, map_topic, odom_topic, pessimistic_topic):
        self.map_msg = None
        self.odom_msg = None
        self.pessimistic_map = None

        self.size = size

        self.map_sub = rospy.Subscriber(map_topic, OccupancyGrid, self.onMapMessage)
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.onOdomMessage)

        self.pub = rospy.Publisher(pessimistic_topic, OccupancyGrid, queue_size=10, latch=True)

    def onMapMessage(self, msg):
        '''
        Callback receiving an OccupancyGrid Message.

        This represents a 2-D grid map, in which each cell represents the probability of
        occupancy.

        * header: Message Header

        * info:   MetaData for the map

        * data:   The map data, in row-major order, starting with (0,0).
                  Occupancy probabilities are in the range [0,100].
                  Unknown is -1.
        '''
        self.map_msg = msg

        if self.pessimistic_map is None:
            dimension = (self.map_msg.info.height, self.map_msg.info.width)
            self.pessimistic_map = np.ones(dimension).astype(bool)
        

    def onOdomMessage(self, msg):
        '''
        Callback receiving an Odometry Message.

        This represents an estimate of a position and velocity in free space.
        The pose in this message should be specified in the coordinate frame given by header.frame_id.
        The twist in this message should be specified in the coordinate frame given by the child_frame_id

        * header: Message Header

        * child_frame_id: String

        * pose:   Estimated Position

        * twist:  Estimated Twist
        '''
        self.odom_msg = msg

    def spin(self, rate):
        r = rospy.Rate(rate)
        while not rospy.is_shutdown():
            if self.odom_msg and self.map_msg:
                computed = self.compute()
                self.pub.publish(computed)
            r.sleep()

    def compute(self):
        pos_xy = np.array((self.odom_msg.pose.pose.position.x, self.odom_msg.pose.pose.position.y))
        origin_xy = np.array((self.map_msg.info.origin.position.x, self.map_msg.info.origin.position.y))
        
        height = self.map_msg.info.height
        width = self.map_msg.info.width
        resolution = self.map_msg.info.resolution

        idx_x, idx_y = ((pos_xy - origin_xy) / resolution).astype(int)
        rospy.loginfo('x: {}, y: {}'.format(idx_x, idx_y))
        
        safe_idx_x_low = max(idx_x - self.size / 2, 0)
        safe_idx_y_low = max(idx_y - self.size / 2, 0)
        safe_idx_x_high = min(idx_x + 1 + self.size / 2, height)
        safe_idx_y_high = min(idx_y + 1 + self.size / 2, width)

        # mark position of robot as visited (can be shown in published map now)
        self.pessimistic_map[safe_idx_y_low:safe_idx_y_high, safe_idx_x_low:safe_idx_x_high] = False

        # bring data in shape
        pessimistic = np.array(self.map_msg.data).reshape((height, width))
        # mark pessimistic fields as Unknown
        pessimistic[self.pessimistic_map==True] = -1
        # turn back into one dimensional array again
        pessimistic = pessimistic.reshape(-1)
        
        pessimistic_msg = OccupancyGrid()
        pessimistic_msg.header = self.map_msg.header
        pessimistic_msg.info = self.map_msg.info
        pessimistic_msg.data = pessimistic
        return pessimistic_msg


def main():
    try:
        rospy.init_node('pessimistic_mapper', anonymous=True)

        size = rospy.get_param('~size', default=3)
        map_topic = rospy.get_param('~map_topic', default='map')
        odom_topic = rospy.get_param('~odom_topic', default='odom')
        pessimistic_topic = rospy.get_param('~pessimistic_topic', default='pessimistic')
        rate = rospy.get_param('~rate', default=1)

        pm = PessimisticMapper(size, map_topic, odom_topic, pessimistic_topic)
        pm.spin(rate)

    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
