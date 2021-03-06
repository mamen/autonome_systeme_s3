#!/usr/bin/env python
import rospy
import tf
import numpy as np

from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped

from pessimistic_mask import createPessimisticMask

class PessimisticMapper(object):
    """
    Creates a Map from an actual map but only with things which have been really "seen" by camera
    """
    def __init__(self, topics, frames, view):
        """
        creates a new instance
        """
        _map, _scan, _full, _update = topics
        
        # frames
        self.frame_map, self.frame_base_link = frames

        # message data incoming
        self.map_msg = None
        self.odom_msg = None
        self.scan_msg = None

        # meta
        self.dimension = None
        self.pessimistic_map = None
        self.view = view

        # frame transformer
        self.tl = tf.TransformListener()

        # publishers
        self.pub_full = rospy.Publisher(_full, OccupancyGrid, queue_size=1, latch=True)
        self.pub_update = rospy.Publisher(_update, OccupancyGridUpdate, queue_size=1, latch=True)

        # subscribers
        self.sub_map = rospy.Subscriber(_map, OccupancyGrid, self.onMapMessage)
        self.sub_scan = rospy.Subscriber(_scan, LaserScan, self.onScanMessage)

    def onMapMessage(self, msg):
        """
        Callback upon receiving map message from slam
        """
        if self.pessimistic_map is None:
            self.dimension = (msg.info.height, msg.info.width)
            self.pessimistic_map = np.ones(self.dimension).astype(bool)
        self.map_msg = msg

    def onScanMessage(self, msg):
        """
        Callback upon receiving a new LIDAR scan message
        """
        self.scan_msg = msg

    def runMapping(self, rate_full, rate_update):
        """
        periodically performs the uncovering of the original map based on current sight
        """
        counter_limit = rate_update / rate_full
        counter = 0

        seq_full = 0
        seq_update = 0

        r = rospy.Rate(rate_update)

        while not rospy.is_shutdown():
            # not much we can do if we get no map or scan
            if self.map_msg and self.scan_msg:
                try:
                    # get current position in map frame
                    point, quaternion = self.tl.lookupTransform(self.frame_map, self.frame_base_link, rospy.Time())
                    pos_xy = np.array(point[:2])
                    yaw = tf.transformations.euler_from_quaternion(quaternion)[2]

                    # create visible area mask based on emulated sight and actualy possible sight
                    mask = createPessimisticMask(self.map_msg, self.scan_msg, self.view, pos_xy, yaw)
                    
                    if mask.any():
                        # mark visible area at current pose as visited (can be shown in map now)
                        self.pessimistic_map[mask == True] = False

                        # bring data in shape
                        pessimistic = np.array(self.map_msg.data).reshape(self.dimension)

                        # mark pessimistic fields as Unknown
                        pessimistic[self.pessimistic_map == True] = -1

                        # determine region of update
                        xs, ys = np.where(mask)

                        x = np.min(xs)
                        y = np.min(ys)
                        height = np.max(xs) - x + 1
                        width = np.max(ys) - y + 1

                        # get current time
                        now = rospy.Time.now()

                        # prepare update message
                        update = OccupancyGridUpdate()
                        update.header.frame_id = self.frame_map
                        update.header.seq = seq_update
                        update.header.stamp = now
                        update.x = x
                        update.y = y
                        update.width = width
                        update.height = height
                        update.data = pessimistic[x:x + height, y:y + width].reshape(-1)

                        # publish update
                        self.pub_update.publish(update)

                        # increment sequence counter
                        seq_update += 1

                        # it's time for a full update
                        if counter == 0:
                            # reset countdown
                            counter = counter_limit

                            # perform full update
                            full = OccupancyGrid()
                            full.header.frame_id = self.frame_map
                            full.header.seq = seq_full
                            full.header.stamp = now
                            full.info = self.map_msg.info
                            full.data = pessimistic.reshape(-1)

                            self.pub_full.publish(full)

                            seq_full += 1

                        counter -= 1
                except (tf.Exception, tf.LookupException, tf.ConnectivityException):
                    rospy.logerr("Transformation failed, sorry")
            r.sleep()


def main():
    try:
        rospy.init_node('pessimistic_mapper', anonymous=True)

        topics = (
            rospy.get_param('~topic_map', default='map'),
            rospy.get_param('~topic_scan', default='scan'),
            rospy.get_param('~topic_full', default='pessimistic'),
            rospy.get_param('~topic_update', default='pessimistic_updates')
        )

        frames = (
            rospy.get_param('~frame_map', default='map'),
            rospy.get_param('~frame_base_link', default='base_link')
        )

        view = (
            (
                rospy.get_param('~sight_distance_min', default=6),
                rospy.get_param('~sight_distance_max', default=20)
            ),
            (
                rospy.get_param('~sight_width_min', default=2),
                rospy.get_param('~sight_width_max', default=15)
            )
        )
        pm = PessimisticMapper(topics, frames, view)

        pm.runMapping(
            rospy.get_param('~rate_full', default=1),
            rospy.get_param('~rate_update', default=10)
        )
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
