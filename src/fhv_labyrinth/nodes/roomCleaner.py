#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, Odometry
from time import sleep

class RoomCleaner(object):
    def __init__(self, rate, map_topic, odom_topic):
        self.has_map = False
        self.has_odom = False
        self.rate = rate
        self.map_subscription = rospy.Subscriber(map_topic, OccupancyGrid, self.onMapChange)
        self.odom_subscription = rospy.Subscriber(odom_topic, Odometry, self.onOdomChange)

    def fieldIndex2Coordinates(self, fieldIndex):
        resolution = self.map_resolution # in cm
        origin = np.array(self.map_origin) # (x, y) in m
        fi = np.array(fieldIndex) # (x, y) in ??

        co = fi * resolution + origin
        return co

    def coordinates2FieldIndex(self, coordinates):
        resolution = self.map_resolution # in cm
        origin = np.array(self.map_origin) # (x, y) in m
        co = np.array(coordinates) # (x, y) in ??

        # position in occupancy grid (x, y)
        fi = ((co - origin) / resolution).astype(int)
        return fi

    def onMapChange(self, map_msg):
        origin_pos = map_msg.info.origin.position
        
        # update info
        self.map_resolution = map_msg.info.resolution
        self.map_dim = (map_msg.info.height, map_msg.info.width)
        self.map_origin = (origin_pos.x, origin_pos.y)
        
        # update data
        self.map_data = np.array(map_msg.data).reshape(self.map_dim)

        # initially you want to visit all fields whether they are reachable or not
        if not self.has_map:
            self.fields_to_visit = np.zeros(self.map_dim)==0
            self.has_map = True

    def onOdomChange(self, odom_msg):
        odom_pos = odom_msg.pose.pose.position

        # update position
        self.odom_pos = (odom_pos.x, odom_pos.y)
        
        if not self.has_odom:
            self.has_odom = True

    def calc(self):
        robot_indexPos = self.coordinates2FieldIndex(self.odom_pos)

        x, y = robot_indexPos
        self.fields_to_visit[x][y] = False

        # m
        # -1
        # 0
        # 100
        
        # 384x384 field -> (-1=undiscovered, 0=free, 100=occupied)
        fields_visitable = self.map_data==0 # actually 1-99 would work too
        
        fields_remaining = fields_visitable & self.fields_to_visit

        # find index of next field which is true
        a, b = np.indices(self.map_dim)
        c = ((a-x)**2+(b-y)**2)**0.5
        c[fields_remaining==False] = np.nan
        
        idx = np.unravel_index(np.nanargmin(c, axis=None), c.shape)

        next_coordinates = self.fieldIndex2Coordinates(idx)

        return next_coordinates

    def run(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            computed = self.calc()
            # self.pub.publish(computed)
            r.sleep()


def run():
    rate = rospy.get_param("~rate")
    map_topic = rospy.get_param("~map_topic")
    odom_topic = rospy.get_param("~odom_topic")
    room_cleaner = RoomCleaner(rate, map_topic, odom_topic)
    room_cleaner.run()

def main():
    rospy.init_node('room_cleaner')

    try:
        run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
