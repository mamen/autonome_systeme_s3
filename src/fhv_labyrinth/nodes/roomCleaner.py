#!/usr/bin/env python

import rospy
from numpy import array, reshape, where, floor, zeros_like
from nav_msgs.msg import OccupancyGrid, Odometry
import cv2 as cv
import numpy as np
from cv2 import imshow, waitKey
from time import sleep

class RoomCleaner(object):
    def __init__(self, map_topic, odom_topic):
        self.has_map = False
        self.has_odom = False
        self.map_subscription = rospy.Subscriber(map_topic, OccupancyGrid, self.onMapChange)
        self.odom_subscription = rospy.Subscriber(odom_topic, Odometry, self.onOdomChange)

    def onMapChange(self, map_msg):
        origin_pos = map_msg.info.origin.position
        
        # update info
        self.map_resolution = map_msg.info.resolution
        self.map_dim = (map_msg.info.height, map_msg.info.width)
        self.map_origin = (origin_pos.x, origin_pos.y)
        
        # update data
        self.map_data = array(map_msg.data).reshape(self.map_dim)

        # initially you want to visit all fields whether they are reachable or not
        if not self.has_map:
            self.fields_to_visit = zeros(self.map_dim)==0
            self.has_map = True
        
        # run calculations with new updated value
        if self.has_odom:
            self.calc()

    def onOdomChange(self, odom_msg):
        odom_pos = odom_msg.pose.pose.position

        # update position
        self.odom_pos = (odom_pos.x, odom_pos.y)
        
        if not self.has_odom:
            self.has_odom = True
        
        # run calculations with new updated value
        if self.has_map:
            self.calc()

    def calc(self):        
        #dim = self.map_dim # (height, width)

        resolution = self.map_resolution # in cm
        origin = array(self.map_origin) # (x, y) in m
        pos = array(self.odom_pos) # (x, y) in ??

        # position in occupancy grid (x, y)
        robot_pos = ((pos - origin) / resolution).astype(int)
        
        # currently visited field is not needed to be visited again
        x, y = robot_pos
        self.fields_to_visit[x][y] = False

        # m
        # -1
        # 0
        # 100
        
        # 384x384 field -> (-1=undiscovered, 0=free, 100=occupied)
        fields_visitable = self.map_data==0 # actually 1-99 would work too
        
        self.fields_remaining = fields_visitable & self.fields_to_visit

        # c
        # True: field i wanna visit
        # False: field i want to avoid
        
        # 1



        data = self.data

        print(robot_pos)

def run():
    map_topic = rospy.get_param("~map_topic")
    odom_topic = rospy.get_param("~odom_topic")
    room_cleaner = RoomCleaner(map_topic, odom_topic)
    rospy.spin()
    # pub = rospy.Publisher('detectTags', TagDetected, queue_size=10)
    #IMAGE_TOPIC = rospy.get_param("~img_topic")
    
    #while not rospy.is_shutdown():
        # rospy.loginfo(text)
    #    sleep(1)


def main():
    rospy.init_node('room_cleaner')

    try:
        run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()

'''
from numpy import array, zeros
dim = (3,3) # dimensions of map
c = zeros(dim)==0 # current fields to visit
m = array([[ -1,   0, 100],
       [ -1,   0,   4],
       [ -1, 100, 100]]) # current map data
p = (array([1]), array([2])) # current position in map

...

c[p] = False # mark current field as visited
(m==0) & c # fields to visit next (true: to visit)
'''
