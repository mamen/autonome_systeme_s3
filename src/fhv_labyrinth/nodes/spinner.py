#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist
from topic_tools.srv import MuxSelect, MuxSelectRequest
from std_msgs.msg import String


BURGER_MAX_ANG_VEL = 2.84

def getTwist(linear_vel, angular_vel):
    twist = Twist()

    twist.linear.x = linear_vel
    twist.linear.y = 0.0
    twist.linear.z = 0.0

    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = angular_vel

    return twist

class Spinner(object):

    def __init__(self, topic, duration):
        self.topic = topic
        self.rate = rospy.Rate(1)
        
        self.pub = rospy.Publisher(self.topic, Twist, queue_size=1, latch=True)
        self.sub = rospy.Subscriber('mux/selected', String, self.onSelect)

        rospy.wait_for_service('mux/select')
        self.mux_select = rospy.ServiceProxy('mux/select', MuxSelect)
        
        mux_select_req = MuxSelectRequest()
        mux_select_req.topic = topic

        self.prev = self.mux_select(mux_select_req).prev_topic


    def onSelect(self, msg):
        print(msg.data, self.topic)
        if msg.data == self.topic:
            i = 2
            while not rospy.is_shutdown() and i:
                self.pub.publish(getTwist(0.0, BURGER_MAX_ANG_VEL))
                self.rate.sleep()
                i = i - 1
            
            self.pub.publish(getTwist(0.0, 0.0))

            mux_select_req = MuxSelectRequest()
            mux_select_req.topic = self.prev
            self.mux_select(mux_select_req)

        else:
            exit(0)

if __name__ == '__main__':
    try:
        rospy.init_node('spinner', anonymous=True)
        
        topic = rospy.get_param('~topic', default='spinner_vel')
    
        s = Spinner(topic, 1)
        
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
