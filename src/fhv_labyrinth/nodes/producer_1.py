#!/usr/bin/env python

import rospy

from std_msgs.msg import String

if __name__ == '__main__':
    try:
        rospy.init_node('producer_1', anonymous=True)
        
        topic = rospy.get_param('~topic', default='prod_1')
        hz = rospy.get_param('~rate', default=1)
        
        rate = rospy.Rate(hz)
        pub = rospy.Publisher(topic, String, queue_size=1, latch=True)

        while not rospy.is_shutdown():
            pub.publish('1 at {}'.format(rospy.get_time()))
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
