#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

class Drive():

    def __init__(self):
        self.test()

    def test(self):

        try:
            while not rospy.is_shutdown():
                # rospy.loginfo('HELLO WORLD')

                pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

                twist = Twist()

                twist.linear.x = 0.0

                pub.publish(twist)

        finally:
            twist = Twist()
            twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
            pub.publish(twist)

def main():
    rospy.init_node('drive_straight')

    try:
        hello = Drive()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
