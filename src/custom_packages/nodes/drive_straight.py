#!/usr/bin/env python

import rospy

class Hello():

    def __init__(self):
        self.test()

    def test(self):
        while not rospy.is_shutdown():
            rospy.loginfo('HELLO WORLD')

def main():
    rospy.init_node('drive_straight')
    try:
        hello = Hello()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
