#!/usr/bin/env python

import rospy

from std_msgs.msg import String

selected = None

def onSelectedChange(msg):
    global selected
    selected = msg

def onMessage(msg):
    print('current topic: {}, message: {}'.format(selected.data, msg.data))

if __name__ == '__main__':
    try:
        rospy.init_node('consumer', anonymous=True)
        data_topic = rospy.get_param('~data_topic', default='cons')
        mux_topic = rospy.get_param('~mux_topic', default='mux/selected')
        data_sub = rospy.Subscriber(data_topic, String, onMessage)
        mux_sub = rospy.Subscriber(mux_topic, String, onSelectedChange)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
