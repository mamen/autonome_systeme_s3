#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Point

class TagSaver(object):
    def __init__(self, topic, filename):
        self.filename = filename
        with open(self.filename, 'w+') as f:
            f.write("id;x;y;z\n")
        self.id = 1
        self.sub = rospy.Subscriber(topic, Point, self.onTag)

    def onTag(self, msg):
        print('found tag', msg)
        with open(self.filename, 'a+') as f:
            f.write("{};{};{};{}\n".format(self.id, msg.x, msg.y, msg.z))
        i += 1

if __name__ == '__main__':
    try:
        rospy.init_node('tag_saver', anonymous=True)
        topic = rospy.get_param('~topic', default='tags_found')
        filename = rospy.get_param('~filename')
        ts = TagSaver(topic, filename)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
