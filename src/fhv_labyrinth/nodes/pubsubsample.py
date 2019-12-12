#!/usr/bin/env python

import rospy
from std_msgs.msg import String

class MyNode(object):
  '''
  asdfasdfasdf
  '''

  def __init__(self, rate, topic_out, topic_in):
    self.pub = rospy.Publisher(topic_out, String, queue_size=10)
    self.sub = rospy.Subscriber(topic_in, String, self.onValue)
    self.rate = rate
    self.value = None

  def onValue(self, msg):
    self.value = msg.data

  def compute(self):
    return 'computed {}'.format(self.value)

  def spin(self):
    r = rospy.Rate(self.rate)
    while not rospy.is_shutdown():
      computed = self.compute()
      self.pub.publish(computed)
      r.sleep()

def main():
  try:
    rospy.init_node('mynode', anonymous=True)
    mn = MyNode(10, '/out', '/in')
    mn.spin()
  except rospy.ROSInterruptException:
    pass
    
if __name__ == '__main__':
  main()
