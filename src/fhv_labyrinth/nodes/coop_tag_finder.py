#!/usr/bin/env python
import rospy
import actionlib

# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point
from std_msgs.msg import Header

from std_srvs.srv import Empty, EmptyRequest

# TODO: use correct type here
coop_data_class = Point

class CoopTagFinder(object):
  def __init__(self, tag_positions, search_topic, found_topic):
    self.search_list = set(tag_positions)
    self.searching_list = set()
    self.found_list = set()

    self.pub_search = rospy.Publisher(search_topic, coop_data_class, queue_size=2)
    self.pub_found = rospy.Publisher(found_topic, coop_data_class, queue_size=2)

    self.sub_search = rospy.Subscriber(search_topic, coop_data_class, self.onSearch)
    self.sub_found = rospy.Subscriber(found_topic, coop_data_class, self.onFound)

    # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        
    # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

  def done(self):
    return (len(self.search_list) + len(self.searching_list)) == 0

  def next_target(self):
    next_search_list = self.search_list if len(self.search_list) else self.searching_list
    return next(iter(next_search_list))

  def onSearch(self, msg):
    # TODO shift tag positions in state sets
    # TODO add tolerance to identify tag
    pass

  def onFound(self, msg):
    # TODO shift tag positions in state sets
    # TODO add tolerance to identify tag
    pass

  def localize(self):
    rospy.wait_for_service('amcl/global_localization')
    amcl_global_localization = rospy.ServiceProxy('amcl/global_localization', Empty)
    amcl_global_localization(EmptyRequest())

  def findAll(self):
    if self.done():
      return

    self.localize()

    while not rospy.is_shutdown() and not self.done():
      (next_x, next_y) = self.next_target()
      target = Point(x=next_x, y=next_y)

      # communicate to others that you look after this point
      self.pub_search.publish(target)

      # Creates a new goal with the MoveBaseGoal constructor      
      goal = MoveBaseGoal()
      
      # set header
      goal.target_pose.header.frame_id = 'map'
      goal.target_pose.header.stamp = rospy.Time.now()
      
      # set position
      goal.target_pose.pose.position = target

      # set orientation
      goal.target_pose.pose.orientation.w = 1.0

      # Sends the goal to the action server.
      client.send_goal(goal)
      # Waits for the server to finish performing the action.
      wait = client.wait_for_result()
      # If the result doesn't arrive, assume the Server is not available
      if not wait:
          rospy.logerr("Action server not available!")
          rospy.signal_shutdown("Action server not available!")
      else:
          # Result of executing the action
          result = client.get_result()

          # TODO analyze result to determine whether goal was found or not
          self.pub_found(target)

def main():
    try:
        rospy.init_node('coop_tag_finder', anonymous=True)

        search_topic = rospy.get_param('~search_topic', default='/search')
        found_topic = rospy.get_param('~found_topic', default='/found')

        tag_positions = [(1, 1)]

        ctf = CoopTagFinder(tag_positions, search_topic, found_topic)
        ctf.findAll()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
