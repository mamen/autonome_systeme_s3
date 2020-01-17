#!/usr/bin/env python

import rospy

from topic_tools.srv import MuxList, MuxSelect, MuxSelectRequest
from std_msgs.msg import String
from random import randint

if __name__ == '__main__':
    try:
        rospy.init_node('mux_controller', anonymous=True)
        
        rate = rospy.Rate(0.2)

        rospy.wait_for_service('mux/list')
        mux_list = rospy.ServiceProxy('mux/list', MuxList)

        topics = mux_list().topics

        rospy.wait_for_service('mux/select')
        mux_select = rospy.ServiceProxy('mux/select', MuxSelect)

        while not rospy.is_shutdown():
            # print('randomly selecting topic of {}'.format(topics))
            next_topic = topics[randint(0,1)]
            # print('randomly chose {}'.format(next_topic))
            
            mux_select_req = MuxSelectRequest()
            mux_select_req.topic = next_topic
            mux_select_res = mux_select(mux_select_req)
            prev_topic = mux_select_res.prev_topic

            # print('topic changes, previous topic was {}'.format(prev_topic))
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
