#!/usr/bin/env python
import rospy
import tf
import numpy as np

# framerate (hz)
hz = 10

# tf frame names
child = 'camera_link'
parent = 'base_link'

# config
translation_xyz = (
    0, # 0m
    0, # 0m
    -0.025, # -2.5cm
)

# for easy configuration https://quaternions.online
# euler: 0, 0, 0
rotation_xyzw = (
    0.0,
    0.0,
    0.0,
    1.0
)

"""
This node transforms the camera_link frame to the base_link frame
"""
def main():
    try:
        rospy.init_node('tf_cam', anonymous=True)
        tb = tf.TransformBroadcaster(queue_size=100)
        rate = rospy.Rate(hz)

        while not rospy.is_shutdown():            
            tb.sendTransform(
                translation_xyz,
                rotation_xyzw,
                rospy.Time.now(),
                child,
                parent
            )
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
