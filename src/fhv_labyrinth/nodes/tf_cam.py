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
    0.045,#4.5cm
    0.008,#0.8cm
    0.14,#14cm
)

# for easy configuration https://quaternions.online
# euler: 0, 10, -5 deg
rotation_xyzw = (
    0.004,
    0.087,
    -0.043,
    0.995
)

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
