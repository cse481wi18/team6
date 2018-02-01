#!/usr/bin/env python

import rospy
import tf
from util import wait_for_time

def main():
    rospy.init_node('ee_demo')
    wait_for_time()

    listener = tf.TransformListener()
    rate = rospy.Rate(1)
    rospy.sleep(0.1)

    while True:
        trans, rot = None, None
        tries = 5
        while tries >= 0:
            try:
                (trans, rot) = listener.lookupTransform('base_link', 'gripper_link', rospy.Time(0))
                break
            except Exception as e:
                tries -= 1

        rospy.loginfo('transform: {} rotation: {}'.format(trans, rot))
        rate.sleep()




if __name__ == '__main__':
    main()
