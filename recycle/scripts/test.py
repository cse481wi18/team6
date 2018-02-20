#!/usr/bin/env python

import rospy
from recycle import Controller

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

if __name__ == '__main__':
	rospy.init_node("recycle_test")
	wait_for_time()

	c = Controller()
	c.test()