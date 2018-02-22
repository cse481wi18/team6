#!/usr/bin/env python

import rospy
from recycle import Classifier

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def main():
    rospy.init_node('recycle_classifier')
    wait_for_time()

    server = Classifier('recycle_classifier/classify')
    rospy.spin()

if __name__ == '__main__':
    main()
