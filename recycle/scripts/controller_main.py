#!/usr/bin/env python

import rospy
from recycle import Controller

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def main():
    rospy.init_node('recycle_controller')
    wait_for_time()

    move_request_topic = rospy.get_param('move_request_topic')
    classify_action = rospy.get_param('classify_action')

    controller = Controller(move_request_topic, classify_action)
    rospy.on_shutdown(controller.shutdown)
    controller.start()


if __name__ == '__main__':
    main()
