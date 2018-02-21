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

	ui_action_pose_topic = rospy.get_param('ui_action_pose_topic')
	classification_action = rospy.get_param('classification_action')

	controller = Controller(ui_action_pose_topic, classification_action)
	controller.start()
	# rospy.spin()


if __name__ == '__main__':
	main()