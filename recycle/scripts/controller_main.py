#!/usr/bin/env python

import rospy
from fetch_api import Torso
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

	# TODO grab from the database??
	category_map = {
		"coffee_cup_sleeve": "compost",
		"coffee_cup_no_sleeve": "recycle",
		"crumpled_paper": "recycle",
		"nature_valley_wrapper": "landfill"
	}

	torso = Torso()
	torso.set_height(0.4)
	
	controller = Controller(move_request_topic, classify_action, category_map)
	controller.start()
	# rospy.spin()


if __name__ == '__main__':
	main()