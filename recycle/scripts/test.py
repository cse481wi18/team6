#!/usr/bin/env python

import rospy
# from recycle import Controller
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

if __name__ == '__main__':
	rospy.init_node("recycle_test")
	wait_for_time()

	# c = Controller()
	# c.test()

	move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	move_base_client.wait_for_server()
	rospy.loginfo('done wait for move base')

	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.pose.position.x = 1.17713165283
	goal.target_pose.pose.position.y = -5.16732311249
	goal.target_pose.pose.orientation.z = 0.0604100547386
	goal.target_pose.pose.orientation.w = 0.998173644857

	rospy.logerr("before send goal")
	move_base_client.send_goal(goal)
	move_base_client.wait_for_result()
	result = move_base_client.get_result()
	rospy.logerr("done: {}".format(result))
