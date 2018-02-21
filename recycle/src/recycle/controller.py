#!/usr/bin/env python                                                                                  
                                      
import actionlib                                                                 
import rospy
import fetch_api

from recycle_msgs.msg import ActionPose
# TODO: from ???_msgs.msg import ClassificationAction, ClassificationActionGoal
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

MOVE_BASE_ACTION = 'move_base'

class Controller(object):

	def __init__(self, action_pose_sub, classification_action):
		self._head = fetch_api.Head()
		# TODO self._arm = fetch_api.Arm()

		self._action_pose_queue = []

		# subscribe to target action_pose from the UI
		self._action_pose_sub = rospy.Subscriber(action_pose_sub, 
											   	 ActionPose, 
											   	 callback=self._action_pose_cb)
		# action client to navigate robot
		self._move_base_client = actionlib.SimpleActionClient(MOVE_BASE_ACTION, MoveBaseAction)
		# action client to classify objects
		# TODO: self._classification_client = actionlib.SimpleActionClient(classification_action, ClassificationAction)

		self._move_base_client.wait_for_server()
		# TODO: self._classification_client.wait_for_server()


	def _action_pose_cb(self, msg):
		# add goal pose to queue
		self._action_pose_queue.append(msg)
		rospy.loginfo("New action pose queued")


	def start(self):
		while True:
			if len(self._action_pose_queue) == 0:
				continue

			action_pose = self._action_pose_queue.pop(0)

			rospy.loginfo("Going to target pose: {}".format(action_pose.target_pose))
			# navigate to target pose
			goal = MoveBaseGoal()
			goal.target_pose = action_pose.target_pose
			self._move_base_client.send_goal(goal)
			self._move_base_client.wait_for_result()
			rospy.loginfo("Arrived")

			# TODO perform action once at target pose
			if action_pose.action == "bus":
				# TODO
				# 1. look down
				# 2. classify
				# 3. bus objects
				rospy.loginfo("BUS action")
				pass

			elif action_pose.action == "home":
				rospy.loginfo("HOME action")
				continue

			else:
				rospy.logerr("Unknown action!")



	def _classify_objects(self):
		# TODO build ClassificationActionGoal
		# self._classification_client.send(goal)
		# self._classification_client.wait_for_result()	# set a timeout?
 
		# return self._classification_client.get_result()
		pass

	