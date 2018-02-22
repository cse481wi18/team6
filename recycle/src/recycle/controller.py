#!/usr/bin/env python

import actionlib
import rospy
import fetch_api

from recycle_msgs.msg import ActionPose
from recycle_msgs.msg import ClassifyAction, ClassifyActionGoal
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

MOVE_BASE_ACTION = 'move_base'

class Controller(object):

    def __init__(self, move_request_topic, classify_action):
        self._head = fetch_api.Head()
        # TODO self._arm = fetch_api.Arm()

        self._request_queue = []

        # subscribe to the move requests from the UI
        self._move_request_sub = rospy.Subscriber(move_request_topic,
                                                 ActionPose,
                                                 callback=self._move_request_cb)
        # action client to navigate robot
        self._move_base_client = actionlib.SimpleActionClient(MOVE_BASE_ACTION, MoveBaseAction)
        # action client to classify objects
        self._classify_client = actionlib.SimpleActionClient(classify_action, ClassifyAction)

        rospy.loginfo("Waiting for action servers...")
        self._move_base_client.wait_for_server()
        rospy.loginfo("Done waiting for move_base server.")
        self._classify_client.wait_for_server()
        rospy.loginfo("Done waiting for classify server.")


    def _move_request_cb(self, msg):
        # add goal pose to queue
        self._request_queue.append(msg)
        rospy.loginfo("New move request queued. Queue size: {}.".format(len(self._request_queue)))


    def start(self):
        # Inifinitely check if there is anything in the queue and process requests
        while True:
            if len(self._request_queue) == 0:
                continue

            request = self._request_queue.pop(0)
            rospy.loginfo("Processing a request:")
            rospy.loginfo(request)
            rospy.loginfo("Num requests left in q: {}".format(len(self._request_queue)))

            # navigate to target pose
            goal = MoveBaseGoal()
            goal.target_pose = request.target_pose
            self._move_base_client.send_goal(goal)
            self._move_base_client.wait_for_result()
            rospy.loginfo("Arrived at target pose.")

            # TODO perform action once at target pose
            if request.action == "bus":
                # TODO
                # 1. look down
                # 2. classify
                # 3. bus objects
                rospy.loginfo("BUS action")
                pass

            elif request.action == "home":
                rospy.loginfo("HOME action")
                continue

            else:
                rospy.logerr("Unknown action!")



    def _classify_objects(self):
        # TODO build ClassificationActionGoal
        # self._classification_client.send(goal)
        # self._classification_client.wait_for_result() # set a timeout?

        # return self._classification_client.get_result()
        pass


