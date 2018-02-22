#!/usr/bin/env python
import math

import actionlib
import fetch_api
import rospy

from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from recycle_msgs.msg import ActionPose
from recycle_msgs.msg import ClassifyAction, ClassifyActionGoal

MOVE_BASE_ACTION = 'move_base'

class Controller(object):

    # angle to look down at the table
    LOOK_DOWN_ANGLE = math.pi / 2

    # TODO fill in position of the bins in PoseStamped (frame in base_link?)
    BIN_POSES = {
        "compost": None,
        "recycle": None,
        "landfill": None
    }

    def __init__(self, move_request_topic, classify_action, category_map):
        self._category_map = category_map
        self._request_queue = []

        # init robot body controllers
        # TODO torso? always set torso to max height??
        self._head = fetch_api.Head()
        # TODO self._arm = fetch_api.Arm()

        # init subscribers and action clients
        # subscribe to the move requests from the UI
        self._move_request_sub = rospy.Subscriber(move_request_topic,
                                                 ActionPose,
                                                 callback=self._move_request_cb)
        # action client to navigate robot
        self._move_base_client = actionlib.SimpleActionClient(MOVE_BASE_ACTION, MoveBaseAction)
        # action client to classify objects
        self._classify_client = actionlib.SimpleActionClient(classify_action, ClassifyAction)

        # wait for action servers
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
            rospy.loginfo("Arrived at target. Performing \'{}\' action...".format(request.action))

            # perform action once at target pose
            if request.action == "bus":
                # TODO can't move head for some reason..
                # "Unable to stop head_controller/point_head when trying to start head_controller/follow_joint_trajectory"
                self._head.pan_tilt(0, self.LOOK_DOWN_ANGLE)  # look down
                classifications = self._classify_objects()
                self._bus_objects(classifications)

            elif request.action == "home":
                continue

            else:
                rospy.logerr("Unknown action!")
            
            rospy.loginfo("Action completed.")


    def _classify_objects(self):
        self._classify_client.send_goal(ClassifyActionGoal())
        self._classify_client.wait_for_result() # set a timeout?
        classifications = self._classify_client.get_result()

        rospy.loginfo("Got classifications.")
        return classifications


    def _bus_objects(self, classifications):
        rospy.loginfo("Start bussing objects... TODO")
        # TODO
        # planning scene??? need to have obstacles for tables, etc
        # pick up each object and put it into the correct bin

        for i in range(classifications.num_objects):
            obj = classifications.bounding_boxes[i]
            label = classifications.classifications[i]
            category = self._category_map[label]

            # TODO
            # pick up object
            # move gripper to correct bin and drop (self.BIN_POSES)

            rospy.loginfo("Done with object {}".format(i))            

        # TODO put arm back to 'home position'
