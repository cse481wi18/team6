#!/usr/bin/env python
import math

import actionlib
import fetch_api
import rospy

from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from recycle_msgs.msg import ActionPose
from recycle_msgs.msg import ClassifyAction, ClassifyActionGoal

from visualization_msgs.msg import Marker

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
        self._head = fetch_api.Head()
        self._arm = fetch_api.Arm()

        # init subscribers and action clients
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


        ########### FOR DEBUGGING ##################
        self._marker_pub = rospy.Publisher('recycle/object_markers', Marker, queue_size=10)

        rospy.loginfo("Done init")


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
            rospy.loginfo("Processing request:")
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
                pointcloud_result = self._classify_pointcloud()

                # add obstacles to PlanningScene for the arm
                self._add_obstacles(pointcloud_result)

                # now, bus the classified objects
                self._bus_objects(pointcloud_result)

            elif request.action == "home":
                continue

            else:
                rospy.logerr("Unknown action!")
            
            rospy.loginfo("Request completed.")


    def _classify_pointcloud(self):
        self._classify_client.send_goal(ClassifyActionGoal())
        self._classify_client.wait_for_result() # set a timeout?
        pointcloud_result = self._classify_client.get_result()

        rospy.loginfo("Got results from classifier.")
        return pointcloud_result


    def _add_obstacles(self, pointcloud_result):
        # TODO add obstacles to planning scene
        pass


    def _bus_objects(self, classifications):
        rospy.loginfo("Start bussing objects...")

        for i in range(classifications.num_objects):
            obj_posestamped = classifications.poses[i]
            obj_dim = classifications.dimensions[i]
            obj_name = classifications.classifications[i]
            category = self._category_map[obj_name]
            bin_pose = self.BIN_POSES[category]

            # DEBUG
            self._pub_object_marker(i, obj_name, obj_posestamped.pose, obj_dim)


            # if confidence < thresh, default to landfill
            self._pickup_object(obj_posestamped, obj_dim, bin_pose)

            rospy.loginfo("Done with object {}: \'{}\' to {}".format(i, obj_name, category))

        # TODO put arm back to 'home position'


    # Given the target object's pose and dimension, and the target
    # bin's pose, pick up object and drop in the bin.
    def _pickup_object(self, obj_posestamped, obj_dim, bin_pose):
        # TODO
        # 1. Move gripper to top of obj_pose
        #    a. gripper face down
        #    b. object in the center of the gripper
        # 2. Move down, grip object, move back up
        # 3. Move to above bin_pose, drop object

        rospy.loginfo("obj_posestamped: {}".format(obj_posestamped))
        rospy.loginfo("obj_dim: {}".format(obj_dim))
        rospy.loginfo("bin_pose: {}".format(bin_pose))
        pass


    ##### FOR DEBUGGING #####
    def _pub_object_marker(self, i, obj_name, obj_pose, obj_dim):
        marker = Marker()

        marker.ns = obj_name
        marker.id = i
        marker.header.frame_id = 'base_link'
        marker.type = Marker.CUBE
        marker.pose = obj_pose
        marker.scale = obj_dim
        marker.color.g = 1
        marker.color.a = 0.3

        self._marker_pub.publish(marker)