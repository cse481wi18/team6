#!/usr/bin/env python
import copy
import math

import actionlib
import fetch_api
import rospy
from moveit_python import PlanningSceneInterface

from geometry_msgs.msg import PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import OrientationConstraint
from recycle_msgs.msg import ActionPose
from recycle_msgs.msg import ClassifyAction, ClassifyActionGoal
from visualization_msgs.msg import Marker

MOVE_BASE_ACTION = 'move_base'

class Controller(object):

    LOOK_DOWN_ANGLE = math.pi / 4  # angle to look down at the table

    GRIPPER_BASE_OFFSET = 0.166 - 0.03  # offset between wrist_roll_link and the base of the gripper
    GRASP_DIST = 0.02  # gripper base 2cm above the object
    
    GRIPPER_FINGERTIP_OFFSET = 0.166 + 0.03   # offset between wrist_roll_link and the END of the fingertips
    PRE_GRASP_DIST = 0.05  # fingertips 5cm above the object

    POST_GRASP_DIST = 0.05 # move 5cm back upwards after grasping

    CRANE_ORIENTATION = Quaternion(0.00878962595016, 
                                   0.711250066757, 
                                   -0.00930198933929,
                                   0.702822685242)

    MIN_CONFIDENCE = 1.0 #TODO 0.5  # threshold for when to default to landfill
    # TODO fill in position of the bins in PoseStamped (frame in base_link?)
    BIN_POSES = {
        "compost": None,
        "recycle": None,
        "landfill": None
    }

    def __init__(self, move_request_topic, classify_action):
        self._request_queue = []

        # init robot body controllers
        self._head = fetch_api.Head()
        self._arm = fetch_api.Arm()
        self._gripper = fetch_api.Gripper()

        # Planning Scene Interface
        self._planning_scene = PlanningSceneInterface('base_link')
        self._planning_scene.clear()
        self._num_prev_obstacles = 0
        self._crane_oc = self._create_crane_oc()

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


    def _create_crane_oc(self):
        # create orientation contraint for the crane gripper position
        # it only needs to be created once, and then passed into arm
        oc = OrientationConstraint()
        oc.header.frame_id = 'base_link'
        oc.link_name = 'wrist_roll_link'
        oc.weight = 1.0
        oc.orientation.w = 0.707
        oc.orientation.y = -0.707
        oc.absolute_x_axis_tolerance = 0.1
        oc.absolute_y_axis_tolerance = 0.1
        oc.absolute_z_axis_tolerance = 3.14
        return oc


    def _move_request_cb(self, msg):
        # add goal pose to queue
        self._request_queue.append(msg)
        rospy.loginfo("New move request queued. Queue size: {}.".format(len(self._request_queue)))


    def shutdown():
        self._arm.cancel_all_goals()


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
            # TODO: Uncomment this
            # goal = MoveBaseGoal()
            # goal.target_pose = request.target_pose
            # self._move_base_client.send_goal(goal)
            # self._move_base_client.wait_for_result()
            rospy.loginfo("Arrived at target. Performing \'{}\' action...".format(request.action))

            # perform action once at target pose
            if request.action == "bus":
                # TODO can't move head for some reason..
                # "Unable to stop head_controller/point_head when trying to start head_controller/follow_joint_trajectory"
                self._head.pan_tilt(0, self.LOOK_DOWN_ANGLE)  # look down
                classifier_result = self._classify_pointcloud()

                # add obstacles to PlanningScene for the arm
                # self._add_obstacles(classifier_result) TODO

                # now, bus the classified objects
                self._bus_objects(classifier_result)

            elif request.action == "home":
                pass

            else:
                rospy.logerr("Unknown action!")
            
            rospy.loginfo("Request completed.")


    def _classify_pointcloud(self):
        self._classify_client.send_goal(ClassifyActionGoal())
        self._classify_client.wait_for_result()
        classifier_result = self._classify_client.get_result()

        rospy.loginfo("Got results from classifier.")
        return classifier_result


    def _add_obstacles(self, classifier_result):
        # Clearing all the previous obstacles in the planning scene
        self._planning_scene.clear()
        for i in range(self._num_prev_obstacles):
            self._planning_scene.removeCollisionObject('obstacle_' + str(i))
        
        self._num_prev_obstacles = classifier_result.num_obstacles

        for i in range(classifier_result.num_obstacles):
            obstacle_pose = classifier_result.obstacle_poses[i]
            obstacle_dim = classifier_result.obstacle_dimensions[i]

            print("TABLE LOCATION")
            print([obstacle_dim.x, obstacle_dim.y, obstacle_dim.z,
                obstacle_pose.pose.position.x, obstacle_pose.pose.position.y, obstacle_pose.pose.position.z])

            # TODO: The parameters might need to be changed when we stop using the mock point cloud.
            # TODO: had to flip the x and y for some reason
            self._planning_scene.addBox('obstalce_' + str(i),
                                        obstacle_dim.y, 
                                        obstacle_dim.x, 
                                        obstacle_dim.z,
                                        obstacle_pose.pose.position.x, 
                                        obstacle_pose.pose.position.y, 
                                        obstacle_dim.z / 2.0) # Hacky fix


    def _bus_objects(self, classifier_result):
        rospy.loginfo("Start bussing objects...")

        for i in range(classifier_result.num_objects):
            obj_posestamped = classifier_result.object_poses[i]
            obj_dim = classifier_result.object_dimensions[i]

            # if confidence < thresh, default to landfill
            if classifier_result.confidence[i] < self.MIN_CONFIDENCE:
                category = 'landfill'
            else:
                category = classifier_result.classifications[i]
            bin_pose = self.BIN_POSES[category]

            # DEBUG
            self._pub_object_marker(i, category, obj_posestamped.pose, obj_dim)

            self._pickup_object(obj_posestamped, obj_dim, bin_pose)

            rospy.loginfo("Done with object {}: {}".format(i, category))

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
        self._gripper.open()

        gripper_goal = copy.deepcopy(obj_posestamped)
        gripper_goal.pose.orientation = self.CRANE_ORIENTATION
        obj_top_z = obj_posestamped.pose.position.z + obj_dim.z / 2.0

        # 1. Move to pre-grasp pose: end of fingertips at Xcm above the object
        gripper_goal.pose.position.z = (obj_top_z 
                                        + self.GRIPPER_FINGERTIP_OFFSET
                                        + self.PRE_GRASP_DIST)
        r = self._arm.move_to_pose(gripper_goal)

        rospy.loginfo("pre grasp result: {}".format(r))

        # 2. TODO: align gripper with shortest orientation
        # modify gripper_goal.pose.orientation

        # 3. Move down to grasp pose: gripper base X cm above the object
        # TODO add orientation constraint
        # TODO min of grasp pose or fingertip 0.5 cm above table
        gripper_goal.pose.position.z = (obj_top_z 
                                        + self.GRIPPER_BASE_OFFSET
                                        + self.GRASP_DIST)
        r = self._arm.move_to_pose(gripper_goal)
        self._gripper.close()

        rospy.loginfo("grasp result: {}".format(r))

        #. Move Xcm back up
        gripper_goal.pose.position.z += self.POST_GRASP_DIST
        r = self._arm.move_to_pose(gripper_goal)

        rospy.loginfo("post grasp result: {}".format(r))

        rospy.loginfo("obj_posestamped: {}".format(obj_posestamped))
        rospy.loginfo("obj_dim: {}".format(obj_dim))
        # rospy.loginfo("pre grasp pose: {}".format(gripper_goal))


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