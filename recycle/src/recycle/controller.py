#!/usr/bin/env python
import copy
import math
import numpy as np

import actionlib
import fetch_api
import rospy
import utils
from joint_state_reader import JointStateReader
from moveit_python import PlanningSceneInterface

from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Vector3
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import OrientationConstraint
from recycle_msgs.msg import ActionPose
from recycle_msgs.msg import ClassifyAction, ClassifyActionGoal
from visualization_msgs.msg import Marker

MOVE_BASE_ACTION = 'move_base'

class Controller(object):

    # Error handling tolerences
    ZERO_OBSTACLES = 1
    TABLE_HEIGHT_FAILURE = 2
    FAILED_TO_BUS = 3
    ITEM_ATTEMPTS = 2

    TORSO_HEIGHT = 0.4

    # head.look_at(frame_id, x, y, z)
    LOOK_AT_TABLE = ('base_link', 0.7, 0.0, 0.75)
    TABLE_HEIGHT_THRESH = 0.7

    GRIPPER_WIDTH = 0.10
    GRIPPER_BASE_OFFSET = 0.166 - 0.03  # offset between wrist_roll_link and the base of the gripper
    GRIPPER_FINGERTIP_OFFSET = 0.166 + 0.03   # offset between wrist_roll_link and the END of the fingertips

    PRE_PRE_GRASP_X = 0.65
    PRE_PRE_GRASP_Z = 1.25

    PRE_GRASP_DIST = 0.03  # fingertips 5cm above the object
    GRASP_DIST = 0.02  # gripper base 2cm above the object
    TABLE_DIST = 0.005  # fingertip 0.5cm above the table
    POST_GRASP_DIST = 0.05 # move 5cm back upwards after grasping

    NUM_ARM_ATTEMPTS = 3

    # Python 2 doesn't have ENUMS FFFFFUUUUUUUU
    BUS_RESULT = {'success':0, 'missed':1, 'failed_all':2}

    CRANE_ORIENTATION = Quaternion(0.00878962595016,
                                   0.711250066757,
                                   -0.00930198933929,
                                   0.702822685242)

    # The center of the top of the bins in base_link frame.
    # Right to left (for Astro): compost, landfill, recycle
    DROPOFF_X_PADDING = 0.1
    DROPOFF_Z_PADDING = 0.0
    BIN_WIDTH = 0.153
    BIN_POSES = {
        'compost': {'x': 0.18, 'y': -0.15955811739, 'z': 0.515},
        'landfill': {'x': 0.18, 'y': -0.0019529312849, 'z': 0.515},
        'recycle': {'x': 0.18, 'y': 0.154245600104, 'z': 0.515}
    }
    # Dimensions of the cardboard ramps
    RAMP_WIDTH = 0.14
    # RAMP_HEIGHT = 0.22
    RAMP_HEIGHT = BIN_WIDTH

    PRE_BIN_POSES = {
        'compost': {'x': 0.3, 'y': -0.36, 'z': 1.05},
        'landfill': {'x': 0.3, 'y': 0.36, 'z': 1.05},
        'recycle': {'x': 0.3, 'y': 0.36, 'z': 1.05}
    }

    def __init__(self, move_request_topic, classify_action):
        self.RAMP_MESH_FILE = rospy.get_param('ramp_mesh_file', "/home/team6/catkin_ws/src/cse481wi18/recycle/src/recycle/rampMesh.stl")
        self.GRIPPER_EFFORT = rospy.get_param('gripper_effort', 70)
        self._request_queue = []
        self._move_summary = []
        # moveit_python planning scene is STUPID. Once you removed an
        # attached object, it is forever added to `_attached_removed`
        # and it will always be "supposed to be removed". Hack to not
        # add with the same name.
        self._attached_i = 0

        # init robot body controllers
        self._head = fetch_api.Head()
        self._arm = fetch_api.Arm()
        self._gripper = fetch_api.Gripper()
        self._torso = fetch_api.Torso()

        # init joint state reader
        self._reader = JointStateReader()

        # Planning Scene Interface
        self._planning_scene = PlanningSceneInterface('base_link')
        self._planning_scene.clear()
        self._attach_bin_obstacles()
        self._num_prev_obstacles = 0
        self._crane_oc = self._create_crane_oc()

        # init subscribers and action clients
        self._move_request_sub = rospy.Subscriber(move_request_topic,
                                                 ActionPose,
                                                 callback=self._move_request_cb)
        # action client to navigate robot
        # self._move_base_client = actionlib.SimpleActionClient(MOVE_BASE_ACTION, MoveBaseAction)
        # action client to classify objects
        self._classify_client = actionlib.SimpleActionClient(classify_action, ClassifyAction)

        # wait for action servers
        rospy.loginfo("Waiting for action servers...")
        # self._move_base_client.wait_for_server()
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


    def _attach_bin_obstacles(self):
        frame_attached_to = 'base_link'
        frames_okay_to_collide_with = ['base_link', 'laser_link']

        for category in self.BIN_POSES:
            rospy.loginfo('add bin obstacle: {}'.format(category))
            # the bin itself
            bin_name = category + "_bin"
            self._planning_scene.attachBox(bin_name,
                                           self.BIN_WIDTH, self.BIN_WIDTH, self.BIN_WIDTH,
                                           self.BIN_POSES[category]['x'],
                                           self.BIN_POSES[category]['y'],
                                           self.BIN_POSES[category]['z'] - self.BIN_WIDTH/2.0,
                                           frame_attached_to,
                                           frames_okay_to_collide_with)
            # Just boxes
            ramp_name = category + "_ramp"
            ramp_x = self.BIN_POSES[category]['x'] + self.BIN_WIDTH/2.0 + self.RAMP_WIDTH/2.0
            ramp_z = self.BIN_POSES[category]['z'] - self.BIN_WIDTH + self.RAMP_HEIGHT/2.0
            self._planning_scene.attachBox(ramp_name,
                                           self.RAMP_WIDTH,
                                           self.BIN_WIDTH,
                                           self.RAMP_HEIGHT,
                                           ramp_x,
                                           self.BIN_POSES[category]['y'],
                                           ramp_z,
                                           frame_attached_to,
                                           frames_okay_to_collide_with)

            # MESH
            # the cardboard ramps
            # name = category + "_ramp"
            # ramp_pose = Pose()
            # ramp_pose.position.x = self.BIN_POSES[category]['x'] + self.BIN_WIDTH/2.0
            # ramp_pose.position.y = self.BIN_POSES[category]['y'] - self.BIN_WIDTH/2.0
            # ramp_pose.position.z = self.BIN_POSES[category]['z'] - self.BIN_WIDTH
            # ramp_pose.orientation.w = 1.0
            # self._planning_scene.attachMesh(name,
            #                                ramp_pose,
            #                                self.RAMP_MESH_FILE,
            #                                frame_attached_to,
            #                                frames_okay_to_collide_with)


    def _move_request_cb(self, msg):
        # add goal pose to queue
        self._request_queue.append(msg)
        rospy.loginfo("New move request queued. Queue size: {}.".format(len(self._request_queue)))


    def shutdown(self):
        self._arm.cancel_all_goals()

        for category in self.BIN_POSES:
            self._planning_scene.removeAttachedObject(category + "_bin", wait=True)
            self._planning_scene.removeAttachedObject(category + "_ramp", wait=True)
            self._planning_scene.removeCollisionObject(category + "_bin", wait=True)
            self._planning_scene.removeCollisionObject(category + "_ramp", wait=True)

        for i in range(self._attached_i):
            self._planning_scene.removeAttachedObject("attached_object_{}".format(i), wait=True)
            self._planning_scene.removeCollisionObject("attached_object_{}".format(i), wait=True)


    def start(self):
        # Inifinitely check if there is anything in the queue and process requests
        while True:
            no_objects = 0
            table_height_fail = 0
            failed_to_bus_all = 0

            if len(self._request_queue) == 0:
                continue

            request = self._request_queue.pop(0)
            rospy.loginfo("Processing request:")
            rospy.loginfo(request)
            rospy.loginfo("Num requests left in q: {}".format(len(self._request_queue)))

            # navigate to target pose
            do_navigation = rospy.get_param("do_navigation", True)
            if do_navigation:
                rospy.loginfo("Navigating..")
                goal = MoveBaseGoal()
                goal.target_pose = request.target_pose
                # self._move_base_client.send_goal(goal)
                # self._move_base_client.wait_for_result()
            rospy.loginfo("Arrived at target. Performing \'{}\' action...".format(request.action))

            # perform action once at target pose
            if request.action == "bus":
                while True:
                    self._torso.set_height(self.TORSO_HEIGHT)
                    self._head.look_at(*self.LOOK_AT_TABLE)
                    rospy.sleep(2) # TODO

                    classifier_result = self._classify_pointcloud()

                    if classifier_result and classifier_result.num_objects == 0:
                        if no_objects < self.ZERO_OBSTACLES:
                            no_objects += 1
                            continue
                        else:
                            rospy.logerr("Failed to find any objects " + str(self.ZERO_OBSTACLES) + " times")
                            break

                    # There is at least one object in the scene.
                    if classifier_result:
                        # add obstacles to PlanningScene for the arm
                        if self._add_env_obstacles(classifier_result):
                            # add objects as obstacles on the table before bussing
                            self._add_object_obstacles(classifier_result)

                            # now, bus the classified objects
                            bussed = self._bus_objects(classifier_result)


                            # remove the obstacles now that we are done bussing
                            self._remove_env_obstacles()
                            self._remove_object_obstacles(classifier_result)
                            
                            if not bussed and failed_to_bus_all < self.FAILED_TO_BUS:
                                failed_to_bus_all += 1
                                continue
                            else:
                                rospy.logerr("Failed to bus any object. Tried " + str(self.FAILED_TO_BUS) + " times")
                                break

                        else:
                            if table_height_fail < self.TABLE_HEIGHT_FAILURE:
                                table_height_fail += 1
                                continue
                            else:
                                rospy.logerr("Table height too low. Tried " + str(self.TABLE_HEIGHT_FAILURE) + " times")
                                break


            elif request.action == "rest":
                pass

            else:
                rospy.logerr("Unknown action!")

            rospy.loginfo("Request completed.")


    def _classify_pointcloud(self):
        self._classify_client.send_goal(ClassifyActionGoal())
        self._classify_client.wait_for_result()
        classifier_result = self._classify_client.get_result()

        if classifier_result:
            rospy.loginfo("Got results from classifier:")
            rospy.loginfo(classifier_result)
        else:
            rospy.logerr("Got None from classifier")

        return classifier_result

    def _remove_env_obstacles(self):
        rospy.loginfo("Removing env obstacles")
        for i in range(self._num_prev_obstacles):
            name = 'obstacle_{}'.format(i)
            self._planning_scene.removeCollisionObject(name, wait=True)

    def _add_env_obstacles(self, classifier_result):
        rospy.loginfo("ADD TABLE")
        self._num_prev_obstacles = classifier_result.num_obstacles

        for i in range(classifier_result.num_obstacles):
            obstacle_pose = classifier_result.obstacle_poses[i]
            obstacle_dim = classifier_result.obstacle_dimensions[i]

            rospy.loginfo(obstacle_pose)
            rospy.loginfo(obstacle_dim)

            flip_obstacles = rospy.has_param('flip_obstacles') and rospy.get_param('flip_obstacles')
            table_extension = rospy.get_param('table_extension', 2)
            if flip_obstacles:
                obstacle_dim.x *= table_extension
                self._planning_scene.addBox('obstalce_' + str(i),
                                        obstacle_dim.y,  # TODO hacky
                                        obstacle_dim.x,
                                        obstacle_dim.z,
                                        obstacle_pose.pose.position.x,
                                        obstacle_pose.pose.position.y,
                                        obstacle_pose.pose.position.z,
                                        wait=True)
            else: # Don't flip. Double y dimension
                obstacle_dim.y *= table_extension
                self._planning_scene.addBox('obstalce_' + str(i),
                                        obstacle_dim.x,
                                        obstacle_dim.y,  # TODO hacky
                                        obstacle_dim.z,
                                        obstacle_pose.pose.position.x,
                                        obstacle_pose.pose.position.y,
                                        obstacle_pose.pose.position.z,
                                        wait=True)

            if obstacle_pose.pose.position.z + obstacle_dim.z/2.0 < self.TABLE_HEIGHT_THRESH:
                rospy.logerr("Table obstacle NOT tall enough!! Aborting!!")
                return False

        return True

    def _remove_object_obstacles(self, classifier_result):
        rospy.loginfo("Removing all object obstacles")
        for i in range(classifier_result.num_objects):
            name = 'collision_object_{}'.format(i)
            self._planning_scene.removeCollisionObject(name, wait=True)

    def _add_object_obstacles(self, classifier_result):
        rospy.loginfo("Adding all objects as obstacles")
        for i in range(classifier_result.num_objects):
            obj_pose = classifier_result.object_poses[i]
            obj_dim = classifier_result.object_dimensions[i]
            name = 'collision_object_{}'.format(i)
            self._planning_scene.addBox(name,
                                        obj_dim.x, obj_dim.y, obj_dim.z,
                                        obj_pose.pose.position.x,
                                        obj_pose.pose.position.y,
                                        obj_pose.pose.position.z,
                                        wait=True)

    def _bus_objects(self, classifier_result):
        rospy.loginfo("Start bussing objects...")



        obj_posestamped = classifier_result.object_poses[0]
        obj_dim = classifier_result.object_dimensions[0]
        category = classifier_result.classifications[0]

        # DEBUG
        # self._pub_bin_markers()
        # self._pub_object_marker(0, category, obj_posestamped.pose, obj_dim)

        # pickedup = self._pickup_object(0, obj_posestamped, obj_dim, category)
        # if pickedup:
        #     rospy.loginfo("Picked up object {}: {}".format(0, category))
        # else:
        #     gripper_goal = copy.deepcopy(obj_posestamped)

        #     pre_bin_pose = self.PRE_BIN_POSES['recycle']
        #     gripper_goal.pose.position.x = pre_bin_pose['x']
        #     gripper_goal.pose.position.y = pre_bin_pose['y']
        #     gripper_goal.pose.position.z = pre_bin_pose['z']
        #     self._arm_move_to_pose_attempt(gripper_goal, "post grasp")
        #     # rospy.logwarn("Failed to pick up object {}, moving on".format(i))
        # self._print_move_summary()

        # TODO: Ariel's old code for iterating through all the objects
        # for i in range(classifier_result.num_objects):
        num_attempts = 0
        i = 0
        while i < classifier_result.num_objects:
            obj_posestamped = classifier_result.object_poses[i]
            obj_dim = classifier_result.object_dimensions[i]
            category = classifier_result.classifications[i]

            # DEBUG
            # self._pub_bin_markers()
            self._pub_object_marker(i, category, obj_posestamped.pose, obj_dim)
            self._print_move_summary()

            pickedup = self._pickup_object(i, obj_posestamped, obj_dim, category)
            if pickedup:
                rospy.loginfo("Picked up object {}: {}".format(i, category))
                return True
            else:
                gripper_goal = copy.deepcopy(obj_posestamped)
                pre_bin_pose = self.PRE_BIN_POSES['recycle']
                gripper_goal.pose.position.x = pre_bin_pose['x']
                gripper_goal.pose.position.y = pre_bin_pose['y']
                gripper_goal.pose.position.z = pre_bin_pose['z']
                self._arm_move_to_pose_attempt(gripper_goal, "post grasp")
                
                num_attempts += 1
                if num_attempts < self.ITEM_ATTEMPTS:
                    continue
                else: 
                    num_attempts = 0
                    i += 1
                    rospy.logwarn("Failed to pick up object {}, moving on".format(i))

        # TODO put arm back to 'home position'
        return False

    # Attempts to align gripper to object's shortest dimension
    # Returns True if success, False otherwise
    def _align_gripper_to_shortest_dim(self, gripper_goal, obj_posestamped, obj_dim):
        # 1. check if there is a grippable dimension
        if obj_dim.x > self.GRIPPER_WIDTH and obj_dim.y > self.GRIPPER_WIDTH:
            rospy.logwarn("Object is too big to grip!")
            return False

        theta = utils.quaternion_to_angle(obj_posestamped.pose.orientation)

        rospy.logerr("theta1: {}".format(theta))

        # Gripper is default to align with base_link's y axis
        if obj_dim.x < obj_dim.y:
            theta = -(np.pi/2.0 - theta)

        rospy.logerr("theta2: {}".format(theta))

        aligned_orien = utils.rotate_quaternion_by_angle(gripper_goal.pose.orientation, theta)
        gripper_goal.pose.orientation = aligned_orien

        return self._arm_move_to_pose_attempt(gripper_goal, "rotate")


    # Given the target object's pose and dimension, and the target
    # bin's pose, pick up object and drop in the bin.
    def _pickup_object(self, i, obj_posestamped, obj_dim, category):
        # -1. make sure gripper is initially open
        self._gripper.open()

        gripper_goal = copy.deepcopy(obj_posestamped)
        gripper_goal.pose.orientation = self.CRANE_ORIENTATION

        # 0. Move to pre-pre-grasp pose:
        gripper_goal.pose.position.x = self.PRE_PRE_GRASP_X
        gripper_goal.pose.position.y = 0
        gripper_goal.pose.position.z = self.PRE_PRE_GRASP_Z
        if not self._arm_move_to_pose_attempt(gripper_goal, "pre pre grasp"):
            # can't even get to pre pre grasp.. # failed, don't need rescan
            return False

        # prep gripper pose to go to object
        gripper_goal.pose.position = copy.deepcopy(obj_posestamped.pose.position)
        obj_top_z = obj_posestamped.pose.position.z + obj_dim.z / 2.0
        obj_bottom_z = obj_posestamped.pose.position.z - obj_dim.z / 2.0

        # 1. Move to pre-grasp pose: fingertips at Xcm above the object
        gripper_goal.pose.position.z = (obj_top_z
                                        + self.GRIPPER_FINGERTIP_OFFSET
                                        + self.PRE_GRASP_DIST)
        if not self._arm_move_to_pose_attempt(gripper_goal, "pre grasp"):
            # if we can't get to the pre grasp pose, move on
            return False # failed, don't need rescan

        # 2. Align gripper with object's shortest dimension
        if not self._align_gripper_to_shortest_dim(gripper_goal, obj_posestamped, obj_dim):
            rospy.logwarn("Cannot align gripper to shortest dimension :(")
            return False # failed, don't need rescan

        # 2.5 Remove object as collision obstacle right before grasping
        # TODO add it back if didnt pick up
        self._planning_scene.removeCollisionObject("collision_object_{}".format(i), wait=True)

        # 3. Move down to grasp pose:
        # max-z(gripper base X cm above the object, fingertip Ycm above table)
        gripper_goal.pose.position.z = max(obj_top_z + self.GRIPPER_BASE_OFFSET + self.GRASP_DIST,
                                           obj_bottom_z + self.GRIPPER_FINGERTIP_OFFSET + self.TABLE_DIST)
        if not self._arm_move_to_pose_attempt(gripper_goal, "grasp"):
            # if we can't get to grasp pose, move on
            return False # rescan. might touch object

        grasp_pose = copy.deepcopy(gripper_goal.pose)

        # 4. Grip object and attach as obstacle
        self._gripper.close(max_effort=self.GRIPPER_EFFORT)
        self._attach_object_obstacle(obj_dim)

        # 5. Move up to post-grasp pose: Xcm back up
        gripper_goal.pose.position.z += self.POST_GRASP_DIST
        if not self._arm_move_to_pose_attempt(gripper_goal, "post grasp"):
            # if we cant get to post grasp, drop the object and move on
            self._drop_object()
            return False # rescan. might touch object

        # 5.25 Check if it's actually holding the object. 
        gripper_vals = self._reader.get_joints(['l_gripper_finger_joint', 'r_gripper_finger_joint'])
        if all([i < 0.01 for i in gripper_vals]):
            # Gripper is completely closed. It's not actually holding anything
            return False # rescan. might touch object

        # 5.5 Move to pre-bin pose
        pre_bin_pose = self.PRE_BIN_POSES[category]
        gripper_goal.pose.position.x = pre_bin_pose['x']
        gripper_goal.pose.position.y = pre_bin_pose['y']
        gripper_goal.pose.position.z = pre_bin_pose['z']
        if not self._arm_move_to_pose_attempt(gripper_goal, "pre bin"):
            self._put_object_down(gripper_goal, grasp_pose, "pre bin")
            return False # rescan. might touch object

        # 6. Move to bin dropoff pose: fingertip obj-height above bin pose
        bin_pose = self.BIN_POSES[category]
        gripper_goal.pose.position.x = bin_pose['x'] + self.DROPOFF_X_PADDING
        gripper_goal.pose.position.y = bin_pose['y']
        gripper_goal.pose.position.z = (bin_pose['z']
                                        + self.GRIPPER_FINGERTIP_OFFSET
                                        + obj_dim.z
                                        + self.DROPOFF_Z_PADDING)
        if not self._arm_move_to_pose_attempt(gripper_goal, "bin"):
            # if we can't move to the bin, try to set the obj back down
            self._put_object_down(gripper_goal, grasp_pose, "bin")
            return False # rescan. might touch object

        # 7. Drop object and detach obstacle
        self._drop_object()

        # DEBUG
        # rospy.loginfo("obj_posestamped: {}".format(obj_posestamped))
        # rospy.loginfo("obj_dim: {}".format(obj_dim))

        return True


    def _put_object_down(self, gripper_goal, grasp_pose, label):
        # if we can't move to the bin, try to set the obj back down
        gripper_goal.pose = grasp_pose
        gripper_goal.pose.position.z += 0.01 # just to leave some room for collision
        if not self._arm_move_to_pose_attempt(gripper_goal, label + " fail, dropoff"):
            rospy.logerr("Could not put object back down, going to drop it anyway! :O")

        self._drop_object()

    def _drop_object(self):
        self._gripper.open()
        self._detach_object_obstacle()


    # return True if succeeded, False otherwise
    def _arm_move_to_pose_attempt(self, goal, action_label):
        succeeded = False
        for i in range(self.NUM_ARM_ATTEMPTS):
            r = self._arm.move_to_pose(goal)
            rospy.loginfo(action_label + " try{}: {}".format(i, r))
            if r is None:
                succeeded = True
                break
        # give robot's arm time to settle down
        rospy.sleep(1.5)

        self._move_summary.append({'label': action_label, 'num_tried': i+1, 'result': r})
        return succeeded


    def _attach_object_obstacle(self, obj_dim):
        # TODO figure out the correct orientation
        rospy.loginfo("ATTACH")
        name = 'attached_object_{}'.format(self._attached_i)

        frame_attached_to = 'gripper_link'
        frames_okay_to_collide_with = ['gripper_link', 'l_gripper_finger_link', 'r_gripper_finger_link']

        # ASSUMPTION: gripper should always be aligned with the shortest dimension at this point
        attach_dim = Vector3()
        attach_dim.x = obj_dim.z + 0.005 # gripper's x axis points outwards, down in this case
        attach_dim.y = min(obj_dim.x, obj_dim.y) # gripper's y axis connects the fingers
        attach_dim.z = max(obj_dim.x, obj_dim.y) # gripper's z axis is perpendicular to the fingers

        self._planning_scene.attachBox(name,
                                       attach_dim.x,
                                       attach_dim.y,
                                       attach_dim.z,
                                       obj_dim.z/2.0, # X pos
                                       0, 0, # y pos, z pos
                                       frame_attached_to,
                                       touch_links=frames_okay_to_collide_with,
                                       wait=True)


    def _detach_object_obstacle(self):
        rospy.loginfo("DETACH")
        name = 'attached_object_{}'.format(self._attached_i)
        self._planning_scene.removeAttachedObject(name, wait=True)
        self._planning_scene.removeCollisionObject(name, wait=True)
        self._attached_i += 1



    ################ FOR DEBUGGING ################
    def _print_move_summary(self):
        rospy.loginfo('Move Summary:')
        for entry in self._move_summary:
            rospy.loginfo(entry['label'] + ": num_tried={}, res={}".format(entry['num_tried'], entry['result']))
        self._move_summary = []

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

        rospy.loginfo(marker)

        self._marker_pub.publish(marker)

    def _pub_bin_markers(self):
        marker = Marker()
        i = 100
        for category in self.BIN_POSES:
            marker.ns = category
            marker.id = i
            marker.header.frame_id = 'base_link'
            marker.type = Marker.CUBE
            marker.pose = Pose()
            marker.pose.position.x = self.BIN_POSES[category]['x']
            marker.pose.position.y = self.BIN_POSES[category]['y']
            marker.pose.position.z = self.BIN_POSES[category]['z'] - 0.153/2.0
            marker.pose.orientation.w = 1
            marker.scale.x = 0.153
            marker.scale.y = 0.153
            marker.scale.z = 0.153
            marker.color.r = 1
            marker.color.a = 0.5
            self._marker_pub.publish(marker)
            i += 1
