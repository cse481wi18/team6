#!/usr/bin/env python
import copy
import math
import numpy as np

import actionlib
import fetch_api
import rospy
import utils
from moveit_python import PlanningSceneInterface

from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Vector3
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import OrientationConstraint
from recycle_msgs.msg import ActionPose
from recycle_msgs.msg import ClassifyAction, ClassifyActionGoal
from visualization_msgs.msg import Marker

MOVE_BASE_ACTION = 'move_base'

class Controller(object):

    # head.look_at(frame_id, x, y, z)
    LOOK_AT_TABLE = ('base_link', 0.7, 0.0, 0.75)

    GRIPPER_WIDTH = 0.10
    GRIPPER_BASE_OFFSET = 0.166 - 0.03  # offset between wrist_roll_link and the base of the gripper
    GRIPPER_FINGERTIP_OFFSET = 0.166 + 0.03   # offset between wrist_roll_link and the END of the fingertips

    PRE_GRASP_DIST = 0.03  # fingertips 5cm above the object
    GRASP_DIST = 0.02  # gripper base 2cm above the object
    TABLE_DIST = 0.005  # fingertip 0.5cm above the table
    POST_GRASP_DIST = 0.05 # move 5cm back upwards after grasping

    NUM_ARM_ATTEMPTS = 3

    CRANE_ORIENTATION = Quaternion(0.00878962595016,
                                   0.711250066757,
                                   -0.00930198933929,
                                   0.702822685242)
    Y_ALIGN_ORIENTATION = Quaternion(0.509480476379,
                                     0.49084764719,
                                     -0.515461504459,
                                     0.483526408672)

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

    def __init__(self, move_request_topic, classify_action):
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


    def _attach_bin_obstacles(self):
        for category in self.BIN_POSES:
            self._planning_scene.removeAttachedObject(category + "_bin")

        frame_attached_to = 'base_link'
        frames_okay_to_collide_with = ['base_link', 'laser_link']

        for category in self.BIN_POSES:
            rospy.loginfo('add bin obstacle: {}'.format(category))
            name = category + "_bin"
            self._planning_scene.attachBox(name,
                                           self.BIN_WIDTH, self.BIN_WIDTH, self.BIN_WIDTH,
                                           self.BIN_POSES[category]['x'],
                                           self.BIN_POSES[category]['y'],
                                           self.BIN_POSES[category]['z'] - self.BIN_WIDTH/2.0,
                                           frame_attached_to,
                                           frames_okay_to_collide_with)


    def _move_request_cb(self, msg):
        # add goal pose to queue
        self._request_queue.append(msg)
        rospy.loginfo("New move request queued. Queue size: {}.".format(len(self._request_queue)))


    def shutdown(self):
        self._arm.cancel_all_goals()

        for category in self.BIN_POSES:
            self._planning_scene.removeAttachedObject(category + "_bin")


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
            # TODO : Test with sleeping
            # rospy.sleep(3)
            rospy.loginfo("Arrived at target. Performing \'{}\' action...".format(request.action))

            # perform action once at target pose
            if request.action == "bus":
                # TODO can't move head for some reason..
                # "Unable to stop head_controller/point_head when trying to start head_controller/follow_joint_trajectory"
                #self._head.pan_tilt(0, self.LOOK_DOWN_ANGLE)  # look down
                self._head.look_at(*self.LOOK_AT_TABLE)
                classifier_result = self._classify_pointcloud()

                # add obstacles to PlanningScene for the arm
                self._add_obstacles(classifier_result)

                # now, bus the classified objects
                self._bus_objects(classifier_result)

            elif request.action == "rest":
                pass

            else:
                rospy.logerr("Unknown action!")

            rospy.loginfo("Request completed.")


    def _classify_pointcloud(self):
        self._classify_client.send_goal(ClassifyActionGoal())
        self._classify_client.wait_for_result()
        classifier_result = self._classify_client.get_result()

        rospy.loginfo("Got results from classifier:")
        rospy.loginfo(classifier_result)

        return classifier_result


    def _add_obstacles(self, classifier_result):
        rospy.loginfo("ADD TABLE")
        # Clearing all the previous obstacles in the planning scene
        for i in range(self._num_prev_obstacles):
            name = 'obstacle_{}'.format(i)
            self._planning_scene.removeCollisionObject(name, wait=True)

        self._num_prev_obstacles = classifier_result.num_obstacles

        for i in range(classifier_result.num_obstacles):
            obstacle_pose = classifier_result.obstacle_poses[i]
            obstacle_dim = classifier_result.obstacle_dimensions[i]

            rospy.loginfo(obstacle_pose)
            rospy.loginfo(obstacle_dim)

            flip_obstacles = rospy.has_param('flip_obstacles') and rospy.get_param('flip_obstacles')
            rospy.logerr(flip_obstacles)
            if flip_obstacles:
                obstacle_dim.x *= 2
                self._planning_scene.addBox('obstalce_' + str(i),
                                        obstacle_dim.y,  # TODO hacky
                                        obstacle_dim.x,
                                        obstacle_dim.z,
                                        obstacle_pose.pose.position.x, # TODO
                                        obstacle_pose.pose.position.y,
                                        obstacle_dim.z / 2.0, # Hacky fix
                                        wait=True)
            else: # Don't flip. Double y dimension
                obstacle_dim.y *= 2
                self._planning_scene.addBox('obstalce_' + str(i),
                                        obstacle_dim.x,
                                        obstacle_dim.y,  # TODO hacky
                                        obstacle_dim.z,
                                        obstacle_pose.pose.position.x, # TODO
                                        obstacle_pose.pose.position.y,
                                        obstacle_dim.z / 2.0, # Hacky fix
                                        wait=True)

            # # TODO:
            # # Note for Ariel:
            # # Perception is going to always return the dimensions in the true order,
            # # i.e. x is always x and y is always y.
            # # In simulation, flipping the dimensions here results in the correct behaviour.
            # # I will test it on the real robot tomorrow morning to see if it has the same
            # # behaviour irl as well.

            # # obstacle_dim.y *= 2
            # obstacle_dim.x *= 2 # Doubling the x dimension here since we're flipping the dimensions
            # # print("TABLE LOCATION")
            # # print([obstacle_dim.x, obstacle_dim.y, obstacle_dim.z,
            # #     obstacle_pose.pose.position.x, obstacle_pose.pose.position.y, obstacle_pose.pose.position.z])

            # # TODO: The parameters might need to be changed when we stop using the mock point cloud.
            # # TODO: had to flip the x and y for some reason
            # self._planning_scene.addBox('obstalce_' + str(i),
            #                             obstacle_dim.y,  # TODO hacky
            #                             obstacle_dim.x,
            #                             obstacle_dim.z,
            #                             obstacle_pose.pose.position.x, # TODO
            #                             obstacle_pose.pose.position.y,
            #                             obstacle_dim.z / 2.0, # Hacky fix
            #                             wait=True)

        rospy.loginfo(self._planning_scene.getKnownCollisionObjects())


    def _bus_objects(self, classifier_result):
        rospy.loginfo("Start bussing objects...")


        for i in range(classifier_result.num_objects):
            obj_posestamped = classifier_result.object_poses[i]
            obj_dim = classifier_result.object_dimensions[i]
            category = classifier_result.classifications[i]
            bin_pose = self.BIN_POSES[category]

            # DEBUG
            # self._pub_bin_markers()
            self._pub_object_marker(i, category, obj_posestamped.pose, obj_dim)

            pickedup = self._pickup_object(obj_posestamped, obj_dim, bin_pose)
            if pickedup:
                rospy.loginfo("Picked up object {}: {}".format(i, category))
            else:
                rospy.logwarn("Failed to pick up object {}, moving on".format(i))
            self._print_move_summary()


        # TODO put arm back to 'home position'
        # TODO remove table obstacle


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
    def _pickup_object(self, obj_posestamped, obj_dim, bin_pose):
        gripper_goal = copy.deepcopy(obj_posestamped)
        gripper_goal.pose.orientation = self.CRANE_ORIENTATION
        obj_top_z = obj_posestamped.pose.position.z + obj_dim.z / 2.0
        obj_bottom_z = obj_posestamped.pose.position.z - obj_dim.z / 2.0

        # make sure gripper is initially open
        self._gripper.open()

        # 1. Move to pre-grasp pose: fingertips at Xcm above the object
        gripper_goal.pose.position.z = (obj_top_z
                                        + self.GRIPPER_FINGERTIP_OFFSET
                                        + self.PRE_GRASP_DIST)
        if not self._arm_move_to_pose_attempt(gripper_goal, "pre grasp"):
            # if we can't get to the pre grasp pose, move on
            return False

        # 2. Align gripper with object's shortest dimension
        if not self._align_gripper_to_shortest_dim(gripper_goal, obj_posestamped, obj_dim):
            rospy.logwarn("Cannot align gripper to shortest dimension :(")
            return False

        # 3. Move down to grasp pose:
        # max-z(gripper base X cm above the object, fingertip Ycm above table)
        gripper_goal.pose.position.z = max(obj_top_z + self.GRIPPER_BASE_OFFSET + self.GRASP_DIST,
                                           obj_bottom_z + self.GRIPPER_FINGERTIP_OFFSET + self.TABLE_DIST)
        if not self._arm_move_to_pose_attempt(gripper_goal, "grasp"):
            # if we can't get to grasp pose, move on
            return False

        grasp_pose = copy.deepcopy(gripper_goal.pose)

        # 4. Grip object and attach as obstacle
        self._gripper.close()
        self._attach_object_obstacle(obj_dim)

        # 5. Move up to post-grasp pose: Xcm back up
        gripper_goal.pose.position.z += self.POST_GRASP_DIST
        if not self._arm_move_to_pose_attempt(gripper_goal, "post grasp"):
            # if we cant get to post grasp, drop the object and move on
            self._drop_object()
            return False

        # 6. Move to bin dropoff pose: fingertip obj-height above bin pose
        gripper_goal.pose.position.x = bin_pose['x'] + self.DROPOFF_X_PADDING
        gripper_goal.pose.position.y = bin_pose['y']
        gripper_goal.pose.position.z = (bin_pose['z']
                                        + self.GRIPPER_FINGERTIP_OFFSET
                                        + obj_dim.z
                                        + self.DROPOFF_Z_PADDING)
        if not self._arm_move_to_pose_attempt(gripper_goal, "bin"):
            # if we can't move to the bin, try to set the obj back down
            gripper_goal.pose = grasp_pose
            gripper_goal.pose.position.z += 0.01 # just to leave some room for collision
            if not self._arm_move_to_pose_attempt(gripper_goal, "bin fail, dropoff"):
                rospy.logerr("Could not put object back down, going to drop it anyway! :O")

            self._drop_object()
            return False

        # 7. Drop object and detach obstacle
        self._drop_object()

        # DEBUG
        # rospy.loginfo("obj_posestamped: {}".format(obj_posestamped))
        # rospy.loginfo("obj_dim: {}".format(obj_dim))

        return True

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
        name = 'object_{}'.format(self._attached_i)
        self._planning_scene.removeAttachedObject(name, wait=True)

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
        name = 'object_{}'.format(self._attached_i)
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
