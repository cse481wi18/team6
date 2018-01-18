#!/usr/bin/env python

import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, PointHeadAction, PointHeadGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import math
import rospy

LOOK_AT_ACTION_NAME = 'head_controller/point_head' # name of the look-at action
PAN_TILT_ACTION_NAME = 'head_controller/follow_joint_trajectory' # name of the pan/tilt action
PAN_JOINT = 'head_pan_joint' # name of the head pan joint
TILT_JOINT = 'head_tilt_joint' # name of the head tilt joint
PAN_TILT_TIME = 2.5 # how many seconds it should take to move the head


class Head(object):
    """Head controls the Fetch's head.

    It provides two interfaces:
        head.look_at(frame_id, x, y, z)
        head.pan_tilt(pan, tilt) # In radians

    For example:
        head = fetch_api.Head()
        head.look_at('base_link', 1, 0, 0.3)
        head.pan_tilt(0, math.pi/4)
    """

    # Pan: positive value is clockwise
    MIN_PAN = -math.pi / 2 # Minimum pan angle, in radians.
    MAX_PAN = math.pi / 2 # Maximum pan angle, in radians.
    # Tilt: positive value is downwards
    MIN_TILT = -math.pi / 4 # Minimum tilt angle, in radians.
    MAX_TILT = math.pi / 2 #  Maximum tilt angle, in radians.

    def __init__(self):
        # Create actionlib clients
        self._look_at_client = actionlib.SimpleActionClient(LOOK_AT_ACTION_NAME,
                                                            PointHeadAction)
        self._pan_tilt_client = actionlib.SimpleActionClient(PAN_TILT_ACTION_NAME,
                                                            FollowJointTrajectoryAction)
        # Wait for both servers
        self._look_at_client.wait_for_server()
        self._pan_tilt_client.wait_for_server()

    def look_at(self, frame_id, x, y, z):
        """Moves the head to look at a point in space.

        Args:
            frame_id: The name of the frame in which x, y, and z are specified.
            x: The x value of the point to look at.
            y: The y value of the point to look at.
            z: The z value of the point to look at.
        """
        # Create goal
        goal = PointHeadGoal()
        # Fill out the goal (we recommend setting min_duration to 1 second)
        goal.target.header.frame_id = frame_id
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.min_duration = rospy.Duration.from_sec(1)
        # Send the goal
        self._look_at_client.send_goal(goal)
        # Wait for result
        self._look_at_client.wait_for_result()

    def pan_tilt(self, pan, tilt):
        """Moves the head by setting pan/tilt angles.

        Args:
            pan: The pan angle, in radians. A positive value is clockwise.
            tilt: The tilt angle, in radians. A positive value is downwards.
        """
        # Check that the pan/tilt angles are within joint limits
        if pan < self.MIN_PAN or pan > self.MAX_PAN or  \
           tilt < self.MIN_TILT or tilt > self.MAX_TILT:
            rospy.loginfo('Head: pan or tilt out of range.')
            return

        # Create a trajectory point
        point = JointTrajectoryPoint()
        # Set positions of the two joints in the trajectory point
        point.positions.append(pan) # index 0
        point.positions.append(tilt) # index 1
        # Set time of the trajectory point
        point.time_from_start = rospy.Duration(PAN_TILT_TIME)

        # Create goal
        goal = FollowJointTrajectoryGoal()
        # Add joint names to the list
        goal.trajectory.joint_names.append(PAN_JOINT) # index 0
        goal.trajectory.joint_names.append(TILT_JOINT) # index 1
        # Add trajectory point created above to trajectory
        goal.trajectory.points.append(point)

        # Send the goal
        self._pan_tilt_client.send_goal(goal)
        # Wait for result
        self._pan_tilt_client.wait_for_result(rospy.Duration.from_sec(5.0))

