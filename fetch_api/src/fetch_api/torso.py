#!/usr/bin/env python

import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import rospy

ACTION_NAME = 'torso_controller/follow_joint_trajectory'
JOINT_NAME = 'torso_lift_joint'
TIME_FROM_START = 5  # How many seconds it should take to set the torso height.


class Torso(object):
    """Torso controls the robot's torso height.
    """
    MIN_HEIGHT = 0.0
    MAX_HEIGHT = 0.4

    def __init__(self):
        # Create actionlib client
        self._client = actionlib.SimpleActionClient(ACTION_NAME, FollowJointTrajectoryAction)
        # Wait for server
        self._client.wait_for_server()

    def set_height(self, height):
        """Sets the torso height.

        This will always take ~5 seconds to execute.

        Args:
            height: The height, in meters, to set the torso to. Values range
                from Torso.MIN_HEIGHT (0.0) to Torso.MAX_HEIGHT(0.4).
        """
        # Check that the height is between MIN_HEIGHT and MAX_HEIGHT.
        if height > self.MAX_HEIGHT or height < self.MIN_HEIGHT:
            rospy.loginfo('Height outside of range')
            return

        # Create a trajectory point
        point = JointTrajectoryPoint()
        # Set position of trajectory point
        point.positions.append(height)
        # Set time of trajectory point
        point.time_from_start = rospy.Duration(TIME_FROM_START)
        # Create goal
        goal = FollowJointTrajectoryGoal()
        # Add joint name to list
        goal.trajectory.joint_names.append(JOINT_NAME)
        # Add the trajectory point created above to trajectory
        goal.trajectory.points.append(point)
        # Send goal
        self._client.send_goal(goal)
        # Wait for result
        self._client.wait_for_result(rospy.Duration.from_sec(5.0))

