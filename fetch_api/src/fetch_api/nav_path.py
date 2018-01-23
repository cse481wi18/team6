#!/usr/bin/env python

import math

from geometry_msgs.msg import Vector3
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker

import rospy

class NavPath(object):

    DIST_THRESH = 0.3

    def __init__(self):
        self._last_pose = None
        self._pub = rospy.Publisher('visualization_marker', Marker, queue_size=5)
        self._point_list = []

    def callback(self, msg):
        #rospy.loginfo(msg)

        cur_pose = msg.pose.pose
        if not self._last_pose:
            self._last_pose = cur_pose
        else:
            # check if it's moved far enough from last pose
            if self.dist(cur_pose.position) > self.DIST_THRESH:
                self._point_list.append(self._last_pose.position)
                marker = Marker(
                            type = Marker.SPHERE_LIST,
                            id = 0,
                            points = self._point_list,
                            scale = Vector3(0.06, 0.06, 0.06),
                            header = Header(frame_id = 'odom'),
                            color = ColorRGBA(0.0, 0.0, 1.0, 1.0))
                self._pub.publish(marker)
                self._last_pose = cur_pose

    def dist(self, cur_position):
        dx = self._last_pose.position.x - cur_position.x
        dy = self._last_pose.position.y - cur_position.y
        dz = self._last_pose.position.z - cur_position.z
        return math.sqrt(dx**2 + dy**2 + dz**2)
