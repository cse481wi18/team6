#! /usr/bin/env python

import os
import pickle

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import rospy

class MapAnnotator(object):

    POSES_FILE = '/home/team6/catkin_ws/src/cse481wi18/map_annotator/pickled_poses/poses.pickle'

    def __init__(self):
        # load poses file if one exists, otherwise create an empty dict
        if os.path.isfile(self.POSES_FILE):
            self._poses = pickle.load(open(self.POSES_FILE, 'rb'))
        else:
            self._poses = {}    # {name: pose}
        self._pose_sub = rospy.Subscriber('amcl_pose',
                                            PoseWithCovarianceStamped,
                                            callback=self._pose_cb)
        self._goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self._cur_pose = None

    def _pose_cb(self, msg):
        self._cur_pose = msg.pose.pose # type geometry_msgs/Pose

    def save_pose(self, name):
        if self._cur_pose is None:
            rospy.logerr("Cur pose is not set!!!")
            return False
        self._poses[name] = self._cur_pose
        pickle.dump(self._poses, open(self.POSES_FILE, 'wb'))
        return True


    def goto_pose_name(self, name):
        if name in self._poses:
            goal_pose = self._poses[name]
            # create msg and publish
            goal_msg = PoseStamped()
            goal_msg.pose = goal_pose
            goal_msg.header.frame_id = 'map'
            self._goal_pub.publish(goal_msg)
            return True

        return False

    def delete_pose(self, name):
        if name in self._poses:
            del self._poses[name]
            pickle.dump(self._poses, open(self.POSES_FILE, 'wb'))
            return True

        return False

    def list_poses(self):
        return self._poses.keys()
