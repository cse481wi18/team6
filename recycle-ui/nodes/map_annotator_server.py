#! /usr/bin/env python

import fetch_api
import rospy
import os
import pickle

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker
from geometry_msgs.msg import PoseStamped
from map_annotator.msg import UserAction, PoseNames
from recycle_msgs.msg import ActionPose

def wait_for_time():
    while rospy.Time().now().to_sec() == 0:
        pass

class MapPoses:
    POSES_FILE = '/home/team6/catkin_ws/src/cse481wi18/map_annotator/pickled_poses/web_poses.pickle'

    def __init__(self):
        if os.path.isfile(self.POSES_FILE):
            with open(self.POSES_FILE, 'rb') as f:
                self._poses = pickle.load(f)
        else:
            self._poses = {}    # {name: pose}

        self._front_sub = rospy.Subscriber('/map_annotator/user_actions',
                UserAction,
                callback=self._handle_user_action)
        self._marker_server = InteractiveMarkerServer('/map_annotator/map_poses')
        self._pose_name_publisher = rospy.Publisher('/map_annotator/pose_names', PoseNames, latch=True)
        self._goal_pub = rospy.Publisher('/recycle/move_request', ActionPose, queue_size=10)
        self._draw_markers()


    def _draw_markers(self):
        for k, v in self._poses.iteritems():
            self._marker_server.insert(v, self._handle_click)
        self._marker_server.applyChanges()
        self._publish_names()


    def _handle_user_action(self, action):
        command = action.command
        if command == 'create':
            self._create(action)
        elif command == 'delete':
            self._delete(action)
        elif command == 'goto':
            self._goto(action)
        elif command == 'update':
            self._marker_server.applyChanges()
            self._dump()


    def _goto(self, action):
        goal_msg = ActionPose()
        goal_msg.action = 'bus' # TODO (jkbach): will have to update this
        goal_msg.target_pose = PoseStamped()
        goal_msg.target_pose.pose = self._poses[action.name].pose
        goal_msg.target_pose.header.frame_id = 'map'
        self._goal_pub.publish(goal_msg)


    def _delete(self, action):
        rospy.logerr('DELETING!')
        self._marker_server.erase(action.name)
        del self._poses[action.name]
        self._publish_names()
        self._marker_server.applyChanges()
        self._dump()


    def _dump(self):
        with open(self.POSES_FILE, 'wb') as f:
            pickle.dump(self._poses, f)


    def _create(self, action):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.name = action.name
        int_marker.description = action.name
        int_marker.pose.position.x = 0
        int_marker.pose.position.y = 0
        int_marker.pose.position.z = 0
        int_marker.pose.orientation.w = 1

        twist_control = InteractiveMarkerControl()
        twist_control.orientation.w = 1
        twist_control.orientation.x = 0
        twist_control.orientation.y = 1
        twist_control.orientation.z = 0
        twist_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        twist_control.always_visible = True
        int_marker.controls.append(twist_control)

        arrow_marker = Marker()
        arrow_marker.type = Marker.ARROW
        arrow_marker.pose.orientation.w = 1
        arrow_marker.scale.x = .5
        arrow_marker.scale.y = 0.1
        arrow_marker.scale.z = 0.1
        arrow_marker.color.r = 0.0
        arrow_marker.color.g = 0.5
        arrow_marker.color.b = 0.5
        arrow_marker.color.a = 1.0

        button_control = InteractiveMarkerControl()
        button_control.orientation.w = 1
        button_control.orientation.x = 0
        button_control.orientation.y = 1
        button_control.orientation.z = 0
        button_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        button_control.always_visible = True
        button_control.markers.append(arrow_marker)
        int_marker.controls.append(button_control)

        self._marker_server.insert(int_marker, self._handle_click)
        self._marker_server.applyChanges()

        self._poses[action.name] = int_marker
        self._publish_names()
        self._dump()


    def _handle_click(self, event):
        self._poses[event.marker_name].pose = event.pose
        self._dump()
        print(self._poses[event.marker_name].pose)


    def _publish_names(self):
        pose_names = PoseNames()
        pose_names.names = list(self._poses.keys())
        self._pose_name_publisher.publish(pose_names)


def main():
    rospy.init_node('recycle_map_annotator')
    wait_for_time()
    map_pose = MapPoses()
    rospy.spin()

if __name__ == '__main__':
    main()
