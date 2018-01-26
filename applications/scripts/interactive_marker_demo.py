#!/usr/bin/env python

import rospy
import fetch_api
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker

def make_marker(server, x, y, z, marker_name):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "odom"
    int_marker.name = marker_name
    int_marker.description = "Simple Click Control"
    int_marker.pose.position.x = x
    int_marker.pose.position.y = y
    int_marker.pose.position.z = z
    int_marker.pose.orientation.w = 1

    box_marker = Marker()
    box_marker.type = Marker.CUBE
    box_marker.pose.orientation.w = 1
    box_marker.scale.x = 0.45
    box_marker.scale.y = 0.45
    box_marker.scale.z = 0.45
    box_marker.color.r = 0.0
    box_marker.color.g = 0.5
    box_marker.color.b = 0.5
    box_marker.color.a = 1.0

    button_control = InteractiveMarkerControl()
    button_control.interaction_mode = InteractiveMarkerControl.BUTTON
    button_control.always_visible = True
    button_control.markers.append(box_marker)
    int_marker.controls.append(button_control)

    server.insert(int_marker, handle_viz_input)


def main():
    rospy.init_node('interactive_marker_node')
    wait_for_time()

    server = InteractiveMarkerServer("simple_marker")

    make_marker(server, 3, 1, 0, "my_marker1")
    make_marker(server, 4.4, -2.8, "my_marker2")
    make_marker(server, 1, 0, 0, "my_marker3")
    server.applyChanges()
    rospy.spin()


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def handle_viz_input(input):
    if (input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK):
        rospy.loginfo('{} was clicked at position ({}, {}, {})'.format(input.marker_name, input.pose.position.x, input.pose.position.y, input.pose.position.z))
        base = fetch_api.Base()
        base.goto_in_odom(input.pose.position)
    else:
        rospy.loginfo('Cannot handle this InteractiveMarker event')

if __name__ == '__main__':
    main()
