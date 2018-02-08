#! /usr/bin/env python

import fetch_api
import rospy

from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped
from util import get_markers, wait_for_time


def main():
    rospy.init_node('hallucination_demo_node')
    wait_for_time()

    start = PoseStamped()
    start.header.frame_id = 'base_link'
    start.pose.position.x = 0.5
    start.pose.position.y = 0.5
    start.pose.position.z = 0.75
    arm = fetch_api.Arm()
    arm.move_to_pose(start)
                                                                               
    markers = get_markers()

    for marker in markers:
        # TODO: get the pose to move to
        error = arm.move_to_pose(marker.pose)
        if error is None:
            rospy.loginfo('Moved to marker {}'.format(marker.id))
            #return
        else:
            rospy.logwarn('Failed to move to marker {}'.format(marker.id))
    rospy.logerr('Failed to move to any markers!')


if __name__ == '__main__':
    main()