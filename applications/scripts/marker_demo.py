#!/usr/bin/env python

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from nav_msgs.msg import Odometry

import fetch_api
import rospy

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def show_text_in_rviz(marker_publisher, text):
    marker = Marker(
                type=Marker.TEXT_VIEW_FACING,
                id=0,
                #lifetime=rospy.Duration(3.5),
                pose=Pose(Point(1.5, 0.5, 1.45), Quaternion(0, 0, 0, 1)),
                scale=Vector3(0.5, 0.5, 0.5),
                header=Header(frame_id='base_link'),
                color=ColorRGBA(1.0, 0.0, 0.0, 0.8),
                text=text)
    marker_publisher.publish(marker)

def main():
    rospy.init_node('marker_node')
    wait_for_time()

    #marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=5)
    #rospy.sleep(1)
    #show_text_in_rviz(marker_publisher, "LOOK AT ME!")

    nav_path = fetch_api.NavPath()
    rospy.Subscriber('odom', Odometry, nav_path.callback)
    rospy.spin()

if __name__ == '__main__':
    main()
