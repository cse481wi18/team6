#! /usr/bin/env python

import fetch_api
import rospy


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('arm_demo')
    wait_for_time()
    argv = rospy.myargv()
    DISCO_POSES = [[0, 0, 0, 0, 0, 0, 1.5]]

    torso = fetch_api.Torso()
    torso.set_height(fetch_api.Torso.MAX_HEIGHT)

    arm = fetch_api.Arm()
    for vals in DISCO_POSES:
        arm.move_to_joints(fetch_api.ArmJoints.from_list(vals))


if __name__ == '__main__':
    main()
