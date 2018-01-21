#!/usr/bin/env python

import fetch_api
import rospy
from web_teleop.srv import SetTorso, SetTorsoResponse,\
        MoveHead, MoveHeadResponse,\
        MoveArm, MoveArmResponse,\
        SetGrip, SetGripResponse
from joint_state_reader import JointStateReader


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class ActuatorServer(object):
    def __init__(self):
        self._torso = fetch_api.Torso()
        self._head = fetch_api.Head()
        self._grip = fetch_api.Gripper()
        self._arm = fetch_api.Arm()
        self._reader = JointStateReader()

    def handle_set_torso(self, request):
        self._torso.set_height(request.height)
        return SetTorsoResponse()

    def handle_move_head(self, move_head_request):
        if (move_head_request.focus_hand):
            self._head.look_at('wrist_flex_link', 0, 0, 0)
        else:
            curr_pan = self._reader.get_joint('head_pan_joint')
            curr_tilt = self._reader.get_joint('head_tilt_joint')
            self._head.pan_tilt(curr_pan + move_head_request.delta_pan,
                    curr_tilt + move_head_request.delta_tilt)
        return MoveHeadResponse()

    def set_grip(self, set_grip_request):
        if not set_grip_request.grip:
            self._grip.open()
        else:
            self._grip.close(set_grip_request.effort)
        return SetGripResponse()


    def handle_move_arm(self, move_arm_request):
        rospy.loginfo(move_arm_request)
        self._arm.move_to_joints(fetch_api.ArmJoints.from_list(move_arm_request.positions))
        return MoveArmResponse()


def main():
    rospy.init_node('web_teleop_actuators')
    wait_for_time()
    server = ActuatorServer()
    torso_service = rospy.Service('web_teleop/set_torso', SetTorso,
                                  server.handle_set_torso)
    head_service = rospy.Service('web_teleop/move_head', MoveHead,
                                  server.handle_move_head)
    arm_service = rospy.Service('web_teleop/move_arm', MoveArm,
                                 server.handle_move_arm)
    gripper_service = rospy.Service('web_teleop/set_grip', SetGrip,
                                     server.set_grip)
    rospy.spin()


if __name__ == '__main__':
    main()
