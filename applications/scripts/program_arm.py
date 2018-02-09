#!/usr/bin/env python

import fetch_api
import numpy as np
import pickle
import rospy
import sys
import tf

from geometry_msgs.msg import PoseStamped
from util import get_markers, wait_for_time, pose_to_matrix, trans_rot_to_matrix, transform_to_pose

class GripperAction:
    def __init__(self, open_gripper):
        self.open = open_gripper

    def __repr__(self):
        return 'GripperAction({})'.format(self.open)

class ArmRelBaseAction:
    def __init__(self, pose):
        self.pose = pose

    def __repr__(self):
        # return 'ArmRelBaseAction({})'.format(self.pose)
        return 'ArmRelBaseAction()'

class ArmRelTagAction:
    def __init__(self, tagid, transform):
        self.tagid = tagid
        self.transform = transform

    def __repr__(self):
        return 'ArmRelTagAction({})'.format(self.tagid)


def execute_program(name, markers, arm, gripper):
    actions = pickle.load(open('arm_programs/{}.pickle'.format(name), 'rb'))

    for action in actions:
        print(action)
        if isinstance(action, GripperAction):
            if action.open:
                gripper.open()
            else:
                gripper.close()
        elif isinstance(action, ArmRelBaseAction):
            arm.move_to_pose(action.pose)
        else: # Arm action relative to tag
            marker = find_marker(markers, action.tagid)
            base_marker_transform = pose_to_matrix(marker.pose)
            base_wrist_transform = base_marker_transform.dot(action.transform)
            pose = transform_to_pose(base_wrist_transform)
            error = arm.move_to_pose(pose)
            if error is None:
                print("moved successfully")
            else:
                print(error)


def find_marker(markers, mid):
    for marker in markers:
        if marker.id == mid:
            return marker
    return None


def record_actions(is_sim, arm, gripper):
    listener = tf.TransformListener()
    actions = []
    if not is_sim:
        arm.relax(True)
        print("arm relaxed")

    markers = get_markers()
    print("start entering actions")

    while True:
        action = raw_input('> action: ')
        if action == 'pose':
            # saves current arm pose
            # prompts more
            frame = raw_input('Should the pose be relative to the base or a tag? ')
            if frame == 'base':
                base_wrist_transform = trans_rot_to_matrix(*listener.lookupTransform('base_link', 'wrist_roll_link', rospy.Time(0)))
                pose = transform_to_pose(base_wrist_transform)
                actions.append(ArmRelBaseAction(pose))
                
            elif frame == 'tag':
                tag = int(raw_input('Which tag {}? '.format([marker.id for marker in markers])))
                # save rel to tag
                base_wrist_transform = trans_rot_to_matrix(*listener.lookupTransform('base_link', 'wrist_roll_link', rospy.Time(0)))
                marker = find_marker(markers, tag)
                base_marker_transform = pose_to_matrix(marker.pose)

                actions.append(ArmRelTagAction(tag, np.linalg.inv(base_marker_transform).dot(base_wrist_transform))) # TODO: lol
        
            print("saved pose")

        elif action == 'open':
            gripper.open()
            actions.append(GripperAction(True))
            print("saved open action")

        elif action == 'close':
            gripper.close()
            actions.append(GripperAction(False))
            print("saved close action")

        elif action == 'save':
            name = raw_input('> Program name: ')
            # save program
            pickle.dump(actions, open('arm_programs/{}.pickle'.format(name), 'wb'))
            print("saved recorded actions under \'{}\'".format(name))
            actions = []
            break
        elif action == 'cancel':
            # reset actions list
            actions = []
            print("canceled recording")
            break
        else:
            print("Unknown action!")
            print_actions()

    if not is_sim:
        arm.relax(False)
        print("arm unrelaxed")


def main():
    print("waiting for time..")
    rospy.init_node('program_arm_node')
    wait_for_time()
    is_sim = rospy.get_param('use_sim_time', False)

    print_cmds()

    arm = fetch_api.Arm()
    gripper = fetch_api.Gripper()

    recording = False
    while True:
        cmd = raw_input('> ')
        cmd = cmd.split()
        if len(cmd) == 0:
            continue

        if cmd[0] == 'record':
            # start saving action inputs
            record_actions(is_sim, arm, gripper)

        elif cmd[0] == 'run':
            name = cmd[1]
            markers = get_markers()
            print("running {}...".format(name))
            execute_program(name, markers, arm, gripper)

        elif cmd[0] == 'exit':
            print("Bye!")
            break
        elif cmd[0] == 'help':
            print_cmds()
        else:
            print 'Unknown command!'
            print_cmds()


def print_cmds():
    print("Commands:")
    print("\t[record]: updates current tags and starts recording actions")
    print_actions()
    print("\t[run <program_name>]: updates current tags and runs the given program")
    print("\t[exit]: exits")
    print("\t[help]: show this list")

def print_actions():
    print("\t\tactions:")
    print("\t\t[pose]: saves current arm's pose")
    print("\t\t[open]: opens gripper")
    print("\t\t[close]: closes gripper")
    print("\t\t[save]: end recording and be prompted to save the program")
    print("\t\t[cancel]: cancels recording")

if __name__ == '__main__':
    main()