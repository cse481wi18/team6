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
        return 'ArmRelBaseAction({})'.format(self.pose)

class ArmRelTagAction:
    def __init__(self, tagid, transform):
        self.tagid = tagid
        self.transform = transform

    def __repr__(self):
        return 'ArmRelTagAction({})'.format(self.tagid)


def execute_program(name, markers):
    actions = pickle.load(open('arm_programs/{}.pickle'.format(name), 'rb'))
    arm = fetch_api.Arm()
    gripper = fetch_api.Gripper()

    arm.relax(False)

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
            print(arm.move_to_pose(pose))


def find_marker(markers, mid):
    for marker in markers:
        if marker.id == mid:
            return marker
    return None

def help():
    pass

def build_program(markers):
    arm = fetch_api.Arm()
    gripper = fetch_api.Gripper()
    listener = tf.TransformListener()

    #start = PoseStamped()
    #start.header.frame_id = 'base_link'
    #start.pose.position.x = 0.5
    #start.pose.position.y = 0.5
    #start.pose.position.z = 0.75
    #arm.move_to_pose(start)
                
    program_name = raw_input('Program Name: ')
    arm.relax(True) # TODO uncomment for real robot

    actions = []
    while True:
        command = raw_input('Command: ')
        if command == 'save pose':
            # save the pose to program
            # saving offset in obj frame
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
        elif command == 'save program':
            # save program to file
            pickle.dump(actions, open('arm_programs/{}.pickle'.format(program_name), 'wb'))
        elif command == 'open gripper':
            # save open gripper command and do 
            gripper.open()
            actions.append(GripperAction(True))
        elif command == 'close gripper':
            # save close gripper command and do
            gripper.close()
            actions.append(GripperAction(False))
        elif command == 'exit':
            break
        elif command == 'help':
            help()
        elif command == 'relax':
            arm.relax(True)
        elif command == 'unrelax':
            arm.relax(False)
        else:
            print 'Unknown command'
            help()
        print(command)

def main():
    rospy.init_node('program_arm_node')
    wait_for_time()

    markers = get_markers()
    if len(sys.argv) > 1:
        execute_program(sys.argv[1], markers)
    else: 
        build_program(markers)

   




if __name__ == '__main__':
    main()