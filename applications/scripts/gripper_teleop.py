#! /usr/bin/env python

import numpy as np
import rospy
from util import wait_for_time
import fetch_api
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import MenuEntry, InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import PoseStamped
from copy import deepcopy
import tf.transformations as tft

GRIPPER_MESH = 'package://fetch_description/meshes/gripper_link.dae'
L_FINGER_MESH = 'package://fetch_description/meshes/l_gripper_finger_link.STL'
R_FINGER_MESH = 'package://fetch_description/meshes/r_gripper_finger_link.STL'
CENTER_OFFSET = 0.166   # offset between wrist_roll_link and gripper_link

# Returns a list of controls for 6 degrees of freedom
# to be appended to the Controls of the Interactive marker
def make_6dof_controls():
    dof_controls = list()

    # Creating base control
    control = InteractiveMarkerControl()
    control.always_visible = True

    orientations = {'x':(1, 1, 0, 0), 'y':(1, 0, 0, 1), 'z':(1, 0, 1, 0)}

    for label, orien in orientations.iteritems():
        move_rotate = single_axis_control(control, label, *orien)
        dof_controls.extend(move_rotate)

    return dof_controls

def single_axis_control(control, axis_label, w, x, y, z):
    rotate, move = None, None

    control.orientation.w = w
    control.orientation.x = x
    control.orientation.y = y
    control.orientation.z = z

    # Modifying it to rotate about axis
    control.name = "rotate_" + axis_label
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    rotate = deepcopy(control)
    # Modifying it to move the axis
    control.name = "move_" + axis_label
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    move = deepcopy(control)

    return [move, rotate]

# Given a list of menu titles ['go to gripper', 'open gripper'],
# returns a list of menu items to be appended to the MenuEntries
# of the Interactive marker
def create_menu(items):
    menu_items = list()
    menu = MenuEntry()

    for i, item in enumerate(items, 1):
        menu.title = item
        menu.id = i
        menu_items.append(deepcopy(menu))

    return menu_items


def _create_gripper_im():
    gripper_im = InteractiveMarker()
    gripper_im.header.frame_id = 'base_link'
    gripper_im.name = 'gripper_im'
    gripper_im.description = 'gripper_im'
    gripper_im.pose.position.x = CENTER_OFFSET
    gripper_im.pose.orientation.w = 1
    gripper_im.scale = 0.4
    return gripper_im

def _create_gripper_markers(x, y, z):
    gripper = Marker()
    gripper.type = Marker.MESH_RESOURCE
    gripper.mesh_resource = GRIPPER_MESH
    gripper.pose.position.x = x + CENTER_OFFSET
    gripper.pose.position.y = y
    gripper.pose.position.z = z
    gripper.color.r = 0.9
    gripper.color.g = 0.9
    gripper.color.b = 0.9
    gripper.color.a = 1.0

    l_finger = Marker()
    l_finger.type = Marker.MESH_RESOURCE
    l_finger.mesh_resource = L_FINGER_MESH
    l_finger.pose.position.x = x + CENTER_OFFSET
    l_finger.pose.position.y = y - 0.06
    l_finger.pose.position.z = z
    l_finger.color.r = 0.3
    l_finger.color.g = 0.3
    l_finger.color.b = 0.3
    l_finger.color.a = 1.0

    r_finger = Marker()
    r_finger.type = Marker.MESH_RESOURCE
    r_finger.mesh_resource = R_FINGER_MESH
    r_finger.pose.position.x = x + CENTER_OFFSET
    r_finger.pose.position.y = y + 0.06
    r_finger.pose.position.z = z
    r_finger.color.r = 0.3
    r_finger.color.g = 0.3
    r_finger.color.b = 0.3
    r_finger.color.a = 1.0

    return [gripper, l_finger, r_finger]

# Assumes grippers are ordered by hand, finger, finger
#  and start_idx is index of the hand
def _change_colour(gripper_im, start_idx, col):
    # gripper_im.pose = pose_stamped.pose
    hand_colour = [0.9, 0.9, 0.9, 1]
    finger_colour = [0.3, 0.3, 0.3, 1]

    colours = {'green':[0, 1, 0, 1], 'red':[1, 0, 0, 1]}

    for i in range(len(hand_colour)):
        hand_colour[i] += colours[col][i]
        hand_colour[i] /= 2
        finger_colour[i] += colours[col][i]
        finger_colour[i] /= 2

    newHandColor = ColorRGBA(*tuple(hand_colour))
    newFingerColor = ColorRGBA(*tuple(finger_colour))

    gripper_im.controls[0].markers[start_idx].color = newHandColor
    gripper_im.controls[0].markers[start_idx + 1].color = newFingerColor
    gripper_im.controls[0].markers[start_idx + 2].color = newFingerColor

class GripperTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server
        self._gripper_im = self._create_gripper()

    def _create_gripper(self):
        gripper_im = _create_gripper_im()
        gripper_mesh = InteractiveMarkerControl()
        gripper_mesh.interaction_mode = InteractiveMarkerControl.MENU
        gripper_mesh.always_visible = True
        gripper_mesh.markers.extend(_create_gripper_markers(0, 0, 0))
        gripper_im.controls.append(gripper_mesh)
        gripper_im.controls.extend(make_6dof_controls())
        gripper_im.menu_entries.extend(create_menu(['Go to Gripper', 'Open Gripper', 'Close Gripper']))

        return gripper_im

    def start(self):
        print("Started")
        self._im_server.insert(self._gripper_im, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()

    def _open_gripper(self, isOpen):
        self._gripper_im.controls[0].markers[1].pose.position.y += 0.045 if isOpen else -0.045
        self._gripper_im.controls[0].markers[2].pose.position.y += -0.045 if isOpen else 0.045
        self._gripper.open() if isOpen else self._gripper.close()

    def handle_feedback(self, feedback):
        event_type = feedback.event_type
        pose_stamped = PoseStamped()
        pose_stamped.pose = feedback.pose
        pose_stamped.header = feedback.header
        if event_type == InteractiveMarkerFeedback.MENU_SELECT:
            print('MENU CLICK')
            if feedback.menu_entry_id == 1:
                self._arm.move_to_pose(pose_stamped)
            elif feedback.menu_entry_id == 2:
                self._open_gripper(True)
            elif feedback.menu_entry_id == 3:
                self._open_gripper(False)
        elif event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self._gripper_im.pose = pose_stamped.pose
            if self._arm.compute_ik(pose_stamped):
                rospy.loginfo('{}'.format(pose_stamped))
                _change_colour(self._gripper_im, 0, 'green')
            else:
                _change_colour(self._gripper_im, 0, 'red')

        self._im_server.insert(self._gripper_im, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()


class AutoPickTeleop(object):

    PRE_MARKER_INDEX = 3
    POST_MARKER_INDEX = 6

    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server
        self._grip_pose = (-CENTER_OFFSET, 0, 0)
        self._pre_pose = (-CENTER_OFFSET - 0.2, 0, 0)
        self._post_pose = (-CENTER_OFFSET, 0, 0.2)
        pose = PoseStamped()
        pose.pose.position.x = CENTER_OFFSET
        pose.pose.orientation.w = 1
        self._auto_pick_im = self._create_auto_pick()

        self._safe_to_move = False

    def _open_gripper(self, isOpen):
        self._auto_pick_im.controls[0].markers[1].pose.position.y += 0.045 if isOpen else -0.045
        self._auto_pick_im.controls[0].markers[2].pose.position.y += -0.045 if isOpen else 0.045
        self._gripper.open() if isOpen else self._gripper.close()

    def _create_auto_pick(self):
        gripper_im = _create_gripper_im()

        gripper_mesh = InteractiveMarkerControl()
        gripper_mesh.interaction_mode = InteractiveMarkerControl.MENU
        gripper_mesh.always_visible = True
        gripper_mesh.markers.extend(_create_gripper_markers(*self._grip_pose))
        gripper_mesh.markers.extend(_create_gripper_markers(*self._pre_pose))
        gripper_mesh.markers.extend(_create_gripper_markers(*self._post_pose))
        box = Marker()
        box.type = Marker.CUBE
        box.scale.x = .05
        box.scale.y = .05
        box.scale.z = .05
        box.color.r = 0.9
        box.color.g = 0.2
        box.color.b = 0.9
        box.color.a = 1.0
        

        gripper_im.controls.append(gripper_mesh)
        gripper_im.controls.extend(make_6dof_controls())
        gripper_mesh.markers.append(box)
        gripper_im.menu_entries.extend(create_menu(['Pick from front', 'Open Gripper']))

        return gripper_im

    def start(self):
        self._im_server.insert(self._auto_pick_im, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()
        print("Started")

    def handle_feedback(self, feedback):
        event_type = feedback.event_type
        pose_stamped = PoseStamped()
        pose_stamped.pose = feedback.pose
        pose_stamped.header = feedback.header

        if event_type == InteractiveMarkerFeedback.MENU_SELECT:
            print('MENU CLICK')
            if feedback.menu_entry_id == 1:
                if self._safe_to_move:
                    # open gripper first
                    self._open_gripper(True)

                    # to go pre pose
                    pose_stamped.pose = self._translate_pose(feedback.pose, self._pre_pose)
                    self._arm.move_to_pose(pose_stamped)
                    # go to middle pose
                    pose_stamped.pose = self._translate_pose(feedback.pose, self._grip_pose)
                    self._arm.move_to_pose(pose_stamped)
                    self._open_gripper(False)
                    # go to lift pose
                    pose_stamped.pose = self._translate_pose(feedback.pose, self._post_pose)
                    self._arm.move_to_pose(pose_stamped)

                print("Finished moving")
            elif feedback.menu_entry_id == 2:
                self._open_gripper(True)

        elif event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self._safe_to_move = True
            pose_stamped.pose = self._translate_pose(feedback.pose, self._grip_pose)
            if self._arm.compute_ik(pose_stamped):
                _change_colour(self._auto_pick_im, 0, 'green')
            else:
                self._safe_to_move = False
                _change_colour(self._auto_pick_im, 0, 'red')

            pose_stamped.pose = self._translate_pose(feedback.pose, self._pre_pose)
            if self._arm.compute_ik(pose_stamped):
                _change_colour(self._auto_pick_im, 3, 'green')
            else:
                self._safe_to_move = False
                _change_colour(self._auto_pick_im, 3, 'red')
            
            pose_stamped.pose = self._translate_pose(feedback.pose, self._post_pose)
            if self._arm.compute_ik(pose_stamped):
                _change_colour(self._auto_pick_im, 6, 'green')
            else:
                self._safe_to_move = False
                _change_colour(self._auto_pick_im, 6, 'red')

        self._auto_pick_im.pose = feedback.pose
        self._im_server.insert(self._auto_pick_im, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()


    def _transform_offset(self, orientation, offset_tup):
        matrix = tft.quaternion_matrix([orientation.x, orientation.y, orientation.z, orientation.w])
        offset = np.ones((4,1))
        offset[:3,0] = offset_tup

        offset = np.matmul(matrix, offset)

        return (offset[0,0], offset[1,0], offset[2,0])

    def _translate_pose(self, pose, offset_tup):
        pose = deepcopy(pose)
        offset = self._transform_offset(pose.orientation, offset_tup)

        pose.position.x += offset[0]
        pose.position.y += offset[1]
        pose.position.z += offset[2]

        return pose


def main():
    rospy.init_node('gripper_teleop')
    wait_for_time()
    arm = fetch_api.Arm()
    gripper = fetch_api.Gripper()
    im_server = InteractiveMarkerServer('gripper_im_server', q_size=2)
    teleop = GripperTeleop(arm, gripper, im_server)
    auto_pick_im_server = InteractiveMarkerServer('auto_pick_im_server', q_size=2)
    auto_pick = AutoPickTeleop(arm, gripper, auto_pick_im_server)
    teleop.start()
    auto_pick.start()
    rospy.spin()

if __name__=='__main__':
    main()

