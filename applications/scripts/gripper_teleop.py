#! /usr/bin/env python

import rospy
from util import wait_for_time
import fetch_api
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import MenuEntry, InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import PoseStamped
from copy import deepcopy

GRIPPER_MESH = 'package://fetch_description/meshes/gripper_link.dae'
L_FINGER_MESH = 'package://fetch_description/meshes/l_gripper_finger_link.STL'
R_FINGER_MESH = 'package://fetch_description/meshes/r_gripper_finger_link.STL'


class GripperTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server
        pose = PoseStamped()
        pose.pose.position.x = 0.166
        pose.pose.orientation.w = 1
        self._gripper_im = self._create_gripper(pose)

    def _single_axis_control(self, control, axis_label, w, x, y, z):
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

    def _create_menu(self):
        menu_items = list()
        menu = MenuEntry()

        menu.title = 'Go to Gripper'
        menu.id = 1
        menu_items.append(deepcopy(menu))

        menu.title = 'Open Gripper'
        menu.id = 2
        menu_items.append(deepcopy(menu))
        
        menu.title = 'Close Gripper'
        menu.id = 3
        menu_items.append(deepcopy(menu))
        
        return menu_items


    def _make_6dof_controls(self):
        dof_controls = list()

        # Creating base control
        control = InteractiveMarkerControl()
        control.always_visible = True

        orientations = {'x':(1, 1, 0, 0), 'y':(1, 0, 0, 1), 'z':(1, 0, 1, 0)}

        for label, orien in orientations.iteritems():
            move_rotate = self._single_axis_control(control, label, *orien)
            dof_controls.extend(move_rotate)
        
        return dof_controls

    def _open_close_gripper(self, isOpen):
        self._gripper_im.controls[0].markers[1].pose.position.y += 0.045 if isOpen else -0.045
        self._gripper_im.controls[0].markers[2].pose.position.y += -0.045 if isOpen else 0.045
        self._gripper.open() if isOpen else self._gripper.close()

    def _change_colour(self, pose_stamped, col):
        self._gripper_im.pose = pose_stamped.pose
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

        self._gripper_im.controls[0].markers[0].color = newHandColor
        self._gripper_im.controls[0].markers[1].color = newFingerColor
        self._gripper_im.controls[0].markers[2].color = newFingerColor

    def _create_gripper(self, pose):
        gripper_im = InteractiveMarker()
        gripper_im.header.frame_id = 'base_link'
        gripper_im.name = 'gripper_im'
        gripper_im.description = 'gripper_im'
        gripper_im.pose.position = pose.pose.position
        gripper_im.pose.orientation = pose.pose.orientation
        gripper_im.scale = 0.4

        gripper_mesh = InteractiveMarkerControl()
        gripper_mesh.always_visible = True

        gripper = Marker()
        gripper.type = Marker.MESH_RESOURCE
        gripper.mesh_resource = GRIPPER_MESH
        gripper.pose.position.x += 0.166
        gripper.color.r = 0.9
        gripper.color.g = 0.9
        gripper.color.b = 0.9
        gripper.color.a = 1.0

        l_finger = Marker()
        l_finger.type = Marker.MESH_RESOURCE
        l_finger.mesh_resource = L_FINGER_MESH
        l_finger.pose.position.x += 0.166
        l_finger.pose.position.y -= 0.06
        l_finger.color.r = 0.3
        l_finger.color.g = 0.3
        l_finger.color.b = 0.3
        l_finger.color.a = 1.0

        r_finger = Marker()
        r_finger.type = Marker.MESH_RESOURCE
        r_finger.mesh_resource = R_FINGER_MESH
        r_finger.pose.position.x += 0.166
        r_finger.pose.position.y += 0.06
        r_finger.color.r = 0.3
        r_finger.color.g = 0.3
        r_finger.color.b = 0.3
        r_finger.color.a = 1.0

        gripper_mesh.markers.extend([gripper, l_finger, r_finger])
        gripper_im.controls.append(gripper_mesh)

        gripper_im.controls.extend(self._make_6dof_controls())
        gripper_im.menu_entries.extend(self._create_menu())

        return gripper_im

    def start(self):
        print("Started")
        self._im_server.insert(self._gripper_im, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()


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
                self._open_close_gripper(True)
            elif feedback.menu_entry_id == 3:
                self._open_close_gripper(False)
        elif event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            if self._arm.compute_ik(pose_stamped):
                self._change_colour(pose_stamped, 'green')
            else: 
                self._change_colour(pose_stamped, 'red')
        
        self._im_server.insert(self._gripper_im, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()


class AutoPickTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server

    def start(self):
        obj_im = InteractiveMarker()
        self._im_server.insert(obj_im, feedback_cb=self.handle_feedback)

    def handle_feedback(self, feedback):
        pass


def main():
    rospy.init_node('gripper_teleop')
    wait_for_time()
    arm = fetch_api.Arm()
    gripper = fetch_api.Gripper()
    # TODO:
    im_server = InteractiveMarkerServer('gripper_im_server')
    # auto_pick_im_server = InteractiveMarkerServer('auto_pick_im_server')
    teleop = GripperTeleop(arm, gripper, im_server)
    # auto_pick = AutoPickTeleop(arm, gripper, auto_pick_im_server)
    teleop.start()
    # auto_pick.start()
    rospy.spin()

if __name__=='__main__':
    main()
"""
TODO

3 markers: 
    - gripper + 2 finger tips 
    - add to single InteractiveMarkerControl
    - add to InteractiveMarker
    - function(PoseStamped) returns InteractiveMarker or [3 markers]

"""