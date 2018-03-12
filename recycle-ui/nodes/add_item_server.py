#!/usr/bin/env python

import actionlib
import rospy

from sensor_msgs.msg import PointCloud2
import actionlib

import fetch_api
from perception import MockCamera
from recycle import Controller
from recycle_msgs.msg import AddItemAction, AddItemGoal, LogItem, DbLogAction, DbLogGoal
from recycle_ui.msg import AddItemAction as UIAddItemAction, AddItemResult as UIAddItemResult, \
        ConfirmItemAction, ConfirmItemResult


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


POINTCLOUD_TOPIC = 'confirm_pointcloud'


class AddItemServer:
    def __init__(self):
        self._head = fetch_api.Head()
        self._torso = fetch_api.Torso()
        self._camera = MockCamera()
        self._log_item = None
        self._dblog_client = actionlib.SimpleActionClient('/recycle/dblog_action', DbLogAction)

        # to talk to frontend
        self._add_item_server = actionlib.SimpleActionServer('/recycle_ui/add_item_action', UIAddItemAction, execute_cb=self._handle_add_item, auto_start=False)
        self._confirm_item_server = actionlib.SimpleActionServer('/recycle_ui/confirm_item_action', ConfirmItemAction, execute_cb=self._handle_confirm_item, auto_start=False)
        self._add_item_server.start()
        self._confirm_item_server.start()

        # publish pointcloud to topic
        self._pointcloud_pub = rospy.Publisher(POINTCLOUD_TOPIC, PointCloud2, latch=True, queue_size=1)

        # to talk to pointcloud processor
        self._add_item_client = actionlib.SimpleActionClient('/recycle_classifier/add_item', AddItemAction)
        self._add_item_client.wait_for_server()

    def _handle_add_item(self, add_item):
        rospy.logerr('AddItem: ' + add_item.category)
        # Adjust torso and head to be at the same place the controller would place it
        self._torso.set_height(Controller.TORSO_HEIGHT)
        self._head.look_at(*Controller.LOOK_AT_TABLE)

        # forward to the real add item ActionLibServer
        goal = AddItemGoal(category=add_item.category)
        self._add_item_client.send_goal(goal)
        self._add_item_client.wait_for_result()
        result = self._add_item_client.get_result()
        self._log_item = result.to_log

        # publish pointcloud
        cloud = self._camera.read_cloud(result.to_log.pointcloud_file_path)
        self._pointcloud_pub.publish(cloud)

        # send back result for confirmation
        new_result = UIAddItemResult(pointcloud_topic=POINTCLOUD_TOPIC)
        self._add_item_server.set_succeeded(new_result)

    def _handle_confirm_item(self, confirm_item):
        rospy.logerr('ConfirmItem:' + str(self._log_item) + str(confirm_item.confirmed))
        successful = True
        if confirm_item.confirmed:
            rospy.logerr('WRITING TO DB')
            goal = DbLogGoal(self._log_item)
            self._dblog_client.send_goal(goal)
            self._dblog_client.wait_for_result()
            if (self._dblog_client.get_result()):
                rospy.logerr('Write succeeded')
            else:
                rospy.logerr('Failed write')
                successful = False

        self._log_item = None
        result = ConfirmItemResult(success=successful)
        self._confirm_item_server.set_succeeded(result)


def main():
    rospy.init_node('recycle_ui_add_item')
    wait_for_time()
    server = AddItemServer()
    rospy.spin()


if __name__ == '__main__':
    main()
