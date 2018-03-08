#!/usr/bin/env python

import actionlib
import rospy

from sensor_msgs.msg import PointCloud2

from perception import MockCamera
from recycle_msgs.msg import AddItemAction, AddItemGoal
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
        self._camera = MockCamera()
        self._log_item = None

        # to talk to frontend
        self._add_item_server = actionlib.SimpleActionServer('/recycle_ui/add_item_action', UIAddItemAction, execute_cb=self._handle_add_item, auto_start=False)
        self._confirm_item_server = actionlib.SimpleActionServer('/recycle_ui/confirm_item_action', ConfirmItemAction, execute_cb=self._handle_confirm_item, auto_start=False)
        self._add_item_server.start()
        self._confirm_item_server.start()

        # publish pointcloud to topic
        self._pointcloud_pub = rospy.Publisher(POINTCLOUD_TOPIC, PointCloud2, latch=True, queue_size=1)

        # to talk to pointcloud processor
        self._add_item_client = actionlib.SimpleActionClient('/recycle/add_item_action', AddItemAction)
        self._add_item_client.wait_for_server()

    def _handle_add_item(self, add_item):
        rospy.logerr('AddItem: ' + add_item.category)

        # forward to the real add item ActionLibServer

        # TODO talk to varun about how to actually talk to this server
        #goal = AddItemGoal(category=add_item.category)
        #self._add_item_client.send_goal(goal)
        #self._add_item_client.wait_for_result()
        #result = self._add_item_client.get_result()
        #self._log_item = result.log_item

        # publish pointcloud
        # TODO use real path
        cloud = self._camera.read_cloud('/home/team6/data/coffee1.bag')
        self._pointcloud_pub.publish(cloud)

        # send back result for confirmation
        # TODO fill this out
        #new_result = UIAddItemResult(pointcloud_topic='foo', log_item=result.log_item)
        new_result = UIAddItemResult(pointcloud_topic=POINTCLOUD_TOPIC)
        self._add_item_server.set_succeeded(new_result)

    def _handle_confirm_item(self, confirm_item):
        rospy.logerr('ConfirmItem:' + str(self._log_item) + str(confirm_item.confirmed))
        if confirm_item.confirmed:
            # TODO publish to logger topic and respond with success
            rospy.logerr('Confirm was true!')

        this._log_item = None
        result = ConfirmItemResult(success=True)
        self._confirm_item_server.set_succeeded(result)


def main():
    rospy.init_node('recycle-ui-add-item')
    wait_for_time()
    server = AddItemServer()
    rospy.spin()


if __name__ == '__main__':
    main()
