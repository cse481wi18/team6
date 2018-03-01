#! /usr/bin/env python

import rospy
from recycle_ui.msg import ActiveLogs, GetLogPage, LogItem
from sensor_msgs.msg import PointCloud2
from perception import MockCamera
import sqlite3

WASTE_TYPES = ['trash', 'recycle', 'compost']

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

DB_FILE = '/home/team6/catkin_ws/src/cse481wi18/database/recycle.db'
class ClassificationServer():

    def __init__(self, waste_type):
        self._waste_type = waste_type
        rospy.logerr("CONSTRUCTING SERVER")
        self._log_page_sub = rospy.Subscriber('/recycle_ui/get_{}_log_page'.format(waste_type),
                GetLogPage,
                callback=self._handle_get_log_page)
        self._active_log_pub = rospy.Publisher('/recycle_ui/active_{}_logs'.format(waste_type),
                ActiveLogs,
                latch=True,
                queue_size=10)
        self._camera = MockCamera()
        self._active_cloud_pubs = []

    def _handle_get_log_page(self, get_log_page):
        rospy.logerr('GOT A MESSAGE')
        conn = sqlite3.connect(DB_FILE)
        rospy.logerr('DB connected')
        c = conn.cursor()
        # TODO Run this query once for each waste_type in waste_types
        c.execute('''SELECT
                cl.log_id,
                pred_ol.label,
                act_ol.label,
                cl.pointcloud_file_path
            FROM classification_log cl
            INNER JOIN object_labels pred_ol ON pred_ol.label_id=cl.predicted_label_id
            INNER JOIN object_labels act_ol ON act_ol.label_id=cl.actual_label_id
            INNER JOIN categories pred_cat ON pred_cat.category_id=pred_ol.category_id
            WHERE pred_cat.name=?
            AND cl.log_id > ?
            LIMIT ?''',
            (self._waste_type, get_log_page.starting_log_num, get_log_page.page_size))
        logs = c.fetchall()

        # Hunter is doing this for testing because He fucked up db
        test_data = {
            'recycle': [(1, 'paper', 'unknown', 'recycle', 'not/sure'), (4, 'coffee', 'unknown', 'recycle', 'not/sure')],
            'trash': [(2, 'wrapper', 'unknown', 'trash', 'not/sure')],
            'compost': [(3, 'apple', 'unknown', 'compost', 'not/sure')]
        };

        logs = test_data[self._waste_type]
        log_items = []
        for log in logs:
            # build response fields
            rospy.logerr(log)
            rospy.logerr("PROCESSING LOG")
            cloud_topic_name = 'log{}_pointcloud'.format(log[0])
            log_items.append(LogItem(log_id=log[0],
                predicted_item_label=str(log[1]),
                predicted_waste_type=self._waste_type,
                actual_label=str(log[2]),
                pointcloud_topic=cloud_topic_name))

            # prepare to publish pointclouds
            #cloud = self._camera.read_cloud(log[3])
            #pub = rospy.Publisher(cloud_topic_name, PointCloud2, latch=True, queue_size=1)
            #pub.publish(cloud)
            #self._active_cloud_pubs.append(pub)

        active_logs = ActiveLogs(log_items=log_items)
        self._active_log_pub.publish(active_logs);
        conn.close()


def main():
    rospy.init_node('recycle-ui-classification')
    wait_for_time()
    servers = [ClassificationServer(waste_type) for waste_type in WASTE_TYPES]
    rospy.spin()

if __name__ == '__main__':
    main()
