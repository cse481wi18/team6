#! /usr/bin/env python

import rospy
from recycle_msgs.msg import  LogItem
from recycle_ui.msg import ActiveLogs, GetLogPage
from sensor_msgs.msg import PointCloud2
from perception import MockCamera
import sqlite3

WASTE_TYPES = ['landfill', 'recycle', 'compost']

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

DB_FILE = '/home/team6/catkin_ws/src/cse481wi18/database/recycle.db'

class ClassificationServer():

    def __init__(self, waste_type):
        self._waste_type = waste_type
        rospy.loginfo("CONSTRUCTING SERVER")
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
        rospy.loginfo('log page handler got a message:')
        rospy.loginfo(get_log_page)
        is_reverse_page = get_log_page.page_size < 0
        sort_direction = 'DESC' if is_reverse_page else 'ASC'
        log_id_inequality = 'log_id < ' if is_reverse_page else 'log_id > '
        log_id_inequality += str(get_log_page.starting_log_num)
        rospy.loginfo(log_id_inequality)
        rospy.loginfo('sort direction {}'.format(sort_direction))

        conn = sqlite3.connect(DB_FILE)
        rospy.loginfo('DB connected')
        c = conn.cursor()
        c.execute('''SELECT
                  log_id,
                  predicted_category,
                  actual_category,
                  pointcloud_file_path,
                  image_file_path,
                  feature_file_path
            FROM classification_log
            WHERE predicted_category=?
            AND (actual_category IS NULL OR actual_category='')
            AND {}
            ORDER BY log_id {}
            LIMIT ?'''.format(log_id_inequality, sort_direction),
            (self._waste_type, abs(get_log_page.page_size)))
        logs = c.fetchall()

        log_items = []
        for log in logs:
            # build response fields
            rospy.loginfo(log)
            rospy.loginfo("PROCESSING LOG")
            cloud_topic_name = '/log{}_pointcloud'.format(log[0])
            log_items.append(LogItem(log_id=log[0],
                predicted_category=str(log[1]),
                actual_category=str(log[2]),
                pointcloud_file_path=str(log[3]),
                image_file_path=str(log[4]),
                feature_file_path=str(log[5])))

            # prepare to publish pointclouds
            cloud = self._camera.read_cloud(log[3])
            pub = rospy.Publisher(cloud_topic_name, PointCloud2, latch=True, queue_size=1)
            pub.publish(cloud)
            rospy.loginfo('published pointcloud to {}'.format(cloud_topic_name))
            self._active_cloud_pubs.append(pub)

        log_items = sorted(log_items, key=lambda x: x.log_id)
        active_logs = ActiveLogs(log_items=log_items)
        rospy.loginfo(active_logs)
        self._active_log_pub.publish(active_logs);
        conn.close()


def main():
    rospy.init_node('recycle-ui-classification')
    wait_for_time()
    servers = [ClassificationServer(waste_type) for waste_type in WASTE_TYPES]
    rospy.spin()

if __name__ == '__main__':
    main()
