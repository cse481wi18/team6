#! /usr/bin/env python

import rospy
from recycle_ui.msg import GetLogPage, ActiveLogs
from sensor_msgs.msg import PointCloud2
from perception import MockCamera
import sqlite3

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

DB_FILE = '/home/team6/catkin_ws/src/cse481wi18/database/recycle.db'
class ClassificationServer():

    def __init__(self):
        rospy.logerr("CONSTRUCTING SERVER")
        self._log_page_sub = rospy.Subscriber('/recycle_ui/get_log_page',
                GetLogPage,
                callback=self._handle_get_log_page)
        self._active_log_pub = rospy.Publisher('/recycle_ui/active_logs',
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
            (get_log_page.waste_type, get_log_page.starting_log_num, get_log_page.page_size))
        logs = c.fetchall()

        predicted_labels = []
        actual_labels = []
        pointcloud_topics = []
        for log in logs:
            # build response fields
            rospy.logerr(log)
            rospy.logerr("PROCESSING LOG")
            predicted_labels.append(str(log[1]))
            actual_labels.append(str(log[2]))
            cloud_topic_name = 'log{}_pointcloud'.format(log[0])
            pointcloud_topics.append(cloud_topic_name)

            # prepare to publish pointclouds
            cloud = self._camera.read_cloud(log[3])
            pub = rospy.Publisher(cloud_topic_name, PointCloud2, latch=True, queue_size=1)
            pub.publish(cloud)
            self._active_cloud_pubs.append(pub)

        active_logs = ActiveLogs()
        active_logs.predicted_labels = predicted_labels
        active_logs.actual_labels = actual_labels
        active_logs.pointcloud_topics = pointcloud_topics
        self._active_log_pub.publish(active_logs);
        conn.close()


def main():
    rospy.init_node('recycle-ui-classification')
    wait_for_time()
    server = ClassificationServer()
    rospy.spin()

if __name__ == '__main__':
    main()
