#!/usr/bin/env python

import rospy
from recycle_msgs.msg import LogItem
import sqlite3

DB_FILE = '/home/team6/catkin_ws/src/cse481wi18/database/temp.db'

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def callback(log_item):
    rospy.logerr('Got a request to ingest log regord')
    rospy.logerr(DB_FILE)
    conn = sqlite3.connect(DB_FILE)
    c = conn.cursor()
    if log_item.log_id >= 0:
        rospy.logerr('updating record')
        c.execute('''
            UPDATE classification_log
            SET predicted_category = ?,
                actual_category = ?
            WHERE log_id = ?''',
            (log_item.predicted_category, log_item.actual_category, log_item.log_id))
    else:
        # new item
        rospy.logerr('inserting new record')
        rospy.logerr(log_item)
        c.execute('''
            INSERT INTO classification_log (
                predicted_category,
                actual_category,
                image_file_path,
                pointcloud_file_path,
                feature_file_path)
            VALUES(?, ?, ?, ?, ?)''',
            (log_item.predicted_category, log_item.actual_category, log_item.image_file_path,
            log_item.pointcloud_file_path, log_item.feature_file_path))
    conn.commit()
    rospy.logerr('ITEM INSERTED')
    conn.close()

def main():
    rospy.init_node('classification_logger')
    wait_for_time()
    rospy.Subscriber('/recycle/log_classification', LogItem, callback)
    rospy.spin()

if __name__ == '__main__':
    main()
