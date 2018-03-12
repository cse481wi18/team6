#!/usr/bin/env python

import rospy
from recycle_msgs.msg import LogItem, DbLogAction, DbLogResult
import sqlite3
import actionlib

DB_FILE = '/home/team6/catkin_ws/src/cse481wi18/database/recycle.db'

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

class LogServer:
    def __init__(self):
        # to talk to frontend
        self._logging_server = actionlib.SimpleActionServer('/recycle/dblog_action',
                DbLogAction, execute_cb=self._handle_logging, auto_start=False)
        self._logging_server.start()

    def _handle_logging(self, log_goal):
        rospy.logerr('Logging item to db')
        log_item = log_goal.log_item

        rospy.logerr(DB_FILE)
        conn = sqlite3.connect(DB_FILE)
        c = conn.cursor()
        if log_item.log_id >= 0:
            rospy.logerr('updating record')
            c.execute('''
                UPDATE classification_log
                SET predicted_category = ?,
                    actual_category = ?,
                    image_file_path = ?,
                    pointcloud_file_path = ?,
                    feature_file_path = ?
                WHERE log_id = ?''',
                (log_item.predicted_category, log_item.actual_category, log_item.image_file_path,
                    log_item.pointcloud_file_path, log_item.feature_file_path, log_item.log_id))
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
        self._logging_server.set_succeeded(DbLogResult(True))

def main():
    rospy.init_node('logger')
    wait_for_time()
    server = LogServer()
    rospy.spin()

if __name__ == '__main__':
    main()
