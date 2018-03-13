#!/usr/bin/env python

import sqlite3

DB_FILE = '/home/team6/catkin_ws/src/cse481wi18/database/recycle_less.db'
TO_INGEST = '/home/team6/catkin_ws/src/cse481wi18/database/less.txt'
def load_item(item, c):
    true_classification, pointcloud, features = item.split(' ')
    print('logging new {} object with pointcloud: {}, features: {}'.format(true_classification, pointcloud, features))
    c.execute('''
            INSERT INTO classification_log (
                actual_category,
                pointcloud_file_path,
                feature_file_path)
            VALUES(?, ?, ?)''',
            (true_classification, pointcloud, features))

def main():
    with open(TO_INGEST) as f:
        lines = f.readlines()

    conn = sqlite3.connect(DB_FILE)
    c = conn.cursor()
    for line in lines:
        load_item(line.strip(), c)
        conn.commit()
    conn.close()

if __name__ == '__main__':
    main()
