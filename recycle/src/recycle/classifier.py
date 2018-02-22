#!/usr/bin/env python                                                                                  
                                      
import actionlib                                                                 
import rospy
from recycle_msgs.msg import ClassifyAction, ClassifyActionResult

class Classifier(object):

    # create messages that are used to publish feedback/result
    # _result = Result()

    def __init__(self, classify_action_name):
        self._action_name = classify_action_name
        self._classify_server = actionlib.SimpleActionServer(self._action_name, ClassifyAction, execute_cb=self.execute_cb, auto_start = False)
        self._classify_server.start()
      
    def execute_cb(self, goal):
        rospy.loginfo('Executing callback. Received goal.')
        # helper variables
        # success = True
        
        # # check that preempt has not been requested by the client
        # if self._as.is_preempt_requested():
        #     rospy.loginfo('%s: Preempted' % self._action_name)
        #     self._as.set_preempted()
        #     success = False
        #     break

        # if success:
        #     self._result.sequence = self._feedback.sequence
        #     rospy.loginfo('%s: Succeeded' % self._action_name)
        #     self._as.set_succeeded(self._result)
