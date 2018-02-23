import actionlib
from recycle_msgs.msg import ClassifyAction, ClassifyActionGoal

client = actionlib.SimpleActionClient('recycle_classifier', ClassifyAction)
client.wait_for_server()

client.send_goal(ClassifyActionGoal())