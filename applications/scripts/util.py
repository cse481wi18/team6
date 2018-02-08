import rospy
import tf.transformations as tft

from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

class ArTagReader(object):
    def __init__(self):
        self.markers = []

    def callback(self, msg):
        self.markers = msg.markers

def get_markers():
    reader = ArTagReader()
    sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, reader.callback) # Subscribe to AR tag poses, use reader.callback
    
    rospy.loginfo('waiting for markers')

    tries = 200
    while len(reader.markers) == 0:
    	tries -= 1
    	if tries == 0:
    		raise Exception("Wtf")
        rospy.sleep(0.1)


    for marker in reader.markers:
    	if len(marker.pose.header.frame_id) == 0:
    		marker.pose.header.frame_id = marker.header.frame_id
    rospy.loginfo('have markers')
    return reader.markers

def trans_rot_to_matrix(translation, orientation):
	result = tft.quaternion_matrix(orientation)  # add rotation
	result[:3 ,3] = translation # add translation
	return result

def pose_to_matrix(pose_stamped):
	translation = pose_stamped.pose.position
	orientation = pose_stamped.pose.orientation
	return trans_rot_to_matrix([translation.x, translation.y, translation.z], [orientation.x, orientation.y, orientation.z, orientation.w])

def transform_to_pose(matrix):
	position_arr = matrix[:3, 3]
	position = Point(x=position_arr[0], y=position_arr[1], z=position_arr[2])
	quaternion = tft.quaternion_from_matrix(matrix)
	orientation = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])
	pose_stamped = PoseStamped(pose=Pose(position=position, orientation=orientation))
	pose_stamped.header.frame_id = 'base_link'

	return pose_stamped