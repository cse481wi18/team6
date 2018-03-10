import numpy as np
import rospy
import tf.transformations as tft
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion

def trans_rot_to_matrix(translation, orientation):
	result = tft.quaternion_matrix(orientation)  # add rotation
	result[:3 ,3] = translation # add translation
	return result

def posestamped_to_matrix(pose_stamped):
	translation = pose_stamped.pose.position
	orientation = pose_stamped.pose.orientation
	return trans_rot_to_matrix([translation.x, translation.y, translation.z], [orientation.x, orientation.y, orientation.z, orientation.w])

def rotate_quaternion_by_angle(quaternion, radian):
	q_matrix = tft.quaternion_matrix([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
	rot_matrix = rotation_matrix(radian)
	q_matrix = np.dot(rot_matrix, q_matrix)
	new_q = tft.quaternion_from_matrix(q_matrix)
	return Quaternion(*new_q)

def rotation_matrix(radian):
	matrix = np.zeros((4, 4))
	matrix[0,0] = np.cos(radian)
	matrix[0,1] = -np.sin(radian)
	matrix[1,0] = np.sin(radian)
	matrix[1,1] = np.cos(radian)
	matrix[2,2] = 1.0
	matrix[3,3] = 1.0
	return matrix

def quaternion_to_angle(q):
	roll, pitch, yaw = tft.euler_from_quaternion((q.x, q.y, q.z, q.w))
	return yaw