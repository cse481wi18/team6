#!/usr/bin/env python

import rospy
from map_annotator import MapAnnotator

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def print_intro():
	print "Welcome to the map annotator!"
	print_cmds()

def print_cmds():
	print "Commands:\n \
			  list: List saved poses.\n \
			  save <name>: Save the robot's current pose as <name>. Overwrites if <name> already exists.\n \
			  delete <name>: Delete the pose given by <name>.\n \
			  goto <name>: Sends the robot to the pose given by <name>.\n \
			  exit: exit\n \
			  help: Show this list of commands"

def main():
	rospy.init_node('map_annotator_demo')
	wait_for_time()
	map_ann = MapAnnotator()

	print_intro()
	while True:
		cmd = raw_input('> ')
		cmd = cmd.split()
		name = " ".join(cmd[1:])

		if cmd[0] == 'list':
			pose_list = map_ann.list_poses()
			if len(pose_list) == 0:
				print 'No poses'
			else:
				print 'Poses:'
				for pose_name in pose_list:
					print '\t' + pose_name

		elif cmd[0] == 'save':
			map_ann.save_pose(name)

		elif cmd[0] == 'delete':
			if not map_ann.delete_pose(name):
				print 'No such pose \'{}\''.format(name)

		elif cmd[0] == 'goto':
			if not map_ann.goto_pose_name(name):
				print 'No such pose \'{}\''.format(name)

		elif cmd[0] == 'help':
			print_cmds()
		elif cmd[0] == 'exit':
			return
		else:
			print 'Not a valid command.'


if __name__ == '__main__':
	main()

