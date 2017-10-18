#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
import math
import rospy
from hsrb_interface import Robot
from hsrb_interface import collision_world

from villa_manipulation.srv import (handOverObject, handOverObjectResponse)

# Force Detection
from geometry_msgs.msg import WrenchStamped

_CONNECTION_TIMEOUT = 10.0

class setRobot:
	def __init__(self):
		self.rob = Robot()
		self.tts = self.rob.try_get('default_tts')
		self.tts.language = tts.ENGLISH ########## May need modification
		self.whole_body = self.rob.try_get('whole_body')
		self.omni_base = self.rob.try_get('omni_base')
		self.gripper = self.rob.try_get('gripper')
		
		collision_world = self.rob.try_get('global_collision_world')
		self.whole_body.collision_world = collision_world

		self.whole_body.end_effector_frame = u'hand_palm_link'


class ForceSensorCapture(object):
	"""Subscribe and hold force sensor data"""

	def __init__(self):
		self._force_data_x = 0.0
		self._force_data_y = 0.0
		self._force_data_z = 0.0

		# Subscribe force torque sensor data from HSRB
		ft_sensor_topic = '/hsrb/wrist_wrench/raw'
		self._wrist_wrench_sub = rospy.Subscriber(ft_sensor_topic, WrenchStamped, self.__ft_sensor_cb)

		# Wait for connection
		try:
			rospy.wait_for_message(ft_sensor_topic, WrenchStamped, timeout=_CONNECTION_TIMEOUT)
		except Exception as e:
			rospy.logerr(e)
			sys.exit(1)

	def get_current_force(self):
		return [self._force_data_x, self._force_data_y, self._force_data_z]

	def __ft_sensor_cb(self, data):
		self._force_data_x = data.wrench.force.x
		self._force_data_y = data.wrench.force.y
		self._force_data_z = data.wrench.force.z


# General Functions
def compute_difference(pre_data_list, post_data_list):
	if (len(pre_data_list) != len(post_data_list)):
		raise ValueError('Argument lists differ in length')
	# Calcurate square sum of difference
	square_sums = sum([math.pow(b - a, 2)
					   for (a, b) in zip(pre_data_list, post_data_list)])
	return math.sqrt(square_sums)


def robot_say(string):
	print string
	robot.tts.say(string)


#### Main callback
def handOver_cb(req):
	res = handOverObjectResponse()
	
	try: 
		## Need to record the playback joints
		whole_body.move_to_joint_positions({'arm_lift_joint': 0.4, 'arm_flex_joint': -1.4, 'wrist_flex_joint': -0.4, 'wrist_roll_joint': 1.57})

		#### Handing over the object till pull is felt
		# Start force sensor capture
		force_sensor_capture = ForceSensorCapture()

		# Get initial data of force sensor
		pre_force_list = force_sensor_capture.get_current_force()

		# Ask user to grab object
		robot_say('Here is the object you want! Please hold it.')
		rospy.sleep(1.0)

		force_difference = 0.0
		while(force_difference < 0.1):
			rospy.sleep(1.0)
			post_force_list = force_sensor_capture.get_current_force()
			force_difference = compute_difference(pre_force_list, post_force_list)

		# Opening the gripper
		robot.gripper.command(1.2)
		rospy.sleep(1.)
		robot.whole_body.move_to_go()
		res.success = True 

	except Exception as e:
		rospy.logerr(e)
		print "Failed to drop off object"
		res.success = False

	return res


if __name__=='__main__':
	global robot
	robot = setRobot()

	rospy.init_node('handOver_server')
	s = rospy.Service('handOver', handOverObject, handOver_cb)
	rospy.spin()


