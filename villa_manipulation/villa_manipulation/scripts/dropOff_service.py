#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from hsrb_interface import Robot
from hsrb_interface import collision_world

from villa_manipulation.srv import (DropOffObject, DropOffObjectResponse)


	  
def get_transform(target, source):
	print "Waiting for transform from ",source,"to ",target, "..."
	while True:
		try:
			transform = whole_body._tf2_buffer.lookup_transform(target, source, rospy.Time(0), rospy.Duration(1.0))
			break
		except:
			continue
	return transform

class setRobot:
	def __init__(self):
		self.rob = Robot()
		self.whole_body = self.rob.try_get('whole_body')
		self.omni_base = self.rob.try_get('omni_base')
		self.gripper = self.rob.try_get('gripper')
		
		collision_world = self.rob.try_get('global_collision_world')
		self.whole_body.collision_world = collision_world

		self.whole_body.end_effector_frame = u'hand_palm_link'

def dropOff(req):
	frame = req.drop_point.header.frame_id
	pos = req.drop_point.pose
	thresh = req.threshold
	res = DropOffObjectResponse()
	
	try: 
		robot.whole_body.move_end_effector_pose( geometry.pose(x=pos.position.x-0.06, y=pos.position.y, z=pos.position.z, ei=3.14,ej=-1.57, ek=0.0), frame)
		robot.whole_body.move_end_effector_by_line((0,0,1), thresh)
		robot.gripper.command(1.2)

		rospy.sleep(1.)
		robot. whole_body.move_end_effector_by_line((0,0,1), -thresh*2)
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

	rospy.init_node('dropOff_server')
	s = rospy.Service('dropOff', DropOffObject, dropOff)
	rospy.spin()


