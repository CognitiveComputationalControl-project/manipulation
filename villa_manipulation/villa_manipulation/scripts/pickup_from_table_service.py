#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import tf, tf2_ros

from hsrb_interface import Robot
from hsrb_interface import geometry
from hsrb_interface import collision_world

from villa_manipulation.srv import (PickupObjectsFromTable2, PickupObjectsFromTable2Response)

from basic_grasp_planner import *

	  
def get_transform(target, source):
	print "Waiting for transform from ",source,"to ",target, "..."
	while True:
		try:
			transform = whole_body._tf2_buffer.lookup_transform(target, source, rospy.Time(0), rospy.Duration(1.0))
			break
		except:
			continue
	return transform


def openGripper(obj_width):
	if obj_width < 0.035:
		return 0.35
	elif obj_width < 0.045:
		return 0.45
	elif obj_width < 0.065:
		return 0.65
	elif obj_width < 0.095:
		return 0.85
	elif obj_width < 0.115:
		return 1.05
	else:
		return 1.25
   


class setRobot:
	def __init__(self):
		self.rob = Robot()
		self.whole_body = self.rob.try_get('whole_body')
		self.omni_base = self.rob.try_get('omni_base')
		self.gripper = self.rob.try_get('gripper')
		
		collision_world = self.rob.try_get('global_collision_world')
		self.whole_body.collision_world = collision_world

		self.whole_body.end_effector_frame = u'hand_palm_link'

		self.g_planner = grasp_planner()


def pickupFromTable(req):
	obj = req.bbox_pose
	aligned_axis = req.table_aligned_axis
	selected_object = obj.header.frame_id
	res = PickupObjectsFromTable2Response()

	try: 

		obj_transform = get_transform(selected_object, 'map')
		patt = robot.g_planner.getGraspWithObjectPose(req, robot.omni_base.pose, obj_transform, aligned_axis, graspDirection=req.grasp_direction)
		if aligned_axis == "y":
			obj_width = req.bbox_width
			obj_length = req.bbox_length
		else:
			obj_width = req.bbox_length
			obj_length = req.bbox_width
		
		robot.gripper.command(openGripper(obj_width))

		print "grasp pattern: ", patt
		robot.whole_body.move_end_effector_pose(patt, selected_object)
		robot.collision_world.remove(collision_objects[selected_object])
		robot.gripper.grasp(-0.02)
		robot.whole_body.move_end_effector_by_line((1,0,0), 0.05)
		robot.whole_body.move_end_effector_by_line((0,0,1), -0.2)
		robot.whole_body.move_to_go()
	   
		print robot.whole_body.joint_positions['hand_motor_joint'] 
		if robot.whole_body.joint_positions['hand_motor_joint'] <= -0.7: #### Can add more robust check for successful grasp, eg. based on the force sensor
			print "Failed grasp for obj"
			res.success = False
		else:
			res.success = True 

	except Exception as e:
		rospy.logerr(e)
		print "Failed to grasp object"
		res.success = False

	return res


if __name__=='__main__':
	global robot
	robot = setRobot()

	rospy.init_node('pickupFromTable_server')
	s = rospy.Service('pickupFromTable', PickupObjectsFromTable2, pickupFromTable)
	rospy.spin()


