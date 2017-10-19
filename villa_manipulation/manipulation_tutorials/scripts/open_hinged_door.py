#!/usr/bin/python
# -*- coding: utf-8 -*-
import hsrb_interface
from hsrb_interface import geometry
import math
import rospy
import sys
from tmc_planning_msgs.srv import PlanWithTsrConstraints
from tmc_planning_msgs.srv import PlanWithTsrConstraintsRequest
from tmc_planning_msgs.msg import TaskSpaceRegion
from tmc_manipulation_msgs.msg import (
    BaseMovementType,
    ArmManipulationErrorCodes
)

_TF_TIMEOUT = 3.0
# _ORIGIN_TF = 'odom'
_ORIGIN_TF ='map'
_ROBOT_TF = 'base_footprint'
_HAND_TF = 'hand_palm_link'

_USE_JOINTS = (
    'wrist_flex_joint',
    'wrist_roll_joint',
    'arm_roll_joint',
    'arm_flex_joint',
    'arm_lift_joint'
)
_PLANNING_MAX_ITERATION = 10000
_PLANNING_GOAL_GENERATION = 0.3
_PLANNING_GOAL_DEVIATION = 0.3

HANDLE_POS = (0.54, 0.0, 0.94)
HANDLE_TO_DOOR_HINGE_POS = 0.70
HANDLE_TO_HANDLE_HINGE_POS = -0.09
HANDLE_TO_HAND_POS = 0.16

def call_tsr_plan_service(whole_body, constraint_tsrs, goal_tsrs):
    odom_to_robot_pose = geometry.tuples_to_pose(get_relative_tuples(_ORIGIN_TF,
                                                                     _ROBOT_TF))
    req = PlanWithTsrConstraintsRequest()
    req.base_movement_type.val = BaseMovementType.PLANAR
    req.origin_to_basejoint = odom_to_robot_pose
    req.initial_joint_state = whole_body._get_joint_state()
    req.use_joints = _USE_JOINTS
    req.probability_goal_generate = _PLANNING_GOAL_GENERATION
    req.timeout = rospy.Duration(whole_body._planning_timeout)
    req.max_iteration = _PLANNING_MAX_ITERATION
    req.uniform_bound_sampling = False
    req.deviation_for_bound_sampling = _PLANNING_GOAL_DEVIATION
    req.weighted_joints = ['_linear_base', '_rotational_base']
    req.weight = [whole_body._linear_weight, whole_body._angular_weight]
    if whole_body._collision_world is not None:
        req.environment_before_planning = whole_body._collision_world.snapshot(_ORIGIN_TF)
    req.constraint_tsrs = constraint_tsrs
    req.goal_tsrs = goal_tsrs
    plan_service = rospy.ServiceProxy('plan_with_constraints',
                                      PlanWithTsrConstraints)
    res = plan_service.call(req)
    return res

def get_relative_tuples(base_frame, target_frame):
    trans = whole_body._tf2_buffer.lookup_transform(base_frame,
                                                    target_frame,
                                                    rospy.Time(0),
                                                    rospy.Duration(_TF_TIMEOUT)).transform
    tuples = geometry.transform_to_tuples(trans)
    return tuples


def main(whole_body, gripper):
    # Grab the handle of door
    whole_body.move_to_neutral()
    gripper.command(1.0)
    
    grab_pose = geometry.multiply_tuples(geometry.pose(x=HANDLE_POS[0]-HANDLE_TO_HAND_POS,
                                                       y=HANDLE_POS[1],
                                                       z=HANDLE_POS[2],
                                                       ej=math.pi/2),
                                         geometry.pose(ek=math.pi/2))
    whole_body.move_end_effector_pose(grab_pose, _ORIGIN_TF)
    gripper.command(0.1)
    rospy.sleep(10.0)

    # Rotate the handle (Angle: math.pi/6)
    odom_to_hand = get_relative_tuples(_ORIGIN_TF, _HAND_TF)
    tsr_to_odom = geometry.pose(x=-(HANDLE_POS[0]-HANDLE_TO_HAND_POS),
                                y=-(HANDLE_POS[1]-HANDLE_TO_HANDLE_HINGE_POS),
                                z=-HANDLE_POS[2])
    tsr_to_hand = geometry.multiply_tuples(tsr_to_odom, odom_to_hand)

    const_tsr = TaskSpaceRegion()
    const_tsr.end_frame_id = _HAND_TF
    const_tsr.origin_to_tsr = geometry.tuples_to_pose(geometry.pose(x=HANDLE_POS[0]-HANDLE_TO_HAND_POS,
                                                                    y=HANDLE_POS[1]-HANDLE_TO_HANDLE_HINGE_POS,
                                                                    z=HANDLE_POS[2]))
    const_tsr.tsr_to_end = geometry.tuples_to_pose(tsr_to_hand)
    const_tsr.min_bounds = [0, 0.0, 0.0, 0, 0, 0]
    const_tsr.max_bounds = [0, 0.0, 0.0, math.pi/6, 0, 0]

    goal_tsr = TaskSpaceRegion()
    goal_tsr.end_frame_id = _HAND_TF
    goal_tsr.origin_to_tsr = geometry.tuples_to_pose(geometry.pose(x=HANDLE_POS[0]-HANDLE_TO_HAND_POS,
                                                                   y=HANDLE_POS[1]-HANDLE_TO_HANDLE_HINGE_POS,
                                                                   z=HANDLE_POS[2]))
    goal_tsr.tsr_to_end = geometry.tuples_to_pose(tsr_to_hand)
    goal_tsr.min_bounds = [0, 0.0, 0.0, math.pi/6, 0, 0]
    goal_tsr.max_bounds = [0, 0.0, 0.0, math.pi/6, 0, 0]

    response = call_tsr_plan_service(whole_body, [const_tsr], [goal_tsr])
    if response.error_code.val != ArmManipulationErrorCodes.SUCCESS:
        rospy.logerr("Planning failed: (Error Code {0})".format(response.error_code.val))
        exit(-1)
    response.base_solution.header.frame_id = _ORIGIN_TF
    constrain_traj = whole_body._constrain_trajectories(response.solution,
                                                        response.base_solution)
    whole_body._execute_trajectory(constrain_traj)
    rospy.sleep(10.0)

    # Open the door (Angle: math.pi/4)
    odom_to_hand = get_relative_tuples(_ORIGIN_TF, _HAND_TF)           #T0h
    tsr_to_odom = geometry.pose(x=-HANDLE_POS[0],
                                y=-(HANDLE_POS[1]-HANDLE_TO_DOOR_HINGE_POS),
                                z=-HANDLE_POS[2]) #Twe
    tsr_to_hand = geometry.multiply_tuples(tsr_to_odom, odom_to_hand) #T0s'

    const_tsr = TaskSpaceRegion()
    const_tsr.end_frame_id = _HAND_TF
    const_tsr.origin_to_tsr = geometry.tuples_to_pose(geometry.pose(x=HANDLE_POS[0],
                                                                    y=HANDLE_POS[1]-HANDLE_TO_DOOR_HINGE_POS,
                                                                    z=HANDLE_POS[2]))
    const_tsr.tsr_to_end = geometry.tuples_to_pose(tsr_to_hand)
    const_tsr.min_bounds = [0, 0.0, 0.0, 0, 0, 0]
    const_tsr.max_bounds = [0, 0.0, 0.0, 0, 0, math.pi/4]

    goal_tsr = TaskSpaceRegion()
    goal_tsr.end_frame_id = _HAND_TF
    goal_tsr.origin_to_tsr = geometry.tuples_to_pose(geometry.pose(x=HANDLE_POS[0],
                                                                   y=HANDLE_POS[1]-HANDLE_TO_DOOR_HINGE_POS,
                                                                   z=HANDLE_POS[2]))
    goal_tsr.tsr_to_end = geometry.tuples_to_pose(tsr_to_hand)
    goal_tsr.min_bounds = [0, 0.0, 0.0, 0, 0, math.pi/4]
    goal_tsr.max_bounds = [0, 0.0, 0.0, 0, 0, math.pi/4]

    response = call_tsr_plan_service(whole_body, [const_tsr], [goal_tsr])
    if response.error_code.val != ArmManipulationErrorCodes.SUCCESS:
        rospy.logerr("Planning failed: (Error Code {0})".format(response.error_code.val))
        exit(-1)
    response.base_solution.header.frame_id = _ORIGIN_TF
    constrain_traj = whole_body._constrain_trajectories(response.solution,
                                                        response.base_solution)
    whole_body._execute_trajectory(constrain_traj)


if __name__=='__main__':
    with hsrb_interface.Robot() as robot:
        whole_body = robot.get('whole_body')
        gripper = robot.get('gripper')
        main(whole_body,  gripper)
