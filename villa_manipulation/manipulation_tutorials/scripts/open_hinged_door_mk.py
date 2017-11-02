#!/usr/bin/python
# -*- coding: utf-8 -*-
import hsrb_interface
from hsrb_interface import geometry
from hsrb_interface import Robot, ItemTypes
from hsrb_impedance_control_example.impedance_control_switch import ImpedanceControlSwitch
import math
import rospy
import sys
import tf
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from tmc_planning_msgs.srv import PlanWithTsrConstraints
from tmc_planning_msgs.srv import PlanWithTsrConstraintsRequest
from tmc_planning_msgs.msg import TaskSpaceRegion
from tmc_manipulation_msgs.msg import (
    BaseMovementType,
    ArmManipulationErrorCodes
)

_TF_TIMEOUT = 3.5
_ODOM_TF = 'odom'
# _ORIGIN_TF ='map'
_ORIGIN_TF ='odom'
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

HANDLE_POS = (5.1, 0.51, 0.947)
RECOG_HANDLE_POS = (0.0, 0.0, 0.0)
HANDLE_TO_DOOR_HINGE_POS = 0.61
HANDLE_TO_HANDLE_HINGE_POS = -0.06
HANDLE_TO_HAND_POS = 0.15
HANDLE_GOAL_OFFSET = 0.5
recog_pos = geometry_msgs.msg.PoseStamped()
recog_pos.pose.position.x=0.00
recog_pos.pose.position.y=-0.0
recog_pos.pose.position.z=0.0

# recog_pos.pose.position.x=0.70
# recog_pos.pose.position.y=-0.2
# recog_pos.pose.position.z=0.947


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

    while not rospy.is_shutdown():
        try:
            trans = whole_body._tf2_buffer.lookup_transform(base_frame,
                                                        target_frame,
                                                        rospy.Time(0),
                                                        rospy.Duration(_TF_TIMEOUT)).transform
            break
        except ExtrapolationException as e:
            continue
            
    tuples = geometry.transform_to_tuples(trans)
    return tuples

def grasp_point_callback(msg):
    recog_pos.pose.position.x=msg.pose.position.x-0.06
    recog_pos.pose.position.y=msg.pose.position.y-0.01
    recog_pos.pose.position.z=msg.pose.position.z
    # dkldsaflk;print recog_pos.pose.position


def listnerfunction():
    # rospy.init_node('listener',anonymous=True)
    rospy.Subscriber("handle_detector/grasp_point",PoseStamped,grasp_point_callback)

def main(whole_body, gripper,wrist_wrench):
    # Grab the handle of door
    # wrist_wrench =wholjjjjjjjjjk
    whole_body.move_to_neutral()
    # whole_body.impedance_config= 'grasping'
    switch = ImpedanceControlSwitch() 
    # wrist_wrench.reset()
    gripper.command(1.0)

    grab_pose = geometry.multiply_tuples(geometry.pose(x=recog_pos.pose.position.x,
                                                       y=recog_pos.pose.position.y,
                                                       z=recog_pos.pose.position.z,
                                                       ej=math.pi/2),
                                         geometry.pose(ek=math.pi/2))
    whole_body.move_end_effector_pose(grab_pose, _ORIGIN_TF)
    wrist_wrench.reset()
    # whole_body.impedance_config= 'compliance_middle'
    switch.activate("grasping")
    # gripper.command(0.01)
    gripper.grasp(-0.0075)
    rospy.sleep(1.0)
    switch.inactivate()

    # wrist_wrench.reset()
    rospy.sleep(8.0)

    listener = tf.TransformListener()
    listener.waitForTransform("/odom", "/hand_palm_link", rospy.Time(), rospy.Duration(3.0))
    # now = rospy.Time.now()
    # listener.waitForTransform("/odom", "/hand_palm_link", now, rospy.Duration(5.0))
    # (trans, rot) = listener.lookupTransform("/map", "/hand_palm_link", now)

    # cur_tuples = geometry.transform_to_tuples(target_trans)
        
    # print trans
    # print rot
 
    #tf frame
    # br = tf2_ros.TransformBroadcaster()
    # t = geometry_msgs.msg.TransformStamped()

    # t.header.stamp = rospy.Time.now()
    # t.header.frame_id = "/map"
    # t.child_frame_id = "/target"
    # t.transform.translation.x = trans[0]
    # t.transform.translation.y = trans[1]
    # t.transform.translation.z = trans[2]
    # q = tf.transformations.quaternion_from_euler(0, 0, 0.0)
    # t.transform.rotation.x = rot[0]
    # t.transform.rotation.y = rot[1]
    # t.transform.rotation.z = rot[2]
    # t.transform.rotation.w = rot[3]
    # br.sendTransform(t)


    # Rotate the handle (Angle: math.pi/6)
    # wrist_wrench.reset()
    # whole_body.end_effector_frame = 'hand_palm_link'
    # whole_body.impedance_config= 'dumper_soft'

    switch.activate("placing")
    odom_to_hand = get_relative_tuples(_ORIGIN_TF, _HAND_TF)
    tsr_to_odom = geometry.pose(x=-(recog_pos.pose.position.x),
                                y=-(recog_pos.pose.position.y+HANDLE_TO_HANDLE_HINGE_POS),
                                z=-recog_pos.pose.position.z)
    tsr_to_hand = geometry.multiply_tuples(tsr_to_odom, odom_to_hand)

    const_tsr = TaskSpaceRegion()
    const_tsr.end_frame_id = _HAND_TF
    const_tsr.origin_to_tsr = geometry.tuples_to_pose(geometry.pose(x=recog_pos.pose.position.x,
                                                                    y=recog_pos.pose.position.y+HANDLE_TO_HANDLE_HINGE_POS,
                                                                    z=recog_pos.pose.position.z))
    const_tsr.tsr_to_end = geometry.tuples_to_pose(tsr_to_hand)
    const_tsr.min_bounds = [0, 0.0, 0.0,-(math.pi/7) , 0, 0]
    const_tsr.max_bounds = [0, 0.0, 0.0, 0, 0, 0]

    goal_tsr = TaskSpaceRegion()
    goal_tsr.end_frame_id = _HAND_TF
    goal_tsr.origin_to_tsr = geometry.tuples_to_pose(geometry.pose(x=recog_pos.pose.position.x,
                                                                   y=recog_pos.pose.position.y+HANDLE_TO_HANDLE_HINGE_POS,
                                                                   z=recog_pos.pose.position.z))
    goal_tsr.tsr_to_end = geometry.tuples_to_pose(tsr_to_hand)
    goal_tsr.min_bounds = [0, 0.0, 0.0,-math.pi/7, 0, 0]
    goal_tsr.max_bounds = [0, 0.0, 0.0,-math.pi/7, 0, 0]

    response = call_tsr_plan_service(whole_body, [const_tsr], [goal_tsr])
    if response.error_code.val != ArmManipulationErrorCodes.SUCCESS:
        rospy.logerr("Planning failed: (Error Code {0})".format(response.error_code.val))
        exit(-1)
    response.base_solution.header.frame_id = _ORIGIN_TF
    constrain_traj = whole_body._constrain_trajectories(response.solution,
                                                        response.base_solution)
    switch.activate("grasping")
    whole_body._execute_trajectory(constrain_traj)
    # whole_body.impedance_config= 'dumper_soft'
    switch.inactivate()
    rospy.sleep(10.0)


    # listener = tf.TransformListener()
    now = rospy.Time.now()
    listener.waitForTransform("/odom", "/hand_palm_link", now, rospy.Duration(3.0))

    rospy.sleep(1.0)
    # Open the door (Angle: math.pi/4)
    wrist_wrench.reset()
    switch.activate("placing")
    odom_to_hand = get_relative_tuples(_ORIGIN_TF, _HAND_TF)           #T0h
    tsr_to_odom = geometry.pose(x=-(recog_pos.pose.position.x),
                                y=-(recog_pos.pose.position.y+HANDLE_TO_DOOR_HINGE_POS+HANDLE_GOAL_OFFSET),
                                z=-recog_pos.pose.position.z) #Twe
    tsr_to_hand = geometry.multiply_tuples(tsr_to_odom, odom_to_hand) #T0s'

    const_tsr = TaskSpaceRegion()
    const_tsr.end_frame_id = _HAND_TF
    const_tsr.origin_to_tsr = geometry.tuples_to_pose(geometry.pose(x=recog_pos.pose.position.x,
                                                                    y=recog_pos.pose.position.y+HANDLE_TO_DOOR_HINGE_POS++HANDLE_GOAL_OFFSET,
                                                                    z=recog_pos.pose.position.z))
    const_tsr.tsr_to_end = geometry.tuples_to_pose(tsr_to_hand)
    const_tsr.min_bounds = [0, 0.0, 0.0, 0, 0, 0]
    const_tsr.max_bounds = [0, 0.0, 0.0, 0, 0, math.pi/4]

    goal_tsr = TaskSpaceRegion()
    goal_tsr.end_frame_id = _HAND_TF
    goal_tsr.origin_to_tsr = geometry.tuples_to_pose(geometry.pose(x=recog_pos.pose.position.x,
                                                                   y=recog_pos.pose.position.y+HANDLE_TO_DOOR_HINGE_POS+HANDLE_GOAL_OFFSET,
                                                                   z=recog_pos.pose.position.z))
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

    # whole_body.impedance_config= None
    switch.inactivate()

    gripper.command(1.0)
    whole_body.move_to_neutral()


if __name__=='__main__':
    with hsrb_interface.Robot() as robot:
        whole_body = robot.get('whole_body')
        gripper = robot.get('gripper')
        wrist_wrench = robot.get('wrist_wrench')
        listnerfunction()
        main(whole_body,  gripper,wrist_wrench)
