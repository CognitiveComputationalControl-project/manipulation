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
from sensor_msgs.msg import JointState
#import moveit_commander
import trajectory_msgs.msg
import moveit_msgs
from copy import deepcopy
import controller_manager_msgs.srv
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
from tmc_planning_msgs.srv import PlanWithTsrConstraints
from tmc_planning_msgs.srv import PlanWithTsrConstraintsRequest
from tmc_planning_msgs.msg import TaskSpaceRegion
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionResult
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
HANDLE_TO_DOOR_HINGE_POS = 0.82
HANDLE_TO_HANDLE_HINGE_POS = -0.06
HANDLE_TO_HAND_POS = 0.067
HANDLE_GOAL_OFFSET = 0.5
recog_pos = geometry_msgs.msg.PoseStamped()
#recog_pos.pose.position.x=0.00
#recog_pos.pose.position.y=-0.0
#recog_pos.pose.position.z=0.0
cur_arm_lift_joint=0.0
cur_wrist_roll_joint=0.0
angle=30
recog_pos.pose.position.x=1.14
recog_pos.pose.position.y=-0.3
recog_pos.pose.position.z=0.947

def publish_arm(lift, flex,roll,wrist_flex,wrist_roll):
    traj = JointTrajectory()
    # This controller requires that all joints have values
    traj.joint_names = ["arm_lift_joint", "arm_flex_joint",
                        "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
    p = JointTrajectoryPoint()
    current_positions = [latest_positions[name] for name in traj.joint_names]
    current_positions[0] = lift
    current_positions[1] = flex
    current_positions[2] = roll
    current_positions[3] = wrist_flex
    current_positions[4] = wrist_roll
    p.positions = current_positions
    p.velocities = [0, 0, 0, 0, 0]
    p.time_from_start = rospy.Time(3)
    traj.points = [p]

    armPub.publish(traj)

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

latest_positions = None

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
    recog_pos.pose.position.x=msg.pose.position.x
    recog_pos.pose.position.y=msg.pose.position.y
    recog_pos.pose.position.z=msg.pose.position.z
    # dkldsaflk;print recog_pos.pose.position
def joint_states_callback(msg):
    global latest_positions
    positions = {}
    for name, i in zip(msg.name, range(len(msg.name))):
        positions[name] = msg.position[i]
    latest_positions = positions

def angle_callback(msg):
    angle = msg.data

def listnerfunction():
    # rospy.init_node('listener',anonymous=True)
    rospy.Subscriber("handle_detector/grasp_point",PoseStamped,grasp_point_callback)
    rospy.Subscriber("hsrb/joint_states",JointState,joint_states_callback)
    #rospy.Subscriber("hsrb/angle_detector", Float32 ,angle_callback)

def main(whole_body, gripper,wrist_wrench):


    
    armPub = rospy.Publisher('/hsrb/arm_trajectory_controller/command', JointTrajectory, queue_size=1)

#    #1.READ THE ANGLE (topic angle_detector)
#    angle_Msg = rospy.wait_for_message("angle_detector", Float32)
#    angle = angle_Msg.data 

    #2.READ THE HANDLE POSITION when the door is closed
#    target_pose_Msg = rospy.wait_for_message("/handle_detector/grasp_point", PoseStamped)
#    recog_pos.pose.position.x=target_pose_Msg.pose.position.x
#    recog_pos.pose.position.y=target_pose_Msg.pose.position.y
#    recog_pos.pose.position.z=target_pose_Msg.pose.position.z

    if (angle > -0.1 and angle < 0.2 ): #the door is closed


            ##3.GRAB THE DOOR HANDLE
            whole_body.move_to_neutral() 
            # whole_body.impedance_config= 'grasping' 
            switch = ImpedanceControlSwitch() 
            # wrist_wrench.reset() 
            gripper.command(1.0) 

            grab_pose = geometry.multiply_tuples(geometry.pose(x=recog_pos.pose.position.x-HANDLE_TO_HAND_POS,
                                                               y=recog_pos.pose.position.y,
                                                               z=recog_pos.pose.position.z,
                                                               ej=math.pi/2),
                                                 geometry.pose(ek=math.pi/2))
            whole_body.move_end_effector_pose(grab_pose, _ROBOT_TF)
            wrist_wrench.reset()
            # whole_body.impedance_config= 'compliance_middle'
            switch.activate("grasping")
            # gripper.command(0.01)
            gripper.grasp(-0.008)
            rospy.sleep(1.0)
            switch.inactivate()

            wrist_wrench.reset()
            rospy.sleep(8.0)

            #### test manipulation
            whole_body.impedance_config = 'grasping'



            #4.ROTATE HANDLE 
            # print(whole_body.impedance_config)
            # desired_rot=-1.95
            # whole_body.move_to_joint_positions({"wrist_roll_joint":desired_rot})
            wrist_roll=latest_positions["wrist_roll_joint"]-0.55

            traj = JointTrajectory()
            # This controller requires that all joints have values
            traj.joint_names = ["arm_lift_joint", "arm_flex_joint",
                                "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
            p = JointTrajectoryPoint()
            current_positions = [latest_positions[name] for name in traj.joint_names]
            current_positions[0] = latest_positions["arm_lift_joint"]-0.04
            current_positions[1] = latest_positions["arm_flex_joint"]-0.015
            current_positions[2] = latest_positions["arm_roll_joint"]
            current_positions[3] = latest_positions["wrist_flex_joint"]
            current_positions[4] = wrist_roll
            p.positions = current_positions
            p.velocities = [0, 0, 0, 0, 0]
            p.time_from_start = rospy.Time(3)
            traj.points = [p]

            armPub.publish(traj)

            rospy.sleep(5.0)

            whole_body.impedance_config = 'grasping'

       
            #5.PUSH MOTION - he moves with the door - first option - streight movement 
            phi = 90*(2*math.pi/360)
            l = HANDLE_TO_DOOR_HINGE_POS
            d = 2 * l *  math.cos(phi)
            #open_pose = geometry.pose(x = d*math.sin(phi), y=d*math.cos(phi), ek=math.pi/2 - angle_rad )
            #whole_body.move_end_effector_pose(open_pose, _ROBOT_TF)
            #angleeee=math.pi/2 - angle_rad
            omni_base.go(d*math.sin(phi), d*math.cos(phi), math.pi/2 - angle_rad, 300.0, relative=True)

            whole_body.move_to_neutral()



    elif (angle >= 0.2 and angle < 60 ): #the door is half-open



            #3.ROTATE PARALLEL TO THE DOOR 
            angle_rad = angle*(2*math.pi/360)
            #omni_base.go_rel(0.0, 0.0, angle_rad , 300.0) 

            ##4.GRAB THE DOOR HANDLE
            whole_body.move_to_neutral() 
            # whole_body.impedance_config= 'grasping' 
            switch = ImpedanceControlSwitch() 
            # wrist_wrench.reset() 
            gripper.command(0.0) 

            #change coordinates of the handle - now the door is open so the handle is moved - the data of the handle position are given for the closed door - I m supposing 
            #that the coordinates of the handle are wrt the base_footprint tf 
            phi = math.pi/2 - angle_rad/2
            l = HANDLE_TO_DOOR_HINGE_POS
            d1 = 2 * l *  math.sin(angle_rad/2)
            recog_pos.pose.position.x = recog_pos.pose.position.x + d1 * math.sin(phi)
            recog_pos.pose.position.y = recog_pos.pose.position.y + d1 * math.cos(phi)

            grab_pose = geometry.multiply_tuples(geometry.pose(x=recog_pos.pose.position.x-HANDLE_TO_HAND_POS,
                                                               y=recog_pos.pose.position.y - 10,
                                                               z=recog_pos.pose.position.z,
                                                               ej=math.pi/2),
                                                 geometry.pose(ek=math.pi/2))
            whole_body.move_end_effector_pose(grab_pose, _ROBOT_TF)
            wrist_wrench.reset()

            rospy.sleep(1.0)
            switch.inactivate()

            wrist_wrench.reset()
            rospy.sleep(8.0)

            #### test manipulation
            whole_body.impedance_config = 'grasping'

            #5.PUSH MOTION - he moves with the door - first option - streight movement 
            phi = math.pi/4 + angle_rad/2
            l = HANDLE_TO_DOOR_HINGE_POS
            d = 2 * l *  math.cos(phi)

            omni_base.go(d*math.sin(phi), d*math.cos(phi), math.pi/2 - angle_rad, 300.0, relative=True)


 





if __name__=='__main__':
    with hsrb_interface.Robot() as robot:
        whole_body = robot.get('whole_body')
        gripper = robot.get('gripper')
        wrist_wrench = robot.get('wrist_wrench')
        omni_base = robot.get('omni_base')
        listnerfunction()
        main(whole_body,  gripper,wrist_wrench)
