#!/usr/bin/python
# -*- coding: utf-8 -*-
import hsrb_interface
from hsrb_interface import geometry
from hsrb_interface import Robot, ItemTypes
from hsrb_impedance_control_example.impedance_control_switch import ImpedanceControlSwitch
import actionlib
import control_msgs.msg
import controller_manager_msgs.srv
import math
import rospy
import sys
import tf
import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import JointState
import moveit_commander
from tf import TransformListener
import trajectory_msgs.msg
import move_base_msgs.msg
import moveit_msgs
from copy import deepcopy
import controller_manager_msgs.srv
from move_base_msgs.msg import MoveBaseActionGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
from handle_tracking.srv import objectfinder
from tmc_planning_msgs.srv import PlanWithTsrConstraints
from tmc_planning_msgs.srv import PlanWithTsrConstraintsRequest
from tmc_planning_msgs.msg import TaskSpaceRegion
from tmc_manipulation_msgs.msg import (
    BaseMovementType,
    ArmManipulationErrorCodes
)

_TF_TIMEOUT = 3.5
_ODOM_TF = 'map'
# _ORIGIN_TF ='map'
_ORIGIN_TF ='map'
_ROBOT_TF = 'base_footprint'
_HAND_TF = 'hand_palm_link'

BASE_STATE_TOPIC = "/hsrb/omni_base_controller/state"
BASE_GOAL_TOPIC = "/move_base/move/goal"

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
HANDLE_TO_HAND_POS = 0.067
HANDLE_GOAL_OFFSET = 0.5
HANDLE_TO_HAND_POS_X = 0.04
HANDLE_TO_HAND_POS_Y=0.012

recog_pos = geometry_msgs.msg.PoseStamped()

cur_arm_lift_joint=0.0
cur_wrist_roll_joint=0.0

# recog_pos.pose.position.x=0.30
# recog_pos.pose.position.y=0.2
# recog_pos.pose.position.z=0.947

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
def grasp_point_client():
    try:
        print "calling grasping point service"
        localize_handle = rospy.ServiceProxy('/track_handle',objectfinder)
        global Is_found
        Is_found=False
        while(Is_found==False):
            Response=localize_handle()
            Is_found =Response.handle_is_found
            print Response.best_grasp_pose
            print Is_found
            if Is_found:
                global recog_pos
                # recog_pos=Response.best_grasp_pose
                recog_pos.pose.position.x=Response.best_grasp_pose.pose.position.x
                recog_pos.pose.position.y=Response.best_grasp_pose.pose.position.y
                recog_pos.pose.position.z=Response.best_grasp_pose.pose.position.z
                recog_pos.pose.orientation.x=Response.best_grasp_pose.pose.orientation.x
                recog_pos.pose.orientation.y=Response.best_grasp_pose.pose.orientation.y
                recog_pos.pose.orientation.z=Response.best_grasp_pose.pose.orientation.z
                recog_pos.pose.orientation.w=Response.best_grasp_pose.pose.orientation.w
            

    except rospy.ServiceException, e:
        print "service grasp point failed"
        
    

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

def listnerfunction():
    # rospy.init_node('listener',anonymous=True)
    # rospy.Subscriber("handle_detector/grasp_point",PoseStamped,grasp_point_callback)
    rospy.Subscriber("hsrb/joint_states",JointState,joint_states_callback)

def main(whole_body, base, gripper,wrist_wrench):
    
    cli = actionlib.SimpleActionClient('/hsrb/omni_base_controller/follow_joint_trajectory', control_msgs.msg.FollowJointTrajectoryAction)
    # publisher for delvering command for base move
    vel_pub = rospy.Publisher('/hsrb/command_velocity', geometry_msgs.msg.Twist, queue_size=10)
    # base_pub = rospy.Publisher('/move_base/move/goal',move_base_msgs.msg.MoveBaseActionGoal,queue_size=10)
    
    armPub = rospy.Publisher('/hsrb/arm_trajectory_controller/command', JointTrajectory, queue_size=1)
    ## Grab the handle of door
    
    #test with service - get the handle position from handle
    grasp_point_client()

    global recog_pos
    global Is_found

    print recog_pos.pose.position
    # target_pose_Msg = rospy.wait_for_message("/handle_detector/grasp_point", PoseStamped)
    # recog_pos.pose.position.x=target_pose_Msg.pose.position.x
    # recog_pos.pose.position.y=target_pose_Msg.pose.position.y
    # recog_pos.pose.position.z=target_pose_Msg.pose.position.z



    whole_body.move_to_neutral()
    # whole_body.impedance_config= 'grasping'
    switch = ImpedanceControlSwitch() 
    # wrist_wrench.reset()
    gripper.command(1.0)

    grab_pose = geometry.multiply_tuples(geometry.pose(x=recog_pos.pose.position.x-HANDLE_TO_HAND_POS_X,
                                                       y=recog_pos.pose.position.y-HANDLE_TO_HAND_POS_Y,
                                                       z=recog_pos.pose.position.z,
                                                       ej=math.pi/2),
                                         geometry.pose(ek=math.pi/2))
    whole_body.move_end_effector_pose(grab_pose, _ORIGIN_TF)
    wrist_wrench.reset()
    # whole_body.impedance_config= 'compliance_middle'
    switch.activate("grasping")
    # gripper.command(0.01)
    gripper.grasp(-0.01)
    rospy.sleep(1.0)
    switch.inactivate()

    wrist_wrench.reset()
    rospy.sleep(8.0)

    #### test manipulation 
    #change the impedance config to grasping
    whole_body.impedance_config = 'grasping'
    ## Direct Joint trajectory
    traj = JointTrajectory()
    # This controller requires that all joints have values
    traj.joint_names = ["arm_lift_joint", "arm_flex_joint",
                        "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
    p = JointTrajectoryPoint()
    current_positions = [latest_positions[name] for name in traj.joint_names]
    current_positions[0] = latest_positions["arm_lift_joint"]-0.07
    current_positions[1] = latest_positions["arm_flex_joint"]-0.02
    current_positions[2] = latest_positions["arm_roll_joint"]
    current_positions[3] = latest_positions["wrist_flex_joint"]
    current_positions[4] = latest_positions["wrist_roll_joint"]-0.65
    p.positions = current_positions
    p.velocities = [0, 0, 0, 0, 0]
    p.time_from_start = rospy.Time(3)
    traj.points = [p]

    armPub.publish(traj)

    rospy.sleep(3.0)
    
    ## Move by End-effector line
    whole_body.impedance_config = 'compliance_hard'
    whole_body.move_end_effector_by_line((0.0,0.0,1), 0.45)
   
    ## Move base with linear & Angular motion
    tw = geometry_msgs.msg.Twist()
    tw.linear.x =0.9
    tw.angular.z = 0.45
    vel_pub.publish(tw)
    rospy.sleep(4.0)
    
    ## Move base with linear & Angular motion second
    tw_cmd0 = geometry_msgs.msg.Twist()
    tw_cmd0.linear.x =0.3
    tw_cmd0.angular.z = 0.45
    vel_pub.publish(tw_cmd0)
    # rospy.sleep(4.0)   
   
    rospy.sleep(4.0) 

    ## Open the gripper
    gripper.command(1.0)
    
    ## Move back 
    tw_cmd = geometry_msgs.msg.Twist()
    tw_cmd.linear.x =-1.2
    vel_pub.publish(tw_cmd)
    rospy.sleep(2.0) 

    ## Move back  2
    tw_cmd2= geometry_msgs.msg.Twist()
    tw_cmd2.linear.x =-1.1
    tw_cmd2.angular.z =-0.4
    vel_pub.publish(tw_cmd2)
    rospy.sleep(4.0) 
    whole_body.move_to_neutral()
    ## Move back  3
    tw_cmd2= geometry_msgs.msg.Twist()
    tw_cmd2.linear.x =-0.6
    tw_cmd2.angular.z =-0.3
    vel_pub.publish(tw_cmd2)


if __name__=='__main__':
    with hsrb_interface.Robot() as robot:
        whole_body = robot.get('whole_body')
        gripper = robot.get('gripper')
        wrist_wrench = robot.get('wrist_wrench')
        base=robot.get('omni_base')
        listnerfunction()
        main(whole_body,base,gripper,wrist_wrench)
