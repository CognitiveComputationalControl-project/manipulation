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
from tf import TransformListener
import geometry_msgs.msg
from sensor_msgs.msg import JointState
import moveit_commander
import trajectory_msgs.msg
import moveit_msgs
from copy import deepcopy
from villa_navi_service.srv import GoTargetPos
from villa_navi_service.srv import GoTargetPosRequest
import controller_manager_msgs.srv
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
from handle_tracking.srv import objectfinder
import move_base_msgs.msg
from move_base_msgs.msg import MoveBaseActionGoal
from tmc_planning_msgs.srv import PlanWithTsrConstraints
from tmc_planning_msgs.srv import PlanWithTsrConstraintsRequest
from tmc_planning_msgs.msg import TaskSpaceRegion
from tmc_manipulation_msgs.msg import (
	BaseMovementType,
	ArmManipulationErrorCodes
)

import roslib
# roslib.load_manifest('villa_manipulation')
import actionlib

from villa_manipulation.msg import OpenDoorAction

_TF_TIMEOUT = 3.5
_ODOM_TF = 'map'
# _ORIGIN_TF ='map'
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

HANDLE_POS = (5.1, 0.51, 0.947)
RECOG_HANDLE_POS = (0.0, 0.0, 0.0)
HANDLE_TO_DOOR_HINGE_POS = 0.61
HANDLE_TO_HANDLE_HINGE_POS = -0.06
HANDLE_TO_HAND_POS = 0.067
HANDLE_GOAL_OFFSET = 0.5
HANDLE_TO_HAND_POS_X = 0.04
HANDLE_TO_HAND_POS_Y=0.012


recog_pos = geometry_msgs.msg.PoseStamped()

latest_positions = None

class SetRobot:

    def __init__(self):
        print("Getting robot infos")
        self.rob = Robot()
        try:
            self.whole_body = self.rob.get('whole_body')
            self.omni_base = self.rob.get('omni_base')
            self.gripper = self.rob.get('gripper')
            self.whole_body.end_effector_frame = u'hand_palm_link'
            print("Infos received")
        except Exception as e:
            print("Fail")

class OpenDoorServer:

    def __init__(self):
        self.server = actionlib.SimpleActionServer('open_door', OpenDoorAction, self.execute, False)
        self.server.start()
        print("Server starting")

    def execute(self, goal):
        print("Start action")
    	# main(whole_body,  gripper,wrist_wrench)
    	# frame = goal.handle_pose.header.frame_id
    	# hanlde_pos = goal.handle_pose.pose
    	# hanlde_pos=geometry_msgs.msg.PoseStamped()
        cli = actionlib.SimpleActionClient('/hsrb/omni_base_controller/follow_joint_trajectory', control_msgs.msg.FollowJointTrajectoryAction)
        vel_pub = rospy.Publisher('/hsrb/command_velocity', geometry_msgs.msg.Twist, queue_size=10)
        armPub = rospy.Publisher('/hsrb/arm_trajectory_controller/command', JointTrajectory, queue_size=1)
	
        robot = hsrb_interface.Robot()
        whole_body = robot.get('whole_body')
        gripper = robot.get('gripper')
        wrist_wrench = robot.get('wrist_wrench')
        base=robot.get('omni_base')
        start_theta=base.pose[2]
    	# with hsrb_interface.Robot() as robot:
                # whole_body = robot.get('whole_body')
                # gripper = robot.get('gripper')
                # wrist_wrench = robot.get('wrist_wrench')

    	try: 
    		# Open the gripper 
            whole_body.move_to_neutral()
            grasp_point_client()
            global recog_pos
            global is_found
            print recog_pos.pose.position

            print("Opening the gripper")
            whole_body.move_to_neutral()
            rospy.sleep(2.5)
            switch = ImpedanceControlSwitch() 
            gripper.command(1.0)

            # Approach to the door
            print("Approach to the door")
            grab_pose = geometry.multiply_tuples(geometry.pose(x=recog_pos.pose.position.x-HANDLE_TO_HAND_POS_X,
                        y=recog_pos.pose.position.y-HANDLE_TO_HAND_POS_Y,
                        z=recog_pos.pose.position.z,
                        ej=math.pi/2),
                        geometry.pose(ek=math.pi/2))

            whole_body.move_end_effector_pose(grab_pose, _ORIGIN_TF)
            # Close the gripper
            wrist_wrench.reset()
            switch.activate("grasping")
            gripper.grasp(-0.01)
            rospy.sleep(1.0)
            switch.inactivate()
    		# Rotate the handle
            whole_body.impedance_config = 'grasping'
            traj = JointTrajectory()

            # This controller requires that all joints have values
            traj.joint_names = ["arm_lift_joint", "arm_flex_joint",
                                                    "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
            p = JointTrajectoryPoint()
            current_positions = [latest_positions[name] for name in traj.joint_names]
            current_positions[0] = latest_positions["arm_lift_joint"]-0.0675
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
            print("finishing rotating handle") 
            ## Move by End-effector line
            whole_body.impedance_config = 'compliance_hard'
            whole_body.move_end_effector_by_line((0.0,0.0,1), 0.45)

            print("push the door") 
            ## Move base with linear & Angular motion
            tw = geometry_msgs.msg.Twist()
            tw.linear.x =0.9
            tw.angular.z = 0.45
            vel_pub.publish(tw)
            rospy.sleep(4.0)

            ## Move base with linear & Angular motion second
            tw_cmd0 = geometry_msgs.msg.Twist()
            tw_cmd0.linear.x =0.5
            tw_cmd0.angular.z = 0.45
            vel_pub.publish(tw_cmd0)
            rospy.sleep(2.0) 

            tw_cmd1 = geometry_msgs.msg.Twist()
            tw_cmd1.linear.x =0.6
            tw_cmd1.linear.y =0.2
            tw_cmd1.angular.z = 0.25
            vel_pub.publish(tw_cmd1)
            # rospy.sleep(4.0)   
            rospy.sleep(3.0) 

            tw_cmd2 = geometry_msgs.msg.Twist()
            tw_cmd2.linear.x =0.65
            tw_cmd2.linear.y =0.5
            tw_cmd2.angular.z = 0.35
            vel_pub.publish(tw_cmd2)
            # rospy.sleep(4.0)   
            rospy.sleep(2.0) 

            ## Open the gripper
            gripper.command(1.0)

            ## Move back 
            base.go_rel(-1.3,0.0,start_theta)

            gripper.command(1.0)
            whole_body.move_to_neutral()
            self.server.set_succeeded(True)

        except Exception as e:
            rospy.logerr(e)
            print "Failed to open door"
            self.server.set_succeeded(False)

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
    print("Publishing trajectory")
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

def grasp_point_client():
    print("trying grasping point client")
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
        
def navi_service_client(x_,y_,theta_):
    try:
        print "calling navi service"
        nav_srv_client = rospy.ServiceProxy("navi_go_base", GoTargetPos)
        nav_req=GoTargetPosRequest()
        nav_req.x_from_map=x_
        nav_req.y_from_map=y_
        nav_req.theta_from_map=theta_
        nav_srv_client(nav_req)

    except rospy.ServiceException, e:
        print "navi service failed"


def joint_states_callback(msg):
    global latest_positions
    positions = {}
    for name, i in zip(msg.name, range(len(msg.name))):
        positions[name] = msg.position[i]
    latest_positions = positions


if __name__=='__main__':
    print("Initializing node")
    # rospy.init_node('open_door_server')

    global robot
    print("Initializing Robot")
    robot = SetRobot()
    OpenDoorServer()
    rospy.spin()
