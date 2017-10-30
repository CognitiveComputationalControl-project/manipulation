#! /usr/bin/env python
# -*- coding: utf-8 -*-
'''
@copyright: Copyright(C) 2016, TOYOTA MOTOR CORPORATION
@author: Keisuke Takeshita
'''
import argparse
import rospy
import tf2_ros
import hsrb_grasp_planner.gripper_pose_planner
import hsrb_interface.geometry as geometry
from geometry_msgs.msg import Wrench
# from hsrb_impedance_control_example.impedance_control_switch import ImpedanceControlSwitch
from hsrb_interface import Robot


with Robot() as robot:
    # parser
    parser = argparse.ArgumentParser(description='This script is ...')
    parser.add_argument('points', action='store', nargs=3, type=float,
                        help='Target points [m]')
    parser.add_argument('--ref-frame', action='store', nargs='?', type=str,
                        default='base_footprint',
                        help='Reference frame of points')
    parser.add_argument('--type', action='store', nargs='?', type=str,
                        default='grasp',
                        help='Action type: grasp or suction')
    args, unknown = parser.parse_known_args()
    if args.type != 'grasp' and args.type != 'suction':
        raise RuntimeError("--type is not invalid.")

    # 必要な物を生成
    whole_body = robot.try_get('whole_body')
    suction = robot.try_get('suction')
    gripper = robot.try_get('gripper')
    wrist_wrench = robot.try_get('wrist_wrench')
    # switch = ImpedanceControlSwitch()
    tf2_buffer = tf2_ros.Buffer()
    tf2_listener = tf2_ros.TransformListener(tf2_buffer)
 
    # 把持姿勢の生成,とりあえずは0番目を採用する
    planner = hsrb_grasp_planner.gripper_pose_planner.GripperPosePlanner()
    grasp_patterns = None
    if args.type == 'grasp':
        grasp_patterns = planner.plan_grasp(args.points, args.ref_frame)
    elif args.type == 'suction':
        grasp_patterns = planner.plan_suction(args.points, args.ref_frame)
    else:
        raise RuntimeError()
    poses = grasp_patterns.get_hand_poses(tf2_buffer, geometry.pose(z=-0.04))

    # 手を伸ばす
    whole_body.move_end_effector_pose(poses[0], grasp_patterns.ref_frame)
  
    # 手を降ろす
    wrist_wrench.reset()
    ff_wrench = Wrench()
    ff_wrench.force.z = -2.0
    result = switch.activate("dumper_soft", ff_wrench)
    if not result:
        raise RuntimeError("Failed to activate impedance control.")
    rate = rospy.Rate(50.0)
    counter = 0
    goal = 0
    if args.type == 'grasp':
        goal = 1
    previous = wrist_wrench.wrench[0][0]
    while not rospy.is_shutdown():
        current = wrist_wrench.wrench[0][0]
        if (current - ff_wrench.force.x) * (previous - ff_wrench.force.x) < 0.0:
            counter = counter + 1
        if counter > goal:
            break
        previous = current
        rate.sleep()
    # switch.inactivate()
  
    # つかむ
    if args.type == 'grasp':
        result = switch.activate("grasping")
        if not result:
            raise RuntimeError("Failed to activate impedance control.")
        gripper.grasp(-0.018)
        # switch.inactivate()
    elif args.type == 'suction':
        suction.command(True)
    else:
        raise RuntimeError()
