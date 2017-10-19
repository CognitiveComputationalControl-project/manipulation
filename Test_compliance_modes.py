#!/usr/bin/python
# -*- coding: utf-8 -*-

import hsrb_interface
import rospy
import sys
from hsrb_interface import geometry

# Move timeout[s]
_MOVE_TIMEOUT=60.0
# Grasp torque[Nm]
_GRASP_TORQUE=float(sys.argv[1])
# TF name of the bottle
_BOTTLE_TF='ar_marker/503'
# TF name of the gripper
_HAND_TF='hand_palm_link'

# Preparation for using the robot functions
robot = hsrb_interface.Robot()
omni_base = robot.get('omni_base')
whole_body = robot.get('whole_body')
whole_body.move_to_neutral()

gripper = robot.get('gripper')
tts = robot.get('default_tts')

if __name__=='__main__':

    rospy.sleep(2.0)

    # try:

    #     gripper.grasp(_GRASP_TORQUE)

    #     # rospy.sleep(2.0)

    #     # whole_body.move_to_neutral()
    # except:
    #     tts.say('Fail to grasp.')
    #     rospy.logerr('fail to grasp')
    #     sys.exit()

    print(whole_body.impedance_config)
    whole_body.impedance_config = 'dumper_soft'
    print(whole_body.impedance_config)
    whole_body.move_to_joint_positions({"arm_flex_joint":-0.8,"arm_lift_joint":0.5,"wrist_flex_joint":-0.6})
    rospy.sleep(20.0)
    whole_body.move_to_neutral()

    print(whole_body.impedance_config)
    whole_body.impedance_config = 'dumper_hard'
    print(whole_body.impedance_config)
    whole_body.move_to_joint_positions({"arm_flex_joint":-0.8,"arm_lift_joint":0.5,"wrist_flex_joint":-0.6})
    rospy.sleep(3.0)
    whole_body.move_to_neutral()

    whole_body.impedance_config = 'grasping'
    print(whole_body.impedance_config)
    whole_body.move_to_joint_positions({"arm_flex_joint":-0.8,"arm_lift_joint":0.5,"wrist_flex_joint":-0.6})
    rospy.sleep(3.0)
    whole_body.move_to_neutral()

    whole_body.impedance_config = 'placing'
    print(whole_body.impedance_config)
    whole_body.move_to_joint_positions({"arm_flex_joint":-0.8,"arm_lift_joint":0.5,"wrist_flex_joint":-0.6})
    rospy.sleep(3.0)
    whole_body.move_to_neutral()
    
    whole_body.impedance_config = 'compliance_soft'
    print(whole_body.impedance_config)
    whole_body.move_to_joint_positions({"arm_flex_joint":-0.8,"arm_lift_joint":0.5,"wrist_flex_joint":-0.6})
    rospy.sleep(3.0)
    whole_body.move_to_neutral()

