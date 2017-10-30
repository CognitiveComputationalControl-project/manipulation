#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from hsrb_interface import Robot, ItemTypes
from hsrb_impedance_control_example.impedance_control_switch import ImpedanceControlSwitch

with Robot() as robot:
    wrist_wrench = robot.try_get('wrist_wrench')
    gripper = robot.try_get('gripper')
    switch = ImpedanceControlSwitch()

    wrist_wrench.reset()

    result = switch.activate("grasping")
    if result:
        print "Activate impedance control."
    else:
        print "Impedance control activation failure."

    print "Grasp object."
    gripper.grasp(-0.018)

    switch.inactivate()
    # print "Inactivate impedance control."
