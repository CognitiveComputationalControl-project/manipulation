#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import sys

from geometry_msgs.msg import (
    Wrench,
)

from hsrb_interface import Robot, ItemTypes
from hsrb_impedance_control_example.impedance_control_switch import ImpedanceControlSwitch
from hsrb_impedance_control_example.utils import CachingSubscriber

with Robot() as robot:
    # 必要なインスタンスを作成
    wrist_wrench = robot.try_get('wrist_wrench')
    gripper = robot.try_get('gripper')
    switch = ImpedanceControlSwitch()

    # 力覚センサのゼロ点リセット
    # 任意の方向にFFWrenchで進ませるため
    # 本当はリセットせずに，現在のセンサ値をキャンセルする成分をFFWrenchに足すべき
    wrist_wrench.reset()

    # FFWrenchの準備，手先(hand_palm_link)座標系で与える
    # このサンプルでは，-x方向に適当な大きさを与えている
    ff_wrench = Wrench()
    ff_wrench.force.x = -2.0
    
    # インピーダンス制御を有効化
    result = switch.activate("placing", ff_wrench)
    if result:
        print "Activate impedance control."
    else:
        print "Impedance control activation failure."
        sys.exit()

    # 終了判定
    # センサ値が与えたFFWrenchを２回横切ったら止める
    # 本当は全ての方向を考慮すべきだけれど，x軸のみチェック
    # 力覚センサの座標系と手先座標系のx軸方向が逆転していることに注意
    rate = rospy.Rate(50.0)
    counter = 0
    previous = wrist_wrench.wrench[0][0]
    while not rospy.is_shutdown():
        current = wrist_wrench.wrench[0][0]
        if (current - ff_wrench.force.x) * (previous - ff_wrench.force.x) < 0.0:
            counter = counter + 1
        if counter > 1:
            break
        previous = current
        rate.sleep()

    # インピーダンス制御を無効化
    switch.inactivate()
    print "Inactivate impedance control."

    # 手放す
    gripper.command(1.1)