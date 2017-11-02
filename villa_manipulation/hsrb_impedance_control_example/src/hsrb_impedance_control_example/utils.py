#!/usr/bin/python
# -*- coding: utf-8 -*-

import copy
import threading
import rospy

def extract_joint_positions(joint_states, joint_names):
    u""" joint_statesからjoint_namesの関節角度を取り出し，
         取り出すことができた関節名とその角度を返す
    """
    names = []
    positions = []
    for name in joint_names:
        for index in range(len(joint_states.name)):
            if name == joint_states.name[index]:
                names.append(name)
                positions.append(joint_states.position[index])
                break
    return (names, positions)


def extract_odom_positions(odom_states, joint_names):
    u""" odom_statesからjoint_namesの関節角度を取り出し，
         取り出すことができた関節名とその角度を返す
    """
    names = []
    positions = []
    for name in joint_names:
        for index in range(len(odom_states.joint_names)):
            if name == odom_states.joint_names[index]:
                names.append(name)
                positions.append(odom_states.actual.positions[index])
                break
    return (names, positions)


class CachingSubscriber(object):
    u"""指定したトピックを購読し、最新の値を保持するクラス

    Args:
        topic (str):                       トピック名
        msg_type (rospy.Message):          ROSメッセージ
    Attributes:
        data (msg_type): 保持する最新の値，何も受け取っていなければNoneを返す
    """
    def __init__(self, topic, msg_type):
        self._lock = threading.Lock()
        self._msg = None
        self._topic = topic
        self._msg_type = msg_type
        self._sub = rospy.Subscriber(topic, msg_type, self._callback)
    
    def wait_for_message(self, timeout):
        rate = rospy.Rate(20.0)
        start = rospy.get_rostime()
        while not rospy.is_shutdown():
            if self.data is not None:
                return True
            elif timeout > 0.0:
                if (rospy.get_rostime() - start).to_sec() > timeout:
                    return False
            rate.sleep()

    def _callback(self, msg):
        if self._lock.acquire(False):
            try:
                self._msg = msg
            finally:
                self._lock.release()

    @property
    def data(self):
        with self._lock:
            return copy.deepcopy(self._msg)