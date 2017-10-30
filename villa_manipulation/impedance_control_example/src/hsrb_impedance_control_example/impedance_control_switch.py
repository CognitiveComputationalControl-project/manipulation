#!/usr/bin/python
# -*- coding: utf-8 -*-

import exceptions
import utils
import actionlib
import rospy

from geometry_msgs.msg import (
    Wrench,
)
from trajectory_msgs.msg import (
    JointTrajectory,
    JointTrajectoryPoint,
)
from sensor_msgs.msg import (
    JointState,
)
from control_msgs.msg import (
    JointTrajectoryControllerState,
)
from tmc_manipulation_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)

_ARM_JOINT_NAMES = ["arm_lift_joint",
                    "arm_flex_joint",
                    "arm_roll_joint",
                    "wrist_flex_joint",
                    "wrist_roll_joint"]
_ODOM_JOINT_NAMES = ["odom_x", "odom_y", "odom_t"]


class ImpedanceControlSwitch(object):
    def __init__(self):
        self._client = actionlib.SimpleActionClient(
            "/hsrb/impedance_control/follow_joint_trajectory_with_config",
            FollowJointTrajectoryAction)
        self._joint_states_cache = utils.CachingSubscriber(
            "/hsrb/joint_states", JointState)
        self._odom_states_cache = utils.CachingSubscriber(
            "/hsrb/omni_base_controller/state", JointTrajectoryControllerState)
        if not self._joint_states_cache.wait_for_message(10.0):
            raise exceptions.RuntimeError(
                "Failed to subscribe /hsrb/joint_states")
        if not self._odom_states_cache.wait_for_message(10.0):
            raise exceptions.RuntimeError(
                "Failed to subscribe /hsrb/omni_base_controller/state")
        self._client.wait_for_server(rospy.Duration(10.0))

    def activate(self, config, ff_wrench=Wrench()):
        # パラメータが存在するかチェック
        configs = rospy.get_param("/hsrb/impedance_control/config_names")
        if config not in configs:
            rospy.logerr(config + "is invalid name.")
            return False

        # 現在の姿勢から非常に長い（１日分）現在姿勢の軌道を作成
        arm_joint, arm_value = utils.extract_joint_positions(
            self._joint_states_cache.data, _ARM_JOINT_NAMES)
        odom_joint, odom_value = utils.extract_odom_positions(
            self._odom_states_cache.data, _ODOM_JOINT_NAMES)

        trajectory = JointTrajectory()
        trajectory.header.stamp = rospy.get_rostime()
        trajectory.joint_names = arm_joint + odom_joint

        trajectory_point = JointTrajectoryPoint()
        trajectory_point.time_from_start = rospy.Duration(86400.0);
        trajectory_point.positions = arm_value + odom_value
        trajectory.points = [trajectory_point]

        # 軌道を投げる
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory
        goal.preset_parameters_name = config
        goal.feed_forward_wrench.wrench = ff_wrench
        self._client.send_goal(goal)
        return True

    def inactivate(self):
        if self._client.simple_state != actionlib.SimpleGoalState.DONE:
            self._client.cancel_goal()
