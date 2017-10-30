#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
@copyright: Copyright(C) 2016, TOYOTA MOTOR CORPORATION
@author: Keisuke Takeshita
"""
import rospy
from tmc_interactive_grasp_planner.srv import (
    GetGraspPattern,
    GetGraspPatternRequest,
)
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from hsrb_interface.utils import CachingSubscriber
import hsrb_interface.geometry as geometry
from tf import transformations as tr

_ROBOT_BASE = "base_footprint"
_HAND_JOINT = "hand_motor_joint"
_HAND_FRAME = "hand_palm_link"
_SUCTION_FRAME = "hand_l_finger_vacuum_frame"

def _distance(pose):
    pos, ori = pose
    return pos.x * pos.x + pos.y * pos.y + pos.z * pos.z

def _inverse_tuples(t):
    trans, rot = t
    euler = tr.euler_from_quaternion(rot)
    m = tr.compose_matrix(translate=trans, angles=euler)
    m_inv = tr.inverse_matrix(m)
    trans_inv = tr.translation_from_matrix(m_inv)
    rot_inv = tr.rotation_matrix(*tr.rotation_from_matrix(m_inv))
    quat_inv = tr.quaternion_from_matrix(rot_inv)
    return (trans_inv, quat_inv)

class GraspPattern(object):
    def __init__(self, approach_frame, object_pose, grasp_pattern_msgs):
        self._ref_frame = object_pose.header.frame_id
        self._approach_frame = approach_frame
        self._ref_to_object = geometry.pose_to_tuples(object_pose.pose)
        object_to_hands = []
        for grasp_pattern in grasp_pattern_msgs:
            object_to_hands.append(
                geometry.pose_to_tuples(grasp_pattern.hand_frame))
        self._object_to_hands = sorted(object_to_hands, key=_distance)

    def get_hand_poses(self, tf_buffer, offset=geometry.pose()):
        ref_to_hands = []
        for object_to_hand in self._object_to_hands:
            ref_to_hands.append(geometry.multiply_tuples(
                self._ref_to_object, object_to_hand))

        if self._approach_frame == _HAND_FRAME:
            ref_to_hand_offsets = []
            for ref_to_hand in ref_to_hands:
                ref_to_hand_offsets.append(geometry.multiply_tuples(
                    ref_to_hand, offset))
            return ref_to_hand_offsets
        else:
            hand_to_approach_tf = tf_buffer.lookup_transform(
                _HAND_FRAME,
                self._approach_frame,
#                rospy.get_rostime(),
                rospy.Time(0),
                rospy.Duration(1.0))
            hand_to_approach = geometry.transform_to_tuples(
                hand_to_approach_tf.transform)
            ref_to_hand_offsets = []
            for ref_to_hand in ref_to_hands:
                ref_to_approach = geometry.multiply_tuples(ref_to_hand, hand_to_approach)
                ref_to_approach_offset = geometry.multiply_tuples(ref_to_approach, offset)
                ref_to_hand_offset = geometry.multiply_tuples(ref_to_approach_offset, _inverse_tuples(hand_to_approach))
                ref_to_hand_offsets.append(ref_to_hand_offset)
            return ref_to_hand_offsets

    @property
    def ref_frame(self):
        return self._ref_frame

def _to_point_stamped_msg(point, ref_frame_id):
    ret_val = PointStamped()
    ret_val.header.frame_id = ref_frame_id
    ret_val.header.stamp = rospy.get_rostime()
    ret_val.point.x = point[0]
    ret_val.point.y = point[1]
    ret_val.point.z = point[2]
    return ret_val

class GripperPosePlanner(object):
    def __init__(self):
        rospy.wait_for_service("/get_grasp_pattern")
        self._planner_srv = rospy.ServiceProxy("/get_grasp_pattern",
                                               GetGraspPattern)
        self._joint_state_sub = CachingSubscriber("/hsrb/joint_states",
                                                  JointState,
                                                  default=JointState())
        self._joint_state_sub.wait_for_message()

    def plan_grasp(self, point, ref_frame_id=_ROBOT_BASE, finger_angle=None):
        req = self._create_request(point, ref_frame_id, finger_angle,
                                   GetGraspPatternRequest.kAbovePlane)
        res = self._planner_srv.call(req)
        return GraspPattern(_HAND_FRAME,
                            res.object_pose,
                            res.grasp_patterns)

    def plan_suction(self, point, ref_frame_id=_ROBOT_BASE, finger_angle=None):
        req = self._create_request(point, ref_frame_id, finger_angle,
                                   GetGraspPatternRequest.kPointPush)
        req.target_hand_frame = _SUCTION_FRAME
        req.target_hand_direction.x = 0.0
        req.target_hand_direction.y = 0.0
        req.target_hand_direction.z = 1.0
        res = self._planner_srv.call(req)
        return GraspPattern(_SUCTION_FRAME,
                            res.object_pose,
                            res.grasp_patterns)

    def _create_request(self, point, ref_frame_id, finger_angle, type):
        req = GetGraspPatternRequest()
        req.points.append(_to_point_stamped_msg(point, ref_frame_id))
        req.search_pattern = type
        req.do_search_all_preshape_angles = False
        preshape = JointState()
        preshape.name.append(_HAND_JOINT)
        if finger_angle is None:
            state = self._joint_state_sub.data
            index = state.name.index(_HAND_JOINT)
            preshape.position.append(state.position[index])
        else:
            preshape.position.append(finger_angle)
        req.preshape_angles.append(preshape)
        return req
