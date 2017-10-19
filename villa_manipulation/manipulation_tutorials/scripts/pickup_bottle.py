#!/usr/bin/python
# -*- coding: utf-8 -*-
import hsrb_interface
from hsrb_interface import geometry
import rospy
import sys
import tf
from tmc_geometry_msgs.msg import Point2DStamped
from tmc_perspective_transformer.srv import (InversePerspectiveTransform,
                                             InversePerspectiveTransformRequest)
from tmc_interactive_grasp_planner.srv import (GetGraspPattern,
                                               GetGraspPatternRequest)

_CONNECTION_TIMEOUT = 10.0
_INV_PERSPECTIVE_TRANSFORM_SRV_NAME = '/inverse_perspective_transform'
_GRASP_PLANNER_SRV_NAME = '/get_grasp_pattern'
_TARGET_POINT_X = 290.0
_TARGET_POINT_Y = 290.0
_VIEW_POSITIONS = {'head_pan_joint': -1.0,
                   'head_tilt_joint': -0.8,
                   'arm_flex_joint': -0.5}

def main(whole_body, gripper):
    # Initialize
    inv_perspective_transform_client = rospy.ServiceProxy(_INV_PERSPECTIVE_TRANSFORM_SRV_NAME,
                                                          InversePerspectiveTransform)
    grasp_planner_client = rospy.ServiceProxy(_GRASP_PLANNER_SRV_NAME,
                                              GetGraspPattern)
    try:
        inv_perspective_transform_client.wait_for_service(timeout=_CONNECTION_TIMEOUT)
        grasp_planner_client.wait_for_service(timeout=_CONNECTION_TIMEOUT)
    except Exception as e:
        rospy.logerr(e)
        sys.exit(1)

    # Gaze at the target object
    gripper.command(1.0)
    whole_body.move_to_joint_positions(_VIEW_POSITIONS)
    rospy.sleep(1.0)

    # Get the position of the target object from a camera image
    inv_perspective_transform_req = InversePerspectiveTransformRequest()
    target_point = Point2DStamped()
    target_point.point.x = _TARGET_POINT_X
    target_point.point.y = _TARGET_POINT_Y
    inv_perspective_transform_req.points_2D.append(target_point)
    inv_perspective_transform_req.depth_registration = True
    inv_perspective_transform_req.target_frame = 'base_footprint'
    try:
        res = inv_perspective_transform_client(inv_perspective_transform_req)
    except rospy.ServiceException as e:
        rospy.logerr(e)
        exit(1)
    if len(res.points_3D) < 1:
        rospy.logerr('There is no detected point')
        exit(1)
    if res.points_3D[0].point.z < 0.01:
        rospy.logerr('The detected point is ground')
        exit(1)

    # Get the grasp patterns
    grasp_planner_req = GetGraspPatternRequest()
    grasp_planner_req.points.append(res.points_3D[0])
    grasp_planner_req.search_pattern = GetGraspPatternRequest.kAbovePlane
    try:
        res = grasp_planner_client(grasp_planner_req)
    except rospy.ServiceException as e:
        rospy.logerr(e)
        exit(1)
    if len(res.grasp_patterns) < 1:
        rospy.logerr('There is no grasp pattern')
        exit(1)

    # Grasp the object
    target_hand_pose = geometry.multiply_tuples(geometry.pose_to_tuples(res.object_pose.pose),
                                                geometry.pose_to_tuples(res.grasp_patterns[0].hand_frame))
    target_hand_euler = tf.transformations.euler_from_quaternion([
        target_hand_pose.ori.x,
        target_hand_pose.ori.y,
        target_hand_pose.ori.z,
        target_hand_pose.ori.w])
    target_pose = hsrb_interface.geometry.pose(x=target_hand_pose.pos.x,
                                               y=target_hand_pose.pos.y,
                                               z=target_hand_pose.pos.z,
                                               ei=target_hand_euler[0],
                                               ej=target_hand_euler[1],
                                               ek=target_hand_euler[2])
    whole_body.move_end_effector_pose(target_pose)
    gripper.grasp(-0.1)

    # Move to neutral position
    whole_body.move_to_neutral()

if __name__ == '__main__':
    with hsrb_interface.Robot() as robot:
        whole_body = robot.get('whole_body')
        gripper = robot.get('gripper')
        main(whole_body, gripper)
