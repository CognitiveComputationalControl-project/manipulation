#!/usr/bin/python
# -*- coding: utf-8 -*-
import hsrb_interface
from hsrb_interface import geometry
import rospy
import sys
import tf
from tmc_geometry_msgs.msg import Point2DStamped
import controller_manager_msgs.srv
from sensor_msgs.msg import JointState
import trajectory_msgs.msg
import geometry_msgs.msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
from tmc_perspective_transformer.srv import (InversePerspectiveTransform,
                                             InversePerspectiveTransformRequest)
from villa_manipulation.srv import (Opendoor, OpendoorRequest, OpendoorResponse)

_CONNECTION_TIMEOUT = 10.0
_INV_PERSPECTIVE_TRANSFORM_SRV_NAME = '/inverse_perspective_transform'
_OPENDOOR_SRV_NAME = 'open_door_service'
_TARGET_POINT_X = 290.0
_TARGET_POINT_Y = 290.0
_VIEW_POSITIONS = {'head_pan_joint': -1.0,
                   'head_tilt_joint': -0.8,
                   'arm_flex_joint': -0.5}

latest_positions = None
recog_pos = geometry_msgs.msg.PoseStamped()
# recog_pos.pose.position.x=0.40
# recog_pos.pose.position.y=-0.0
# recog_pos.pose.position.z=0.8


# def joint_states_callback(msg):
#     global latest_positions
#     positions = {}
#     for name, i in zip(msg.name, range(len(msg.name))):
#         positions[name] = msg.position[i]
#     latest_positions = positions

def grasp_point_callback(msg):
    recog_pos.pose.position.x=msg.pose.position.x
    recog_pos.pose.position.y=msg.pose.position.y
    recog_pos.pose.position.z=msg.pose.position.z

#   def listnerfunction():
    # rospy.Subscriber("hsrb/joint_states",JointState,joint_states_callback)
    # rospy.Subscriber("handle_detector/grasp_point",PoseStamped,grasp_point_callback)

def mains():
    # Initialize
    rospy.wait_for_service(_OPENDOOR_SRV_NAME)  
    try:
        open_door_client = rospy.ServiceProxy(_OPENDOOR_SRV_NAME,Opendoor)
        # Get the grasp patterns
        open_door_req = OpendoorRequest()
        target_pose_Msg = rospy.wait_for_message("/handle_detector/grasp_point", PoseStamped)
        recog_pos.pose.position.x=target_pose_Msg.pose.position.x
        recog_pos.pose.position.y=target_pose_Msg.pose.position.y
        recog_pos.pose.position.z=target_pose_Msg.pose.position.z


        open_door_req.handle_pose=recog_pos
        print recog_pos
        open_door_req.angle=0.0
        open_door_req.push=True
        try:
            res = open_door_client(open_door_req)    
        except rospy.ServiceException as e:
            rospy.logerr(e)
            exit(1)
        if res.success==False:
            rospy.logerr('Failed')
            exit(1)
            # open_door_client.wait_for_service(timeout=_CONNECTION_TIMEOUT)
    except Exception as e:
        rospy.logerr(e)
        sys.exit(1)

    # whole_body.move_to_neutral()

if __name__ == '__main__':
    # rospy.Subscriber("hsrb/joint_states",JointState,joint_states_callback)
    rospy.Subscriber("handle_detector/grasp_point",PoseStamped,grasp_point_callback)
    mains()
    # with hsrb_interface.Robot() as robot:
        # whole_body = robot.get('whole_body')
        # gripper = robot.get('gripper')
