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
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
from tmc_perspective_transformer.srv import (InversePerspectiveTransform,
                                             InversePerspectiveTransformRequest)
from villa_manipulation.srv import (Opendoor, OpendoorRequest, OpendoorResponse)
from villa_navi_service.srv import (GoTargetName, GoTargetNameRequest)

_CONNECTION_TIMEOUT = 10.0
_INV_PERSPECTIVE_TRANSFORM_SRV_NAME = '/inverse_perspective_transform'
_OPENDOOR_SRV_NAME = 'open_door_service'
_NAV_SRV_NAME = 'waypoint_go'
_TARGET_POINT_X = 290.0
_TARGET_POINT_Y = 290.0
_VIEW_POSITIONS = {'head_pan_joint': -1.0,
                   'head_tilt_joint': -0.8,
                   'arm_flex_joint': -0.5}

# recog_pos = geometry_msgs.msg.PoseStamped()
# waypoint_pub = rospy.Publisher("/way_point", String, queue_size=1)

class TaskManager(object):
        def __init__(self, wait=0.0):
            self.waypoint_pub=rospy.Publisher("way_point", String, queue_size=1)
            self.recog_pos = geometry_msgs.msg.PoseStamped()

        def waypoint_navi_service(self,goal_str):
            waypoint_client = rospy.ServiceProxy(_NAV_SRV_NAME,GoTargetName)
            waypoint_req = GoTargetNameRequest()
            waypoint_req.loc_name=  goal_str
            try:
                res = waypoint_client(waypoint_req)
            except rospy.ServiceException as e:
                rospy.logerr(e)
                exit(1)

	# def listener(self):
            # rospy.spin()

        def mains(self):
            # Initialize
            # rospy.wait_for_service(_NAV_SRV_NAME)  
            self.waypoint_navi_service("door")
            rospy.sleep(35)
            # rospy.wait_for_service(_OPENDOOR_SRV_NAME)  
            # whole_body.move_to_neutral()
            try:
                print "calling grasp"
                open_door_client = rospy.ServiceProxy(_OPENDOOR_SRV_NAME,Opendoor)
                # Get the grasp patterns
                open_door_req = OpendoorRequest()

                self.recog_pos.pose.position.x=0.0
                self.recog_pos.pose.position.y=0.0
                self.recog_pos.pose.position.z=0.0
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

            self.waypoint_navi_service("home_pos")

if __name__ == '__main__':
    rospy.init_node('opendoor_client_node')
    task_manager = TaskManager()
    print("Object created")
    task_manager.mains()

