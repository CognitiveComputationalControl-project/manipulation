import math
from hsrb_interface import geometry
import rospy
import tf2_geometry_msgs
import geometry_msgs.msg


class BasicGraspPlanner:

    def __init__(self):
        pass

    @staticmethod
    def get_grasp(object, style='side'):

        # Planning wrt /base_link frame
        thres = 0.05
        if style == 'top':
            target_pose = geometry.pose(x=object.bbox_pose.pose.position.x,
                                        y=object.bbox_pose.pose.position.y,
                                        z=object.bbox_pose.pose.position.z + object.bbox_height / 2 + thres,
                                        ei=3.14,
                                        ej=0.,
                                        ek=0.)
        else:
            target_pose = geometry.pose(x=object.bbox_pose.pose.position.x - object.bbox_width / 2 - thres,
                                        y=object.bbox_pose.pose.position.y,
                                        z=object.bbox_pose.pose.position.z,
                                        ei=3.14,
                                        ej=-1.57,
                                        ek=0.)

        return target_pose

    @staticmethod
    def gripper_width_for_object_dimension(dimension):
        if dimension < 0.032:
            return 0.3
        elif dimension < 0.042:
            return 0.4
        elif dimension < 0.062:
            return 0.6
        elif dimension < 0.092:
            return 0.8
        elif dimension < 0.112:
            return 1.0
        else:
            return 1.2

    @staticmethod
    def get_grasp_for_pose(object, distance_from_object_side=0.04):
        MAX_GRASPABLE_WIDTH = 0.12
        MAX_GRASPABLE_LENGTH = 0.12
        MIN_SIDE_GRASPABLE_HEIGHT = 0.04
        GRIPPER_TO_PALM_LENGTH = 0.08
        MIN_TOP_GRASPABLE_HEIGHT= 0.03
        TOP_GRASP_DIST_TO_OBJECT_ADJUST = -0.04

        possible_styles = {"top", "side"}

        if object.width > MAX_GRASPABLE_WIDTH:
            rospy.loginfo("Too wide to top grasp")
            possible_styles.discard("top")

        if object.length > MAX_GRASPABLE_LENGTH:
            rospy.loginfo("Too long to side grasp")
            possible_styles.discard("side")

        if object.height < MIN_SIDE_GRASPABLE_HEIGHT:
            rospy.loginfo("Too high to side grasp")
            possible_styles.discard("side")

        if object.height < MIN_TOP_GRASPABLE_HEIGHT:
            rospy.loginfo("Too low to top grasp")
            possible_styles.discard("top")

        if "side" in possible_styles:
            style = "side"
            # The robot approaches from the negative x direction
            delta_x = -(object.width / 2 + distance_from_object_side)
            delta_y = 0
            target_pose = geometry.pose(x=delta_x, y=delta_y, z=object.height/4, ei=3.14, ej=-1.57, ek=0)
            gripper_width = BasicGraspPlanner.gripper_width_for_object_dimension(object.width)
            # Z axis extends from the palm

        elif "top" in possible_styles:
            style = "top"
            delta_x = 0
            delta_y = 0
            delta_z = object.height / 2 + distance_from_object_side + TOP_GRASP_DIST_TO_OBJECT_ADJUST
            target_pose = geometry.pose(x=delta_x, y=delta_y, z=delta_z, ei=3.14, ej=0., ek=1.57)
            gripper_width = BasicGraspPlanner.gripper_width_for_object_dimension(object.length)
        else:
            return None, None, None

        return style, target_pose, gripper_width

    @staticmethod
    def get_grasp_for_pose_in_base_frame(object, robot_pose, obj_transform, aligned_axis='x',  distance_from_object_side=0.08):
        MAX_GRASPABLE_WIDTH = 0.13
        MAX_GRASPABLE_LENGTH = 0.13
        MIN_SIDE_GRASPABLE_HEIGHT = 0.04
        GRIPPER_TO_PALM_LENGTH = 0.08
        MIN_TOP_GRASPABLE_HEIGHT= 0.02
        TOP_GRASP_DIST_TO_OBJECT_ADJUST = -0.04
        
        possible_styles = {"top", "side"}

        if object.width > MAX_GRASPABLE_WIDTH:
            rospy.loginfo("Too wide to top grasp")
            possible_styles.discard("top")

        if object.length > MAX_GRASPABLE_LENGTH:
            rospy.loginfo("Too long to side grasp")
            possible_styles.discard("side")

        if object.height < MIN_SIDE_GRASPABLE_HEIGHT:
            rospy.loginfo("Too high to side grasp")
            possible_styles.discard("side")

        if object.height < MIN_TOP_GRASPABLE_HEIGHT:
            rospy.loginfo("Too low to top grasp")
            possible_styles.discard("top")

        if "side" in possible_styles:
            style = "side"
        elif "top" in possible_styles:
            style = "top"
        else:
            return None, None, None

        # Planning wrt /base_link frame
        ps = geometry_msgs.msg.PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = 'map'
        ps.pose.position.x = robot_pose[0]
        ps.pose.position.y = robot_pose[1]
        ps.pose.position.z = 0
        ps.pose.orientation.x = object.bbox.pose.orientation.x
        ps.pose.orientation.y = object.bbox.pose.orientation.y
        ps.pose.orientation.z = object.bbox.pose.orientation.z
        ps.pose.orientation.w = object.bbox.pose.orientation.w

        pt = tf2_geometry_msgs.do_transform_pose(ps, obj_transform)

        dist = (pt.pose.position.x, pt.pose.position.y)

        theta = 0.0

        if style == "side":
            # The robot approaches from the negative x direction
            delta_x = object.x- (object.width / 2 + distance_from_object_side)
            delta_y = object.y
            delta_z = object.z + object.height / 4
            target_pose = geometry.pose(x=delta_x, y=delta_y, z=delta_z, ei=3.14, ej=-1.57, ek=0)
            gripper_width = BasicGraspPlanner.gripper_width_for_object_dimension(object.width)
            # Z axis extends from the palm
        else:
            delta_x = object.x
            delta_y = object.y
            delta_z = object.z + object.height / 2 + distance_from_object_side + TOP_GRASP_DIST_TO_OBJECT_ADJUST
            target_pose = geometry.pose(x=delta_x, y=delta_y, z=delta_z, ei=3.14, ej=0., ek=1.57)
            gripper_width = BasicGraspPlanner.gripper_width_for_object_dimension(object.length)


        return style, target_pose, gripper_width

