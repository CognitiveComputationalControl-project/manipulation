#!/usr/bin/env python
import math
import sys

import rospy
import geometry_msgs.msg
import moveit_commander
import trajectory_msgs.msg
import moveit_msgs
from copy import deepcopy



class MoveItCartesianDemo(object):
    def __init__(self, wait=0.0):
        # initialize
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("moveit_demo", anonymous=True)

        arm = moveit_commander.MoveGroupCommander("arm")
        base = moveit_commander.MoveGroupCommander("base")
        head = moveit_commander.MoveGroupCommander("head")
        whole_body = moveit_commander.MoveGroupCommander("whole_body")
        gripper = moveit_commander.MoveGroupCommander("gripper")
        gripper.set_goal_joint_tolerance(0.05)
        scene = moveit_commander.PlanningSceneInterface()
        rospy.sleep(1)

        # open gripper
        gripper.set_joint_value_target("hand_motor_joint", 1.2)
        gripper.go()
        rospy.loginfo("step1: open")
        rospy.sleep(wait)

        # close gripper
        gripper.set_joint_value_target("hand_motor_joint", 0.0)
        gripper.go()
        rospy.loginfo("step2: close")
        rospy.sleep(wait)

        # open again
        gripper.set_joint_value_target("hand_motor_joint", 1.2)
        gripper.go()
        rospy.loginfo("step3: open")
        rospy.sleep(wait)

        # grasp (no planning)
        t = moveit_msgs.msg.RobotTrajectory()
        t.joint_trajectory.joint_names = ["hand_motor_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [0.0]
        p.effort = [-0.01]
        p.time_from_start = rospy.Duration(1.0)
        t.joint_trajectory.points.append(p)
        gripper.execute(t)
        rospy.loginfo("step4: grasp")
        rospy.sleep(wait)        
        

        # remove all objects
        end_effector_link = whole_body.get_end_effector_link()
        scene.remove_attached_object(end_effector_link)
        scene.remove_world_object()
        rospy.sleep(1)

        # move_to_neutral
        rospy.loginfo("step5: move_to_neutral")
        base.go()
        arm.set_named_target("neutral")
        arm.go()
        head.set_named_target("neutral")
        head.go()
        rospy.logdebug("done")
        rospy.sleep(wait)

        # add wall
        table_size = [0.01, 0.6, 0.6]
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = "odom"
        p.pose.position.x = 0.5
        p.pose.position.y = 0.0
        p.pose.position.z = 0.6
        p.pose.orientation.w = 1.0
        scene.add_box("wall", p, table_size)
        rospy.sleep(1)

        # draw circle
        rospy.loginfo("step6: draw circle")
        waypoints = []
        p = geometry_msgs.msg.Pose()
        p.position.x = 0.4
        p.orientation.x = 0.707
        p.orientation.y = 0
        p.orientation.z = 0.707
        p.orientation.w = 0

        r = 0.2
        num_waypoints = 30
        for t in range(num_waypoints):
            th = math.pi * 2.0 / num_waypoints * t
            p.position.z = r * math.cos(th) + 0.6
            p.position.y = r * math.sin(th)
            waypoints.append(deepcopy(p))

        (plan, fraction) = whole_body.compute_cartesian_path(waypoints,
                                                             0.01,
                                                             0.0,
                                                             True)
        whole_body.execute(plan)
        rospy.logdebug("done")
        rospy.sleep(wait)

        # finalize
        scene.remove_attached_object(end_effector_link)
        scene.remove_world_object()
        rospy.sleep(1)
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


if __name__ == "__main__":
    MoveItCartesianDemo(1)
