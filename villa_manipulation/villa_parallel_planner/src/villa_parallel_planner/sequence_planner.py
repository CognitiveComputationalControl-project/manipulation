import rospy

import tf.transformations as T
import numpy as np

from tmc_msgs.msg import ObjectIdentifier
from tmc_manipulation_msgs.msg import ArmManipulationErrorCodes, BaseMovementType
from tmc_planning_msgs.srv import PlanWithHandGoals, PlanWithHandGoalsRequest
from tmc_planning_msgs.srv import PlanWithHandLine, PlanWithHandLineRequest

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

from villa_parallel_planner.msg import SequencePlan, Action

from hsrb_interface import geometry
from hsrb_interface import trajectory
from hsrb_interface import settings

from copy import deepcopy

_DEBUG = False

# Timeout for motion planning [sec]
_PLANNING_ARM_TIMEOUT = 10.0

# Max number of iteration of moition planning
_PLANNING_MAX_ITERATION = 10000

# Goal generation probability in moition planning
_PLANNING_GOAL_GENERATION = 0.3

# Goal deviation in motion planning
_PLANNING_GOAL_DEVIATION = 0.3

# Timeout to receive a tf message [sec]
_TF_TIMEOUT = 1.0

# Base frame of a mobile base in moition planning
_BASE_TRAJECTORY_ORIGIN = "odom"

class SequencePlanner(object):
    def __init__(self, hand_goal_service, hand_line_service):
        self.hand_goal_service = hand_goal_service
        self.hand_line_service = hand_line_service

        self._end_effector_frame = settings.get_entry('joint_group', 'whole_body')['end_effector_frames'][0]

        self._linear_weight = 3.0
        self._angular_weight = 1.0

        self._use_base_timeopt = True
        self._base_joint_names = ['odom_x', 'odom_y', 'odom_t']

        self._end_effector_joint = settings.get_entry('end_effector', 'gripper')['joint_names'][0]

        self._robot_actions = [Action.POSE, Action.LINE, Action.GRIPPER]
        self._world_actions = [Action.WORLD]

    def _is_robot_action(self, type):
        return type in self._robot_actions

    def _is_world_action(self, type):
        return type in self._world_actions

    def _transform_base_trajectory(self, base_traj, odom_to_frame_pose):
        odom_to_frame = geometry.pose_to_tuples(odom_to_frame_pose)

        num_points = len(base_traj.points)
        odom_base_traj = JointTrajectory()
        odom_base_traj.points = list(utils.iterate(JointTrajectoryPoint,
                                                   num_points))
        odom_base_traj.header = base_traj.header
        odom_base_traj.joint_names = self._base_joint_names

        # Transform each point into odom frame
        previous_theta = 0.0
        for i in range(num_points):
            t = base_traj.points[i].transforms[0]
            frame_to_base = geometry.transform_to_tuples(t)

            # odom_to_base = odom_to_frame * frame_to_base
            (odom_to_base_trans, odom_to_base_rot) = geometry.multiply_tuples(
                odom_to_frame,
                frame_to_base
            )

            odom_base_traj.points[i].positions = [odom_to_base_trans[0],
                                                  odom_to_base_trans[1],
                                                  0]
            roll, pitch, yaw = T.euler_from_quaternion(
                odom_to_base_rot)
            dtheta = geometry.shortest_angular_distance(previous_theta, yaw)
            theta = previous_theta + dtheta

            odom_base_traj.points[i].positions[2] = theta
            previous_theta = theta
        return odom_base_traj

    def _constrain_trajectories(self, joint_trajectory, base_trajectory, odom_to_robot_pose):
        odom_base_trajectory = self._transform_base_trajectory(base_trajectory, odom_to_robot_pose)
        merged_traj = trajectory.merge(joint_trajectory, odom_base_trajectory)

        filtered_merged_traj = trajectory.constraint_filter(merged_traj)
        if not self._use_base_timeopt:
            return filtered_merged_traj

        # Transform arm and base trajectories to time-series trajectories
        filtered_joint_traj = trajectory.constraint_filter(joint_trajectory)
        filtered_base_traj = trajectory.timeopt_filter(odom_base_trajectory)
        last_joint_point = filtered_joint_traj.points[-1]
        last_base_point = filtered_base_traj.points[-1]
        arm_joint_time = last_joint_point.time_from_start.to_sec()
        base_timeopt_time = last_base_point.time_from_start.to_sec()

        # If end of a base trajectory is later than arm one,
        # an arm trajectory is made slower.
        # (1) arm_joint_time < base_timeopt_time: use timeopt trajectory
        if arm_joint_time < base_timeopt_time:
            # Adjusting arm trajectory points to base ones as much as possible
            trajectory.adjust_time(filtered_base_traj, filtered_joint_traj)
            return trajectory.merge(filtered_joint_traj, filtered_base_traj)
        else:
            return filtered_merged_traj

    def _move_end_effector_pose(self, odom_to_hand_pose, odom_to_robot_pose, initial_joint_state, collision_env=None):
        use_joints = (
            b'wrist_flex_joint',
            b'wrist_roll_joint',
            b'arm_roll_joint',
            b'arm_flex_joint',
            b'arm_lift_joint'
        )

        req = PlanWithHandGoalsRequest()
        req.base_movement_type.val = BaseMovementType.PLANAR
        req.origin_to_basejoint = odom_to_robot_pose
        req.initial_joint_state = initial_joint_state
        req.use_joints = use_joints
        req.weighted_joints = [b'_linear_base', b'_rotational_base']
        req.weight = [self._linear_weight, self._angular_weight]
        req.origin_to_hand_goals.append(odom_to_hand_pose)
        req.ref_frame_id = self._end_effector_frame
        req.probability_goal_generate = _PLANNING_GOAL_GENERATION
        req.timeout = rospy.Duration(_PLANNING_ARM_TIMEOUT)
        req.max_iteration = _PLANNING_MAX_ITERATION
        req.uniform_bound_sampling = False
        req.deviation_for_bound_sampling = _PLANNING_GOAL_DEVIATION
        if collision_env is not None:
            req.environment_before_planning = collision_env

        service_name = self.hand_goal_service
        plan_service = rospy.ServiceProxy(service_name,
                                          PlanWithHandGoals)
        res = plan_service.call(req)

        if res.error_code.val != ArmManipulationErrorCodes.SUCCESS:
            msg = "Fail to plan move_endpoint"
            raise exceptions.MotionPlanningError(msg, res.error_code)

        res.base_solution.header.frame_id = settings.get_frame('odom')
        constrained_traj = self._constrain_trajectories(res.solution,
                                                        res.base_solution,
                                                        odom_to_robot_pose)

        return constrained_traj

    def _move_end_effector_by_line(self, axis, distance, odom_to_robot_pose, initial_joint_state, collision_env=None):
        axis_length = np.linalg.norm(np.array(axis, dtype='float64'))

        if axis_length < sys.float_info.epsilon:
            raise ValueError("The axis is zero vector.")

        use_joints = (
            b'wrist_flex_joint',
            b'wrist_roll_joint',
            b'arm_roll_joint',
            b'arm_flex_joint',
            b'arm_lift_joint'
        )

        req = PlanWithHandLineRequest()
        req.base_movement_type.val = BaseMovementType.PLANAR
        req.origin_to_basejoint = odom_to_robot_pose
        req.initial_joint_state = initial_joint_state
        req.use_joints = use_joints
        req.weighted_joints = [b'_linear_base', b'_rotational_base']
        req.weight = [self._linear_weight, self._angular_weight]
        req.axis.x = axis[0]
        req.axis.y = axis[1]
        req.axis.z = axis[2]
        req.local_origin_of_axis = True
        req.ref_frame_id = self._end_effector_frame
        req.goal_value = distance
        req.probability_goal_generate = _PLANNING_GOAL_GENERATION
        req.attached_objects = []
        req.timeout = rospy.Duration(_PLANNING_ARM_TIMEOUT)
        req.max_iteration = _PLANNING_MAX_ITERATION
        req.uniform_bound_sampling = False
        req.deviation_for_bound_sampling = _PLANNING_GOAL_DEVIATION
        req.extra_goal_constraints = []
        if collision_env is not None:
            req.environment_before_planning = collision_env

        service_name = self._hand_line_service
        plan_service = rospy.ServiceProxy(service_name,
                                          PlanWithHandLine)
        res = plan_service.call(req)

        if res.error_code.val != ArmManipulationErrorCodes.SUCCESS:
            msg = "Fail to plan move_hand_line"
            raise exceptions.MotionPlanningError(msg, res.error_code)

        res.base_solution.header.frame_id = settings.get_frame('odom')
        constrained_traj = self._constrain_trajectories(res.solution,
                                                        res.base_solution,
                                                        odom_to_robot_pose)

        return constrained_traj

    def _command_gripper(self, open_angle, motion_time=1.0):
        if motion_time <= 0:
            motion_time = 1

        traj = JointTrajectory()
        traj.joint_names = [self._end_effector_joint]
        traj.points = [JointTrajectoryPoint(positions=[open_angle],
                                            time_from_start=rospy.Duration(motion_time))]
        return traj

    def _remove_object(self, object_id, collision_env):
        new_collision_env = deepcopy(collision_env)
        collision_env.collision_map.subtracted_object_ids.append(deepcopy(object_id))
        return new_collision_env

    def _plan_robot_action(self, action, odom_to_robot_pose, initial_joint_state, collision_env=None):
        if action.type == Action.POSE:
            x = action.odom_to_hand_pose.position.x
            y = action.odom_to_hand_pose.position.y
            z = action.odom_to_hand_pose.position.z
            vec3 = geometry.Vector3(*(x, y, z))

            x = action.odom_to_hand_pose.orientation.x
            y = action.odom_to_hand_pose.orientation.y
            z = action.odom_to_hand_pose.orientation.z
            w = action.odom_to_hand_pose.orientation.w
            quaternion = geometry.Quaternion(*(x, y, z, w))
            
            pose = geometry.Pose(vec3, quaternion)

            return self._move_end_effector_pose(pose, odom_to_robot_pose, initial_joint_state, collision_env)

        elif action.type == Action.LINE:
            x = action.axis.x
            y = action.axis.y
            z = action.axis.z
            axis = geometry.Vector3(*(x, y, z))

            return self._move_end_effector_by_line(axis, action.distance, odom_to_robot_pose, initial_joint_state, collision_env)

        elif action.type == Action.GRIPPER:
            return self._command_gripper(action.open_angle, action.motion_time) 

        else:
            raise Exception("type must be pose, line, or gripper")

    def _plan_world_action(self, action, collision_env):
        if action.type == Action.WORLD:
            if action.op == Action.REMOVE:
                return self._remove_object(action.id, collision_env)

            else:
                raise Exception("op must be remove") 

        else:
            return deepcopy(collision_env)              

    def _final_odom_to_robot_pose(self, traj):
        idxs = [i for i, n in enumerate(traj.joint_names) if n in self._base_joint_names]
        point = traj.points[-1]
        return geometry.tuples_to_pose(((point.positions[idx[0]], point.positions[idx[1]], 0), (0, 0, point.positions[idx[2]])))

    def _final_joint_state(self, traj):
        idxs = [i for i, n in enumerate(traj.joint_names) if not n in self._base_joint_names]
        point = traj.points[-1]

        joint_state = JointState()
        joint_state.header = traj.header

        if len(idxs) > 0:
            joint_state.name = [traj.joints_names[idx] for idx in idxs]

        if len(point.positions) > 0:
            joint_state.position = [point.positions[idx] for idx in idxs]

        if len(point.velocity) > 0:
            joint_state.velocity = [point.velocities[idx] for idx in idxs]

        if len(point.effort) > 0:
            joint_state.effort = [point.effort[idx] for idx in idxs]

        return joint_state

    def _update_robot_state(self, type, traj, robot_state):
        pose, joint_state = robot_state

        if type == Action.POSE or type == Action.LINE:
            new_pose = self._final_odom_to_robot_pose(traj)
            new_joint_state = self._final_joint_state(traj)

        elif type == Action.GRIPPER:
            new_pose = deepcopy(pose)
            new_joint_state = deepcopy(joint_state)

            idx = joint_state.name.index(self._end_effector_joint)
            new_joint_state.position[idx] = traj.points[0].positions[0]

        else:
            new_pose = deepcopy(pose)
            new_joint_state = deepcopy(joint_state)

        return new_pose, new_joint_state 

    # seq: [d1, ..., dn]
    def plan_sequence(self, seq, odom_to_robot_pose, initial_joint_state, collision_env=None):
        traj_seq = []

        for action in seq:
            if self._is_robot_action(action.type):
                traj = self._plan_robot_action(action, odom_to_robot_pose, initial_joint_state, collision_env)
                traj_seq.append(traj)

                odom_to_robot_pose, initial_joint_state = self._update_robot_state(action.type, traj, 
                                                                                   (odom_to_robot_pose, initial_joint_state))
            elif self._is_world_action(action.type):
                collision_env = self._plan_world_action(action.type, collision_env)

        return traj_seq

