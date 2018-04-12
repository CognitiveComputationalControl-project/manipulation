#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <hsr_kinematics/head_kinematics.hpp>
#include <tmc_eigen_bridge/eigen_bridge.hpp>
#include <tmc_manipulation_types_bridge/manipulation_msg_convertor.hpp>
#include <tmc_utils/pose_utils.hpp>

#include <boost/thread.hpp>
#include <boost/format.hpp>

namespace {
  const double kRate = 10.0;
  const char* kOriginLink = "odom";
  const char* kRobotBaseLink = "base_link";
}

class GazedPointFixing {
 public:
  GazedPointFixing() :
    is_activating_(false),
    tf_buffer_(),
    listener_(tf_buffer_) {
    ros::NodeHandle nh("gazed_point_fixing_node");
    std::string robot_model_config;
    if (!nh.getParam("/robot_description",
                     robot_model_config)) {
      ROS_ERROR("No robot_description in parameter server.");
      exit(EXIT_FAILURE);
    }
    std::vector<std::string> head_joint_names;
    head_joint_names.push_back("head_pan_joint");
    head_joint_names.push_back("head_tilt_joint");
    head_kinematics_.reset(new hsr_kinematics::HsrHeadKinematics(robot_model_config,
                                                                 head_joint_names));

    joint_state_sub_ = nh.subscribe("/hsrb/joint_states", 1,
                                    &GazedPointFixing::JointStateCallback_, this);
    target_point_sub_ = nh.subscribe("target_point",
                                     1,
                                     &GazedPointFixing::TargetPointCallback_, this);
    activation_sub_ = nh.subscribe("activate",
                                   1,
                                   &GazedPointFixing::ActivationCallback_, this);
    head_trajectory_pub_ = \
      nh.advertise<trajectory_msgs::JointTrajectory>("/hsrb/head_trajectory_controller/collisionable_command", 10);
  }


  void Fixing() {
    if (!is_activating_) {
      return;
    }
    geometry_msgs::TransformStamped transform_stamped;
    try {
    transform_stamped =
      tf_buffer_.lookupTransform(kOriginLink,
                                 kRobotBaseLink,
                                 ros::Time(0),
                                 ros::Duration(1.0));
    } catch(const tf2::TransformException& ex) {
      ROS_ERROR_STREAM((boost::format("Cannot transform goal pose from '%1%' frame to '%2%' frame.")
                        % kOriginLink % kRobotBaseLink).str());
      return;
    }
    sensor_msgs::JointState target_head_angle;
    geometry_msgs::Pose origin_to_robot;
    origin_to_robot.position.x = transform_stamped.transform.translation.x;
    origin_to_robot.position.y = transform_stamped.transform.translation.y;
    origin_to_robot.orientation = transform_stamped.transform.rotation;
    if (!ComputeGazeTargetPointHeadAngle_(origin_to_robot,
                                          origin_to_target_point_,
                                          &target_head_angle)) {
      ROS_ERROR("Failed to compute head angle to gaze target.");
      return;
    }
    trajectory_msgs::JointTrajectory traj;
    traj.joint_names.push_back("head_pan_joint");
    traj.joint_names.push_back("head_tilt_joint");
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.push_back(target_head_angle.position[0]);
    point.positions.push_back(target_head_angle.position[1]);
    point.time_from_start = ros::Duration(0.01);
    traj.points.push_back(point);
    head_trajectory_pub_.publish(traj);
  }

 private:
  ros::Publisher head_trajectory_pub_;
  ros::Subscriber activation_sub_;
  ros::Subscriber joint_state_sub_;
  ros::Subscriber target_point_sub_;
  tf2_ros::Buffer tf_buffer_;

  mutable boost::mutex mutex_;
  bool is_activating_;
  hsr_kinematics::HsrHeadKinematics::Ptr head_kinematics_;
  geometry_msgs::Point origin_to_target_point_;
  tf2_ros::TransformListener listener_;
  sensor_msgs::JointState current_joint_state_;

  void ActivationCallback_(const std_msgs::BoolConstPtr& msg) {
    is_activating_ = msg->data;
  }

  void JointStateCallback_(const sensor_msgs::JointState& msg) {
    boost::mutex::scoped_lock lock(mutex_);
    current_joint_state_ = msg;
  }

  void TargetPointCallback_(const geometry_msgs::PointConstPtr& point) {
    origin_to_target_point_.x = point->x;
    origin_to_target_point_.y = point->y;
    origin_to_target_point_.z = point->z;
    ROS_INFO("Target point received.");
  }

  bool ComputeGazeTargetPointHeadAngle_(const geometry_msgs::Pose& origin_to_robot,
                                        const geometry_msgs::Point& origin_to_gaze_point,
                                        sensor_msgs::JointState* dst_head_angle) {
    geometry_msgs::Pose robot_base_to_origin_msg = tmc_utils::InvertPoseMsg(origin_to_robot);
    Eigen::Affine3d robot_base_to_origin;
    tmc_eigen_bridge::PoseMsgToAffine3d(robot_base_to_origin_msg,
                                        robot_base_to_origin);
    Eigen::Vector3d origin_to_gaze;
    tmc_eigen_bridge::PointMsgToVector3d(origin_to_gaze_point, origin_to_gaze);
    Eigen::Vector3d robot_base_to_gaze_point = robot_base_to_origin * origin_to_gaze;
    sensor_msgs::JointState joint_state;
    {
      boost::mutex::scoped_lock lock(mutex_);
      joint_state = current_joint_state_;
    }
    tmc_manipulation_types::JointState current_joint_state;
    tmc_manipulation_types_bridge::JointStateMsgToJointState(joint_state,
                                                             current_joint_state);
    tmc_manipulation_types::JointState head_angle;
    if (!head_kinematics_->CalculateAngleToGazePoint(kRobotBaseLink,
                                                     Eigen::Translation3d(robot_base_to_gaze_point),
                                                     "head_rgbd_sensor_link",
                                                     current_joint_state,
                                                     head_angle)) {
      ROS_WARN("Failed to calculate angle to gaze point.");
      return false;
    }
    tmc_manipulation_types_bridge::JointStateToJointStateMsg(head_angle,
                                                             *dst_head_angle);
    return true;
  }
};

void Run() {
  GazedPointFixing node;
  ros::Rate rate(kRate);
  while (ros::ok()) {
    node.Fixing();
    ros::spinOnce();
    rate.sleep();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "gazed_point_fixing_node");
  Run();
  return EXIT_SUCCESS;
}
