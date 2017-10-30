#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/ControllerState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <tmc_manipulation_msgs/FollowJointTrajectoryAction.h>
#include <tmc_manipulation_msgs/FollowJointTrajectoryGoal.h>
#include <sensor_msgs/JointState.h>
#include <vector>

using namespace std;

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client;

std::vector<double> tempJointTraj(5,0.0);


void joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg)
{

    tempJointTraj.resize(5);
    tempJointTraj[0]=static_cast<double>( msg->position[0]);
    tempJointTraj[1]=static_cast<double>( msg->position[1]);
    tempJointTraj[2]=static_cast<double>( msg->position[2]);
    tempJointTraj[3]=static_cast<double>( msg->position[11]);
    tempJointTraj[4]=static_cast<double>( msg->position[12]);

    ROS_INFO("sensor message :%.3lf, %.3lf, %.3lf, %.3lf, %.3lf \n ",tempJointTraj[0],tempJointTraj[1],tempJointTraj[2],tempJointTraj[3], tempJointTraj[4]);


}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");

    // initialize action client
  // Client cli("/hsrb/impedance_control/follow_joint_trajectory_with_config", true);
    Client cli("/hsrb/arm_trajectory_controller/follow_joint_trajectory", true);

  // wait for the action server to establish connection
  cli.waitForServer();
  
  // initalize ROS publisher
  ros::NodeHandle n;
  
  ros::Publisher pub = n.advertise<trajectory_msgs::JointTrajectory>("/hsrb/arm_trajectory_controller/command", 10);
  ros::Subscriber sub = n.subscribe<sensor_msgs::JointState>("hsrb/joint_states",10, joint_state_callback);

  // wait to establish connection between the controller
  while(pub.getNumSubscribers() == 0) {
    ros::Duration(0.1).sleep();
  }

// make sure the controller is running
   ros::ServiceClient client = n.serviceClient<controller_manager_msgs::ListControllers>("/hsrb/controller_manager/list_controllers");
  controller_manager_msgs::ListControllers list_controllers;
  bool running = false;
  while (running == false) {
    ros::Duration(0.1).sleep();
    if (client.call(list_controllers)) {
      for (unsigned int i = 0; i < list_controllers.response.controller.size(); i++) {
        controller_manager_msgs::ControllerState c = list_controllers.response.controller[i];
        if (c.name == "arm_trajectory_controller" && c.state == "running") {
          running = true;
        }
      }
    }
  }

  ros::Rate loop_rate(50);

  // tmc_manipulation_msgs::FollowJointTrajectoryGoal goal;
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory.joint_names.push_back("arm_flex_joint");
  goal.trajectory.joint_names.push_back("arm_lift_joint");
  goal.trajectory.joint_names.push_back("arm_roll_joint");
  goal.trajectory.joint_names.push_back("wrist_flex_joint");
  goal.trajectory.joint_names.push_back("wrist_roll_joint");



  while (ros::ok())
  {

      // fill ROS message
 
    goal.trajectory.points.resize(1);
    
    goal.trajectory.points[0].positions.resize(5);
    goal.trajectory.points[0].velocities.resize(5);

    for(int i(0);i<tempJointTraj.size();i++)
    {
      goal.trajectory.points[0].positions[i] = tempJointTraj[i];
      goal.trajectory.points[0].velocities[i] = 0.0;
    }
    
    // traj.points[0].positions[2]=0.34;

    goal.trajectory.points[0].time_from_start = ros::Duration(2.0);

    // publish ROS message

    // pub.publish(traj);
    cli.sendGoal(goal);
    cli.waitForResult(ros::Duration(5.0));
    ros::spinOnce();  
    loop_rate.sleep();  
  }

  ros::spin();


  return 0;
}
