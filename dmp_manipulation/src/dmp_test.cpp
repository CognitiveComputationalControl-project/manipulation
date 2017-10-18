#include "ros/ros.h"
#include "dmp_manager.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseArray.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <sstream>
#include <boost/thread/thread.hpp>
#include <dmp/LearnDMPFromDemo.h>
#include <dmp/SetActiveDMP.h>
#include <dmp/GetDMPPlan.h>


using namespace Eigen;

static int saveiter=0;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dmp_manager");

  ros::NodeHandle n;
  ros::Subscriber global_pos_sub;
  ros::Subscriber jointstates_sub;

  DMP_Manager dmpmanager;

  global_pos_sub= n.subscribe<geometry_msgs::PoseStamped>("/global_pose", 10, &DMP_Manager::global_pose_callback,&dmpmanager);
  jointstates_sub =n.subscribe<sensor_msgs::JointState>("/hsrb/joint_states", 10, &DMP_Manager::joint_state_callback,&dmpmanager);
  dmpmanager.client_lfd = n.serviceClient<dmp::LearnDMPFromDemo>("learn_dmp_from_demo");
  dmpmanager.client_sap = n.serviceClient<dmp::SetActiveDMP>("set_active_dmp");
  dmpmanager.client_gdp = n.serviceClient<dmp::GetDMPPlan>("get_dmp_plan");

  ros::Rate loop_rate(5);

  dmpmanager.loadTrajectory();
  bool bool_learned_DMP = dmpmanager.makeLFDRequest();

  stvd x_0(2,0.0);
  stvd x_dot_0(2,0.0);
  double t_0 = 0;
  stvd goal(2,0.0);
  goal[0]= 8.0;
  goal[1]= 7.0;
  stvd goal_thresh(2,0.0);
  goal_thresh[0]=0.2;
  goal_thresh[1]=0.2;
  double seg_length =-1;
  double tau= 2*dmpmanager.m_tau;
  double dt=1.0;
  int integrate_iter = 5; 

  //if(bool_learned_DMP)
    //dmpmanager.makeGetPlanRequest(x_0,x_dot_0,t_0,goal,goal_thresh,seg_length,tau,dt,integrate_iter);

  while (ros::ok())
  {
    ros::spinOnce();
  	
     //if(saveiter%50==0) 
     //{
         //dmpmanager.saveTrajectory();
         //saveiter=0;
     //}
   
    loop_rate.sleep();
    //saveiter++;
  }
  ros::spin();
  return 0;
}
