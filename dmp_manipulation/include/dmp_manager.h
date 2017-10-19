#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <stdint.h>
#include <math.h>
#include "ros/ros.h"
#include <ros/package.h>
#include <Eigen/Dense>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int8.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/PoseArray.h"
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>

//Msg & Srvs
#include <dmp/LearnDMPFromDemo.h>
#include <dmp/SetActiveDMP.h>
#include <dmp/GetDMPPlan.h>
#include <dmp/DMPData.h>
#include <dmp/DMPPoint.h>
#include <dmp/DMPTraj.h>

using namespace std;

#define arm_flex_joint 0
#define arm_lift_joint 1
#define arm_roll_joint 2
#define wrist_flex_joint 3
#define wrist_roll_joint 4

typedef std::vector<double> stvd;

class DMP_Manager{

public:
	DMP_Manager();
	DMP_Manager(int numofhuman);
	~DMP_Manager();

    int dimension;
    
	ros::Publisher dmp_pub;
	ros::Publisher jointtrajectory_pub;

    ofstream basisTrajectory;
    
    vector< stvd >          JointTrajectoryset;       //[time x joint number]
    sensor_msgs::JointState JointStates_traj;
    map<int,sensor_msgs::JointState> trajectory_map;
    std::vector< dmp::DMPData > m_dmp_dataset;

    dmp::DMPTraj m_dmptrajectory;

    stvd Kgains;
    stvd Dgains;

    ros::ServiceClient client_lfd;
    ros::ServiceClient client_sap;
    ros::ServiceClient client_gdp;

    double m_tau;
	int index;
	void Init_parameters();

    //ros callback function
    void global_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg);
    
    void set_tau(double tau_);
    //managing trajectory functions
    void saveTrajectory();
    void loadTrajectory();
    void printTrajectory();
    
    //service functions
    bool makeLFDRequest();
    bool makeSetActiveRequest();
    bool makeGetPlanRequest(stvd& x_0_, stvd& x_dot_0_, double t_0_, stvd& goal_, stvd& goal_thresh_, double seg_length_, double tau_, double dt_, int integrate_iter_);
    void print_dmp_trajectory();
};
