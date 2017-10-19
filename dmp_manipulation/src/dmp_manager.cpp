#include "dmp_manager.h"

DMP_Manager::DMP_Manager(){
    m_tau=0.0;
	// Init_parameters();
}
DMP_Manager::~DMP_Manager(){}

void DMP_Manager::Init_parameters()
{

    int dim=2;
    JointTrajectoryset.resize(dim);  //create size of joint trajectory
}

void DMP_Manager::global_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{



}

void DMP_Manager::joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg)
{
//ROS_INFO("recieved sensor_msgs");
    stvd tempJointTraj(5,0.0);
    
    tempJointTraj[0]=static_cast<double>( msg->position[0]);
    tempJointTraj[1]=static_cast<double>( msg->position[1]);
    tempJointTraj[2]=static_cast<double>( msg->position[2]);
    tempJointTraj[3]=static_cast<double>( msg->position[11]);
    tempJointTraj[4]=static_cast<double>( msg->position[12]);

    JointTrajectoryset.push_back(tempJointTraj);
    
}

void DMP_Manager::saveTrajectory()
{

    ROS_INFO("save trajectory");
    basisTrajectory.open("src/dmp_manipulation/data/dmp_trajectories.csv");

    for(int i=0; i<JointTrajectoryset.size();i++ )
    {
        basisTrajectory<<i<<",";

        for(int j=0;j<JointTrajectoryset[i].size();j++) 
            basisTrajectory<<JointTrajectoryset[i][j]<<",";
    
        basisTrajectory<<endl;
    }
    
    basisTrajectory.close();
}

void DMP_Manager::printTrajectory()
{
    int time_size = JointTrajectoryset.size();
    int dof_trj=JointTrajectoryset[0].size();
    ROS_INFO("joint trajectory : time : %d , dof : %d \n", time_size, dof_trj); 
    
     for(int i(0);i<JointTrajectoryset.size();i++)
     {
         std::cout<<"time : "<<i<<": \t" ;
         for(int j=0;j<JointTrajectoryset[i].size();j++) 
         {
             std::cout<<JointTrajectoryset[i][j]<<",";
         }
         std::cout<<std::endl;
     }
}

void DMP_Manager::loadTrajectory()
{
    char    spell[150]; 
    int     iter=0;
    float   tempdata=0.0;
    char    *data_seg[50];
    string  token =",";
    int     i,j=0;
    int     res;
    string  str;
    int     Dim=0;
    stvd    tempDataVec;

    ROS_INFO("load trajectory");
    ifstream laodtrjfile;
    laodtrjfile.open("src/dmp_manipulation/data/dmp_trajectories.csv");

    //clear joint trajectory
    JointTrajectoryset.clear();

    //read data file
    if(!laodtrjfile.is_open()){
            cout << "Data file load error... check the data file" << endl;
            exit(0);
        }
    else{
        iter=0;
        while(!laodtrjfile.eof()) 
        {
            laodtrjfile.getline(spell, 100);              //read lines

            if (spell[0]=='\0')     //empty line
                continue;

            i=0;
            data_seg[i] = strtok(spell,token.c_str());

            while(data_seg[i]!=NULL)                       //write data_segment with "\t"
                data_seg[++i] = strtok(NULL,token.c_str());
            
            Dim=i-1;                                         //check dimension : time-1

            //Dim=4;
            cout<<"Feature dimension is"<<Dim<<endl;
            tempDataVec.resize(Dim);
            
            for(j=1;j<Dim+1;j++)
            {
                str=data_seg[j];
                str.erase(str.length(),1) ;
                tempdata = static_cast<float>(atof(str.c_str())) ;
                tempDataVec[j-1]=tempdata;
            }

            JointTrajectoryset.push_back(tempDataVec);
            iter++;
        } 
    }

   laodtrjfile.close();

   //setDimension
   dimension=Dim;

   //print trajectory
   //printTrajectory();
}

bool DMP_Manager::makeLFDRequest()
{
    dmp::DMPTraj trajectory_msgs;

    //test with same data with sample.py
    //dimension=2;
    //int time_length = 4;
    //int trj_length =dimension;

    //JointTrajectoryset.resize(time_length);
    
    //for(int k(0);k<time_length;k++)
    //{   
        //JointTrajectoryset[k].resize(dimension);

    //}

    //JointTrajectoryset[0][0]= 1.0;
    //JointTrajectoryset[0][1]= 1.0;

    //JointTrajectoryset[1][0]= 2.0;
    //JointTrajectoryset[1][1]= 2.0;

    //JointTrajectoryset[2][0]= 3.0;
    //JointTrajectoryset[2][1]= 4.0;

    //JointTrajectoryset[3][0]= 6.0;
    //JointTrajectoryset[3][1]= 8.0;

    int time_length = JointTrajectoryset.size();
    int joint_dof = JointTrajectoryset[0].size();
    std::cout<<"time : "<<time_length<<", joint_dof : " << joint_dof <<", dim :"<<dimension<<endl;
    double dt =1;
    double K  = 100.0;
    double D  = 2.0 * sqrt(K);

    for(int i(0);i<time_length;i++)
    {
        dmp::DMPPoint dmp_point;        
        for(int j(0);j<joint_dof;j++)
        {
            double tempdata = static_cast<double> (JointTrajectoryset[i][j]);
            dmp_point.positions.push_back(tempdata);
            dmp_point.velocities.push_back(0.0);
        }

       trajectory_msgs.points.push_back(dmp_point);
       trajectory_msgs.times.push_back(static_cast<double>(i*dt));
    }

    Kgains.resize(dimension);
    Dgains.resize(dimension);
           
    for(int i(0);i<dimension;i++)
    {
        Kgains[i]=K;
        Dgains[i]=D;
    }

    //calling service learnfromdemonstration
    dmp::LearnDMPFromDemo lfd_srv;  
    lfd_srv.request.demo = trajectory_msgs;
    lfd_srv.request.k_gains = Kgains;
    lfd_srv.request.d_gains = Dgains;
    lfd_srv.request.num_bases = 10;

    if(client_lfd.call(lfd_srv))
    {
        ROS_INFO("Learning From Demonstration srv is done \n");
        m_dmp_dataset.clear();
        // dmp_dataset=lfd_srv.response.dmp_list;
        int list_size =lfd_srv.response.dmp_list.size();
        std::cout<<"list size : "<<list_size<<std::endl;
        std::cout<<"tau size is : "<<lfd_srv.response.tau<<std::endl;
        
       set_tau(lfd_srv.response.tau);
        
        for(int i(0);i<lfd_srv.response.dmp_list.size();i++)
        {
          m_dmp_dataset.push_back(lfd_srv.response.dmp_list[i]);
          // cout<<lfd_srv.response.dmp_list[i]<<", ";
        }
        //calling set_activeset_dmp service;
        dmp::SetActiveDMP sap_srv;
        sap_srv.request.dmp_list=lfd_srv.response.dmp_list;
        if(client_sap.call(sap_srv))
        {
           ROS_INFO("set_Active_DMP_srv succeed!");
           return true;
        }
        else
        {
           ROS_INFO("Unable to call set_Active_DMP_srv");
           return false;
        }

    }
    else{

        ROS_INFO("Unable to call LearnDMPFromDemo_srv.");
        return false;
    }
    
}

void DMP_Manager::set_tau(double tau_)
{
    m_tau=tau_;
}

bool DMP_Manager::makeSetActiveRequest()
{
    //this service will be called in learning from demonstration service
}

bool DMP_Manager::makeGetPlanRequest(stvd& x_0_, stvd& x_dot_0_, double t_0_, stvd& goal_, stvd& goal_thresh_, double seg_length_, double tau_, double dt_, int integrate_iter_)
{
    dmp::GetDMPPlan gdp_srv;
    //setting parameter for calling service
    gdp_srv.request.x_0            = x_0_;
    gdp_srv.request.x_dot_0        = x_dot_0_ ;
    gdp_srv.request.t_0            = t_0_;
    gdp_srv.request.goal           = goal_;
    gdp_srv.request.goal_thresh    = goal_thresh_;
    gdp_srv.request.seg_length     = seg_length_;
    gdp_srv.request.tau            = tau_;
    gdp_srv.request.dt             = dt_;
    gdp_srv.request.integrate_iter = integrate_iter_;

    if(client_gdp.call(gdp_srv))
    {
        ROS_INFO("get_DMP_Plan_srv succeed!");
        m_dmptrajectory=gdp_srv.response.plan;
        print_dmp_trajectory();

        return true;
    }
    else
    {
        ROS_INFO("Unable to call get_DMP_plan_srv");
        return false;
    }

}

void DMP_Manager::print_dmp_trajectory(){

    int data_size = m_dmptrajectory.points.size();

    for(int i(0);i<data_size;i++)
    {
        std::cout<<"time : "<<m_dmptrajectory.times[i]<<" , ";
        for(int j(0);j<m_dmptrajectory.points[0].positions.size();j++)
        {
            std::cout<<"pos : "<<m_dmptrajectory.points[i].positions[j]<<", ";
            std::cout<<"===vel : "<<m_dmptrajectory.points[i].velocities[j]<<", ";
        }
        std::cout<<std::endl;
    }

}
