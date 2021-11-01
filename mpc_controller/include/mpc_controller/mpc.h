//kaidi wang, 2021.9.14
//define a class and declearation of class member
#ifndef MPC_H__
#define MPC_H__
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
//#include <casadi/casadi.hpp>
//#include <mpc_controller/param.h>
#include <dynamic_reconfigure/server.h>
#include <Eigen/Dense>
#include <math.h>
//include memory file
#include <memory>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>
#include <iomanip>
#include <cstring>
#include <cmath>
#include <cstdlib>

using namespace std;

//acado library
#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>
#include <acado_optimal_control.hpp>

#include "acado_common.h"
#include "acado_auxiliary_functions.h"


//define usual param
#define PI 3.1415926
#define G 9.8
#define T 0.01//this param need to change 

USING_NAMESPACE_ACADO
DMatrix r(6,6);

static constexpr int kSamples = ACADO_N;      // number of samples
static constexpr int kStateSize = ACADO_NX;   // number of states
static constexpr int kRefSize = ACADO_NY;     // number of reference states
static constexpr int kEndRefSize = ACADO_NYN; // number of end reference states
static constexpr int kInputSize = ACADO_NU;   // number of inputs
static constexpr int kCostSize = ACADO_NY - ACADO_NU; // number of state costs
static constexpr int kOdSize = ACADO_NOD;     // number of online data


// OCP ocp(0,1,20);
// OCPexport mpc_obj( ocp);
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;
//AdaptiveReferenceTrajectory traj;
//class of some_xyz
class some_xyz
{
private:
    //none private param
public:
    double x;
    double y;
    double z;
    some_xyz(/* args */);
    ~some_xyz();
};
some_xyz::some_xyz(/* args */)
{
    x=0;
    y=0;
    z=0;
}
some_xyz::~some_xyz()
{

}

//class of controller_base
class controller_base
{
private:

public:
    //will be used in controller_node.cpp
    /* define data list */
    some_xyz fly1_pos;
    some_xyz fly2_pos;
    some_xyz fly3_pos;
    
    some_xyz I_center;

    double center_mass;
    double fly1_mass;
    double fly2_mass;
    double fly3_mass;

    some_xyz I_sys;
    double S3Q_mass;//define the mass param
    // Eigen::Matrix3f Mt;//define the inertia of the AVA platform

    some_xyz tool_pos;
    controller_base(/* args */);
    ~controller_base();
};
controller_base::controller_base(/* args */)
{

    //fly1 position
    fly1_pos.x = 0.50882;
    fly1_pos.y = 0;
    fly1_pos.z = -0.04135;

    //fly2_positon
    fly2_pos.x = -0.26956;
    fly2_pos.y = 0.43641;
    fly2_pos.z = -0.04135;

    //fly3_positon
    fly3_pos.x = -0.26956;
    fly3_pos.y = -0.43641;
    fly3_pos.z = -0.04135;

    //tool position
    tool_pos.x = 0;
    tool_pos.y = 0;
    tool_pos.z = 0;

    //mass param
    fly1_mass = 1.597;
    fly2_mass = 1.604;
    fly3_mass = 1.583;
    center_mass = 1.607;

    //I center param
    I_center.x = 0.056804546;
    I_center.y = 0.057436539;
    I_center.z = 0.108610089;

    //calc mass of S3Q platform
    S3Q_mass = fly1_mass+fly2_mass+fly3_mass+center_mass;
    I_sys.x =  I_center.x + 
               fly1_mass*(fly1_pos.y*fly1_pos.y+fly1_pos.z*fly1_pos.z)+
               fly2_mass*(fly2_pos.y*fly2_pos.y+fly2_pos.z*fly2_pos.z)+
               fly3_mass*(fly3_pos.y*fly3_pos.y+fly3_pos.z*fly3_pos.z);

    I_sys.y =  I_center.y + 
               fly1_mass*(fly1_pos.x*fly1_pos.x+fly1_pos.z*fly1_pos.z)+
               fly2_mass*(fly2_pos.x*fly2_pos.x+fly2_pos.z*fly2_pos.z)+
               fly3_mass*(fly3_pos.x*fly3_pos.x+fly3_pos.z*fly3_pos.z);

    I_sys.z =  I_center.z + 
               fly1_mass*(fly1_pos.x*fly1_pos.x+fly1_pos.y*fly1_pos.y)+
               fly2_mass*(fly2_pos.x*fly2_pos.x+fly2_pos.y*fly2_pos.y)+
               fly3_mass*(fly3_pos.x*fly3_pos.x+fly3_pos.y*fly3_pos.y);
    //assignment to Mt matrix
}
controller_base::~controller_base()
{

}

//class of mpc
class mpc
{
public:
    /* data */
    //private member
    ros::NodeHandle nh;
    //define a controller_base class
    //Eigen::Matrix<double, kRefSize, kSamples, Eigen::ColMajor> acado_reference_states_;
    controller_base param;

    Eigen::Matrix3f J;
    Eigen::Matrix3f pos_s1;      //position matrix
    Eigen::Matrix3f pos_s2;
    Eigen::Matrix3f pos_s3;
    //define rotation matrix 
    Eigen::Matrix3f r_mat;       //main_euler_angle rotation mat
    Eigen::Matrix3f r_mat_d;     //nominal_euler_angle rotation mat

    //distributor param list
    Eigen::Vector3f thrust_u;     //thrust vector
    Eigen::Vector3f torques_u;    //torque vector

    //before filter function thrust and torque
    Eigen::Vector3f in_thrust;
    Eigen::Vector3f in_torque;

    Eigen::Vector3f lamda1;
    Eigen::Vector3f lamda2;
    Eigen::Vector3f lamda3;

    //mpc controller variable list
    DifferentialState pos_x;
    DifferentialState pos_y;
    DifferentialState pos_z;
    
    DifferentialState ang_roll;
    DifferentialState ang_pitch;
    DifferentialState ang_yaw;

    DifferentialState pos_vel_x;
    DifferentialState pos_vel_y;
    DifferentialState pos_vel_z;

    DifferentialState ang_vel_roll;
    DifferentialState ang_vel_pitch;
    DifferentialState ang_vel_yaw;
    //inter mediate state list
    IntermediateState pos[3];
    IntermediateState att[3];
    IntermediateState pos_vel[3];
    // IntermediateState pos_vel_ground[3];
    // IntermediateState pos_vel_body[3];
    IntermediateState ang_vel[3];
    IntermediateState rotation[3][3];
    IntermediateState force[3];
    IntermediateState tao[3];
    IntermediateState acc[3];
    IntermediateState ang_acc[3];

    IntermediateState MtMangv[3];//Mt*ang_v
    IntermediateState mapMtMangv[3][3];//map(Mt*ang_v)
    IntermediateState mapMang_v[3];//map(Mt*ang_v)*ang_v
    
    double states_var[12];
    //OCP ocp( 0, 1, 20 );
    //define controls
    Control u_thu_x; 
    Control u_thu_y; 
    Control u_thu_z; 
    Control u_tor_x; 
    Control u_tor_y; 
    Control u_tor_z;

    //define differentialEquation
    DifferentialEquation f;
    Function h;
    Function hN;

    int NX    = ACADO_NX;   /* Number of differential state variables.  */
    int NXA   = ACADO_NXA;  /* Number of algebraic variables. */
    int NU    = ACADO_NU;   /* Number of control inputs. */
    int NOD   = ACADO_NOD;  /* Number of online data values. */
    int NY    =      ACADO_NY;  /* Number of measurements/references on nodes 0..N - 1. */
    int NYN   =      ACADO_NYN; /* Number of measurements/references on node N. */
    int N     =      ACADO_N;   /* Number of intervals in the horizon. */
    int NUM_STEPS  = 20;        /* Number of real-time iterations. */
    int VERBOSE    = 1 ;        /* Show iterations: 1, silent: 0.  */

    int iter;

    int count_num_test = 0;

    //acado up and low bounder value
    //position bound
    double u_pos_x;
    double l_pos_x;
    double u_pos_y;
    double l_pos_y;
    double u_pos_z;
    double l_pos_z; 
    //attitude bound
    double u_ang_x;
    double l_ang_x;
    double u_ang_y;
    double l_ang_y;
    double u_ang_z;
    double l_ang_z;
    //velocity bound
    double u_vel_x;
    double l_vel_x;
    double u_vel_y;
    double l_vel_y;
    double u_vel_z;
    double l_vel_z;
    //ang velocity bound
    double u_ang_vel_x;
    double l_ang_vel_x;
    double u_ang_vel_y;
    double l_ang_vel_y;
    double u_ang_vel_z;
    double l_ang_vel_z;
    //thrust and torque bound
    double u_thrust_x;
    double l_thrust_x;
    double u_thrust_y;
    double l_thrust_y;
    double u_thrust_z;
    double l_thrust_z;
    double u_torque_x;
    double l_torque_x;
    double u_torque_y;
    double l_torque_y;
    double u_torque_z;
    double l_torque_z;
    //Controller controller;
    StaticReferenceTrajectory reference;

    //param of first-order filter
    double a_thu_x; 
    double a_thu_y; 
    double a_thu_z; 
    double a_tor_x; 
    double a_tor_y; 
    double a_tor_z;

    //output list
    geometry_msgs::Point ang1;
    geometry_msgs::Point ang2;
    geometry_msgs::Point ang3;
    std_msgs::Float64 thu1;
    std_msgs::Float64 thu2;
    std_msgs::Float64 thu3;
    //output mark value
    std_msgs::Float64 mark;

    //publister list
    ros::Publisher angle1_pub;
    ros::Publisher angle2_pub;
    ros::Publisher angle3_pub;
    ros::Publisher thrust1_pub;
    ros::Publisher thrust2_pub;
    ros::Publisher thrust3_pub;
    //mpc_mark to record when change to mpc controller, bool value
    ros::Publisher mpc_mark_pub;

    //input list
    geometry_msgs::Point nominal_position;      //setpoint position
    geometry_msgs::Point nominal_euler_angles;  //euler angle setpoint position
    geometry_msgs::Point main_position;
    geometry_msgs::Point main_velocity;
    geometry_msgs::Point main_eular_angles;
    geometry_msgs::Point main_body_rates;
    geometry_msgs::Point init_euler_angles;
    geometry_msgs::Point init_position;
    geometry_msgs::Point init_body_rate;
    geometry_msgs::Point init_velocity;

    //define some varable to store reference value
    geometry_msgs::Point r_pos;
    geometry_msgs::Point r_att;
    geometry_msgs::Point r_vel_pos;
    geometry_msgs::Point r_vel_ang;

    //kaidi wang 2021.10.15: define two variables to store main_position and main_eular_angles
    geometry_msgs::Point hover_position;
    geometry_msgs::Point hover_attitude;

    //mode switch msg
    std_msgs::Bool mode_switch;
    //init switch
    int init_fun_switch ;
    std_msgs::Bool start_pub_att;
    std_msgs::Float64 node1_yaw;
    std_msgs::Float64 node2_yaw;
    std_msgs::Float64 node3_yaw;

    double ref_n[ACADO_NY]; 

    real_t t1,t2;
    real_t fdbSum = 0.0;
	real_t prepSum = 0.0;
	int status;
	acado_timer t;

    //subscriber list
    //geometry_msgs::Point
    ros::Subscriber nominal_position_sub ;      //nominal_position
    ros::Subscriber nominal_eular_angles_sub ;  //nominal_eular_angles
    ros::Subscriber main_position_sub ;	        //controller input of main_position
    ros::Subscriber main_velocity_sub ;	        //controller input of main_velocity
    ros::Subscriber main_eular_angles_sub ;	    //controller input of main_eular_angles
    ros::Subscriber main_body_rates_sub ;	    //controller input of main_body_rates
    ros::Subscriber init_euler_angles_cmd_sub ;	//init cmd publish: euler angles
    ros::Subscriber init_pos_cmd_sub ;	        //init cmd publish: position
    ros::Subscriber init_body_rates_cmd_sub ;	//init cmd publish: body_rates
    ros::Subscriber init_velocity_cmd_sub ;	    //init cmd publish: velocity

    //std_msgs::Bool
    ros::Subscriber control_start_sub_att ;	    //control switch of when publish attitude to child node, besides control when start the controller  
    ros::Subscriber mode_switch_sub;
    
    //subscribe the bias of psi angle that each child flight needed
    //std_msgs::Float64
    ros::Subscriber psi_bias_1_sub;
    ros::Subscriber psi_bias_2_sub;
    ros::Subscriber psi_bias_3_sub;

    //own function list
    Eigen::Matrix3f pos_mat(double x,double y,double z);
    //the state function of mpc controller
    void mpc_state_function();
    //the slover of mpc controller
    void mpc_solver();
    //real init funciton of mpc controller
    void init_mpc_fun();

    void get_input();
    void get_state();
    void update(
        geometry_msgs::Point position, 
        geometry_msgs::Point attitude,
        geometry_msgs::Point vel_pos,
        geometry_msgs::Point vel_ang,
        geometry_msgs::Point ref_pos,
        geometry_msgs::Point ref_att,
        geometry_msgs::Point ref_vel_pos,
        geometry_msgs::Point ref_vel_ang,
        Eigen::Vector3f thrust,    //thrust vector
        Eigen::Vector3f torque);
    //filter function 
    void fisrt_order_filter(Eigen::Vector3f in_data_thu,Eigen::Vector3f in_data_tor);

    //the distributor of the whole thrust and troque
    void distributor(Eigen::Vector3f thu,Eigen::Vector3f tor);
    //alloc function
    Eigen::Vector4f alloc(Eigen::Vector3f f,double psi_cmd);

    //convert euler angle to rotation matrix
    Eigen::Matrix3f euler_to_rotation_mat(geometry_msgs::Point angle);

    //basic function list
    void init_publisher();
    void init_subscriber();
    void output_publish(geometry_msgs::Point ang1,geometry_msgs::Point ang2,geometry_msgs::Point ang3,std_msgs::Float64 thu1,std_msgs::Float64 thu2,std_msgs::Float64 thu3);
    
    
    //callback function list
    void nominal_position_sub_cb(const geometry_msgs::Point::ConstPtr& msg );
    void nominal_eular_angles_sub_cb(const geometry_msgs::Point::ConstPtr& msg);
    void main_position_sub_cb(const geometry_msgs::Point::ConstPtr& msg);
    void main_velocity_sub_cb(const geometry_msgs::Point::ConstPtr& msg);
    void main_eular_angles_sub_cb(const geometry_msgs::Point::ConstPtr& msg);
    void main_body_rates_sub_cb(const geometry_msgs::Point::ConstPtr& msg);
    void init_euler_angles_cmd_sub_cb(const geometry_msgs::Point::ConstPtr& msg);
    void init_pos_cmd_sub_cb(const geometry_msgs::Point::ConstPtr& msg);
    void init_body_rates_cmd_sub_cb(const geometry_msgs::Point::ConstPtr& msg);
    void init_velocity_cmd_sub_cb(const geometry_msgs::Point::ConstPtr& msg);

    void control_start_sub_att_cb(const std_msgs::Bool::ConstPtr& msg);
    void mode_switch_sub_cb(const std_msgs::Bool::ConstPtr& msg);

    void psi_bias_1_sub_cb(const std_msgs::Float64::ConstPtr& msg);
    void psi_bias_2_sub_cb(const std_msgs::Float64::ConstPtr& msg);
    void psi_bias_3_sub_cb(const std_msgs::Float64::ConstPtr& msg);

    //read data from file 
    bool readDataFromFile(const char* fileName, vector<vector<double>>& data);
    //get rand data
    double getRandData(double min,double max);
public:
    //define a init node handle function
    mpc(ros::NodeHandle* nodehandle);
    //define a timer for period calc controller
    ros::Timer calc_timer;
    void calc_cb(const ros::TimerEvent&);
    ~mpc();
};
mpc::~mpc()
{

}
#endif
