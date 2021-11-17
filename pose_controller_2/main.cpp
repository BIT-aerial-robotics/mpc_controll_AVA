// Copyright 2019-2020 The MathWorks, Inc.
// Generated 08-Feb-2021 21:19:06

#include <stdio.h>
#include "rosnodeinterface.h"
#include "slros_initialize.h"
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <pose_controller_2/paramConfig.h>

// kaidi wang 2021.10.14
#include <std_msgs/Bool.h>
std_msgs::Bool pub;
extern P_pose_controller_2_T pose_controller_2_P;

extern void slros_node_init(int argc, char** argv);
namespace ros
{
  namespace matlab
  {
    std::shared_ptr<ros::matlab::NodeInterface> gMatlabNodeIntr;
    std::shared_ptr<ros::matlab::NodeInterface> getNodeInterface()
    {
      return gMatlabNodeIntr;
    }
  }                                    //namespace matlab
}                                      //namespace ros

void callback(pose_controller_2::paramConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure Request: %f %f %f %f %f %f ",             
			config.kp_x, 
      config.kp_y,             
			config.kp_z,
      config.ki_x,
      config.ki_y,
      config.ki_z);

  pose_controller_2_P.kp_x_Value =config.kp_x;      
  pose_controller_2_P.kp_y_Value =config.kp_y;     
  pose_controller_2_P.kp_z_Value =config.kp_z;     

  pose_controller_2_P.ki_x_Value =config.ki_x;      
  pose_controller_2_P.ki_y_Value =config.ki_y;     
  pose_controller_2_P.ki_z_Value =config.ki_z;     

  pose_controller_2_P.kd_x_Value =config.kd_x;      
  pose_controller_2_P.kd_y_Value =config.kd_y;     
  pose_controller_2_P.kd_z_Value =config.kd_z;     


  pose_controller_2_P.kp_phi_Value = config.kp_phi;
  pose_controller_2_P.kp_theta_Value = config.kp_theta;
  pose_controller_2_P.kp_psi_Value = config.kp_psi;

  pose_controller_2_P.ki_phi_Value = config.ki_phi;
  pose_controller_2_P.ki_theta_Value = config.ki_theta;
  pose_controller_2_P.ki_psi_Value = config.ki_psi;


  pose_controller_2_P.kd_phi_Value = config.kd_phi;
  pose_controller_2_P.kd_theta_Value = config.kd_theta;
  pose_controller_2_P.kd_psi_Value = config.kd_psi;



}

//kaidi wang 10.14, mode switch callback function 
std_msgs::Bool mode_switch;
void mode_switch_sub_cb(const std_msgs::Bool::ConstPtr& msg)
{
	mode_switch = *msg;
  if (mode_switch.data)
  {
    ROS_INFO_STREAM("mpc controller running...");
    pub.data = false;
  }
  else
  {
    ROS_INFO_STREAM("ordinary controller runnning...");
    pub.data = true;
  }
}

void timerCallback(const ros::TimerEvent& event)
{

}

int main(int argc, char* argv[])
{
  // create the Node specified in Model
  slros_node_init(argc, argv);

  dynamic_reconfigure::Server<pose_controller_2::paramConfig> server;
  dynamic_reconfigure::Server<pose_controller_2::paramConfig>::CallbackType f; 

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ROS_INFO("Spinning node");
  ros::matlab::gMatlabNodeIntr = std::make_shared<ros::matlab::NodeInterface>();
  ros::matlab::gMatlabNodeIntr->initialize(argc, argv);
  
  //kaidi wang and a mode switch
  ros::init(argc, argv, SLROSNodeName);
  ros::NodeHandle nh;
  ros::Subscriber mode_switch_sub = nh.subscribe<std_msgs::Bool>("/mode_switch",1,mode_switch_sub_cb);

  ros::Timer timer = nh.createTimer(ros::Duration(0.1), timerCallback);


  auto ret = ros::matlab::gMatlabNodeIntr->run();
  ros::matlab::gMatlabNodeIntr->terminate();
  ros::matlab::gMatlabNodeIntr.reset();
   // ROS_INFO("start controller node");

  //ros::spin();

  return ret;
}
