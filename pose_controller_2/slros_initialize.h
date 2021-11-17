#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block pose_controller_2/Subscribe10
extern SimulinkSubscriber<geometry_msgs::Point, SL_Bus_pose_controller_2_geometry_msgs_Point> Sub_pose_controller_2_2548;

// For Block pose_controller_2/Subscribe9
extern SimulinkSubscriber<geometry_msgs::Point, SL_Bus_pose_controller_2_geometry_msgs_Point> Sub_pose_controller_2_2544;

// For Block pose_controller_2/Subsystem/Subscribe
extern SimulinkSubscriber<geometry_msgs::Point, SL_Bus_pose_controller_2_geometry_msgs_Point> Sub_pose_controller_2_222;

// For Block pose_controller_2/Subsystem/Subscribe1
extern SimulinkSubscriber<geometry_msgs::Point, SL_Bus_pose_controller_2_geometry_msgs_Point> Sub_pose_controller_2_225;

// For Block pose_controller_2/Subsystem/Subscribe2
extern SimulinkSubscriber<geometry_msgs::Point, SL_Bus_pose_controller_2_geometry_msgs_Point> Sub_pose_controller_2_229;

// For Block pose_controller_2/Subsystem/Subscribe3
extern SimulinkSubscriber<geometry_msgs::Point, SL_Bus_pose_controller_2_geometry_msgs_Point> Sub_pose_controller_2_232;

// For Block pose_controller_2/Subsystem/Subscribe4
extern SimulinkSubscriber<geometry_msgs::Point, SL_Bus_pose_controller_2_geometry_msgs_Point> Sub_pose_controller_2_235;

// For Block pose_controller_2/Subsystem/Subscribe5
extern SimulinkSubscriber<geometry_msgs::Point, SL_Bus_pose_controller_2_geometry_msgs_Point> Sub_pose_controller_2_238;

// For Block pose_controller_2/Subsystem/Subscribe6
extern SimulinkSubscriber<std_msgs::Float64, SL_Bus_pose_controller_2_std_msgs_Float64> Sub_pose_controller_2_2529;

// For Block pose_controller_2/Subsystem/Subscribe7
extern SimulinkSubscriber<std_msgs::Float64, SL_Bus_pose_controller_2_std_msgs_Float64> Sub_pose_controller_2_2533;

// For Block pose_controller_2/Subsystem/Subscribe8
extern SimulinkSubscriber<std_msgs::Float64, SL_Bus_pose_controller_2_std_msgs_Float64> Sub_pose_controller_2_2536;

// For Block pose_controller_2/Subsystem/Publish
extern SimulinkPublisher<std_msgs::Float64, SL_Bus_pose_controller_2_std_msgs_Float64> Pub_pose_controller_2_265;

// For Block pose_controller_2/Subsystem/Publish1
extern SimulinkPublisher<geometry_msgs::Point, SL_Bus_pose_controller_2_geometry_msgs_Point> Pub_pose_controller_2_266;

// For Block pose_controller_2/Subsystem/Publish2
extern SimulinkPublisher<std_msgs::Float64, SL_Bus_pose_controller_2_std_msgs_Float64> Pub_pose_controller_2_267;

// For Block pose_controller_2/Subsystem/Publish3
extern SimulinkPublisher<geometry_msgs::Point, SL_Bus_pose_controller_2_geometry_msgs_Point> Pub_pose_controller_2_268;

// For Block pose_controller_2/Subsystem/Publish4
extern SimulinkPublisher<std_msgs::Float64, SL_Bus_pose_controller_2_std_msgs_Float64> Pub_pose_controller_2_269;

// For Block pose_controller_2/Subsystem/Publish5
extern SimulinkPublisher<geometry_msgs::Point, SL_Bus_pose_controller_2_geometry_msgs_Point> Pub_pose_controller_2_270;

void slros_node_init(int argc, char** argv);

#endif
