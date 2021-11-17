#ifndef _SLROS_BUSMSG_CONVERSION_H_
#define _SLROS_BUSMSG_CONVERSION_H_

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64.h>
#include "pose_controller_2_types.h"
#include "slros_msgconvert_utils.h"


void convertFromBus(geometry_msgs::Point* msgPtr, SL_Bus_pose_controller_2_geometry_msgs_Point const* busPtr);
void convertToBus(SL_Bus_pose_controller_2_geometry_msgs_Point* busPtr, geometry_msgs::Point const* msgPtr);

void convertFromBus(std_msgs::Float64* msgPtr, SL_Bus_pose_controller_2_std_msgs_Float64 const* busPtr);
void convertToBus(SL_Bus_pose_controller_2_std_msgs_Float64* busPtr, std_msgs::Float64 const* msgPtr);


#endif
