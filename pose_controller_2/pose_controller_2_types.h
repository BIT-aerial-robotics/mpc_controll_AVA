//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: pose_controller_2_types.h
//
// Code generated for Simulink model 'pose_controller_2'.
//
// Model version                  : 1.27
// Simulink Coder version         : 9.4 (R2020b) 29-Jul-2020
// C/C++ source code generated on : Tue Jan 19 20:05:41 2021
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_pose_controller_2_types_h_
#define RTW_HEADER_pose_controller_2_types_h_
#include "rtwtypes.h"

// Model Code Variants
#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_pose_controller_2_geometry_msgs_Point_
#define DEFINED_TYPEDEF_FOR_SL_Bus_pose_controller_2_geometry_msgs_Point_

// MsgType=geometry_msgs/Point
typedef struct {
  real_T X;
  real_T Y;
  real_T Z;
} SL_Bus_pose_controller_2_geometry_msgs_Point;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_pose_controller_2_std_msgs_Float64_
#define DEFINED_TYPEDEF_FOR_SL_Bus_pose_controller_2_std_msgs_Float64_

// MsgType=std_msgs/Float64
typedef struct {
  real_T Data;
} SL_Bus_pose_controller_2_std_msgs_Float64;

#endif

#ifndef struct_tag_eFCXAaC7vLdwjksE0MwgOD
#define struct_tag_eFCXAaC7vLdwjksE0MwgOD

struct tag_eFCXAaC7vLdwjksE0MwgOD
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
};

#endif                                 //struct_tag_eFCXAaC7vLdwjksE0MwgOD

#ifndef typedef_ros_slroscpp_internal_block_S_T
#define typedef_ros_slroscpp_internal_block_S_T

typedef tag_eFCXAaC7vLdwjksE0MwgOD ros_slroscpp_internal_block_S_T;

#endif                                 //typedef_ros_slroscpp_internal_block_S_T

#ifndef struct_tag_D6QYIVJDjoKN0blLVEpT8F
#define struct_tag_D6QYIVJDjoKN0blLVEpT8F

struct tag_D6QYIVJDjoKN0blLVEpT8F
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
};

#endif                                 //struct_tag_D6QYIVJDjoKN0blLVEpT8F

#ifndef typedef_ros_slroscpp_internal_block_P_T
#define typedef_ros_slroscpp_internal_block_P_T

typedef tag_D6QYIVJDjoKN0blLVEpT8F ros_slroscpp_internal_block_P_T;

#endif                                 //typedef_ros_slroscpp_internal_block_P_T

// Parameters (default storage)
typedef struct P_pose_controller_2_T_ P_pose_controller_2_T;

// Forward declaration for rtModel
typedef struct tag_RTM_pose_controller_2_T RT_MODEL_pose_controller_2_T;

#endif                                 // RTW_HEADER_pose_controller_2_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
