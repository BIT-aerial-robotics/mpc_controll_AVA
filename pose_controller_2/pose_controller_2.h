//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: pose_controller_2.h
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
#ifndef RTW_HEADER_pose_controller_2_h_
#define RTW_HEADER_pose_controller_2_h_
#include <math.h>
#include <string.h>
#include <stddef.h>
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "slros_initialize.h"
#include "pose_controller_2_types.h"
#include "rtGetInf.h"
#include "rt_nonfinite.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetContStateDisabled
#define rtmGetContStateDisabled(rtm)   ((rtm)->contStateDisabled)
#endif

#ifndef rtmSetContStateDisabled
#define rtmSetContStateDisabled(rtm, val) ((rtm)->contStateDisabled = (val))
#endif

#ifndef rtmGetContStates
#define rtmGetContStates(rtm)          ((rtm)->contStates)
#endif

#ifndef rtmSetContStates
#define rtmSetContStates(rtm, val)     ((rtm)->contStates = (val))
#endif

#ifndef rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag
#define rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm) ((rtm)->CTOutputIncnstWithState)
#endif

#ifndef rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag
#define rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm, val) ((rtm)->CTOutputIncnstWithState = (val))
#endif

#ifndef rtmGetDerivCacheNeedsReset
#define rtmGetDerivCacheNeedsReset(rtm) ((rtm)->derivCacheNeedsReset)
#endif

#ifndef rtmSetDerivCacheNeedsReset
#define rtmSetDerivCacheNeedsReset(rtm, val) ((rtm)->derivCacheNeedsReset = (val))
#endif

#ifndef rtmGetIntgData
#define rtmGetIntgData(rtm)            ((rtm)->intgData)
#endif

#ifndef rtmSetIntgData
#define rtmSetIntgData(rtm, val)       ((rtm)->intgData = (val))
#endif

#ifndef rtmGetOdeF
#define rtmGetOdeF(rtm)                ((rtm)->odeF)
#endif

#ifndef rtmSetOdeF
#define rtmSetOdeF(rtm, val)           ((rtm)->odeF = (val))
#endif

#ifndef rtmGetOdeY
#define rtmGetOdeY(rtm)                ((rtm)->odeY)
#endif

#ifndef rtmSetOdeY
#define rtmSetOdeY(rtm, val)           ((rtm)->odeY = (val))
#endif

#ifndef rtmGetPeriodicContStateIndices
#define rtmGetPeriodicContStateIndices(rtm) ((rtm)->periodicContStateIndices)
#endif

#ifndef rtmSetPeriodicContStateIndices
#define rtmSetPeriodicContStateIndices(rtm, val) ((rtm)->periodicContStateIndices = (val))
#endif

#ifndef rtmGetPeriodicContStateRanges
#define rtmGetPeriodicContStateRanges(rtm) ((rtm)->periodicContStateRanges)
#endif

#ifndef rtmSetPeriodicContStateRanges
#define rtmSetPeriodicContStateRanges(rtm, val) ((rtm)->periodicContStateRanges = (val))
#endif

#ifndef rtmGetZCCacheNeedsReset
#define rtmGetZCCacheNeedsReset(rtm)   ((rtm)->zCCacheNeedsReset)
#endif

#ifndef rtmSetZCCacheNeedsReset
#define rtmSetZCCacheNeedsReset(rtm, val) ((rtm)->zCCacheNeedsReset = (val))
#endif

#ifndef rtmGetdX
#define rtmGetdX(rtm)                  ((rtm)->derivs)
#endif

#ifndef rtmSetdX
#define rtmSetdX(rtm, val)             ((rtm)->derivs = (val))
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

// Block signals for system '<S13>/MATLAB Function1'
typedef struct {
  real_T fw_tmp[9];
  real_T fw[3];
} B_MATLABFunction1_pose_contro_T;

// Block signals (default storage)
typedef struct {
  real_T B[54];
  real_T B_m[36];
  real_T b_A[36];
  real_T Rt[18];
  real_T Rd[9];
  real_T R[9];
  real_T Rt_c[9];
  real_T S1[9];
  real_T dv[9];
  real_T Rd_k[9];
  real_T Rt_cx[9];
  real_T dv1[9];
  real_T dv2[9];
  real_T LAMDA1_tmp[6];
  SL_Bus_pose_controller_2_geometry_msgs_Point In1;// '<S108>/In1'
  SL_Bus_pose_controller_2_geometry_msgs_Point In1_k;// '<S107>/In1'
  SL_Bus_pose_controller_2_geometry_msgs_Point In1_e;// '<S106>/In1'
  SL_Bus_pose_controller_2_geometry_msgs_Point In1_h;// '<S105>/In1'
  SL_Bus_pose_controller_2_geometry_msgs_Point In1_f;// '<S104>/In1'
  SL_Bus_pose_controller_2_geometry_msgs_Point In1_l;// '<S103>/In1'
  SL_Bus_pose_controller_2_geometry_msgs_Point In1_a;// '<S6>/In1'
  SL_Bus_pose_controller_2_geometry_msgs_Point In1_fj;// '<S5>/In1'
  SL_Bus_pose_controller_2_geometry_msgs_Point b_varargout_2;
  SL_Bus_pose_controller_2_geometry_msgs_Point BusAssignment5;// '<S4>/Bus Assignment5' 
  SL_Bus_pose_controller_2_geometry_msgs_Point BusAssignment4;// '<S4>/Bus Assignment4' 
  SL_Bus_pose_controller_2_geometry_msgs_Point BusAssignment3;// '<S4>/Bus Assignment3' 
  real_T vd[3];
  real_T epsilon_d_v[3];
  real_T de[3];
  real_T position_error[3];
  real_T Sum1_b[3];                    // '<S14>/Sum1'
  real_T ex[3];                        // '<S14>/Sum'
  real_T Rd_p[3];
  real_T position_error_c[3];
  char_T b_zeroDelimTopic[23];
  char_T b_zeroDelimTopic_f[22];
  char_T b_zeroDelimTopic_g[19];
  char_T b_zeroDelimTopic_g1[18];
  char_T b_zeroDelimTopic_m[17];
  real_T Integrator;                   // '<S56>/Integrator'
  real_T Integrator_e;                 // '<S58>/Integrator'
  real_T Integrator_i;                 // '<S57>/Integrator'
  real_T Integrator_k;                 // '<S69>/Integrator'
  real_T Integrator_n;                 // '<S70>/Integrator'
  real_T Integrator_a;                 // '<S71>/Integrator'
  real_T Integrator_aj;                // '<S49>/Integrator'
  real_T Integrator_j;                 // '<S50>/Integrator'
  real_T Integrator_h;                 // '<S51>/Integrator'
  real_T Integrator_jx;                // '<S52>/Integrator'
  real_T Integrator_a3;                // '<S53>/Integrator'
  real_T Integrator_jg;                // '<S54>/Integrator'
  real_T Add;                          // '<S32>/Add'
  real_T Sum3;                         // '<S13>/Sum3'
  real_T Sum4;                         // '<S13>/Sum4'
  real_T Sum1;                         // '<S13>/Sum1'
  real_T Integrator1;                  // '<S54>/Integrator1'
  real_T Sum;                          // '<S54>/Sum'
  real_T Integrator1_e;                // '<S53>/Integrator1'
  real_T Sum_n;                        // '<S53>/Sum'
  real_T Integrator1_h;                // '<S52>/Integrator1'
  real_T Sum_l;                        // '<S52>/Sum'
  real_T Integrator1_l;                // '<S51>/Integrator1'
  real_T Sum_j;                        // '<S51>/Sum'
  real_T Integrator1_ln;               // '<S50>/Integrator1'
  real_T Sum_k;                        // '<S50>/Sum'
  real_T Integrator1_n;                // '<S49>/Integrator1'
  real_T Sum_ng;                       // '<S49>/Sum'
  real_T Integrator1_k;                // '<S71>/Integrator1'
  real_T Sum_o;                        // '<S71>/Sum'
  real_T Integrator1_d;                // '<S70>/Integrator1'
  real_T Sum_ns;                       // '<S70>/Sum'
  real_T Integrator1_i;                // '<S69>/Integrator1'
  real_T Sum_e;                        // '<S69>/Sum'
  real_T Integrator1_g;                // '<S57>/Integrator1'
  real_T Sum_p;                        // '<S57>/Sum'
  real_T Integrator1_a;                // '<S58>/Integrator1'
  real_T Sum_c;                        // '<S58>/Sum'
  real_T Integrator1_f;                // '<S56>/Integrator1'
  real_T Sum_g;                        // '<S56>/Sum'
  real_T Ix_sys;                       // '<S15>/calc_J_sys'
  real_T Iy_sys;                       // '<S15>/calc_J_sys'
  real_T Iz_sys;                       // '<S15>/calc_J_sys'
  real_T grad1;                        // '<S40>/MATLAB Function2'
  real_T grad2;                        // '<S40>/MATLAB Function2'
  real_T grad3;                        // '<S40>/MATLAB Function2'
  real_T phi_error;                    // '<S40>/MATLAB Function2'
  real_T theta_error;                  // '<S40>/MATLAB Function2'
  real_T psi_error;                    // '<S40>/MATLAB Function2'
  real_T grad1_g;                      // '<S40>/MATLAB Function1'
  real_T grad2_p;                      // '<S40>/MATLAB Function1'
  real_T grad3_g;                      // '<S40>/MATLAB Function1'
  real_T x_error;                      // '<S40>/MATLAB Function1'
  real_T y_error;                      // '<S40>/MATLAB Function1'
  real_T z_error;                      // '<S40>/MATLAB Function1'
  real_T y;                            // '<S71>/MATLAB Function1'
  real_T y_a;                          // '<S70>/MATLAB Function1'
  real_T y_f;                          // '<S69>/MATLAB Function1'
  real_T y_n;                          // '<S58>/MATLAB Function1'
  real_T y_j;                          // '<S57>/MATLAB Function1'
  real_T y_l;                          // '<S56>/MATLAB Function1'
  real_T q_bar;                        // '<S47>/MATLAB Function'
  real_T Derivative_j;                 // '<S70>/Derivative'
  real_T Derivative_o;                 // '<S69>/Derivative'
  real_T Derivative_d;                 // '<S57>/Derivative'
  real_T Derivative_i;                 // '<S58>/Derivative'
  real_T Derivative;                   // '<S56>/Derivative'
  real_T vd_tmp;
  real_T vd_tmp_n;
  real_T rtb_q_bar_tmp;
  real_T rtb_q_bar_tmp_p;
  real_T rtb_q_bar_tmp_l;
  real_T rtb_p_bar_tmp;
  real_T Rd_tmp;
  real_T Rd_tmp_j;
  real_T Rd_tmp_d;
  real_T Rd_tmp_g;
  real_T Rd_tmp_l;
  real_T Rd_tmp_dh;
  real_T Rd_tmp_dy;
  real_T smax;
  real_T y_lx;
  SL_Bus_pose_controller_2_std_msgs_Float64 In1_g;// '<S111>/In1'
  SL_Bus_pose_controller_2_std_msgs_Float64 In1_c;// '<S110>/In1'
  SL_Bus_pose_controller_2_std_msgs_Float64 In1_p;// '<S109>/In1'
  SL_Bus_pose_controller_2_std_msgs_Float64 b_varargout_2_o;
  SL_Bus_pose_controller_2_std_msgs_Float64 BusAssignment2;// '<S4>/Bus Assignment2' 
  SL_Bus_pose_controller_2_std_msgs_Float64 BusAssignment1;// '<S4>/Bus Assignment1' 
  SL_Bus_pose_controller_2_std_msgs_Float64 BusAssignment;// '<S4>/Bus Assignment' 
  int8_T ipiv[6];
  int32_T i;
  int32_T Rt_tmp;
  int32_T R_tmp;
  int32_T i1;
  int32_T B_tmp;
  int32_T j;
  int32_T kAcol;
  int32_T ix;
  int32_T iy;
  int32_T jA;
  int32_T c_ix;
  int32_T d;
  int32_T ijA;
  B_MATLABFunction1_pose_contro_T sf_MATLABFunction3;// '<S13>/MATLAB Function3' 
  B_MATLABFunction1_pose_contro_T sf_MATLABFunction2;// '<S13>/MATLAB Function2' 
  B_MATLABFunction1_pose_contro_T sf_MATLABFunction1;// '<S13>/MATLAB Function1' 
} B_pose_controller_2_T;

// Block states (default storage) for system '<Root>'
typedef struct {
  ros_slroscpp_internal_block_S_T obj; // '<S30>/SourceBlock'
  ros_slroscpp_internal_block_S_T obj_g;// '<S29>/SourceBlock'
  ros_slroscpp_internal_block_S_T obj_f;// '<S28>/SourceBlock'
  ros_slroscpp_internal_block_S_T obj_m;// '<S27>/SourceBlock'
  ros_slroscpp_internal_block_S_T obj_a;// '<S26>/SourceBlock'
  ros_slroscpp_internal_block_S_T obj_o;// '<S25>/SourceBlock'
  ros_slroscpp_internal_block_S_T obj_ou;// '<S24>/SourceBlock'
  ros_slroscpp_internal_block_S_T obj_l;// '<S23>/SourceBlock'
  ros_slroscpp_internal_block_S_T obj_mx;// '<S22>/SourceBlock'
  ros_slroscpp_internal_block_S_T obj_j;// '<S3>/SourceBlock'
  ros_slroscpp_internal_block_S_T obj_lc;// '<S2>/SourceBlock'
  ros_slroscpp_internal_block_P_T obj_j4;// '<S21>/SinkBlock'
  ros_slroscpp_internal_block_P_T obj_of;// '<S20>/SinkBlock'
  ros_slroscpp_internal_block_P_T obj_b;// '<S19>/SinkBlock'
  ros_slroscpp_internal_block_P_T obj_k;// '<S18>/SinkBlock'
  ros_slroscpp_internal_block_P_T obj_e;// '<S17>/SinkBlock'
  ros_slroscpp_internal_block_P_T obj_gb;// '<S16>/SinkBlock'
  real_T TimeStampA;                   // '<S56>/Derivative'
  real_T LastUAtTimeA;                 // '<S56>/Derivative'
  real_T TimeStampB;                   // '<S56>/Derivative'
  real_T LastUAtTimeB;                 // '<S56>/Derivative'
  real_T TimeStampA_n;                 // '<S58>/Derivative'
  real_T LastUAtTimeA_l;               // '<S58>/Derivative'
  real_T TimeStampB_g;                 // '<S58>/Derivative'
  real_T LastUAtTimeB_h;               // '<S58>/Derivative'
  real_T TimeStampA_nq;                // '<S57>/Derivative'
  real_T LastUAtTimeA_e;               // '<S57>/Derivative'
  real_T TimeStampB_e;                 // '<S57>/Derivative'
  real_T LastUAtTimeB_n;               // '<S57>/Derivative'
  real_T TimeStampA_f;                 // '<S69>/Derivative'
  real_T LastUAtTimeA_d;               // '<S69>/Derivative'
  real_T TimeStampB_f;                 // '<S69>/Derivative'
  real_T LastUAtTimeB_f;               // '<S69>/Derivative'
  real_T TimeStampA_j;                 // '<S70>/Derivative'
  real_T LastUAtTimeA_n;               // '<S70>/Derivative'
  real_T TimeStampB_f4;                // '<S70>/Derivative'
  real_T LastUAtTimeB_l;               // '<S70>/Derivative'
  real_T TimeStampA_m;                 // '<S71>/Derivative'
  real_T LastUAtTimeA_j;               // '<S71>/Derivative'
  real_T TimeStampB_l;                 // '<S71>/Derivative'
  real_T LastUAtTimeB_b;               // '<S71>/Derivative'
  real_T TimeStampA_fq;                // '<S49>/Derivative'
  real_T LastUAtTimeA_et;              // '<S49>/Derivative'
  real_T TimeStampB_m;                 // '<S49>/Derivative'
  real_T LastUAtTimeB_ba;              // '<S49>/Derivative'
  real_T TimeStampA_a;                 // '<S50>/Derivative'
  real_T LastUAtTimeA_p;               // '<S50>/Derivative'
  real_T TimeStampB_gd;                // '<S50>/Derivative'
  real_T LastUAtTimeB_d;               // '<S50>/Derivative'
  real_T TimeStampA_l;                 // '<S51>/Derivative'
  real_T LastUAtTimeA_g;               // '<S51>/Derivative'
  real_T TimeStampB_p;                 // '<S51>/Derivative'
  real_T LastUAtTimeB_lt;              // '<S51>/Derivative'
  real_T TimeStampA_h;                 // '<S52>/Derivative'
  real_T LastUAtTimeA_b;               // '<S52>/Derivative'
  real_T TimeStampB_eu;                // '<S52>/Derivative'
  real_T LastUAtTimeB_hq;              // '<S52>/Derivative'
  real_T TimeStampA_hg;                // '<S53>/Derivative'
  real_T LastUAtTimeA_o;               // '<S53>/Derivative'
  real_T TimeStampB_h;                 // '<S53>/Derivative'
  real_T LastUAtTimeB_g;               // '<S53>/Derivative'
  real_T TimeStampA_p;                 // '<S54>/Derivative'
  real_T LastUAtTimeA_m;               // '<S54>/Derivative'
  real_T TimeStampB_gm;                // '<S54>/Derivative'
  real_T LastUAtTimeB_fn;              // '<S54>/Derivative'
  int_T Integrator_IWORK;              // '<S56>/Integrator'
  int_T Integrator_IWORK_e;            // '<S58>/Integrator'
  int_T Integrator_IWORK_l;            // '<S57>/Integrator'
  int_T Integrator_IWORK_g;            // '<S69>/Integrator'
  int_T Integrator_IWORK_k;            // '<S70>/Integrator'
  int_T Integrator_IWORK_d;            // '<S71>/Integrator'
  int_T Integrator_IWORK_lt;           // '<S44>/Integrator'
  int_T Integrator1_IWORK;             // '<S44>/Integrator1'
  int_T Integrator2_IWORK;             // '<S44>/Integrator2'
  int_T Integrator_IWORK_df;           // '<S42>/Integrator'
  int_T Integrator1_IWORK_g;           // '<S42>/Integrator1'
  int_T Integrator2_IWORK_m;           // '<S42>/Integrator2'
  boolean_T Subsystem_MODE;            // '<Root>/Subsystem'
} DW_pose_controller_2_T;

// Continuous states (default storage)
typedef struct {
  real_T Integrator_CSTATE;            // '<S56>/Integrator'
  real_T Integrator_CSTATE_k;          // '<S58>/Integrator'
  real_T Integrator_CSTATE_j;          // '<S57>/Integrator'
  real_T Integrator_CSTATE_k3;         // '<S69>/Integrator'
  real_T Integrator_CSTATE_c;          // '<S70>/Integrator'
  real_T Integrator_CSTATE_i;          // '<S71>/Integrator'
  real_T Integrator_CSTATE_cv;         // '<S49>/Integrator'
  real_T Integrator_CSTATE_b;          // '<S50>/Integrator'
  real_T Integrator_CSTATE_o;          // '<S51>/Integrator'
  real_T Integrator_CSTATE_a;          // '<S44>/Integrator'
  real_T Integrator1_CSTATE;           // '<S44>/Integrator1'
  real_T Integrator2_CSTATE;           // '<S44>/Integrator2'
  real_T Integrator_CSTATE_jf;         // '<S52>/Integrator'
  real_T Integrator_CSTATE_p;          // '<S53>/Integrator'
  real_T Integrator_CSTATE_m;          // '<S54>/Integrator'
  real_T Integrator_CSTATE_e;          // '<S42>/Integrator'
  real_T Integrator1_CSTATE_p;         // '<S42>/Integrator1'
  real_T Integrator2_CSTATE_c;         // '<S42>/Integrator2'
  real_T Integrator1_CSTATE_k;         // '<S54>/Integrator1'
  real_T Integrator1_CSTATE_h;         // '<S53>/Integrator1'
  real_T Integrator1_CSTATE_g;         // '<S52>/Integrator1'
  real_T Integrator1_CSTATE_b;         // '<S51>/Integrator1'
  real_T Integrator1_CSTATE_d;         // '<S50>/Integrator1'
  real_T Integrator1_CSTATE_n;         // '<S49>/Integrator1'
  real_T Integrator1_CSTATE_pj;        // '<S71>/Integrator1'
  real_T Integrator1_CSTATE_po;        // '<S70>/Integrator1'
  real_T Integrator1_CSTATE_b2;        // '<S69>/Integrator1'
  real_T Integrator1_CSTATE_j;         // '<S57>/Integrator1'
  real_T Integrator1_CSTATE_n2;        // '<S58>/Integrator1'
  real_T Integrator1_CSTATE_i;         // '<S56>/Integrator1'
} X_pose_controller_2_T;

// State derivatives (default storage)
typedef struct {
  real_T Integrator_CSTATE;            // '<S56>/Integrator'
  real_T Integrator_CSTATE_k;          // '<S58>/Integrator'
  real_T Integrator_CSTATE_j;          // '<S57>/Integrator'
  real_T Integrator_CSTATE_k3;         // '<S69>/Integrator'
  real_T Integrator_CSTATE_c;          // '<S70>/Integrator'
  real_T Integrator_CSTATE_i;          // '<S71>/Integrator'
  real_T Integrator_CSTATE_cv;         // '<S49>/Integrator'
  real_T Integrator_CSTATE_b;          // '<S50>/Integrator'
  real_T Integrator_CSTATE_o;          // '<S51>/Integrator'
  real_T Integrator_CSTATE_a;          // '<S44>/Integrator'
  real_T Integrator1_CSTATE;           // '<S44>/Integrator1'
  real_T Integrator2_CSTATE;           // '<S44>/Integrator2'
  real_T Integrator_CSTATE_jf;         // '<S52>/Integrator'
  real_T Integrator_CSTATE_p;          // '<S53>/Integrator'
  real_T Integrator_CSTATE_m;          // '<S54>/Integrator'
  real_T Integrator_CSTATE_e;          // '<S42>/Integrator'
  real_T Integrator1_CSTATE_p;         // '<S42>/Integrator1'
  real_T Integrator2_CSTATE_c;         // '<S42>/Integrator2'
  real_T Integrator1_CSTATE_k;         // '<S54>/Integrator1'
  real_T Integrator1_CSTATE_h;         // '<S53>/Integrator1'
  real_T Integrator1_CSTATE_g;         // '<S52>/Integrator1'
  real_T Integrator1_CSTATE_b;         // '<S51>/Integrator1'
  real_T Integrator1_CSTATE_d;         // '<S50>/Integrator1'
  real_T Integrator1_CSTATE_n;         // '<S49>/Integrator1'
  real_T Integrator1_CSTATE_pj;        // '<S71>/Integrator1'
  real_T Integrator1_CSTATE_po;        // '<S70>/Integrator1'
  real_T Integrator1_CSTATE_b2;        // '<S69>/Integrator1'
  real_T Integrator1_CSTATE_j;         // '<S57>/Integrator1'
  real_T Integrator1_CSTATE_n2;        // '<S58>/Integrator1'
  real_T Integrator1_CSTATE_i;         // '<S56>/Integrator1'
} XDot_pose_controller_2_T;

// State disabled
typedef struct {
  boolean_T Integrator_CSTATE;         // '<S56>/Integrator'
  boolean_T Integrator_CSTATE_k;       // '<S58>/Integrator'
  boolean_T Integrator_CSTATE_j;       // '<S57>/Integrator'
  boolean_T Integrator_CSTATE_k3;      // '<S69>/Integrator'
  boolean_T Integrator_CSTATE_c;       // '<S70>/Integrator'
  boolean_T Integrator_CSTATE_i;       // '<S71>/Integrator'
  boolean_T Integrator_CSTATE_cv;      // '<S49>/Integrator'
  boolean_T Integrator_CSTATE_b;       // '<S50>/Integrator'
  boolean_T Integrator_CSTATE_o;       // '<S51>/Integrator'
  boolean_T Integrator_CSTATE_a;       // '<S44>/Integrator'
  boolean_T Integrator1_CSTATE;        // '<S44>/Integrator1'
  boolean_T Integrator2_CSTATE;        // '<S44>/Integrator2'
  boolean_T Integrator_CSTATE_jf;      // '<S52>/Integrator'
  boolean_T Integrator_CSTATE_p;       // '<S53>/Integrator'
  boolean_T Integrator_CSTATE_m;       // '<S54>/Integrator'
  boolean_T Integrator_CSTATE_e;       // '<S42>/Integrator'
  boolean_T Integrator1_CSTATE_p;      // '<S42>/Integrator1'
  boolean_T Integrator2_CSTATE_c;      // '<S42>/Integrator2'
  boolean_T Integrator1_CSTATE_k;      // '<S54>/Integrator1'
  boolean_T Integrator1_CSTATE_h;      // '<S53>/Integrator1'
  boolean_T Integrator1_CSTATE_g;      // '<S52>/Integrator1'
  boolean_T Integrator1_CSTATE_b;      // '<S51>/Integrator1'
  boolean_T Integrator1_CSTATE_d;      // '<S50>/Integrator1'
  boolean_T Integrator1_CSTATE_n;      // '<S49>/Integrator1'
  boolean_T Integrator1_CSTATE_pj;     // '<S71>/Integrator1'
  boolean_T Integrator1_CSTATE_po;     // '<S70>/Integrator1'
  boolean_T Integrator1_CSTATE_b2;     // '<S69>/Integrator1'
  boolean_T Integrator1_CSTATE_j;      // '<S57>/Integrator1'
  boolean_T Integrator1_CSTATE_n2;     // '<S58>/Integrator1'
  boolean_T Integrator1_CSTATE_i;      // '<S56>/Integrator1'
} XDis_pose_controller_2_T;

#ifndef ODE3_INTG
#define ODE3_INTG

// ODE3 Integration Data
typedef struct {
  real_T *y;                           // output
  real_T *f[3];                        // derivatives
} ODE3_IntgData;

#endif

// Parameters (default storage)
struct P_pose_controller_2_T_ {
  SL_Bus_pose_controller_2_geometry_msgs_Point Out1_Y0;// Computed Parameter: Out1_Y0
                                                          //  Referenced by: '<S5>/Out1'

  SL_Bus_pose_controller_2_geometry_msgs_Point Constant_Value;// Computed Parameter: Constant_Value
                                                                 //  Referenced by: '<S2>/Constant'

  SL_Bus_pose_controller_2_geometry_msgs_Point Out1_Y0_p;// Computed Parameter: Out1_Y0_p
                                                            //  Referenced by: '<S6>/Out1'

  SL_Bus_pose_controller_2_geometry_msgs_Point Constant_Value_i;// Computed Parameter: Constant_Value_i
                                                                   //  Referenced by: '<S3>/Constant'

  SL_Bus_pose_controller_2_geometry_msgs_Point Constant_Value_o;// Computed Parameter: Constant_Value_o
                                                                   //  Referenced by: '<S7>/Constant'

  SL_Bus_pose_controller_2_geometry_msgs_Point Constant_Value_b;// Computed Parameter: Constant_Value_b
                                                                   //  Referenced by: '<S8>/Constant'

  SL_Bus_pose_controller_2_geometry_msgs_Point Constant_Value_a;// Computed Parameter: Constant_Value_a
                                                                   //  Referenced by: '<S9>/Constant'

  SL_Bus_pose_controller_2_geometry_msgs_Point Out1_Y0_p4;// Computed Parameter: Out1_Y0_p4
                                                             //  Referenced by: '<S103>/Out1'

  SL_Bus_pose_controller_2_geometry_msgs_Point Constant_Value_ii;// Computed Parameter: Constant_Value_ii
                                                                    //  Referenced by: '<S22>/Constant'

  SL_Bus_pose_controller_2_geometry_msgs_Point Out1_Y0_d;// Computed Parameter: Out1_Y0_d
                                                            //  Referenced by: '<S104>/Out1'

  SL_Bus_pose_controller_2_geometry_msgs_Point Constant_Value_c;// Computed Parameter: Constant_Value_c
                                                                   //  Referenced by: '<S23>/Constant'

  SL_Bus_pose_controller_2_geometry_msgs_Point Out1_Y0_f;// Computed Parameter: Out1_Y0_f
                                                            //  Referenced by: '<S105>/Out1'

  SL_Bus_pose_controller_2_geometry_msgs_Point Constant_Value_d;// Computed Parameter: Constant_Value_d
                                                                   //  Referenced by: '<S24>/Constant'

  SL_Bus_pose_controller_2_geometry_msgs_Point Out1_Y0_m;// Computed Parameter: Out1_Y0_m
                                                            //  Referenced by: '<S106>/Out1'

  SL_Bus_pose_controller_2_geometry_msgs_Point Constant_Value_cf;// Computed Parameter: Constant_Value_cf
                                                                    //  Referenced by: '<S25>/Constant'

  SL_Bus_pose_controller_2_geometry_msgs_Point Out1_Y0_h;// Computed Parameter: Out1_Y0_h
                                                            //  Referenced by: '<S107>/Out1'

  SL_Bus_pose_controller_2_geometry_msgs_Point Constant_Value_b1;// Computed Parameter: Constant_Value_b1
                                                                    //  Referenced by: '<S26>/Constant'

  SL_Bus_pose_controller_2_geometry_msgs_Point Out1_Y0_k;// Computed Parameter: Out1_Y0_k
                                                            //  Referenced by: '<S108>/Out1'

  SL_Bus_pose_controller_2_geometry_msgs_Point Constant_Value_h;// Computed Parameter: Constant_Value_h
                                                                   //  Referenced by: '<S27>/Constant'

  SL_Bus_pose_controller_2_std_msgs_Float64 Constant_Value_cn;// Computed Parameter: Constant_Value_cn
                                                                 //  Referenced by: '<S10>/Constant'

  SL_Bus_pose_controller_2_std_msgs_Float64 Constant_Value_n;// Computed Parameter: Constant_Value_n
                                                                //  Referenced by: '<S11>/Constant'

  SL_Bus_pose_controller_2_std_msgs_Float64 Constant_Value_g;// Computed Parameter: Constant_Value_g
                                                                //  Referenced by: '<S12>/Constant'

  SL_Bus_pose_controller_2_std_msgs_Float64 Out1_Y0_p3;// Computed Parameter: Out1_Y0_p3
                                                          //  Referenced by: '<S109>/Out1'

  SL_Bus_pose_controller_2_std_msgs_Float64 Constant_Value_j;// Computed Parameter: Constant_Value_j
                                                                //  Referenced by: '<S28>/Constant'

  SL_Bus_pose_controller_2_std_msgs_Float64 Out1_Y0_n;// Computed Parameter: Out1_Y0_n
                                                         //  Referenced by: '<S110>/Out1'

  SL_Bus_pose_controller_2_std_msgs_Float64 Constant_Value_aw;// Computed Parameter: Constant_Value_aw
                                                                 //  Referenced by: '<S29>/Constant'

  SL_Bus_pose_controller_2_std_msgs_Float64 Out1_Y0_kk;// Computed Parameter: Out1_Y0_kk
                                                          //  Referenced by: '<S111>/Out1'

  SL_Bus_pose_controller_2_std_msgs_Float64 Constant_Value_l;// Computed Parameter: Constant_Value_l
                                                                //  Referenced by: '<S30>/Constant'

  real_T tool_x_Value;                 // Expression: 0
                                          //  Referenced by: '<S31>/tool_x'

  real_T tool_y_Value;                 // Expression: 0
                                          //  Referenced by: '<S31>/tool_y'

  real_T tool_z_Value;                 // Expression: 0
                                          //  Referenced by: '<S31>/tool_z'

  real_T Integrator_IC;                // Expression: 0
                                          //  Referenced by: '<S49>/Integrator'

  real_T Integrator_IC_d;              // Expression: 0
                                          //  Referenced by: '<S50>/Integrator'

  real_T Integrator_IC_a;              // Expression: 0
                                          //  Referenced by: '<S51>/Integrator'

  real_T kp_x_Value;                   // Expression: 5
                                          //  Referenced by: '<S40>/kp_x'

  real_T kp_y_Value;                   // Expression: 5
                                          //  Referenced by: '<S40>/kp_y'

  real_T kp_z_Value;                   // Expression: 5
                                          //  Referenced by: '<S40>/kp_z'

  real_T kp_phi_Value;                 // Expression: 3
                                          //  Referenced by: '<S40>/kp_phi'

  real_T kp_theta_Value;               // Expression: 3
                                          //  Referenced by: '<S40>/kp_theta'

  real_T kp_psi_Value;                 // Expression: 5
                                          //  Referenced by: '<S40>/kp_psi'

  real_T drone1_x_Value;               // Expression: 0.6737
                                          //  Referenced by: '<S31>/drone1_x'

  real_T drone1_y_Value;               // Expression: 0
                                          //  Referenced by: '<S31>/drone1_y'

  real_T drone1_z_Value;               // Expression: -0.12468
                                          //  Referenced by: '<S31>/drone1_z'

  real_T drone2_x_Value;               // Expression: -0.3368
                                          //  Referenced by: '<S31>/drone2_x'

  real_T drone2_y_Value;               // Expression: 0.5834
                                          //  Referenced by: '<S31>/drone2_y'

  real_T drone2_z_Value;               // Expression: -0.12468
                                          //  Referenced by: '<S31>/drone2_z'

  real_T drone3_x_Value;               // Expression: -0.3368
                                          //  Referenced by: '<S31>/drone3_x'

  real_T drone3_y_Value;               // Expression: -0.5834
                                          //  Referenced by: '<S31>/drone3_y'

  real_T drone3_z_Value;               // Expression: -0.12468
                                          //  Referenced by: '<S31>/drone3_z'

  real_T center_mass_Value;            // Expression: 2.5
                                          //  Referenced by: '<S32>/center_mass'

  real_T drone1_mass_Value;            // Expression: 2.93
                                          //  Referenced by: '<S32>/drone1_mass'

  real_T drone2_mass_Value;            // Expression: 2.975
                                          //  Referenced by: '<S32>/drone2_mass'

  real_T drone3_mass_Value;            // Expression: 2.85
                                          //  Referenced by: '<S32>/drone3_mass'

  real_T Ixx_Value;                    // Expression: 0.06250784647
                                          //  Referenced by: '<S101>/Ixx'

  real_T Iyy_Value;                    // Expression: 0.06280979995
                                          //  Referenced by: '<S101>/Iyy'

  real_T Izz_Value;                    // Expression: 0.11503523247
                                          //  Referenced by: '<S101>/Izz'

  real_T kd_phi_Value;                 // Expression: 9
                                          //  Referenced by: '<S38>/kd_phi'

  real_T kd_theta_Value;               // Expression: 9
                                          //  Referenced by: '<S38>/kd_theta'

  real_T kd_psi_Value;                 // Expression: 15
                                          //  Referenced by: '<S38>/kd_psi'

  real_T ki_phi_Value;                 // Expression: 1.4
                                          //  Referenced by: '<S38>/ki_phi'

  real_T ki_theta_Value;               // Expression: 1.4
                                          //  Referenced by: '<S38>/ki_theta'

  real_T ki_psi_Value;                 // Expression: 2
                                          //  Referenced by: '<S38>/ki_psi'

  real_T Integrator_IC_g;              // Expression: 0
                                          //  Referenced by: '<S52>/Integrator'

  real_T Integrator_IC_b;              // Expression: 0
                                          //  Referenced by: '<S53>/Integrator'

  real_T Integrator_IC_ad;             // Expression: 0
                                          //  Referenced by: '<S54>/Integrator'

  real_T kd_x_Value;                   // Expression: 15
                                          //  Referenced by: '<S37>/kd_x'

  real_T kd_y_Value;                   // Expression: 15
                                          //  Referenced by: '<S37>/kd_y'

  real_T kd_z_Value;                   // Expression: 15
                                          //  Referenced by: '<S37>/kd_z'

  real_T ki_x_Value;                   // Expression: 2
                                          //  Referenced by: '<S37>/ki_x'

  real_T ki_y_Value;                   // Expression: 2
                                          //  Referenced by: '<S37>/ki_y'

  real_T ki_z_Value;                   // Expression: 2
                                          //  Referenced by: '<S37>/ki_z'

  real_T omega_w_Value;                // Expression: 10
                                          //  Referenced by: '<S54>/omega_w'

  real_T Integrator1_IC;               // Expression: 0
                                          //  Referenced by: '<S54>/Integrator1'

  real_T epsilon_w_Value;              // Expression: 1.414
                                          //  Referenced by: '<S54>/epsilon_w'

  real_T omega_v_Value;                // Expression: 24
                                          //  Referenced by: '<S53>/omega_v'

  real_T Integrator1_IC_j;             // Expression: 0
                                          //  Referenced by: '<S53>/Integrator1'

  real_T epsilon_v_Value;              // Expression: 1.414
                                          //  Referenced by: '<S53>/epsilon_v'

  real_T omega_u_Value;                // Expression: 15
                                          //  Referenced by: '<S52>/omega_u'

  real_T Integrator1_IC_p;             // Expression: 0
                                          //  Referenced by: '<S52>/Integrator1'

  real_T epsilon_u_Value;              // Expression: 1.414
                                          //  Referenced by: '<S52>/epsilon_u'

  real_T omega_r_Value;                // Expression: 20
                                          //  Referenced by: '<S51>/omega_r'

  real_T Integrator1_IC_b;             // Expression: 0
                                          //  Referenced by: '<S51>/Integrator1'

  real_T epsilon_r_Value;              // Expression: 1.414
                                          //  Referenced by: '<S51>/epsilon_r'

  real_T omega_q_Value;                // Expression: 20
                                          //  Referenced by: '<S50>/omega_q'

  real_T Integrator1_IC_c;             // Expression: 0
                                          //  Referenced by: '<S50>/Integrator1'

  real_T epsilon_q_Value;              // Expression: 1.414
                                          //  Referenced by: '<S50>/epsilon_q'

  real_T omega_p_Value;                // Expression: 24
                                          //  Referenced by: '<S49>/omega_p'

  real_T Integrator1_IC_g;             // Expression: 0
                                          //  Referenced by: '<S49>/Integrator1'

  real_T epsilon_p_Value;              // Expression: 1.414
                                          //  Referenced by: '<S49>/epsilon_p'

  real_T omega_z_Value;                // Expression: 5
                                          //  Referenced by: '<S71>/omega_z'

  real_T Integrator1_IC_h;             // Expression: 0
                                          //  Referenced by: '<S71>/Integrator1'

  real_T epsilon_z_Value;              // Expression: 1.414
                                          //  Referenced by: '<S71>/epsilon_z'

  real_T omega_y_Value;                // Expression: 8
                                          //  Referenced by: '<S70>/omega_y'

  real_T Integrator1_IC_pm;            // Expression: 0
                                          //  Referenced by: '<S70>/Integrator1'

  real_T epsilon_y_Value;              // Expression: 1.414
                                          //  Referenced by: '<S70>/epsilon_y'

  real_T omega_x_Value;                // Expression: 5
                                          //  Referenced by: '<S69>/omega_x'

  real_T Integrator1_IC_a;             // Expression: 0
                                          //  Referenced by: '<S69>/Integrator1'

  real_T epsilon_x_Value;              // Expression: 1.414
                                          //  Referenced by: '<S69>/epsilon_x'

  real_T omega_psi_Value;              // Expression: 10
                                          //  Referenced by: '<S57>/omega_psi'

  real_T Integrator1_IC_bk;            // Expression: 0
                                          //  Referenced by: '<S57>/Integrator1'

  real_T epsilon_psi_Value;            // Expression: 1.414
                                          //  Referenced by: '<S57>/epsilon_psi'

  real_T omega_theta_Value;            // Expression: 6.9
                                          //  Referenced by: '<S58>/omega_theta'

  real_T Integrator1_IC_e;             // Expression: 0
                                          //  Referenced by: '<S58>/Integrator1'

  real_T epsilon_theta_Value;          // Expression: 1.414
                                          //  Referenced by: '<S58>/epsilon_theta'

  real_T omega_phi_Value;              // Expression: 8
                                          //  Referenced by: '<S56>/omega_phi'

  real_T Integrator1_IC_m;             // Expression: 0
                                          //  Referenced by: '<S56>/Integrator1'

  real_T epsilon_phi_Value;            // Expression: 1.414
                                          //  Referenced by: '<S56>/epsilon_phi'

};

// Real-time Model Data Structure
struct tag_RTM_pose_controller_2_T {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  X_pose_controller_2_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  boolean_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[30];
  real_T odeF[3][30];
  ODE3_IntgData intgData;

  //
  //  Sizes:
  //  The following substructure contains sizes information
  //  for many of the model attributes such as inputs, outputs,
  //  dwork, sample times, etc.

  struct {
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  } Sizes;

  //
  //  Timing:
  //  The following substructure contains information regarding
  //  the timing information for the model.

  struct {
    uint32_T clockTick0;
    time_T stepSize0;
    uint32_T clockTick1;
    boolean_T firstInitCondFlag;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

// Block parameters (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern P_pose_controller_2_T pose_controller_2_P;

#ifdef __cplusplus

}
#endif

// Block signals (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern B_pose_controller_2_T pose_controller_2_B;

#ifdef __cplusplus

}
#endif

// Continuous states (default storage)
extern X_pose_controller_2_T pose_controller_2_X;

// Block states (default storage)
extern DW_pose_controller_2_T pose_controller_2_DW;

#ifdef __cplusplus

extern "C" {

#endif

  // Model entry point functions
  extern void pose_controller_2_initialize(void);
  extern void pose_controller_2_step(void);
  extern void pose_controller_2_terminate(void);

#ifdef __cplusplus

}
#endif

// Real-time Model object
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_pose_controller_2_T *const pose_controller_2_M;

#ifdef __cplusplus

}
#endif

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S13>/Scope1' : Unused code path elimination
//  Block '<S13>/Scope2' : Unused code path elimination
//  Block '<S13>/Scope4' : Unused code path elimination


//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'pose_controller_2'
//  '<S1>'   : 'pose_controller_2/MATLAB Function'
//  '<S2>'   : 'pose_controller_2/Subscribe10'
//  '<S3>'   : 'pose_controller_2/Subscribe9'
//  '<S4>'   : 'pose_controller_2/Subsystem'
//  '<S5>'   : 'pose_controller_2/Subscribe10/Enabled Subsystem'
//  '<S6>'   : 'pose_controller_2/Subscribe9/Enabled Subsystem'
//  '<S7>'   : 'pose_controller_2/Subsystem/Blank Message'
//  '<S8>'   : 'pose_controller_2/Subsystem/Blank Message1'
//  '<S9>'   : 'pose_controller_2/Subsystem/Blank Message2'
//  '<S10>'  : 'pose_controller_2/Subsystem/Blank Message3'
//  '<S11>'  : 'pose_controller_2/Subsystem/Blank Message4'
//  '<S12>'  : 'pose_controller_2/Subsystem/Blank Message6'
//  '<S13>'  : 'pose_controller_2/Subsystem/Control Allocarion'
//  '<S14>'  : 'pose_controller_2/Subsystem/Controller'
//  '<S15>'  : 'pose_controller_2/Subsystem/J'
//  '<S16>'  : 'pose_controller_2/Subsystem/Publish'
//  '<S17>'  : 'pose_controller_2/Subsystem/Publish1'
//  '<S18>'  : 'pose_controller_2/Subsystem/Publish2'
//  '<S19>'  : 'pose_controller_2/Subsystem/Publish3'
//  '<S20>'  : 'pose_controller_2/Subsystem/Publish4'
//  '<S21>'  : 'pose_controller_2/Subsystem/Publish5'
//  '<S22>'  : 'pose_controller_2/Subsystem/Subscribe'
//  '<S23>'  : 'pose_controller_2/Subsystem/Subscribe1'
//  '<S24>'  : 'pose_controller_2/Subsystem/Subscribe2'
//  '<S25>'  : 'pose_controller_2/Subsystem/Subscribe3'
//  '<S26>'  : 'pose_controller_2/Subsystem/Subscribe4'
//  '<S27>'  : 'pose_controller_2/Subsystem/Subscribe5'
//  '<S28>'  : 'pose_controller_2/Subsystem/Subscribe6'
//  '<S29>'  : 'pose_controller_2/Subsystem/Subscribe7'
//  '<S30>'  : 'pose_controller_2/Subsystem/Subscribe8'
//  '<S31>'  : 'pose_controller_2/Subsystem/X_0'
//  '<S32>'  : 'pose_controller_2/Subsystem/mass'
//  '<S33>'  : 'pose_controller_2/Subsystem/Control Allocarion/MATLAB Function'
//  '<S34>'  : 'pose_controller_2/Subsystem/Control Allocarion/MATLAB Function1'
//  '<S35>'  : 'pose_controller_2/Subsystem/Control Allocarion/MATLAB Function2'
//  '<S36>'  : 'pose_controller_2/Subsystem/Control Allocarion/MATLAB Function3'
//  '<S37>'  : 'pose_controller_2/Subsystem/Controller/calc_Thrusts'
//  '<S38>'  : 'pose_controller_2/Subsystem/Controller/calc_Torques'
//  '<S39>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d'
//  '<S40>'  : 'pose_controller_2/Subsystem/Controller/calc_grad'
//  '<S41>'  : 'pose_controller_2/Subsystem/Controller/calc_Thrusts/MATLAB Function'
//  '<S42>'  : 'pose_controller_2/Subsystem/Controller/calc_Thrusts/Subsystem'
//  '<S43>'  : 'pose_controller_2/Subsystem/Controller/calc_Torques/MATLAB Function'
//  '<S44>'  : 'pose_controller_2/Subsystem/Controller/calc_Torques/Subsystem'
//  '<S45>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/MATLAB Function'
//  '<S46>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/MATLAB Function1'
//  '<S47>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/Rotational kinematic inverse'
//  '<S48>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/Translational kinematic inverse'
//  '<S49>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/pseudo second order differentiator p'
//  '<S50>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/pseudo second order differentiator q'
//  '<S51>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/pseudo second order differentiator r'
//  '<S52>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/pseudo second order differentiator u'
//  '<S53>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/pseudo second order differentiator v'
//  '<S54>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/pseudo second order differentiator w'
//  '<S55>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/Rotational kinematic inverse/MATLAB Function'
//  '<S56>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/Rotational kinematic inverse/pseudo second order differentiator phi'
//  '<S57>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/Rotational kinematic inverse/pseudo second order differentiator psi'
//  '<S58>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/Rotational kinematic inverse/pseudo second order differentiator theta'
//  '<S59>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/Rotational kinematic inverse/pseudo second order differentiator phi/MATLAB Function1'
//  '<S60>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/Rotational kinematic inverse/pseudo second order differentiator phi/MATLAB Function2'
//  '<S61>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/Rotational kinematic inverse/pseudo second order differentiator phi/MATLAB Function3'
//  '<S62>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/Rotational kinematic inverse/pseudo second order differentiator psi/MATLAB Function1'
//  '<S63>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/Rotational kinematic inverse/pseudo second order differentiator psi/MATLAB Function2'
//  '<S64>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/Rotational kinematic inverse/pseudo second order differentiator psi/MATLAB Function3'
//  '<S65>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/Rotational kinematic inverse/pseudo second order differentiator theta/MATLAB Function1'
//  '<S66>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/Rotational kinematic inverse/pseudo second order differentiator theta/MATLAB Function2'
//  '<S67>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/Rotational kinematic inverse/pseudo second order differentiator theta/MATLAB Function3'
//  '<S68>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/Translational kinematic inverse/MATLAB Function'
//  '<S69>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/Translational kinematic inverse/pseudo second order differentiator x'
//  '<S70>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/Translational kinematic inverse/pseudo second order differentiator y'
//  '<S71>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/Translational kinematic inverse/pseudo second order differentiator z'
//  '<S72>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/Translational kinematic inverse/pseudo second order differentiator x/MATLAB Function1'
//  '<S73>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/Translational kinematic inverse/pseudo second order differentiator x/MATLAB Function2'
//  '<S74>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/Translational kinematic inverse/pseudo second order differentiator x/MATLAB Function3'
//  '<S75>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/Translational kinematic inverse/pseudo second order differentiator y/MATLAB Function1'
//  '<S76>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/Translational kinematic inverse/pseudo second order differentiator y/MATLAB Function2'
//  '<S77>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/Translational kinematic inverse/pseudo second order differentiator y/MATLAB Function3'
//  '<S78>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/Translational kinematic inverse/pseudo second order differentiator z/MATLAB Function1'
//  '<S79>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/Translational kinematic inverse/pseudo second order differentiator z/MATLAB Function2'
//  '<S80>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/Translational kinematic inverse/pseudo second order differentiator z/MATLAB Function3'
//  '<S81>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/pseudo second order differentiator p/MATLAB Function1'
//  '<S82>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/pseudo second order differentiator p/MATLAB Function2'
//  '<S83>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/pseudo second order differentiator p/MATLAB Function3'
//  '<S84>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/pseudo second order differentiator q/MATLAB Function1'
//  '<S85>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/pseudo second order differentiator q/MATLAB Function2'
//  '<S86>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/pseudo second order differentiator q/MATLAB Function3'
//  '<S87>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/pseudo second order differentiator r/MATLAB Function1'
//  '<S88>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/pseudo second order differentiator r/MATLAB Function2'
//  '<S89>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/pseudo second order differentiator r/MATLAB Function3'
//  '<S90>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/pseudo second order differentiator u/MATLAB Function1'
//  '<S91>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/pseudo second order differentiator u/MATLAB Function2'
//  '<S92>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/pseudo second order differentiator u/MATLAB Function3'
//  '<S93>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/pseudo second order differentiator v/MATLAB Function1'
//  '<S94>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/pseudo second order differentiator v/MATLAB Function2'
//  '<S95>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/pseudo second order differentiator v/MATLAB Function3'
//  '<S96>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/pseudo second order differentiator w/MATLAB Function1'
//  '<S97>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/pseudo second order differentiator w/MATLAB Function2'
//  '<S98>'  : 'pose_controller_2/Subsystem/Controller/calc_epsilon_d/pseudo second order differentiator w/MATLAB Function3'
//  '<S99>'  : 'pose_controller_2/Subsystem/Controller/calc_grad/MATLAB Function1'
//  '<S100>' : 'pose_controller_2/Subsystem/Controller/calc_grad/MATLAB Function2'
//  '<S101>' : 'pose_controller_2/Subsystem/J/J'
//  '<S102>' : 'pose_controller_2/Subsystem/J/calc_J_sys'
//  '<S103>' : 'pose_controller_2/Subsystem/Subscribe/Enabled Subsystem'
//  '<S104>' : 'pose_controller_2/Subsystem/Subscribe1/Enabled Subsystem'
//  '<S105>' : 'pose_controller_2/Subsystem/Subscribe2/Enabled Subsystem'
//  '<S106>' : 'pose_controller_2/Subsystem/Subscribe3/Enabled Subsystem'
//  '<S107>' : 'pose_controller_2/Subsystem/Subscribe4/Enabled Subsystem'
//  '<S108>' : 'pose_controller_2/Subsystem/Subscribe5/Enabled Subsystem'
//  '<S109>' : 'pose_controller_2/Subsystem/Subscribe6/Enabled Subsystem'
//  '<S110>' : 'pose_controller_2/Subsystem/Subscribe7/Enabled Subsystem'
//  '<S111>' : 'pose_controller_2/Subsystem/Subscribe8/Enabled Subsystem'

#endif                                 // RTW_HEADER_pose_controller_2_h_

//
// File trailer for generated code.
//
// [EOF]
//
