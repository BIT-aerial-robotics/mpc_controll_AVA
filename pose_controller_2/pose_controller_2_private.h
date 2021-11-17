//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: pose_controller_2_private.h
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
#ifndef RTW_HEADER_pose_controller_2_private_h_
#define RTW_HEADER_pose_controller_2_private_h_
#include "rtwtypes.h"
#include "pose_controller_2.h"

// Private macros used by the generated code to access rtModel
#ifndef rtmSetFirstInitCond
#define rtmSetFirstInitCond(rtm, val)  ((rtm)->Timing.firstInitCondFlag = (val))
#endif

#ifndef rtmIsFirstInitCond
#define rtmIsFirstInitCond(rtm)        ((rtm)->Timing.firstInitCondFlag)
#endif

#ifndef rtmIsMajorTimeStep
#define rtmIsMajorTimeStep(rtm)        (((rtm)->Timing.simTimeStep) == MAJOR_TIME_STEP)
#endif

#ifndef rtmIsMinorTimeStep
#define rtmIsMinorTimeStep(rtm)        (((rtm)->Timing.simTimeStep) == MINOR_TIME_STEP)
#endif

#ifndef rtmSetTPtr
#define rtmSetTPtr(rtm, val)           ((rtm)->Timing.t = (val))
#endif

extern void pose_controller_MATLABFunction1(real_T rtu_phi, real_T rtu_theta,
  real_T rtu_psi, real_T rtu_fx, real_T rtu_fy, real_T rtu_fz, real_T
  rtu_psi_cmd, real_T *rty_thrust, real_T *rty_phi_i, real_T *rty_theta_i,
  B_MATLABFunction1_pose_contro_T *localB);
extern void pose_controll_MATLABFunction1_a(real_T rtu_u, real_T rtu_omega,
  real_T *rty_y);
extern void pose_controller_MATLABFunction2(real_T rtu_u, real_T rtu_omega,
  real_T rtu_epsilon, real_T *rty_y);

// private model entry point functions
extern void pose_controller_2_derivatives(void);

#endif                               // RTW_HEADER_pose_controller_2_private_h_

//
// File trailer for generated code.
//
// [EOF]
//
