//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: pose_controller_2.cpp
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
#include "pose_controller_2.h"
#include "pose_controller_2_private.h"
#include <std_msgs/Bool.h>
// Block signals (default storage)
B_pose_controller_2_T pose_controller_2_B;

// Continuous states
X_pose_controller_2_T pose_controller_2_X;

// Block states (default storage)
DW_pose_controller_2_T pose_controller_2_DW;

// Real-time model
RT_MODEL_pose_controller_2_T pose_controller_2_M_ = RT_MODEL_pose_controller_2_T
  ();
RT_MODEL_pose_controller_2_T *const pose_controller_2_M = &pose_controller_2_M_;

//kaidiwang code this in 2021.11.17
extern std_msgs::Bool pub;

// Forward declaration for local functions
static void pose_controller_2_mldivide_g(const real_T A[36], real_T B[6]);

//
// This function updates continuous states using the ODE3 fixed-step
// solver algorithm
//
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  // Solver Matrices
  static const real_T rt_ODE3_A[3] = {
    1.0/2.0, 3.0/4.0, 1.0
  };

  static const real_T rt_ODE3_B[3][3] = {
    { 1.0/2.0, 0.0, 0.0 },

    { 0.0, 3.0/4.0, 0.0 },

    { 2.0/9.0, 1.0/3.0, 4.0/9.0 }
  };

  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE3_IntgData *id = static_cast<ODE3_IntgData *>(rtsiGetSolverData(si));
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T hB[3];
  int_T i;
  int_T nXc = 30;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  // Save the state values at time t in y, we'll use x as ynew.
  (void) memcpy(y, x,
                static_cast<uint_T>(nXc)*sizeof(real_T));

  // Assumes that rtsiSetT and ModelOutputs are up-to-date
  // f0 = f(t,y)
  rtsiSetdX(si, f0);
  pose_controller_2_derivatives();

  // f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*));
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  pose_controller_2_step();
  pose_controller_2_derivatives();

  // f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*));
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  pose_controller_2_step();
  pose_controller_2_derivatives();

  // tnew = t + hA(3);
  // ynew = y + f*hB(:,3);
  for (i = 0; i <= 2; i++) {
    hB[i] = h * rt_ODE3_B[2][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2]);
  }

  rtsiSetT(si, tnew);
  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

//
// Output and update for atomic system:
//    '<S13>/MATLAB Function1'
//    '<S13>/MATLAB Function2'
//    '<S13>/MATLAB Function3'
//
void pose_controller_MATLABFunction1(real_T rtu_phi, real_T rtu_theta, real_T
  rtu_psi, real_T rtu_fx, real_T rtu_fy, real_T rtu_fz, real_T rtu_psi_cmd,
  real_T *rty_thrust, real_T *rty_phi_i, real_T *rty_theta_i,
  B_MATLABFunction1_pose_contro_T *localB)
{
  real_T fw_tmp;
  real_T fw_tmp_0;
  real_T fw_tmp_1;
  real_T fw_tmp_2;
  real_T fw_tmp_3;
  real_T fw_tmp_4;
  int32_T i;
  fw_tmp = cos(rtu_psi);
  fw_tmp_0 = cos(rtu_phi);
  fw_tmp_1 = sin(rtu_theta);
  fw_tmp_2 = sin(rtu_phi);
  fw_tmp_3 = sin(rtu_psi);
  fw_tmp_4 = cos(rtu_theta);
  localB->fw_tmp[0] = fw_tmp_4 * fw_tmp;
  localB->fw_tmp[3] = fw_tmp_2 * fw_tmp_1 * fw_tmp - fw_tmp_0 * fw_tmp_3;
  localB->fw_tmp[6] = fw_tmp_0 * fw_tmp_1 * fw_tmp + fw_tmp_2 * fw_tmp_3;
  localB->fw_tmp[1] = fw_tmp_4 * fw_tmp_3;
  localB->fw_tmp[4] = sin(rtu_phi) * sin(rtu_theta) * fw_tmp_3 + fw_tmp_0 *
    fw_tmp;
  localB->fw_tmp[7] = cos(rtu_phi) * sin(rtu_theta) * fw_tmp_3 - fw_tmp_2 *
    fw_tmp;
  localB->fw_tmp[2] = -fw_tmp_1;
  localB->fw_tmp[5] = fw_tmp_2 * fw_tmp_4;
  localB->fw_tmp[8] = fw_tmp_0 * fw_tmp_4;
  for (i = 0; i < 3; i++) {
    localB->fw[i] = localB->fw_tmp[i + 6] * rtu_fz + (localB->fw_tmp[i + 3] *
      rtu_fy + localB->fw_tmp[i] * rtu_fx);
  }

  fw_tmp = localB->fw[2] * localB->fw[2];
  fw_tmp_0 = sqrt((localB->fw[0] * localB->fw[0] + localB->fw[1] * localB->fw[1])
                  + fw_tmp);
  fw_tmp_1 = cos(rtu_psi_cmd);
  fw_tmp_2 = sin(rtu_psi_cmd);
  *rty_phi_i = (localB->fw[1] * fw_tmp_1 - localB->fw[0] * fw_tmp_2) / fw_tmp_0;
  *rty_phi_i = asin(*rty_phi_i);
  fw_tmp_1 = localB->fw[0] * fw_tmp_1 + localB->fw[1] * fw_tmp_2;
  *rty_theta_i = -(localB->fw[0] * cos(rtu_psi_cmd) + localB->fw[1] * sin
                   (rtu_psi_cmd)) / sqrt(fw_tmp_1 * fw_tmp_1 + fw_tmp);
  *rty_theta_i = asin(*rty_theta_i);
  *rty_thrust = fw_tmp_0;
}

//
// Output and update for atomic system:
//    '<S56>/MATLAB Function1'
//    '<S56>/MATLAB Function3'
//    '<S57>/MATLAB Function1'
//    '<S57>/MATLAB Function3'
//    '<S58>/MATLAB Function1'
//    '<S58>/MATLAB Function3'
//    '<S69>/MATLAB Function1'
//    '<S69>/MATLAB Function3'
//    '<S70>/MATLAB Function1'
//    '<S70>/MATLAB Function3'
//    ...
//
void pose_controll_MATLABFunction1_a(real_T rtu_u, real_T rtu_omega, real_T
  *rty_y)
{
  *rty_y = rtu_omega * rtu_omega * rtu_u;
}

//
// Output and update for atomic system:
//    '<S56>/MATLAB Function2'
//    '<S57>/MATLAB Function2'
//    '<S58>/MATLAB Function2'
//    '<S69>/MATLAB Function2'
//    '<S70>/MATLAB Function2'
//    '<S71>/MATLAB Function2'
//    '<S49>/MATLAB Function2'
//    '<S50>/MATLAB Function2'
//    '<S51>/MATLAB Function2'
//    '<S52>/MATLAB Function2'
//    ...
//
void pose_controller_MATLABFunction2(real_T rtu_u, real_T rtu_omega, real_T
  rtu_epsilon, real_T *rty_y)
{
  *rty_y = 2.0 * rtu_omega * rtu_epsilon * rtu_u;
}

// Function for MATLAB Function: '<S13>/MATLAB Function'
static void pose_controller_2_mldivide_g(const real_T A[36], real_T B[6])
{
  memcpy(&pose_controller_2_B.b_A[0], &A[0], 36U * sizeof(real_T));
  for (pose_controller_2_B.kAcol = 0; pose_controller_2_B.kAcol < 6;
       pose_controller_2_B.kAcol++) {
    pose_controller_2_B.ipiv[pose_controller_2_B.kAcol] = static_cast<int8_T>
      (pose_controller_2_B.kAcol + 1);
  }

  for (pose_controller_2_B.j = 0; pose_controller_2_B.j < 5;
       pose_controller_2_B.j++) {
    pose_controller_2_B.kAcol = pose_controller_2_B.j * 7;
    pose_controller_2_B.jA = 0;
    pose_controller_2_B.ix = pose_controller_2_B.kAcol;
    pose_controller_2_B.smax = fabs
      (pose_controller_2_B.b_A[pose_controller_2_B.kAcol]);
    pose_controller_2_B.iy = 2;
    while (pose_controller_2_B.iy <= 6 - pose_controller_2_B.j) {
      pose_controller_2_B.ix++;
      pose_controller_2_B.y_lx = fabs
        (pose_controller_2_B.b_A[pose_controller_2_B.ix]);
      if (pose_controller_2_B.y_lx > pose_controller_2_B.smax) {
        pose_controller_2_B.jA = pose_controller_2_B.iy - 1;
        pose_controller_2_B.smax = pose_controller_2_B.y_lx;
      }

      pose_controller_2_B.iy++;
    }

    if (pose_controller_2_B.b_A[pose_controller_2_B.kAcol +
        pose_controller_2_B.jA] != 0.0) {
      if (pose_controller_2_B.jA != 0) {
        pose_controller_2_B.iy = pose_controller_2_B.j + pose_controller_2_B.jA;
        pose_controller_2_B.ipiv[pose_controller_2_B.j] = static_cast<int8_T>
          (pose_controller_2_B.iy + 1);
        pose_controller_2_B.ix = pose_controller_2_B.j;
        for (pose_controller_2_B.jA = 0; pose_controller_2_B.jA < 6;
             pose_controller_2_B.jA++) {
          pose_controller_2_B.smax =
            pose_controller_2_B.b_A[pose_controller_2_B.ix];
          pose_controller_2_B.b_A[pose_controller_2_B.ix] =
            pose_controller_2_B.b_A[pose_controller_2_B.iy];
          pose_controller_2_B.b_A[pose_controller_2_B.iy] =
            pose_controller_2_B.smax;
          pose_controller_2_B.ix += 6;
          pose_controller_2_B.iy += 6;
        }
      }

      pose_controller_2_B.jA = (pose_controller_2_B.kAcol -
        pose_controller_2_B.j) + 6;
      pose_controller_2_B.ix = pose_controller_2_B.kAcol + 1;
      while (pose_controller_2_B.ix + 1 <= pose_controller_2_B.jA) {
        pose_controller_2_B.b_A[pose_controller_2_B.ix] /=
          pose_controller_2_B.b_A[pose_controller_2_B.kAcol];
        pose_controller_2_B.ix++;
      }
    }

    pose_controller_2_B.jA = pose_controller_2_B.kAcol;
    pose_controller_2_B.ix = pose_controller_2_B.kAcol + 6;
    pose_controller_2_B.iy = 0;
    while (pose_controller_2_B.iy <= 4 - pose_controller_2_B.j) {
      if (pose_controller_2_B.b_A[pose_controller_2_B.ix] != 0.0) {
        pose_controller_2_B.smax =
          -pose_controller_2_B.b_A[pose_controller_2_B.ix];
        pose_controller_2_B.c_ix = pose_controller_2_B.kAcol + 1;
        pose_controller_2_B.d = (pose_controller_2_B.jA - pose_controller_2_B.j)
          + 12;
        pose_controller_2_B.ijA = pose_controller_2_B.jA + 7;
        while (pose_controller_2_B.ijA + 1 <= pose_controller_2_B.d) {
          pose_controller_2_B.b_A[pose_controller_2_B.ijA] +=
            pose_controller_2_B.b_A[pose_controller_2_B.c_ix] *
            pose_controller_2_B.smax;
          pose_controller_2_B.c_ix++;
          pose_controller_2_B.ijA++;
        }
      }

      pose_controller_2_B.ix += 6;
      pose_controller_2_B.jA += 6;
      pose_controller_2_B.iy++;
    }

    if (pose_controller_2_B.j + 1 !=
        pose_controller_2_B.ipiv[pose_controller_2_B.j]) {
      pose_controller_2_B.smax = B[pose_controller_2_B.j];
      pose_controller_2_B.kAcol = pose_controller_2_B.ipiv[pose_controller_2_B.j]
        - 1;
      B[pose_controller_2_B.j] = B[pose_controller_2_B.kAcol];
      B[pose_controller_2_B.kAcol] = pose_controller_2_B.smax;
    }
  }

  for (pose_controller_2_B.j = 0; pose_controller_2_B.j < 6;
       pose_controller_2_B.j++) {
    pose_controller_2_B.kAcol = 6 * pose_controller_2_B.j;
    if (B[pose_controller_2_B.j] != 0.0) {
      pose_controller_2_B.jA = pose_controller_2_B.j + 1;
      while (pose_controller_2_B.jA + 1 < 7) {
        B[pose_controller_2_B.jA] -=
          pose_controller_2_B.b_A[pose_controller_2_B.jA +
          pose_controller_2_B.kAcol] * B[pose_controller_2_B.j];
        pose_controller_2_B.jA++;
      }
    }
  }

  for (pose_controller_2_B.j = 5; pose_controller_2_B.j >= 0;
       pose_controller_2_B.j--) {
    pose_controller_2_B.kAcol = 6 * pose_controller_2_B.j;
    if (B[pose_controller_2_B.j] != 0.0) {
      B[pose_controller_2_B.j] /= pose_controller_2_B.b_A[pose_controller_2_B.j
        + pose_controller_2_B.kAcol];
      pose_controller_2_B.jA = 0;
      while (pose_controller_2_B.jA <= pose_controller_2_B.j - 1) {
        B[pose_controller_2_B.jA] -=
          pose_controller_2_B.b_A[pose_controller_2_B.jA +
          pose_controller_2_B.kAcol] * B[pose_controller_2_B.j];
        pose_controller_2_B.jA++;
      }
    }
  }
}

// Model step function
void pose_controller_2_step(void)
{
  static const int8_T c[27] = { 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0,
    0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  static const int8_T d[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  real_T *lastU;
  boolean_T b_varargout_1;
  if (rtmIsMajorTimeStep(pose_controller_2_M)) {
    // set solver stop time
    rtsiSetSolverStopTime(&pose_controller_2_M->solverInfo,
                          ((pose_controller_2_M->Timing.clockTick0+1)*
      pose_controller_2_M->Timing.stepSize0));
  }                                    // end MajorTimeStep

  // Update absolute time of base rate at minor time step
  if (rtmIsMinorTimeStep(pose_controller_2_M)) {
    pose_controller_2_M->Timing.t[0] = rtsiGetT(&pose_controller_2_M->solverInfo);
  }

  if (rtmIsMajorTimeStep(pose_controller_2_M)) {
    // Outputs for Atomic SubSystem: '<Root>/Subscribe9'
    // MATLABSystem: '<S3>/SourceBlock' incorporates:
    //   Inport: '<S6>/In1'

    b_varargout_1 = Sub_pose_controller_2_2544.getLatestMessage
      (&pose_controller_2_B.b_varargout_2);

    // Outputs for Enabled SubSystem: '<S3>/Enabled Subsystem' incorporates:
    //   EnablePort: '<S6>/Enable'

    if (b_varargout_1) {
      pose_controller_2_B.In1_a = pose_controller_2_B.b_varargout_2;
    }

    // End of MATLABSystem: '<S3>/SourceBlock'
    // End of Outputs for SubSystem: '<S3>/Enabled Subsystem'
    // End of Outputs for SubSystem: '<Root>/Subscribe9'

    // Outputs for Atomic SubSystem: '<Root>/Subscribe10'
    // MATLABSystem: '<S2>/SourceBlock' incorporates:
    //   Inport: '<S5>/In1'

    b_varargout_1 = Sub_pose_controller_2_2548.getLatestMessage
      (&pose_controller_2_B.b_varargout_2);

    // Outputs for Enabled SubSystem: '<S2>/Enabled Subsystem' incorporates:
    //   EnablePort: '<S5>/Enable'

    if (b_varargout_1) {
      pose_controller_2_B.In1_fj = pose_controller_2_B.b_varargout_2;
    }

    // End of MATLABSystem: '<S2>/SourceBlock'
    // End of Outputs for SubSystem: '<S2>/Enabled Subsystem'
    // End of Outputs for SubSystem: '<Root>/Subscribe10'

    // Outputs for Enabled SubSystem: '<Root>/Subsystem' incorporates:
    //   EnablePort: '<S4>/Enable'

    if (rtmIsMajorTimeStep(pose_controller_2_M)) {
      // MATLAB Function: '<Root>/MATLAB Function'
      if ((pose_controller_2_B.In1_a.X == 0.0) && (pose_controller_2_B.In1_a.Y ==
           0.0) && (pose_controller_2_B.In1_a.Z == 0.0) &&
          (pose_controller_2_B.In1_fj.Z == 0.0)) {
        pose_controller_2_B.i = -1;
      } else {
        pose_controller_2_B.i = 1;
      }

      // End of MATLAB Function: '<Root>/MATLAB Function'
      pose_controller_2_DW.Subsystem_MODE = (pose_controller_2_B.i > 0);
    }

    // End of Outputs for SubSystem: '<Root>/Subsystem'
  }

  // Outputs for Enabled SubSystem: '<Root>/Subsystem' incorporates:
  //   EnablePort: '<S4>/Enable'

  if (pose_controller_2_DW.Subsystem_MODE) {
    if (rtmIsMajorTimeStep(pose_controller_2_M)) {
      // Outputs for Atomic SubSystem: '<S4>/Subscribe3'
      // MATLABSystem: '<S25>/SourceBlock' incorporates:
      //   Inport: '<S106>/In1'

      b_varargout_1 = Sub_pose_controller_2_232.getLatestMessage
        (&pose_controller_2_B.b_varargout_2);

      // Outputs for Enabled SubSystem: '<S25>/Enabled Subsystem' incorporates:
      //   EnablePort: '<S106>/Enable'

      if (b_varargout_1) {
        pose_controller_2_B.In1_e = pose_controller_2_B.b_varargout_2;
      }

      // End of MATLABSystem: '<S25>/SourceBlock'
      // End of Outputs for SubSystem: '<S25>/Enabled Subsystem'
      // End of Outputs for SubSystem: '<S4>/Subscribe3'

      // Outputs for Atomic SubSystem: '<S4>/Subscribe4'
      // MATLABSystem: '<S26>/SourceBlock' incorporates:
      //   Inport: '<S107>/In1'

      b_varargout_1 = Sub_pose_controller_2_235.getLatestMessage
        (&pose_controller_2_B.b_varargout_2);

      // Outputs for Enabled SubSystem: '<S26>/Enabled Subsystem' incorporates:
      //   EnablePort: '<S107>/Enable'

      if (b_varargout_1) {
        pose_controller_2_B.In1_k = pose_controller_2_B.b_varargout_2;
      }

      // End of MATLABSystem: '<S26>/SourceBlock'
      // End of Outputs for SubSystem: '<S26>/Enabled Subsystem'
      // End of Outputs for SubSystem: '<S4>/Subscribe4'

      // Sum: '<S14>/Sum'
      pose_controller_2_B.ex[0] = pose_controller_2_B.In1_e.X -
        pose_controller_2_B.In1_k.X;
      pose_controller_2_B.ex[1] = pose_controller_2_B.In1_e.Y -
        pose_controller_2_B.In1_k.Y;
      pose_controller_2_B.ex[2] = pose_controller_2_B.In1_e.Z -
        pose_controller_2_B.In1_k.Z;

      // Outputs for Atomic SubSystem: '<S4>/Subscribe'
      // MATLABSystem: '<S22>/SourceBlock' incorporates:
      //   Inport: '<S103>/In1'

      b_varargout_1 = Sub_pose_controller_2_222.getLatestMessage
        (&pose_controller_2_B.b_varargout_2);

      // Outputs for Enabled SubSystem: '<S22>/Enabled Subsystem' incorporates:
      //   EnablePort: '<S103>/Enable'

      if (b_varargout_1) {
        pose_controller_2_B.In1_l = pose_controller_2_B.b_varargout_2;
      }

      // End of MATLABSystem: '<S22>/SourceBlock'
      // End of Outputs for SubSystem: '<S22>/Enabled Subsystem'
      // End of Outputs for SubSystem: '<S4>/Subscribe'

      // Outputs for Atomic SubSystem: '<S4>/Subscribe5'
      // MATLABSystem: '<S27>/SourceBlock' incorporates:
      //   Inport: '<S108>/In1'

      b_varargout_1 = Sub_pose_controller_2_238.getLatestMessage
        (&pose_controller_2_B.b_varargout_2);

      // Outputs for Enabled SubSystem: '<S27>/Enabled Subsystem' incorporates:
      //   EnablePort: '<S108>/Enable'

      if (b_varargout_1) {
        pose_controller_2_B.In1 = pose_controller_2_B.b_varargout_2;
      }

      // End of MATLABSystem: '<S27>/SourceBlock'
      // End of Outputs for SubSystem: '<S27>/Enabled Subsystem'
      // End of Outputs for SubSystem: '<S4>/Subscribe5'
    }

    // Integrator: '<S56>/Integrator'
    if (pose_controller_2_DW.Integrator_IWORK != 0) {
      pose_controller_2_X.Integrator_CSTATE = pose_controller_2_B.In1_fj.X;
    }

    // Integrator: '<S56>/Integrator'
    pose_controller_2_B.Integrator = pose_controller_2_X.Integrator_CSTATE;

    // Derivative: '<S56>/Derivative'
    if ((pose_controller_2_DW.TimeStampA >= pose_controller_2_M->Timing.t[0]) &&
        (pose_controller_2_DW.TimeStampB >= pose_controller_2_M->Timing.t[0])) {
      pose_controller_2_B.Derivative = 0.0;
    } else {
      pose_controller_2_B.Derivative_d = pose_controller_2_DW.TimeStampA;
      lastU = &pose_controller_2_DW.LastUAtTimeA;
      if (pose_controller_2_DW.TimeStampA < pose_controller_2_DW.TimeStampB) {
        if (pose_controller_2_DW.TimeStampB < pose_controller_2_M->Timing.t[0])
        {
          pose_controller_2_B.Derivative_d = pose_controller_2_DW.TimeStampB;
          lastU = &pose_controller_2_DW.LastUAtTimeB;
        }
      } else {
        if (pose_controller_2_DW.TimeStampA >= pose_controller_2_M->Timing.t[0])
        {
          pose_controller_2_B.Derivative_d = pose_controller_2_DW.TimeStampB;
          lastU = &pose_controller_2_DW.LastUAtTimeB;
        }
      }

      pose_controller_2_B.Derivative = (pose_controller_2_B.Integrator - *lastU)
        / (pose_controller_2_M->Timing.t[0] - pose_controller_2_B.Derivative_d);
    }

    // End of Derivative: '<S56>/Derivative'

    // Integrator: '<S58>/Integrator'
    if (pose_controller_2_DW.Integrator_IWORK_e != 0) {
      pose_controller_2_X.Integrator_CSTATE_k = pose_controller_2_B.In1_fj.Y;
    }

    // Integrator: '<S58>/Integrator'
    pose_controller_2_B.Integrator_e = pose_controller_2_X.Integrator_CSTATE_k;

    // Derivative: '<S58>/Derivative'
    if ((pose_controller_2_DW.TimeStampA_n >= pose_controller_2_M->Timing.t[0]) &&
        (pose_controller_2_DW.TimeStampB_g >= pose_controller_2_M->Timing.t[0]))
    {
      pose_controller_2_B.Derivative_i = 0.0;
    } else {
      pose_controller_2_B.Derivative_d = pose_controller_2_DW.TimeStampA_n;
      lastU = &pose_controller_2_DW.LastUAtTimeA_l;
      if (pose_controller_2_DW.TimeStampA_n < pose_controller_2_DW.TimeStampB_g)
      {
        if (pose_controller_2_DW.TimeStampB_g < pose_controller_2_M->Timing.t[0])
        {
          pose_controller_2_B.Derivative_d = pose_controller_2_DW.TimeStampB_g;
          lastU = &pose_controller_2_DW.LastUAtTimeB_h;
        }
      } else {
        if (pose_controller_2_DW.TimeStampA_n >= pose_controller_2_M->Timing.t[0])
        {
          pose_controller_2_B.Derivative_d = pose_controller_2_DW.TimeStampB_g;
          lastU = &pose_controller_2_DW.LastUAtTimeB_h;
        }
      }

      pose_controller_2_B.Derivative_i = (pose_controller_2_B.Integrator_e -
        *lastU) / (pose_controller_2_M->Timing.t[0] -
                   pose_controller_2_B.Derivative_d);
    }

    // End of Derivative: '<S58>/Derivative'

    // Integrator: '<S57>/Integrator'
    if (pose_controller_2_DW.Integrator_IWORK_l != 0) {
      pose_controller_2_X.Integrator_CSTATE_j = pose_controller_2_B.In1_fj.Z;
    }

    // Integrator: '<S57>/Integrator'
    pose_controller_2_B.Integrator_i = pose_controller_2_X.Integrator_CSTATE_j;

    // Derivative: '<S57>/Derivative'
    if ((pose_controller_2_DW.TimeStampA_nq >= pose_controller_2_M->Timing.t[0])
        && (pose_controller_2_DW.TimeStampB_e >= pose_controller_2_M->Timing.t[0]))
    {
      pose_controller_2_B.Derivative_d = 0.0;
    } else {
      pose_controller_2_B.Derivative_d = pose_controller_2_DW.TimeStampA_nq;
      lastU = &pose_controller_2_DW.LastUAtTimeA_e;
      if (pose_controller_2_DW.TimeStampA_nq < pose_controller_2_DW.TimeStampB_e)
      {
        if (pose_controller_2_DW.TimeStampB_e < pose_controller_2_M->Timing.t[0])
        {
          pose_controller_2_B.Derivative_d = pose_controller_2_DW.TimeStampB_e;
          lastU = &pose_controller_2_DW.LastUAtTimeB_n;
        }
      } else {
        if (pose_controller_2_DW.TimeStampA_nq >= pose_controller_2_M->Timing.t
            [0]) {
          pose_controller_2_B.Derivative_d = pose_controller_2_DW.TimeStampB_e;
          lastU = &pose_controller_2_DW.LastUAtTimeB_n;
        }
      }

      pose_controller_2_B.Derivative_d = (pose_controller_2_B.Integrator_i -
        *lastU) / (pose_controller_2_M->Timing.t[0] -
                   pose_controller_2_B.Derivative_d);
    }

    // End of Derivative: '<S57>/Derivative'

    // MATLAB Function: '<S47>/MATLAB Function' incorporates:
    //   MATLAB Function: '<S48>/MATLAB Function'

    pose_controller_2_B.rtb_p_bar_tmp = sin(pose_controller_2_B.In1.Y);
    pose_controller_2_B.Derivative -= pose_controller_2_B.Derivative_d *
      pose_controller_2_B.rtb_p_bar_tmp;
    pose_controller_2_B.rtb_q_bar_tmp = cos(pose_controller_2_B.In1.Y);
    pose_controller_2_B.rtb_q_bar_tmp_p = sin(pose_controller_2_B.In1.X);
    pose_controller_2_B.rtb_q_bar_tmp_l = cos(pose_controller_2_B.In1.X);
    pose_controller_2_B.q_bar = pose_controller_2_B.Derivative_d *
      pose_controller_2_B.rtb_q_bar_tmp_p * pose_controller_2_B.rtb_q_bar_tmp +
      pose_controller_2_B.Derivative_i * pose_controller_2_B.rtb_q_bar_tmp_l;
    pose_controller_2_B.Derivative_i = pose_controller_2_B.Derivative_d *
      pose_controller_2_B.rtb_q_bar_tmp_l * pose_controller_2_B.rtb_q_bar_tmp +
      -pose_controller_2_B.Derivative_i * pose_controller_2_B.rtb_q_bar_tmp_p;

    // Integrator: '<S69>/Integrator'
    if (pose_controller_2_DW.Integrator_IWORK_g != 0) {
      pose_controller_2_X.Integrator_CSTATE_k3 = pose_controller_2_B.In1_a.X;
    }

    // Integrator: '<S69>/Integrator'
    pose_controller_2_B.Integrator_k = pose_controller_2_X.Integrator_CSTATE_k3;

    // Derivative: '<S69>/Derivative'
    if ((pose_controller_2_DW.TimeStampA_f >= pose_controller_2_M->Timing.t[0]) &&
        (pose_controller_2_DW.TimeStampB_f >= pose_controller_2_M->Timing.t[0]))
    {
      pose_controller_2_B.Derivative_o = 0.0;
    } else {
      pose_controller_2_B.Derivative_d = pose_controller_2_DW.TimeStampA_f;
      lastU = &pose_controller_2_DW.LastUAtTimeA_d;
      if (pose_controller_2_DW.TimeStampA_f < pose_controller_2_DW.TimeStampB_f)
      {
        if (pose_controller_2_DW.TimeStampB_f < pose_controller_2_M->Timing.t[0])
        {
          pose_controller_2_B.Derivative_d = pose_controller_2_DW.TimeStampB_f;
          lastU = &pose_controller_2_DW.LastUAtTimeB_f;
        }
      } else {
        if (pose_controller_2_DW.TimeStampA_f >= pose_controller_2_M->Timing.t[0])
        {
          pose_controller_2_B.Derivative_d = pose_controller_2_DW.TimeStampB_f;
          lastU = &pose_controller_2_DW.LastUAtTimeB_f;
        }
      }

      pose_controller_2_B.Derivative_o = (pose_controller_2_B.Integrator_k -
        *lastU) / (pose_controller_2_M->Timing.t[0] -
                   pose_controller_2_B.Derivative_d);
    }

    // End of Derivative: '<S69>/Derivative'

    // Integrator: '<S70>/Integrator'
    if (pose_controller_2_DW.Integrator_IWORK_k != 0) {
      pose_controller_2_X.Integrator_CSTATE_c = pose_controller_2_B.In1_a.Y;
    }

    // Integrator: '<S70>/Integrator'
    pose_controller_2_B.Integrator_n = pose_controller_2_X.Integrator_CSTATE_c;

    // Derivative: '<S70>/Derivative'
    if ((pose_controller_2_DW.TimeStampA_j >= pose_controller_2_M->Timing.t[0]) &&
        (pose_controller_2_DW.TimeStampB_f4 >= pose_controller_2_M->Timing.t[0]))
    {
      pose_controller_2_B.Derivative_j = 0.0;
    } else {
      pose_controller_2_B.Derivative_d = pose_controller_2_DW.TimeStampA_j;
      lastU = &pose_controller_2_DW.LastUAtTimeA_n;
      if (pose_controller_2_DW.TimeStampA_j < pose_controller_2_DW.TimeStampB_f4)
      {
        if (pose_controller_2_DW.TimeStampB_f4 < pose_controller_2_M->Timing.t[0])
        {
          pose_controller_2_B.Derivative_d = pose_controller_2_DW.TimeStampB_f4;
          lastU = &pose_controller_2_DW.LastUAtTimeB_l;
        }
      } else {
        if (pose_controller_2_DW.TimeStampA_j >= pose_controller_2_M->Timing.t[0])
        {
          pose_controller_2_B.Derivative_d = pose_controller_2_DW.TimeStampB_f4;
          lastU = &pose_controller_2_DW.LastUAtTimeB_l;
        }
      }

      pose_controller_2_B.Derivative_j = (pose_controller_2_B.Integrator_n -
        *lastU) / (pose_controller_2_M->Timing.t[0] -
                   pose_controller_2_B.Derivative_d);
    }

    // End of Derivative: '<S70>/Derivative'

    // Integrator: '<S71>/Integrator'
    if (pose_controller_2_DW.Integrator_IWORK_d != 0) {
      pose_controller_2_X.Integrator_CSTATE_i = pose_controller_2_B.In1_a.Z;
    }

    // Integrator: '<S71>/Integrator'
    pose_controller_2_B.Integrator_a = pose_controller_2_X.Integrator_CSTATE_i;

    // Derivative: '<S71>/Derivative'
    if ((pose_controller_2_DW.TimeStampA_m >= pose_controller_2_M->Timing.t[0]) &&
        (pose_controller_2_DW.TimeStampB_l >= pose_controller_2_M->Timing.t[0]))
    {
      pose_controller_2_B.Derivative_d = 0.0;
    } else {
      pose_controller_2_B.Derivative_d = pose_controller_2_DW.TimeStampA_m;
      lastU = &pose_controller_2_DW.LastUAtTimeA_j;
      if (pose_controller_2_DW.TimeStampA_m < pose_controller_2_DW.TimeStampB_l)
      {
        if (pose_controller_2_DW.TimeStampB_l < pose_controller_2_M->Timing.t[0])
        {
          pose_controller_2_B.Derivative_d = pose_controller_2_DW.TimeStampB_l;
          lastU = &pose_controller_2_DW.LastUAtTimeB_b;
        }
      } else {
        if (pose_controller_2_DW.TimeStampA_m >= pose_controller_2_M->Timing.t[0])
        {
          pose_controller_2_B.Derivative_d = pose_controller_2_DW.TimeStampB_l;
          lastU = &pose_controller_2_DW.LastUAtTimeB_b;
        }
      }

      pose_controller_2_B.Derivative_d = (pose_controller_2_B.Integrator_a -
        *lastU) / (pose_controller_2_M->Timing.t[0] -
                   pose_controller_2_B.Derivative_d);
    }

    // End of Derivative: '<S71>/Derivative'

    // MATLAB Function: '<S48>/MATLAB Function'
    pose_controller_2_B.vd_tmp = cos(pose_controller_2_B.In1.Z);
    pose_controller_2_B.vd_tmp_n = sin(pose_controller_2_B.In1.Z);
    pose_controller_2_B.S1[0] = pose_controller_2_B.rtb_q_bar_tmp *
      pose_controller_2_B.vd_tmp;
    pose_controller_2_B.S1[1] = pose_controller_2_B.rtb_q_bar_tmp_p *
      pose_controller_2_B.rtb_p_bar_tmp * pose_controller_2_B.vd_tmp -
      pose_controller_2_B.rtb_q_bar_tmp_l * pose_controller_2_B.vd_tmp_n;
    pose_controller_2_B.S1[2] = pose_controller_2_B.rtb_q_bar_tmp_l *
      pose_controller_2_B.rtb_p_bar_tmp * pose_controller_2_B.vd_tmp +
      pose_controller_2_B.rtb_q_bar_tmp_p * pose_controller_2_B.vd_tmp_n;
    pose_controller_2_B.S1[3] = pose_controller_2_B.rtb_q_bar_tmp *
      pose_controller_2_B.vd_tmp_n;
    pose_controller_2_B.S1[4] = sin(pose_controller_2_B.In1.X) * sin
      (pose_controller_2_B.In1.Y) * pose_controller_2_B.vd_tmp_n +
      pose_controller_2_B.rtb_q_bar_tmp_l * pose_controller_2_B.vd_tmp;
    pose_controller_2_B.S1[5] = cos(pose_controller_2_B.In1.X) * sin
      (pose_controller_2_B.In1.Y) * pose_controller_2_B.vd_tmp_n -
      pose_controller_2_B.rtb_q_bar_tmp_p * pose_controller_2_B.vd_tmp;
    pose_controller_2_B.S1[6] = -pose_controller_2_B.rtb_p_bar_tmp;
    pose_controller_2_B.S1[7] = pose_controller_2_B.rtb_q_bar_tmp_p *
      pose_controller_2_B.rtb_q_bar_tmp;
    pose_controller_2_B.S1[8] = pose_controller_2_B.rtb_q_bar_tmp_l *
      pose_controller_2_B.rtb_q_bar_tmp;
    for (pose_controller_2_B.i = 0; pose_controller_2_B.i < 3;
         pose_controller_2_B.i++) {
      pose_controller_2_B.vd[pose_controller_2_B.i] =
        pose_controller_2_B.S1[pose_controller_2_B.i + 6] *
        pose_controller_2_B.Derivative_d +
        (pose_controller_2_B.S1[pose_controller_2_B.i + 3] *
         pose_controller_2_B.Derivative_j +
         pose_controller_2_B.S1[pose_controller_2_B.i] *
         pose_controller_2_B.Derivative_o);
    }

    // MATLAB Function: '<S39>/MATLAB Function' incorporates:
    //   Constant: '<S31>/tool_x'
    //   Constant: '<S31>/tool_y'
    //   Constant: '<S31>/tool_z'
    //   MATLAB Function: '<S37>/MATLAB Function'
    //   MATLAB Function: '<S39>/MATLAB Function1'
    //   MATLAB Function: '<S40>/MATLAB Function1'
    //   MATLAB Function: '<S48>/MATLAB Function'

    pose_controller_2_B.Derivative_d = cos(pose_controller_2_B.In1_l.Z);
    pose_controller_2_B.rtb_p_bar_tmp = cos(pose_controller_2_B.In1_l.X);
    pose_controller_2_B.rtb_q_bar_tmp = sin(pose_controller_2_B.In1_l.Y);
    pose_controller_2_B.rtb_q_bar_tmp_p = sin(pose_controller_2_B.In1_l.X);
    pose_controller_2_B.rtb_q_bar_tmp_l = sin(pose_controller_2_B.In1_l.Z);
    pose_controller_2_B.Derivative_o = cos(pose_controller_2_B.In1_l.Y);
    pose_controller_2_B.S1[0] = pose_controller_2_B.Derivative_o *
      pose_controller_2_B.Derivative_d;
    pose_controller_2_B.S1[1] = pose_controller_2_B.rtb_q_bar_tmp_p *
      pose_controller_2_B.rtb_q_bar_tmp * pose_controller_2_B.Derivative_d -
      pose_controller_2_B.rtb_p_bar_tmp * pose_controller_2_B.rtb_q_bar_tmp_l;
    pose_controller_2_B.S1[2] = pose_controller_2_B.rtb_p_bar_tmp *
      pose_controller_2_B.rtb_q_bar_tmp * pose_controller_2_B.Derivative_d +
      pose_controller_2_B.rtb_q_bar_tmp_p * pose_controller_2_B.rtb_q_bar_tmp_l;
    pose_controller_2_B.S1[3] = pose_controller_2_B.Derivative_o *
      pose_controller_2_B.rtb_q_bar_tmp_l;
    pose_controller_2_B.S1[4] = sin(pose_controller_2_B.In1_l.X) * sin
      (pose_controller_2_B.In1_l.Y) * pose_controller_2_B.rtb_q_bar_tmp_l +
      pose_controller_2_B.rtb_p_bar_tmp * pose_controller_2_B.Derivative_d;
    pose_controller_2_B.S1[5] = cos(pose_controller_2_B.In1_l.X) * sin
      (pose_controller_2_B.In1_l.Y) * pose_controller_2_B.rtb_q_bar_tmp_l -
      pose_controller_2_B.rtb_q_bar_tmp_p * pose_controller_2_B.Derivative_d;
    pose_controller_2_B.S1[6] = -pose_controller_2_B.rtb_q_bar_tmp;
    pose_controller_2_B.S1[7] = pose_controller_2_B.rtb_q_bar_tmp_p *
      pose_controller_2_B.Derivative_o;
    pose_controller_2_B.S1[8] = pose_controller_2_B.rtb_p_bar_tmp *
      pose_controller_2_B.Derivative_o;
    pose_controller_2_B.Derivative_d = cos(pose_controller_2_B.In1.Y) * cos
      (pose_controller_2_B.In1.Z);
    pose_controller_2_B.Rd[0] = pose_controller_2_B.Derivative_d;
    pose_controller_2_B.Rd_tmp = sin(pose_controller_2_B.In1.X) * sin
      (pose_controller_2_B.In1.Y) * cos(pose_controller_2_B.In1.Z) - cos
      (pose_controller_2_B.In1.X) * sin(pose_controller_2_B.In1.Z);
    pose_controller_2_B.Rd[3] = pose_controller_2_B.Rd_tmp;
    pose_controller_2_B.Rd_tmp_j = cos(pose_controller_2_B.In1.X) * sin
      (pose_controller_2_B.In1.Y) * cos(pose_controller_2_B.In1.Z) + sin
      (pose_controller_2_B.In1.X) * sin(pose_controller_2_B.In1.Z);
    pose_controller_2_B.Rd[6] = pose_controller_2_B.Rd_tmp_j;
    pose_controller_2_B.Rd_tmp_d = cos(pose_controller_2_B.In1.Y) * sin
      (pose_controller_2_B.In1.Z);
    pose_controller_2_B.Rd[1] = pose_controller_2_B.Rd_tmp_d;
    pose_controller_2_B.Rd_tmp_g = sin(pose_controller_2_B.In1.X) * sin
      (pose_controller_2_B.In1.Y) * sin(pose_controller_2_B.In1.Z) + cos
      (pose_controller_2_B.In1.X) * cos(pose_controller_2_B.In1.Z);
    pose_controller_2_B.Rd[4] = pose_controller_2_B.Rd_tmp_g;
    pose_controller_2_B.Rd_tmp_l = cos(pose_controller_2_B.In1.X) * sin
      (pose_controller_2_B.In1.Y) * sin(pose_controller_2_B.In1.Z) - sin
      (pose_controller_2_B.In1.X) * cos(pose_controller_2_B.In1.Z);
    pose_controller_2_B.Rd[7] = pose_controller_2_B.Rd_tmp_l;
    pose_controller_2_B.Rd[2] = -sin(pose_controller_2_B.In1.Y);
    pose_controller_2_B.Rd_tmp_dh = sin(pose_controller_2_B.In1.X) * cos
      (pose_controller_2_B.In1.Y);
    pose_controller_2_B.Rd[5] = pose_controller_2_B.Rd_tmp_dh;
    pose_controller_2_B.Rd_tmp_dy = cos(pose_controller_2_B.In1.X) * cos
      (pose_controller_2_B.In1.Y);
    pose_controller_2_B.Rd[8] = pose_controller_2_B.Rd_tmp_dy;
    pose_controller_2_B.Rd_k[0] = 0.0;
    pose_controller_2_B.Rd_k[3] = -pose_controller_2_P.tool_z_Value;
    pose_controller_2_B.Rd_k[6] = pose_controller_2_P.tool_y_Value;
    pose_controller_2_B.Rd_k[1] = pose_controller_2_P.tool_z_Value;
    pose_controller_2_B.Rd_k[4] = 0.0;
    pose_controller_2_B.Rd_k[7] = -pose_controller_2_P.tool_x_Value;
    pose_controller_2_B.Rd_k[2] = -pose_controller_2_P.tool_y_Value;
    pose_controller_2_B.Rd_k[5] = pose_controller_2_P.tool_x_Value;
    pose_controller_2_B.Rd_k[8] = 0.0;
    pose_controller_2_B.position_error[0] = pose_controller_2_B.Derivative;
    pose_controller_2_B.position_error[1] = pose_controller_2_B.q_bar;
    pose_controller_2_B.position_error[2] = pose_controller_2_B.Derivative_i;
    pose_controller_2_B.de[0] = pose_controller_2_B.vd[0];
    pose_controller_2_B.de[1] = pose_controller_2_B.vd[1];
    pose_controller_2_B.de[2] = pose_controller_2_B.vd[2];
    for (pose_controller_2_B.i = 0; pose_controller_2_B.i < 3;
         pose_controller_2_B.i++) {
      for (pose_controller_2_B.B_tmp = 0; pose_controller_2_B.B_tmp < 3;
           pose_controller_2_B.B_tmp++) {
        pose_controller_2_B.Rt_tmp = pose_controller_2_B.i + 3 *
          pose_controller_2_B.B_tmp;
        pose_controller_2_B.Rt_cx[pose_controller_2_B.Rt_tmp] = 0.0;
        pose_controller_2_B.Rt_cx[pose_controller_2_B.Rt_tmp] +=
          pose_controller_2_B.S1[3 * pose_controller_2_B.B_tmp] *
          pose_controller_2_B.Rd_k[pose_controller_2_B.i];
        pose_controller_2_B.Rt_cx[pose_controller_2_B.Rt_tmp] +=
          pose_controller_2_B.S1[3 * pose_controller_2_B.B_tmp + 1] *
          pose_controller_2_B.Rd_k[pose_controller_2_B.i + 3];
        pose_controller_2_B.Rt_cx[pose_controller_2_B.Rt_tmp] +=
          pose_controller_2_B.S1[3 * pose_controller_2_B.B_tmp + 2] *
          pose_controller_2_B.Rd_k[pose_controller_2_B.i + 6];
      }

      pose_controller_2_B.Sum1_b[pose_controller_2_B.i] = 0.0;
      pose_controller_2_B.vd[pose_controller_2_B.i] = 0.0;
      for (pose_controller_2_B.B_tmp = 0; pose_controller_2_B.B_tmp < 3;
           pose_controller_2_B.B_tmp++) {
        pose_controller_2_B.Rt_tmp = pose_controller_2_B.i + 3 *
          pose_controller_2_B.B_tmp;
        pose_controller_2_B.dv[pose_controller_2_B.Rt_tmp] = 0.0;
        pose_controller_2_B.rtb_p_bar_tmp = pose_controller_2_B.Rd[3 *
          pose_controller_2_B.B_tmp];
        pose_controller_2_B.dv[pose_controller_2_B.Rt_tmp] +=
          pose_controller_2_B.rtb_p_bar_tmp *
          pose_controller_2_B.Rt_cx[pose_controller_2_B.i];
        pose_controller_2_B.rtb_q_bar_tmp = pose_controller_2_B.Rd[3 *
          pose_controller_2_B.B_tmp + 1];
        pose_controller_2_B.dv[pose_controller_2_B.Rt_tmp] +=
          pose_controller_2_B.rtb_q_bar_tmp *
          pose_controller_2_B.Rt_cx[pose_controller_2_B.i + 3];
        pose_controller_2_B.rtb_q_bar_tmp_p = pose_controller_2_B.Rd[3 *
          pose_controller_2_B.B_tmp + 2];
        pose_controller_2_B.dv[pose_controller_2_B.Rt_tmp] +=
          pose_controller_2_B.rtb_q_bar_tmp_p *
          pose_controller_2_B.Rt_cx[pose_controller_2_B.i + 6];
        pose_controller_2_B.Rt_c[pose_controller_2_B.Rt_tmp] = 0.0;
        pose_controller_2_B.Rt_c[pose_controller_2_B.Rt_tmp] +=
          pose_controller_2_B.rtb_p_bar_tmp *
          pose_controller_2_B.S1[pose_controller_2_B.i];
        pose_controller_2_B.Rt_c[pose_controller_2_B.Rt_tmp] +=
          pose_controller_2_B.rtb_q_bar_tmp *
          pose_controller_2_B.S1[pose_controller_2_B.i + 3];
        pose_controller_2_B.Rt_c[pose_controller_2_B.Rt_tmp] +=
          pose_controller_2_B.rtb_q_bar_tmp_p *
          pose_controller_2_B.S1[pose_controller_2_B.i + 6];
        pose_controller_2_B.Sum1_b[pose_controller_2_B.i] +=
          pose_controller_2_B.dv[pose_controller_2_B.Rt_tmp] *
          pose_controller_2_B.position_error[pose_controller_2_B.B_tmp];
        pose_controller_2_B.vd[pose_controller_2_B.i] +=
          pose_controller_2_B.Rt_c[pose_controller_2_B.Rt_tmp] *
          pose_controller_2_B.de[pose_controller_2_B.B_tmp];
      }

      pose_controller_2_B.epsilon_d_v[pose_controller_2_B.i] =
        pose_controller_2_B.Sum1_b[pose_controller_2_B.i] +
        pose_controller_2_B.vd[pose_controller_2_B.i];
    }

    if (rtmIsMajorTimeStep(pose_controller_2_M)) {
      // Outputs for Atomic SubSystem: '<S4>/Subscribe2'
      // MATLABSystem: '<S24>/SourceBlock' incorporates:
      //   Inport: '<S105>/In1'

      b_varargout_1 = Sub_pose_controller_2_229.getLatestMessage
        (&pose_controller_2_B.b_varargout_2);

      // Outputs for Enabled SubSystem: '<S24>/Enabled Subsystem' incorporates:
      //   EnablePort: '<S105>/Enable'

      if (b_varargout_1) {
        pose_controller_2_B.In1_h = pose_controller_2_B.b_varargout_2;
      }

      // End of MATLABSystem: '<S24>/SourceBlock'
      // End of Outputs for SubSystem: '<S24>/Enabled Subsystem'
      // End of Outputs for SubSystem: '<S4>/Subscribe2'

      // Outputs for Atomic SubSystem: '<S4>/Subscribe1'
      // MATLABSystem: '<S23>/SourceBlock' incorporates:
      //   Inport: '<S104>/In1'

      b_varargout_1 = Sub_pose_controller_2_225.getLatestMessage
        (&pose_controller_2_B.b_varargout_2);

      // Outputs for Enabled SubSystem: '<S23>/Enabled Subsystem' incorporates:
      //   EnablePort: '<S104>/Enable'

      if (b_varargout_1) {
        pose_controller_2_B.In1_f = pose_controller_2_B.b_varargout_2;
      }

      // End of MATLABSystem: '<S23>/SourceBlock'
      // End of Outputs for SubSystem: '<S23>/Enabled Subsystem'
      // End of Outputs for SubSystem: '<S4>/Subscribe1'
    }

    // MATLAB Function: '<S39>/MATLAB Function1' incorporates:
    //   MATLAB Function: '<S40>/MATLAB Function2'

    pose_controller_2_B.rtb_p_bar_tmp = cos(pose_controller_2_B.In1_l.Y) * cos
      (pose_controller_2_B.In1_l.Z);
    pose_controller_2_B.Rd_k[0] = pose_controller_2_B.rtb_p_bar_tmp;
    pose_controller_2_B.rtb_q_bar_tmp = sin(pose_controller_2_B.In1_l.X) * sin
      (pose_controller_2_B.In1_l.Y) * cos(pose_controller_2_B.In1_l.Z) - cos
      (pose_controller_2_B.In1_l.X) * sin(pose_controller_2_B.In1_l.Z);
    pose_controller_2_B.Rd_k[1] = pose_controller_2_B.rtb_q_bar_tmp;
    pose_controller_2_B.rtb_q_bar_tmp_p = cos(pose_controller_2_B.In1_l.X) * sin
      (pose_controller_2_B.In1_l.Y) * cos(pose_controller_2_B.In1_l.Z) + sin
      (pose_controller_2_B.In1_l.X) * sin(pose_controller_2_B.In1_l.Z);
    pose_controller_2_B.Rd_k[2] = pose_controller_2_B.rtb_q_bar_tmp_p;
    pose_controller_2_B.rtb_q_bar_tmp_l = cos(pose_controller_2_B.In1_l.Y) * sin
      (pose_controller_2_B.In1_l.Z);
    pose_controller_2_B.Rd_k[3] = pose_controller_2_B.rtb_q_bar_tmp_l;
    pose_controller_2_B.Derivative_o = sin(pose_controller_2_B.In1_l.X) * sin
      (pose_controller_2_B.In1_l.Y) * sin(pose_controller_2_B.In1_l.Z) + cos
      (pose_controller_2_B.In1_l.X) * cos(pose_controller_2_B.In1_l.Z);
    pose_controller_2_B.Rd_k[4] = pose_controller_2_B.Derivative_o;
    pose_controller_2_B.Derivative_j = cos(pose_controller_2_B.In1_l.X) * sin
      (pose_controller_2_B.In1_l.Y) * sin(pose_controller_2_B.In1_l.Z) - sin
      (pose_controller_2_B.In1_l.X) * cos(pose_controller_2_B.In1_l.Z);
    pose_controller_2_B.Rd_k[5] = pose_controller_2_B.Derivative_j;
    pose_controller_2_B.Rd_k[6] = -sin(pose_controller_2_B.In1_l.Y);
    pose_controller_2_B.vd_tmp = sin(pose_controller_2_B.In1_l.X) * cos
      (pose_controller_2_B.In1_l.Y);
    pose_controller_2_B.Rd_k[7] = pose_controller_2_B.vd_tmp;
    pose_controller_2_B.vd_tmp_n = cos(pose_controller_2_B.In1_l.X) * cos
      (pose_controller_2_B.In1_l.Y);
    pose_controller_2_B.Rd_k[8] = pose_controller_2_B.vd_tmp_n;
    pose_controller_2_B.Rt_cx[0] = pose_controller_2_B.Derivative_d;
    pose_controller_2_B.Rt_cx[3] = pose_controller_2_B.Rd_tmp;
    pose_controller_2_B.Rt_cx[6] = pose_controller_2_B.Rd_tmp_j;
    pose_controller_2_B.Rt_cx[1] = pose_controller_2_B.Rd_tmp_d;
    pose_controller_2_B.Rt_cx[4] = pose_controller_2_B.Rd_tmp_g;
    pose_controller_2_B.Rt_cx[7] = pose_controller_2_B.Rd_tmp_l;
    pose_controller_2_B.Rt_cx[2] = -sin(pose_controller_2_B.In1.Y);
    pose_controller_2_B.Rt_cx[5] = pose_controller_2_B.Rd_tmp_dh;
    pose_controller_2_B.Rt_cx[8] = pose_controller_2_B.Rd_tmp_dy;
    pose_controller_2_B.position_error[0] = pose_controller_2_B.Derivative;
    pose_controller_2_B.position_error[1] = pose_controller_2_B.q_bar;
    pose_controller_2_B.position_error[2] = pose_controller_2_B.Derivative_i;
    for (pose_controller_2_B.i = 0; pose_controller_2_B.i < 3;
         pose_controller_2_B.i++) {
      pose_controller_2_B.vd[pose_controller_2_B.i] = 0.0;
      for (pose_controller_2_B.B_tmp = 0; pose_controller_2_B.B_tmp < 3;
           pose_controller_2_B.B_tmp++) {
        pose_controller_2_B.Rt_tmp = pose_controller_2_B.i + 3 *
          pose_controller_2_B.B_tmp;
        pose_controller_2_B.dv[pose_controller_2_B.Rt_tmp] = 0.0;
        pose_controller_2_B.dv[pose_controller_2_B.Rt_tmp] +=
          pose_controller_2_B.Rt_cx[3 * pose_controller_2_B.B_tmp] *
          pose_controller_2_B.Rd_k[pose_controller_2_B.i];
        pose_controller_2_B.dv[pose_controller_2_B.Rt_tmp] +=
          pose_controller_2_B.Rt_cx[3 * pose_controller_2_B.B_tmp + 1] *
          pose_controller_2_B.Rd_k[pose_controller_2_B.i + 3];
        pose_controller_2_B.dv[pose_controller_2_B.Rt_tmp] +=
          pose_controller_2_B.Rt_cx[3 * pose_controller_2_B.B_tmp + 2] *
          pose_controller_2_B.Rd_k[pose_controller_2_B.i + 6];
        pose_controller_2_B.vd[pose_controller_2_B.i] +=
          pose_controller_2_B.dv[pose_controller_2_B.Rt_tmp] *
          pose_controller_2_B.position_error[pose_controller_2_B.B_tmp];
      }
    }

    // Integrator: '<S49>/Integrator'
    pose_controller_2_B.Integrator_aj = pose_controller_2_X.Integrator_CSTATE_cv;

    // Derivative: '<S49>/Derivative'
    if ((pose_controller_2_DW.TimeStampA_fq >= pose_controller_2_M->Timing.t[0])
        && (pose_controller_2_DW.TimeStampB_m >= pose_controller_2_M->Timing.t[0]))
    {
      pose_controller_2_B.Derivative = 0.0;
    } else {
      pose_controller_2_B.Derivative_d = pose_controller_2_DW.TimeStampA_fq;
      lastU = &pose_controller_2_DW.LastUAtTimeA_et;
      if (pose_controller_2_DW.TimeStampA_fq < pose_controller_2_DW.TimeStampB_m)
      {
        if (pose_controller_2_DW.TimeStampB_m < pose_controller_2_M->Timing.t[0])
        {
          pose_controller_2_B.Derivative_d = pose_controller_2_DW.TimeStampB_m;
          lastU = &pose_controller_2_DW.LastUAtTimeB_ba;
        }
      } else {
        if (pose_controller_2_DW.TimeStampA_fq >= pose_controller_2_M->Timing.t
            [0]) {
          pose_controller_2_B.Derivative_d = pose_controller_2_DW.TimeStampB_m;
          lastU = &pose_controller_2_DW.LastUAtTimeB_ba;
        }
      }

      pose_controller_2_B.Derivative = (pose_controller_2_B.Integrator_aj -
        *lastU) / (pose_controller_2_M->Timing.t[0] -
                   pose_controller_2_B.Derivative_d);
    }

    // End of Derivative: '<S49>/Derivative'

    // Integrator: '<S50>/Integrator'
    pose_controller_2_B.Integrator_j = pose_controller_2_X.Integrator_CSTATE_b;

    // Derivative: '<S50>/Derivative'
    if ((pose_controller_2_DW.TimeStampA_a >= pose_controller_2_M->Timing.t[0]) &&
        (pose_controller_2_DW.TimeStampB_gd >= pose_controller_2_M->Timing.t[0]))
    {
      pose_controller_2_B.q_bar = 0.0;
    } else {
      pose_controller_2_B.Derivative_d = pose_controller_2_DW.TimeStampA_a;
      lastU = &pose_controller_2_DW.LastUAtTimeA_p;
      if (pose_controller_2_DW.TimeStampA_a < pose_controller_2_DW.TimeStampB_gd)
      {
        if (pose_controller_2_DW.TimeStampB_gd < pose_controller_2_M->Timing.t[0])
        {
          pose_controller_2_B.Derivative_d = pose_controller_2_DW.TimeStampB_gd;
          lastU = &pose_controller_2_DW.LastUAtTimeB_d;
        }
      } else {
        if (pose_controller_2_DW.TimeStampA_a >= pose_controller_2_M->Timing.t[0])
        {
          pose_controller_2_B.Derivative_d = pose_controller_2_DW.TimeStampB_gd;
          lastU = &pose_controller_2_DW.LastUAtTimeB_d;
        }
      }

      pose_controller_2_B.q_bar = (pose_controller_2_B.Integrator_j - *lastU) /
        (pose_controller_2_M->Timing.t[0] - pose_controller_2_B.Derivative_d);
    }

    // End of Derivative: '<S50>/Derivative'

    // Integrator: '<S51>/Integrator'
    pose_controller_2_B.Integrator_h = pose_controller_2_X.Integrator_CSTATE_o;

    // Derivative: '<S51>/Derivative'
    if ((pose_controller_2_DW.TimeStampA_l >= pose_controller_2_M->Timing.t[0]) &&
        (pose_controller_2_DW.TimeStampB_p >= pose_controller_2_M->Timing.t[0]))
    {
      pose_controller_2_B.Derivative_i = 0.0;
    } else {
      pose_controller_2_B.Derivative_d = pose_controller_2_DW.TimeStampA_l;
      lastU = &pose_controller_2_DW.LastUAtTimeA_g;
      if (pose_controller_2_DW.TimeStampA_l < pose_controller_2_DW.TimeStampB_p)
      {
        if (pose_controller_2_DW.TimeStampB_p < pose_controller_2_M->Timing.t[0])
        {
          pose_controller_2_B.Derivative_d = pose_controller_2_DW.TimeStampB_p;
          lastU = &pose_controller_2_DW.LastUAtTimeB_lt;
        }
      } else {
        if (pose_controller_2_DW.TimeStampA_l >= pose_controller_2_M->Timing.t[0])
        {
          pose_controller_2_B.Derivative_d = pose_controller_2_DW.TimeStampB_p;
          lastU = &pose_controller_2_DW.LastUAtTimeB_lt;
        }
      }

      pose_controller_2_B.Derivative_i = (pose_controller_2_B.Integrator_h -
        *lastU) / (pose_controller_2_M->Timing.t[0] -
                   pose_controller_2_B.Derivative_d);
    }

    // End of Derivative: '<S51>/Derivative'
    if (rtmIsMajorTimeStep(pose_controller_2_M)) {
      // MATLAB Function: '<S40>/MATLAB Function2' incorporates:
      //   Constant: '<S31>/tool_x'
      //   Constant: '<S31>/tool_y'
      //   Constant: '<S31>/tool_z'
      //   Constant: '<S40>/kp_phi'
      //   Constant: '<S40>/kp_psi'
      //   Constant: '<S40>/kp_theta'
      //   Constant: '<S40>/kp_x'
      //   Constant: '<S40>/kp_y'
      //   Constant: '<S40>/kp_z'

      pose_controller_2_B.R[0] = pose_controller_2_B.rtb_p_bar_tmp;
      pose_controller_2_B.R[3] = pose_controller_2_B.rtb_q_bar_tmp;
      pose_controller_2_B.R[6] = pose_controller_2_B.rtb_q_bar_tmp_p;
      pose_controller_2_B.R[1] = pose_controller_2_B.rtb_q_bar_tmp_l;
      pose_controller_2_B.R[4] = pose_controller_2_B.Derivative_o;
      pose_controller_2_B.R[7] = pose_controller_2_B.Derivative_j;
      pose_controller_2_B.R[2] = -sin(pose_controller_2_B.In1_l.Y);
      pose_controller_2_B.R[5] = pose_controller_2_B.vd_tmp;
      pose_controller_2_B.R[8] = pose_controller_2_B.vd_tmp_n;
      for (pose_controller_2_B.i = 0; pose_controller_2_B.i < 3;
           pose_controller_2_B.i++) {
        for (pose_controller_2_B.B_tmp = 0; pose_controller_2_B.B_tmp < 3;
             pose_controller_2_B.B_tmp++) {
          pose_controller_2_B.Rt_tmp = 3 * pose_controller_2_B.B_tmp +
            pose_controller_2_B.i;
          pose_controller_2_B.Rt_c[pose_controller_2_B.B_tmp + 3 *
            pose_controller_2_B.i] =
            pose_controller_2_B.R[pose_controller_2_B.Rt_tmp];
          pose_controller_2_B.Rd_k[pose_controller_2_B.Rt_tmp] = 0.0;
          pose_controller_2_B.Rd_k[pose_controller_2_B.Rt_tmp] +=
            pose_controller_2_B.Rd[3 * pose_controller_2_B.i] *
            pose_controller_2_B.R[3 * pose_controller_2_B.B_tmp];
          pose_controller_2_B.Rd_k[pose_controller_2_B.Rt_tmp] +=
            pose_controller_2_B.Rd[3 * pose_controller_2_B.i + 1] *
            pose_controller_2_B.R[3 * pose_controller_2_B.B_tmp + 1];
          pose_controller_2_B.Rd_k[pose_controller_2_B.Rt_tmp] +=
            pose_controller_2_B.Rd[3 * pose_controller_2_B.i + 2] *
            pose_controller_2_B.R[3 * pose_controller_2_B.B_tmp + 2];
        }
      }

      for (pose_controller_2_B.i = 0; pose_controller_2_B.i < 3;
           pose_controller_2_B.i++) {
        for (pose_controller_2_B.B_tmp = 0; pose_controller_2_B.B_tmp < 3;
             pose_controller_2_B.B_tmp++) {
          pose_controller_2_B.Rt_tmp = pose_controller_2_B.B_tmp + 3 *
            pose_controller_2_B.i;
          pose_controller_2_B.Rt_cx[pose_controller_2_B.Rt_tmp] = 0.0;
          pose_controller_2_B.Rt_cx[pose_controller_2_B.Rt_tmp] +=
            pose_controller_2_B.Rd[3 * pose_controller_2_B.i] *
            pose_controller_2_B.Rt_c[pose_controller_2_B.B_tmp];
          pose_controller_2_B.Rt_cx[pose_controller_2_B.Rt_tmp] +=
            pose_controller_2_B.Rd[3 * pose_controller_2_B.i + 1] *
            pose_controller_2_B.Rt_c[pose_controller_2_B.B_tmp + 3];
          pose_controller_2_B.Rt_cx[pose_controller_2_B.Rt_tmp] +=
            pose_controller_2_B.Rd[3 * pose_controller_2_B.i + 2] *
            pose_controller_2_B.Rt_c[pose_controller_2_B.B_tmp + 6];
        }
      }

      for (pose_controller_2_B.i = 0; pose_controller_2_B.i < 9;
           pose_controller_2_B.i++) {
        pose_controller_2_B.Rd[pose_controller_2_B.i] =
          pose_controller_2_B.Rd_k[pose_controller_2_B.i] -
          pose_controller_2_B.Rt_cx[pose_controller_2_B.i];
      }

      pose_controller_2_B.Rd_k[0] = pose_controller_2_P.kp_phi_Value;
      pose_controller_2_B.Rd_k[3] = 0.0;
      pose_controller_2_B.Rd_k[6] = 0.0;
      pose_controller_2_B.Rd_k[1] = 0.0;
      pose_controller_2_B.Rd_k[4] = pose_controller_2_P.kp_theta_Value;
      pose_controller_2_B.Rd_k[7] = 0.0;
      pose_controller_2_B.Rd_k[2] = 0.0;
      pose_controller_2_B.Rd_k[5] = 0.0;
      pose_controller_2_B.Rd_k[8] = pose_controller_2_P.kp_psi_Value;
      pose_controller_2_B.Rt_cx[0] = pose_controller_2_P.kp_x_Value;
      pose_controller_2_B.Rt_cx[3] = 0.0;
      pose_controller_2_B.Rt_cx[6] = 0.0;
      pose_controller_2_B.Rt_cx[1] = 0.0;
      pose_controller_2_B.Rt_cx[4] = pose_controller_2_P.kp_y_Value;
      pose_controller_2_B.Rt_cx[7] = 0.0;
      pose_controller_2_B.Rt_cx[2] = 0.0;
      pose_controller_2_B.Rt_cx[5] = 0.0;
      pose_controller_2_B.Rt_cx[8] = pose_controller_2_P.kp_z_Value;
      pose_controller_2_B.Sum1_b[0] = pose_controller_2_B.ex[0];
      pose_controller_2_B.Sum1_b[1] = pose_controller_2_B.ex[1];
      pose_controller_2_B.Sum1_b[2] = pose_controller_2_B.ex[2];
      for (pose_controller_2_B.i = 0; pose_controller_2_B.i < 3;
           pose_controller_2_B.i++) {
        pose_controller_2_B.position_error[pose_controller_2_B.i] = 0.0;
        for (pose_controller_2_B.B_tmp = 0; pose_controller_2_B.B_tmp < 3;
             pose_controller_2_B.B_tmp++) {
          pose_controller_2_B.Rt_tmp = pose_controller_2_B.i + 3 *
            pose_controller_2_B.B_tmp;
          pose_controller_2_B.R[pose_controller_2_B.Rt_tmp] = 0.0;
          pose_controller_2_B.dv[pose_controller_2_B.Rt_tmp] = 0.0;
          pose_controller_2_B.R[pose_controller_2_B.Rt_tmp] +=
            pose_controller_2_B.Rd[3 * pose_controller_2_B.B_tmp] *
            pose_controller_2_B.Rd_k[pose_controller_2_B.i];
          pose_controller_2_B.dv[pose_controller_2_B.Rt_tmp] +=
            pose_controller_2_B.Rt_c[3 * pose_controller_2_B.B_tmp] *
            pose_controller_2_B.Rt_cx[pose_controller_2_B.i];
          pose_controller_2_B.R_tmp = 3 * pose_controller_2_B.B_tmp + 1;
          pose_controller_2_B.R[pose_controller_2_B.Rt_tmp] +=
            pose_controller_2_B.Rd[pose_controller_2_B.R_tmp] *
            pose_controller_2_B.Rd_k[pose_controller_2_B.i + 3];
          pose_controller_2_B.dv[pose_controller_2_B.Rt_tmp] +=
            pose_controller_2_B.Rt_c[pose_controller_2_B.R_tmp] *
            pose_controller_2_B.Rt_cx[pose_controller_2_B.i + 3];
          pose_controller_2_B.R_tmp = 3 * pose_controller_2_B.B_tmp + 2;
          pose_controller_2_B.R[pose_controller_2_B.Rt_tmp] +=
            pose_controller_2_B.Rd[pose_controller_2_B.R_tmp] *
            pose_controller_2_B.Rd_k[pose_controller_2_B.i + 6];
          pose_controller_2_B.dv[pose_controller_2_B.Rt_tmp] +=
            pose_controller_2_B.Rt_c[pose_controller_2_B.R_tmp] *
            pose_controller_2_B.Rt_cx[pose_controller_2_B.i + 6];
          pose_controller_2_B.position_error[pose_controller_2_B.i] +=
            pose_controller_2_B.dv[pose_controller_2_B.Rt_tmp] *
            pose_controller_2_B.Sum1_b[pose_controller_2_B.B_tmp];
        }
      }

      pose_controller_2_B.grad1 = -(pose_controller_2_P.tool_y_Value *
        pose_controller_2_B.position_error[2] - pose_controller_2_P.tool_z_Value
        * pose_controller_2_B.position_error[1]) + pose_controller_2_B.R[5];
      pose_controller_2_B.grad2 = -(pose_controller_2_P.tool_z_Value *
        pose_controller_2_B.position_error[0] - pose_controller_2_P.tool_x_Value
        * pose_controller_2_B.position_error[2]) + pose_controller_2_B.R[6];
      pose_controller_2_B.grad3 = -(pose_controller_2_P.tool_x_Value *
        pose_controller_2_B.position_error[1] - pose_controller_2_P.tool_y_Value
        * pose_controller_2_B.position_error[0]) + pose_controller_2_B.R[1];
      pose_controller_2_B.phi_error = pose_controller_2_B.Rd[5];
      pose_controller_2_B.theta_error = pose_controller_2_B.Rd[6];
      pose_controller_2_B.psi_error = pose_controller_2_B.Rd[1];

      // MATLAB Function: '<S15>/calc_J_sys' incorporates:
      //   Constant: '<S101>/Ixx'
      //   Constant: '<S101>/Iyy'
      //   Constant: '<S101>/Izz'
      //   Constant: '<S31>/drone1_x'
      //   Constant: '<S31>/drone1_y'
      //   Constant: '<S31>/drone1_z'
      //   Constant: '<S31>/drone2_x'
      //   Constant: '<S31>/drone2_y'
      //   Constant: '<S31>/drone2_z'
      //   Constant: '<S31>/drone3_x'
      //   Constant: '<S31>/drone3_y'
      //   Constant: '<S31>/drone3_z'
      //   Constant: '<S32>/drone1_mass'
      //   Constant: '<S32>/drone2_mass'
      //   Constant: '<S32>/drone3_mass'

      pose_controller_2_B.Rd_k[0] = pose_controller_2_P.drone1_mass_Value * 0.0;
      pose_controller_2_B.Rd_k[3] = pose_controller_2_P.drone1_mass_Value *
        -pose_controller_2_P.drone1_z_Value;
      pose_controller_2_B.Rd_k[6] = pose_controller_2_P.drone1_mass_Value *
        pose_controller_2_P.drone1_y_Value;
      pose_controller_2_B.Rd_k[1] = pose_controller_2_P.drone1_mass_Value *
        pose_controller_2_P.drone1_z_Value;
      pose_controller_2_B.Rd_k[4] = pose_controller_2_P.drone1_mass_Value * 0.0;
      pose_controller_2_B.Rd_k[7] = pose_controller_2_P.drone1_mass_Value *
        -pose_controller_2_P.drone1_x_Value;
      pose_controller_2_B.Rd_k[2] = pose_controller_2_P.drone1_mass_Value *
        -pose_controller_2_P.drone1_y_Value;
      pose_controller_2_B.Rd_k[5] = pose_controller_2_P.drone1_mass_Value *
        pose_controller_2_P.drone1_x_Value;
      pose_controller_2_B.Rd_k[8] = pose_controller_2_P.drone1_mass_Value * 0.0;
      pose_controller_2_B.Rt_cx[0] = 0.0;
      pose_controller_2_B.Rt_cx[3] = -pose_controller_2_P.drone1_z_Value;
      pose_controller_2_B.Rt_cx[6] = pose_controller_2_P.drone1_y_Value;
      pose_controller_2_B.Rt_cx[1] = pose_controller_2_P.drone1_z_Value;
      pose_controller_2_B.Rt_cx[4] = 0.0;
      pose_controller_2_B.Rt_cx[7] = -pose_controller_2_P.drone1_x_Value;
      pose_controller_2_B.Rt_cx[2] = -pose_controller_2_P.drone1_y_Value;
      pose_controller_2_B.Rt_cx[5] = pose_controller_2_P.drone1_x_Value;
      pose_controller_2_B.Rt_cx[8] = 0.0;
      pose_controller_2_B.dv[0] = pose_controller_2_P.Ixx_Value;
      pose_controller_2_B.dv[3] = 0.0;
      pose_controller_2_B.dv[6] = 0.0;
      pose_controller_2_B.dv[1] = 0.0;
      pose_controller_2_B.dv[4] = pose_controller_2_P.Iyy_Value;
      pose_controller_2_B.dv[7] = 0.0;
      pose_controller_2_B.dv[2] = 0.0;
      pose_controller_2_B.dv[5] = 0.0;
      pose_controller_2_B.dv[8] = pose_controller_2_P.Izz_Value;
      for (pose_controller_2_B.i = 0; pose_controller_2_B.i < 3;
           pose_controller_2_B.i++) {
        for (pose_controller_2_B.B_tmp = 0; pose_controller_2_B.B_tmp < 3;
             pose_controller_2_B.B_tmp++) {
          pose_controller_2_B.Rt_tmp = pose_controller_2_B.B_tmp + 3 *
            pose_controller_2_B.i;
          pose_controller_2_B.Rd[pose_controller_2_B.Rt_tmp] = 0.0;
          pose_controller_2_B.Rd[pose_controller_2_B.Rt_tmp] +=
            pose_controller_2_B.Rt_cx[3 * pose_controller_2_B.i] *
            pose_controller_2_B.Rd_k[pose_controller_2_B.B_tmp];
          pose_controller_2_B.Rd[pose_controller_2_B.Rt_tmp] +=
            pose_controller_2_B.Rt_cx[3 * pose_controller_2_B.i + 1] *
            pose_controller_2_B.Rd_k[pose_controller_2_B.B_tmp + 3];
          pose_controller_2_B.Rd[pose_controller_2_B.Rt_tmp] +=
            pose_controller_2_B.Rt_cx[3 * pose_controller_2_B.i + 2] *
            pose_controller_2_B.Rd_k[pose_controller_2_B.B_tmp + 6];
        }
      }

      pose_controller_2_B.Rd_k[0] = pose_controller_2_P.drone2_mass_Value * 0.0;
      pose_controller_2_B.Rd_k[3] = pose_controller_2_P.drone2_mass_Value *
        -pose_controller_2_P.drone2_z_Value;
      pose_controller_2_B.Rd_k[6] = pose_controller_2_P.drone2_mass_Value *
        pose_controller_2_P.drone2_y_Value;
      pose_controller_2_B.Rd_k[1] = pose_controller_2_P.drone2_mass_Value *
        pose_controller_2_P.drone2_z_Value;
      pose_controller_2_B.Rd_k[4] = pose_controller_2_P.drone2_mass_Value * 0.0;
      pose_controller_2_B.Rd_k[7] = pose_controller_2_P.drone2_mass_Value *
        -pose_controller_2_P.drone2_x_Value;
      pose_controller_2_B.Rd_k[2] = pose_controller_2_P.drone2_mass_Value *
        -pose_controller_2_P.drone2_y_Value;
      pose_controller_2_B.Rd_k[5] = pose_controller_2_P.drone2_mass_Value *
        pose_controller_2_P.drone2_x_Value;
      pose_controller_2_B.Rd_k[8] = pose_controller_2_P.drone2_mass_Value * 0.0;
      pose_controller_2_B.Rt_cx[0] = 0.0;
      pose_controller_2_B.Rt_cx[3] = -pose_controller_2_P.drone2_z_Value;
      pose_controller_2_B.Rt_cx[6] = pose_controller_2_P.drone2_y_Value;
      pose_controller_2_B.Rt_cx[1] = pose_controller_2_P.drone2_z_Value;
      pose_controller_2_B.Rt_cx[4] = 0.0;
      pose_controller_2_B.Rt_cx[7] = -pose_controller_2_P.drone2_x_Value;
      pose_controller_2_B.Rt_cx[2] = -pose_controller_2_P.drone2_y_Value;
      pose_controller_2_B.Rt_cx[5] = pose_controller_2_P.drone2_x_Value;
      pose_controller_2_B.Rt_cx[8] = 0.0;
      pose_controller_2_B.dv1[0] = pose_controller_2_P.drone3_mass_Value * 0.0;
      pose_controller_2_B.dv1[3] = pose_controller_2_P.drone3_mass_Value *
        -pose_controller_2_P.drone3_z_Value;
      pose_controller_2_B.dv1[6] = pose_controller_2_P.drone3_mass_Value *
        pose_controller_2_P.drone3_y_Value;
      pose_controller_2_B.dv1[1] = pose_controller_2_P.drone3_mass_Value *
        pose_controller_2_P.drone3_z_Value;
      pose_controller_2_B.dv1[4] = pose_controller_2_P.drone3_mass_Value * 0.0;
      pose_controller_2_B.dv1[7] = pose_controller_2_P.drone3_mass_Value *
        -pose_controller_2_P.drone3_x_Value;
      pose_controller_2_B.dv1[2] = pose_controller_2_P.drone3_mass_Value *
        -pose_controller_2_P.drone3_y_Value;
      pose_controller_2_B.dv1[5] = pose_controller_2_P.drone3_mass_Value *
        pose_controller_2_P.drone3_x_Value;
      pose_controller_2_B.dv1[8] = pose_controller_2_P.drone3_mass_Value * 0.0;
      pose_controller_2_B.dv2[0] = 0.0;
      pose_controller_2_B.dv2[3] = -pose_controller_2_P.drone3_z_Value;
      pose_controller_2_B.dv2[6] = pose_controller_2_P.drone3_y_Value;
      pose_controller_2_B.dv2[1] = pose_controller_2_P.drone3_z_Value;
      pose_controller_2_B.dv2[4] = 0.0;
      pose_controller_2_B.dv2[7] = -pose_controller_2_P.drone3_x_Value;
      pose_controller_2_B.dv2[2] = -pose_controller_2_P.drone3_y_Value;
      pose_controller_2_B.dv2[5] = pose_controller_2_P.drone3_x_Value;
      pose_controller_2_B.dv2[8] = 0.0;
      for (pose_controller_2_B.i = 0; pose_controller_2_B.i < 3;
           pose_controller_2_B.i++) {
        for (pose_controller_2_B.B_tmp = 0; pose_controller_2_B.B_tmp < 3;
             pose_controller_2_B.B_tmp++) {
          pose_controller_2_B.Rt_tmp = pose_controller_2_B.i + 3 *
            pose_controller_2_B.B_tmp;
          pose_controller_2_B.R[pose_controller_2_B.Rt_tmp] = 0.0;
          pose_controller_2_B.R[pose_controller_2_B.Rt_tmp] +=
            pose_controller_2_B.dv2[3 * pose_controller_2_B.B_tmp] *
            pose_controller_2_B.dv1[pose_controller_2_B.i];
          pose_controller_2_B.R_tmp = 3 * pose_controller_2_B.B_tmp + 1;
          pose_controller_2_B.R[pose_controller_2_B.Rt_tmp] +=
            pose_controller_2_B.dv2[pose_controller_2_B.R_tmp] *
            pose_controller_2_B.dv1[pose_controller_2_B.i + 3];
          pose_controller_2_B.i1 = 3 * pose_controller_2_B.B_tmp + 2;
          pose_controller_2_B.R[pose_controller_2_B.Rt_tmp] +=
            pose_controller_2_B.dv2[pose_controller_2_B.i1] *
            pose_controller_2_B.dv1[pose_controller_2_B.i + 6];
          pose_controller_2_B.Rt_c[pose_controller_2_B.Rt_tmp] =
            (pose_controller_2_B.dv[pose_controller_2_B.Rt_tmp] -
             pose_controller_2_B.Rd[pose_controller_2_B.Rt_tmp]) -
            ((pose_controller_2_B.Rt_cx[pose_controller_2_B.R_tmp] *
              pose_controller_2_B.Rd_k[pose_controller_2_B.i + 3] +
              pose_controller_2_B.Rt_cx[3 * pose_controller_2_B.B_tmp] *
              pose_controller_2_B.Rd_k[pose_controller_2_B.i]) +
             pose_controller_2_B.Rt_cx[pose_controller_2_B.i1] *
             pose_controller_2_B.Rd_k[pose_controller_2_B.i + 6]);
        }
      }

      for (pose_controller_2_B.i = 0; pose_controller_2_B.i < 9;
           pose_controller_2_B.i++) {
        pose_controller_2_B.Rd[pose_controller_2_B.i] =
          pose_controller_2_B.Rt_c[pose_controller_2_B.i] -
          pose_controller_2_B.R[pose_controller_2_B.i];
      }

      pose_controller_2_B.Ix_sys = pose_controller_2_B.Rd[0];
      pose_controller_2_B.Iy_sys = pose_controller_2_B.Rd[4];
      pose_controller_2_B.Iz_sys = pose_controller_2_B.Rd[8];

      // End of MATLAB Function: '<S15>/calc_J_sys'
    }

    // Integrator: '<S44>/Integrator'
    if (pose_controller_2_DW.Integrator_IWORK_lt != 0) {
      pose_controller_2_X.Integrator_CSTATE_a = pose_controller_2_B.In1_fj.X;
    }

    // Integrator: '<S44>/Integrator1'
    if (pose_controller_2_DW.Integrator1_IWORK != 0) {
      pose_controller_2_X.Integrator1_CSTATE = pose_controller_2_B.In1_fj.Y;
    }

    // Integrator: '<S44>/Integrator2'
    if (pose_controller_2_DW.Integrator2_IWORK != 0) {
      pose_controller_2_X.Integrator2_CSTATE = pose_controller_2_B.In1_fj.Z;
    }

    // MATLAB Function: '<S38>/MATLAB Function' incorporates:
    //   Constant: '<S38>/kd_phi'
    //   Constant: '<S38>/kd_psi'
    //   Constant: '<S38>/kd_theta'
    //   Constant: '<S38>/ki_phi'
    //   Constant: '<S38>/ki_psi'
    //   Constant: '<S38>/ki_theta'
    //   Integrator: '<S44>/Integrator'
    //   Integrator: '<S44>/Integrator1'
    //   Integrator: '<S44>/Integrator2'
    //   MATLAB Function: '<S39>/MATLAB Function1'
    //   Sum: '<S14>/Sum2'

    pose_controller_2_B.Rd[0] = pose_controller_2_B.Ix_sys;
    pose_controller_2_B.Rd[3] = 0.0;
    pose_controller_2_B.Rd[6] = 0.0;
    pose_controller_2_B.Rd[1] = 0.0;
    pose_controller_2_B.Rd[4] = pose_controller_2_B.Iy_sys;
    pose_controller_2_B.Rd[7] = 0.0;
    pose_controller_2_B.Rd[2] = 0.0;
    pose_controller_2_B.Rd[5] = 0.0;
    pose_controller_2_B.Rd[8] = pose_controller_2_B.Iz_sys;
    for (pose_controller_2_B.i = 0; pose_controller_2_B.i < 3;
         pose_controller_2_B.i++) {
      pose_controller_2_B.Derivative_d =
        pose_controller_2_B.Rd[pose_controller_2_B.i];
      pose_controller_2_B.rtb_p_bar_tmp = pose_controller_2_B.Derivative_d *
        pose_controller_2_B.In1_f.X;
      pose_controller_2_B.rtb_q_bar_tmp = pose_controller_2_B.Derivative_d *
        pose_controller_2_B.Derivative;
      pose_controller_2_B.Derivative_d =
        pose_controller_2_B.Rd[pose_controller_2_B.i + 3];
      pose_controller_2_B.rtb_p_bar_tmp += pose_controller_2_B.Derivative_d *
        pose_controller_2_B.In1_f.Y;
      pose_controller_2_B.rtb_q_bar_tmp += pose_controller_2_B.Derivative_d *
        pose_controller_2_B.q_bar;
      pose_controller_2_B.Derivative_d =
        pose_controller_2_B.Rd[pose_controller_2_B.i + 6];
      pose_controller_2_B.rtb_p_bar_tmp += pose_controller_2_B.Derivative_d *
        pose_controller_2_B.In1_f.Z;
      pose_controller_2_B.rtb_q_bar_tmp += pose_controller_2_B.Derivative_d *
        pose_controller_2_B.Derivative_i;
      pose_controller_2_B.position_error[pose_controller_2_B.i] =
        pose_controller_2_B.rtb_p_bar_tmp;
      pose_controller_2_B.Rd_p[pose_controller_2_B.i] =
        pose_controller_2_B.rtb_q_bar_tmp;
    }

    pose_controller_2_B.position_error_c[0] =
      pose_controller_2_B.position_error[1] * pose_controller_2_B.vd[2] -
      pose_controller_2_B.position_error[2] * pose_controller_2_B.vd[1];
    pose_controller_2_B.position_error_c[1] =
      pose_controller_2_B.position_error[2] * pose_controller_2_B.vd[0] -
      pose_controller_2_B.position_error[0] * pose_controller_2_B.vd[2];
    pose_controller_2_B.position_error_c[2] =
      pose_controller_2_B.position_error[0] * pose_controller_2_B.vd[1] -
      pose_controller_2_B.position_error[1] * pose_controller_2_B.vd[0];
    pose_controller_2_B.Rd_k[0] = pose_controller_2_P.kd_phi_Value;
    pose_controller_2_B.Rd_k[3] = 0.0;
    pose_controller_2_B.Rd_k[6] = 0.0;
    pose_controller_2_B.Rd_k[1] = 0.0;
    pose_controller_2_B.Rd_k[4] = pose_controller_2_P.kd_theta_Value;
    pose_controller_2_B.Rd_k[7] = 0.0;
    pose_controller_2_B.Rd_k[2] = 0.0;
    pose_controller_2_B.Rd_k[5] = 0.0;
    pose_controller_2_B.Rd_k[8] = pose_controller_2_P.kd_psi_Value;
    pose_controller_2_B.Derivative = pose_controller_2_B.In1_f.X -
      pose_controller_2_B.vd[0];
    pose_controller_2_B.q_bar = pose_controller_2_B.In1_f.Y -
      pose_controller_2_B.vd[1];
    pose_controller_2_B.Derivative_i = pose_controller_2_B.In1_f.Z -
      pose_controller_2_B.vd[2];
    pose_controller_2_B.Rt_cx[0] = pose_controller_2_P.ki_phi_Value;
    pose_controller_2_B.Rt_cx[3] = 0.0;
    pose_controller_2_B.Rt_cx[6] = 0.0;
    pose_controller_2_B.Rt_cx[1] = 0.0;
    pose_controller_2_B.Rt_cx[4] = pose_controller_2_P.ki_theta_Value;
    pose_controller_2_B.Rt_cx[7] = 0.0;
    pose_controller_2_B.Rt_cx[2] = 0.0;
    pose_controller_2_B.Rt_cx[5] = 0.0;
    pose_controller_2_B.Rt_cx[8] = pose_controller_2_P.ki_psi_Value;
    pose_controller_2_B.Sum1_b[0] = pose_controller_2_B.grad1;
    pose_controller_2_B.Sum1_b[1] = pose_controller_2_B.grad2;
    pose_controller_2_B.Sum1_b[2] = pose_controller_2_B.grad3;
    for (pose_controller_2_B.i = 0; pose_controller_2_B.i < 3;
         pose_controller_2_B.i++) {
      pose_controller_2_B.de[pose_controller_2_B.i] =
        (((pose_controller_2_B.Rd_p[pose_controller_2_B.i] -
           pose_controller_2_B.position_error_c[pose_controller_2_B.i]) -
          ((pose_controller_2_B.Rd_k[pose_controller_2_B.i + 3] *
            pose_controller_2_B.q_bar +
            pose_controller_2_B.Rd_k[pose_controller_2_B.i] *
            pose_controller_2_B.Derivative) +
           pose_controller_2_B.Rd_k[pose_controller_2_B.i + 6] *
           pose_controller_2_B.Derivative_i)) -
         (pose_controller_2_B.Rt_cx[pose_controller_2_B.i + 6] *
          pose_controller_2_X.Integrator2_CSTATE +
          (pose_controller_2_B.Rt_cx[pose_controller_2_B.i + 3] *
           pose_controller_2_X.Integrator1_CSTATE +
           pose_controller_2_B.Rt_cx[pose_controller_2_B.i] *
           pose_controller_2_X.Integrator_CSTATE_a))) -
        pose_controller_2_B.Sum1_b[pose_controller_2_B.i];
    }

    // Integrator: '<S52>/Integrator'
    pose_controller_2_B.Integrator_jx = pose_controller_2_X.Integrator_CSTATE_jf;

    // Derivative: '<S52>/Derivative'
    if ((pose_controller_2_DW.TimeStampA_h >= pose_controller_2_M->Timing.t[0]) &&
        (pose_controller_2_DW.TimeStampB_eu >= pose_controller_2_M->Timing.t[0]))
    {
      pose_controller_2_B.Derivative = 0.0;
    } else {
      pose_controller_2_B.Derivative_d = pose_controller_2_DW.TimeStampA_h;
      lastU = &pose_controller_2_DW.LastUAtTimeA_b;
      if (pose_controller_2_DW.TimeStampA_h < pose_controller_2_DW.TimeStampB_eu)
      {
        if (pose_controller_2_DW.TimeStampB_eu < pose_controller_2_M->Timing.t[0])
        {
          pose_controller_2_B.Derivative_d = pose_controller_2_DW.TimeStampB_eu;
          lastU = &pose_controller_2_DW.LastUAtTimeB_hq;
        }
      } else {
        if (pose_controller_2_DW.TimeStampA_h >= pose_controller_2_M->Timing.t[0])
        {
          pose_controller_2_B.Derivative_d = pose_controller_2_DW.TimeStampB_eu;
          lastU = &pose_controller_2_DW.LastUAtTimeB_hq;
        }
      }

      pose_controller_2_B.Derivative = (pose_controller_2_B.Integrator_jx -
        *lastU) / (pose_controller_2_M->Timing.t[0] -
                   pose_controller_2_B.Derivative_d);
    }

    // End of Derivative: '<S52>/Derivative'

    // Integrator: '<S53>/Integrator'
    pose_controller_2_B.Integrator_a3 = pose_controller_2_X.Integrator_CSTATE_p;

    // Derivative: '<S53>/Derivative'
    if ((pose_controller_2_DW.TimeStampA_hg >= pose_controller_2_M->Timing.t[0])
        && (pose_controller_2_DW.TimeStampB_h >= pose_controller_2_M->Timing.t[0]))
    {
      pose_controller_2_B.q_bar = 0.0;
    } else {
      pose_controller_2_B.Derivative_d = pose_controller_2_DW.TimeStampA_hg;
      lastU = &pose_controller_2_DW.LastUAtTimeA_o;
      if (pose_controller_2_DW.TimeStampA_hg < pose_controller_2_DW.TimeStampB_h)
      {
        if (pose_controller_2_DW.TimeStampB_h < pose_controller_2_M->Timing.t[0])
        {
          pose_controller_2_B.Derivative_d = pose_controller_2_DW.TimeStampB_h;
          lastU = &pose_controller_2_DW.LastUAtTimeB_g;
        }
      } else {
        if (pose_controller_2_DW.TimeStampA_hg >= pose_controller_2_M->Timing.t
            [0]) {
          pose_controller_2_B.Derivative_d = pose_controller_2_DW.TimeStampB_h;
          lastU = &pose_controller_2_DW.LastUAtTimeB_g;
        }
      }

      pose_controller_2_B.q_bar = (pose_controller_2_B.Integrator_a3 - *lastU) /
        (pose_controller_2_M->Timing.t[0] - pose_controller_2_B.Derivative_d);
    }

    // End of Derivative: '<S53>/Derivative'

    // Integrator: '<S54>/Integrator'
    pose_controller_2_B.Integrator_jg = pose_controller_2_X.Integrator_CSTATE_m;

    // Derivative: '<S54>/Derivative'
    if ((pose_controller_2_DW.TimeStampA_p >= pose_controller_2_M->Timing.t[0]) &&
        (pose_controller_2_DW.TimeStampB_gm >= pose_controller_2_M->Timing.t[0]))
    {
      pose_controller_2_B.Derivative_i = 0.0;
    } else {
      pose_controller_2_B.Derivative_d = pose_controller_2_DW.TimeStampA_p;
      lastU = &pose_controller_2_DW.LastUAtTimeA_m;
      if (pose_controller_2_DW.TimeStampA_p < pose_controller_2_DW.TimeStampB_gm)
      {
        if (pose_controller_2_DW.TimeStampB_gm < pose_controller_2_M->Timing.t[0])
        {
          pose_controller_2_B.Derivative_d = pose_controller_2_DW.TimeStampB_gm;
          lastU = &pose_controller_2_DW.LastUAtTimeB_fn;
        }
      } else {
        if (pose_controller_2_DW.TimeStampA_p >= pose_controller_2_M->Timing.t[0])
        {
          pose_controller_2_B.Derivative_d = pose_controller_2_DW.TimeStampB_gm;
          lastU = &pose_controller_2_DW.LastUAtTimeB_fn;
        }
      }

      pose_controller_2_B.Derivative_i = (pose_controller_2_B.Integrator_jg -
        *lastU) / (pose_controller_2_M->Timing.t[0] -
                   pose_controller_2_B.Derivative_d);
    }

    // End of Derivative: '<S54>/Derivative'
    if (rtmIsMajorTimeStep(pose_controller_2_M)) {
      // MATLAB Function: '<S40>/MATLAB Function1' incorporates:
      //   Constant: '<S40>/kp_x'
      //   Constant: '<S40>/kp_y'
      //   Constant: '<S40>/kp_z'

      pose_controller_2_B.Derivative_d = pose_controller_2_B.ex[0];
      pose_controller_2_B.rtb_p_bar_tmp = pose_controller_2_B.ex[1];
      pose_controller_2_B.rtb_q_bar_tmp = pose_controller_2_B.ex[2];
      for (pose_controller_2_B.i = 0; pose_controller_2_B.i < 3;
           pose_controller_2_B.i++) {
        pose_controller_2_B.position_error[pose_controller_2_B.i] =
          pose_controller_2_B.S1[pose_controller_2_B.i + 6] *
          pose_controller_2_B.rtb_q_bar_tmp +
          (pose_controller_2_B.S1[pose_controller_2_B.i + 3] *
           pose_controller_2_B.rtb_p_bar_tmp +
           pose_controller_2_B.S1[pose_controller_2_B.i] *
           pose_controller_2_B.Derivative_d);
      }

      pose_controller_2_B.Rd_k[0] = pose_controller_2_P.kp_x_Value;
      pose_controller_2_B.Rd_k[3] = 0.0;
      pose_controller_2_B.Rd_k[6] = 0.0;
      pose_controller_2_B.Rd_k[1] = 0.0;
      pose_controller_2_B.Rd_k[4] = pose_controller_2_P.kp_y_Value;
      pose_controller_2_B.Rd_k[7] = 0.0;
      pose_controller_2_B.Rd_k[2] = 0.0;
      pose_controller_2_B.Rd_k[5] = 0.0;
      pose_controller_2_B.Rd_k[8] = pose_controller_2_P.kp_z_Value;
      for (pose_controller_2_B.i = 0; pose_controller_2_B.i < 3;
           pose_controller_2_B.i++) {
        pose_controller_2_B.ex[pose_controller_2_B.i] =
          pose_controller_2_B.Rd_k[pose_controller_2_B.i + 6] *
          pose_controller_2_B.position_error[2] +
          (pose_controller_2_B.Rd_k[pose_controller_2_B.i + 3] *
           pose_controller_2_B.position_error[1] +
           pose_controller_2_B.Rd_k[pose_controller_2_B.i] *
           pose_controller_2_B.position_error[0]);
      }

      pose_controller_2_B.grad1_g = pose_controller_2_B.ex[0];
      pose_controller_2_B.grad2_p = pose_controller_2_B.ex[1];
      pose_controller_2_B.grad3_g = pose_controller_2_B.ex[2];
      pose_controller_2_B.x_error = pose_controller_2_B.position_error[0];
      pose_controller_2_B.y_error = pose_controller_2_B.position_error[1];
      pose_controller_2_B.z_error = pose_controller_2_B.position_error[2];

      // Sum: '<S32>/Add' incorporates:
      //   Constant: '<S32>/center_mass'
      //   Constant: '<S32>/drone1_mass'
      //   Constant: '<S32>/drone2_mass'
      //   Constant: '<S32>/drone3_mass'

      pose_controller_2_B.Add = ((pose_controller_2_P.center_mass_Value +
        pose_controller_2_P.drone1_mass_Value) +
        pose_controller_2_P.drone2_mass_Value) +
        pose_controller_2_P.drone3_mass_Value;
    }

    // Integrator: '<S42>/Integrator'
    if (pose_controller_2_DW.Integrator_IWORK_df != 0) {
      pose_controller_2_X.Integrator_CSTATE_e = pose_controller_2_B.In1_a.X;
    }

    // Integrator: '<S42>/Integrator1'
    if (pose_controller_2_DW.Integrator1_IWORK_g != 0) {
      pose_controller_2_X.Integrator1_CSTATE_p = pose_controller_2_B.In1_a.Y;
    }

    // Integrator: '<S42>/Integrator2'
    if (pose_controller_2_DW.Integrator2_IWORK_m != 0) {
      pose_controller_2_X.Integrator2_CSTATE_c = pose_controller_2_B.In1_a.Z;
    }

    // MATLAB Function: '<S37>/MATLAB Function' incorporates:
    //   Constant: '<S37>/kd_x'
    //   Constant: '<S37>/kd_y'
    //   Constant: '<S37>/kd_z'
    //   Constant: '<S37>/ki_x'
    //   Constant: '<S37>/ki_y'
    //   Constant: '<S37>/ki_z'
    //   Integrator: '<S42>/Integrator'
    //   Integrator: '<S42>/Integrator1'
    //   Integrator: '<S42>/Integrator2'
    //   MATLAB Function: '<S39>/MATLAB Function'
    //   Sum: '<S14>/Sum1'

    pose_controller_2_B.Derivative_d = -pose_controller_2_B.Add * 9.8;
    pose_controller_2_B.position_error[0] = pose_controller_2_B.Add *
      pose_controller_2_B.In1_f.X;
    pose_controller_2_B.position_error[1] = pose_controller_2_B.Add *
      pose_controller_2_B.In1_f.Y;
    pose_controller_2_B.position_error[2] = pose_controller_2_B.Add *
      pose_controller_2_B.In1_f.Z;
    pose_controller_2_B.Sum1_b[0] = pose_controller_2_B.Add *
      pose_controller_2_B.Derivative;
    pose_controller_2_B.Sum1_b[1] = pose_controller_2_B.Add *
      pose_controller_2_B.q_bar;
    pose_controller_2_B.Sum1_b[2] = pose_controller_2_B.Add *
      pose_controller_2_B.Derivative_i;
    pose_controller_2_B.position_error_c[0] =
      pose_controller_2_B.position_error[1] * pose_controller_2_B.epsilon_d_v[2]
      - pose_controller_2_B.position_error[2] * pose_controller_2_B.epsilon_d_v
      [1];
    pose_controller_2_B.position_error_c[1] =
      pose_controller_2_B.position_error[2] * pose_controller_2_B.epsilon_d_v[0]
      - pose_controller_2_B.position_error[0] * pose_controller_2_B.epsilon_d_v
      [2];
    pose_controller_2_B.position_error_c[2] =
      pose_controller_2_B.position_error[0] * pose_controller_2_B.epsilon_d_v[1]
      - pose_controller_2_B.position_error[1] * pose_controller_2_B.epsilon_d_v
      [0];
    pose_controller_2_B.Rd_k[0] = pose_controller_2_P.kd_x_Value;
    pose_controller_2_B.Rd_k[3] = 0.0;
    pose_controller_2_B.Rd_k[6] = 0.0;
    pose_controller_2_B.Rd_k[1] = 0.0;
    pose_controller_2_B.Rd_k[4] = pose_controller_2_P.kd_y_Value;
    pose_controller_2_B.Rd_k[7] = 0.0;
    pose_controller_2_B.Rd_k[2] = 0.0;
    pose_controller_2_B.Rd_k[5] = 0.0;
    pose_controller_2_B.Rd_k[8] = pose_controller_2_P.kd_z_Value;
    pose_controller_2_B.Derivative = pose_controller_2_B.In1_h.X -
      pose_controller_2_B.epsilon_d_v[0];
    pose_controller_2_B.q_bar = pose_controller_2_B.In1_h.Y -
      pose_controller_2_B.epsilon_d_v[1];
    pose_controller_2_B.Derivative_i = pose_controller_2_B.In1_h.Z -
      pose_controller_2_B.epsilon_d_v[2];
    for (pose_controller_2_B.i = 0; pose_controller_2_B.i < 3;
         pose_controller_2_B.i++) {
      pose_controller_2_B.ex[pose_controller_2_B.i] =
        (pose_controller_2_B.Sum1_b[pose_controller_2_B.i] +
         pose_controller_2_B.position_error_c[pose_controller_2_B.i]) -
        ((pose_controller_2_B.Rd_k[pose_controller_2_B.i + 3] *
          pose_controller_2_B.q_bar +
          pose_controller_2_B.Rd_k[pose_controller_2_B.i] *
          pose_controller_2_B.Derivative) +
         pose_controller_2_B.Rd_k[pose_controller_2_B.i + 6] *
         pose_controller_2_B.Derivative_i);
    }

    pose_controller_2_B.Sum1_b[0] = pose_controller_2_B.grad1_g;
    pose_controller_2_B.Sum1_b[1] = pose_controller_2_B.grad2_p;
    pose_controller_2_B.Sum1_b[2] = pose_controller_2_B.grad3_g;
    pose_controller_2_B.Rd_k[0] = pose_controller_2_P.ki_x_Value;
    pose_controller_2_B.Rd_k[3] = 0.0;
    pose_controller_2_B.Rd_k[6] = 0.0;
    pose_controller_2_B.Rd_k[1] = 0.0;
    pose_controller_2_B.Rd_k[4] = pose_controller_2_P.ki_y_Value;
    pose_controller_2_B.Rd_k[7] = 0.0;
    pose_controller_2_B.Rd_k[2] = 0.0;
    pose_controller_2_B.Rd_k[5] = 0.0;
    pose_controller_2_B.Rd_k[8] = pose_controller_2_P.ki_z_Value;
    for (pose_controller_2_B.i = 0; pose_controller_2_B.i < 3;
         pose_controller_2_B.i++) {
      pose_controller_2_B.position_error[pose_controller_2_B.i] =
        ((pose_controller_2_B.ex[pose_controller_2_B.i] -
          pose_controller_2_B.Sum1_b[pose_controller_2_B.i]) -
         ((pose_controller_2_B.Rd_k[pose_controller_2_B.i + 3] *
           pose_controller_2_X.Integrator1_CSTATE_p +
           pose_controller_2_B.Rd_k[pose_controller_2_B.i] *
           pose_controller_2_X.Integrator_CSTATE_e) +
          pose_controller_2_B.Rd_k[pose_controller_2_B.i + 6] *
          pose_controller_2_X.Integrator2_CSTATE_c)) +
        (pose_controller_2_B.S1[pose_controller_2_B.i + 6] *
         pose_controller_2_B.Derivative_d +
         (pose_controller_2_B.S1[pose_controller_2_B.i + 3] *
          pose_controller_2_B.Derivative_d * 0.0 +
          pose_controller_2_B.Derivative_d *
          pose_controller_2_B.S1[pose_controller_2_B.i] * 0.0));
    }

    // MATLAB Function: '<S13>/MATLAB Function' incorporates:
    //   Constant: '<S31>/drone1_x'
    //   Constant: '<S31>/drone1_y'
    //   Constant: '<S31>/drone1_z'
    //   Constant: '<S31>/drone2_x'
    //   Constant: '<S31>/drone2_y'
    //   Constant: '<S31>/drone2_z'
    //   Constant: '<S31>/drone3_x'
    //   Constant: '<S31>/drone3_y'
    //   Constant: '<S31>/drone3_z'
    //   MATLAB Function: '<S37>/MATLAB Function'
    //   MATLAB Function: '<S38>/MATLAB Function'

    pose_controller_2_B.S1[0] = 0.0;
    pose_controller_2_B.S1[3] = -pose_controller_2_P.drone1_z_Value;
    pose_controller_2_B.S1[6] = pose_controller_2_P.drone1_y_Value;
    pose_controller_2_B.S1[1] = pose_controller_2_P.drone1_z_Value;
    pose_controller_2_B.S1[4] = 0.0;
    pose_controller_2_B.S1[7] = -pose_controller_2_P.drone1_x_Value;
    pose_controller_2_B.S1[2] = -pose_controller_2_P.drone1_y_Value;
    pose_controller_2_B.S1[5] = pose_controller_2_P.drone1_x_Value;
    pose_controller_2_B.S1[8] = 0.0;
    pose_controller_2_B.Rd[0] = 0.0;
    pose_controller_2_B.Rd[3] = -pose_controller_2_P.drone2_z_Value;
    pose_controller_2_B.Rd[6] = pose_controller_2_P.drone2_y_Value;
    pose_controller_2_B.Rd[1] = pose_controller_2_P.drone2_z_Value;
    pose_controller_2_B.Rd[4] = 0.0;
    pose_controller_2_B.Rd[7] = -pose_controller_2_P.drone2_x_Value;
    pose_controller_2_B.Rd[2] = -pose_controller_2_P.drone2_y_Value;
    pose_controller_2_B.Rd[5] = pose_controller_2_P.drone2_x_Value;
    pose_controller_2_B.Rd[8] = 0.0;
    pose_controller_2_B.R[0] = 0.0;
    pose_controller_2_B.R[3] = -pose_controller_2_P.drone3_z_Value;
    pose_controller_2_B.R[6] = pose_controller_2_P.drone3_y_Value;
    pose_controller_2_B.R[1] = pose_controller_2_P.drone3_z_Value;
    pose_controller_2_B.R[4] = 0.0;
    pose_controller_2_B.R[7] = -pose_controller_2_P.drone3_x_Value;
    pose_controller_2_B.R[2] = -pose_controller_2_P.drone3_y_Value;
    pose_controller_2_B.R[5] = pose_controller_2_P.drone3_x_Value;
    pose_controller_2_B.R[8] = 0.0;
    for (pose_controller_2_B.i = 0; pose_controller_2_B.i < 9;
         pose_controller_2_B.i++) {
      pose_controller_2_B.B[6 * pose_controller_2_B.i] = c[3 *
        pose_controller_2_B.i];
      pose_controller_2_B.B[6 * pose_controller_2_B.i + 1] = c[3 *
        pose_controller_2_B.i + 1];
      pose_controller_2_B.B[6 * pose_controller_2_B.i + 2] = c[3 *
        pose_controller_2_B.i + 2];
    }

    for (pose_controller_2_B.i = 0; pose_controller_2_B.i < 3;
         pose_controller_2_B.i++) {
      pose_controller_2_B.B[6 * pose_controller_2_B.i + 3] =
        pose_controller_2_B.S1[3 * pose_controller_2_B.i];
      pose_controller_2_B.R_tmp = 6 * (pose_controller_2_B.i + 3);
      pose_controller_2_B.B[pose_controller_2_B.R_tmp + 3] =
        pose_controller_2_B.Rd[3 * pose_controller_2_B.i];
      pose_controller_2_B.B_tmp = 6 * (pose_controller_2_B.i + 6);
      pose_controller_2_B.B[pose_controller_2_B.B_tmp + 3] =
        pose_controller_2_B.R[3 * pose_controller_2_B.i];
      pose_controller_2_B.Rt_tmp = 3 * pose_controller_2_B.i + 1;
      pose_controller_2_B.B[6 * pose_controller_2_B.i + 4] =
        pose_controller_2_B.S1[pose_controller_2_B.Rt_tmp];
      pose_controller_2_B.B[pose_controller_2_B.R_tmp + 4] =
        pose_controller_2_B.Rd[pose_controller_2_B.Rt_tmp];
      pose_controller_2_B.B[pose_controller_2_B.B_tmp + 4] =
        pose_controller_2_B.R[pose_controller_2_B.Rt_tmp];
      pose_controller_2_B.Rt_tmp = 3 * pose_controller_2_B.i + 2;
      pose_controller_2_B.B[6 * pose_controller_2_B.i + 5] =
        pose_controller_2_B.S1[pose_controller_2_B.Rt_tmp];
      pose_controller_2_B.B[pose_controller_2_B.R_tmp + 5] =
        pose_controller_2_B.Rd[pose_controller_2_B.Rt_tmp];
      pose_controller_2_B.B[pose_controller_2_B.B_tmp + 5] =
        pose_controller_2_B.R[pose_controller_2_B.Rt_tmp];
    }

    for (pose_controller_2_B.i = 0; pose_controller_2_B.i < 9;
         pose_controller_2_B.i++) {
      pose_controller_2_B.Rt_c[pose_controller_2_B.i] = d[pose_controller_2_B.i];
    }

    pose_controller_2_B.LAMDA1_tmp[0] = pose_controller_2_B.position_error[0];
    pose_controller_2_B.LAMDA1_tmp[1] = pose_controller_2_B.position_error[1];
    pose_controller_2_B.LAMDA1_tmp[2] = pose_controller_2_B.position_error[2];
    pose_controller_2_B.LAMDA1_tmp[3] = pose_controller_2_B.de[0];
    pose_controller_2_B.LAMDA1_tmp[4] = pose_controller_2_B.de[1];
    pose_controller_2_B.LAMDA1_tmp[5] = pose_controller_2_B.de[2];
    for (pose_controller_2_B.i = 0; pose_controller_2_B.i < 6;
         pose_controller_2_B.i++) {
      for (pose_controller_2_B.B_tmp = 0; pose_controller_2_B.B_tmp < 6;
           pose_controller_2_B.B_tmp++) {
        pose_controller_2_B.R_tmp = pose_controller_2_B.i + 6 *
          pose_controller_2_B.B_tmp;
        pose_controller_2_B.B_m[pose_controller_2_B.R_tmp] = 0.0;
        for (pose_controller_2_B.Rt_tmp = 0; pose_controller_2_B.Rt_tmp < 9;
             pose_controller_2_B.Rt_tmp++) {
          pose_controller_2_B.B_m[pose_controller_2_B.R_tmp] +=
            pose_controller_2_B.B[6 * pose_controller_2_B.Rt_tmp +
            pose_controller_2_B.i] * pose_controller_2_B.B[6 *
            pose_controller_2_B.Rt_tmp + pose_controller_2_B.B_tmp];
        }
      }
    }

    pose_controller_2_mldivide_g(pose_controller_2_B.B_m,
      pose_controller_2_B.LAMDA1_tmp);
    for (pose_controller_2_B.i = 0; pose_controller_2_B.i < 3;
         pose_controller_2_B.i++) {
      pose_controller_2_B.Rt[3 * pose_controller_2_B.i] =
        pose_controller_2_B.Rt_c[3 * pose_controller_2_B.i];
      pose_controller_2_B.Rt_tmp = 3 * (pose_controller_2_B.i + 3);
      pose_controller_2_B.Rt[pose_controller_2_B.Rt_tmp] =
        pose_controller_2_B.S1[pose_controller_2_B.i];
      pose_controller_2_B.B_tmp = 3 * pose_controller_2_B.i + 1;
      pose_controller_2_B.Rt[pose_controller_2_B.B_tmp] =
        pose_controller_2_B.Rt_c[pose_controller_2_B.B_tmp];
      pose_controller_2_B.Rt[pose_controller_2_B.Rt_tmp + 1] =
        pose_controller_2_B.S1[pose_controller_2_B.i + 3];
      pose_controller_2_B.B_tmp = 3 * pose_controller_2_B.i + 2;
      pose_controller_2_B.Rt[pose_controller_2_B.B_tmp] =
        pose_controller_2_B.Rt_c[pose_controller_2_B.B_tmp];
      pose_controller_2_B.Rt[pose_controller_2_B.Rt_tmp + 2] =
        pose_controller_2_B.S1[pose_controller_2_B.i + 6];
    }

    for (pose_controller_2_B.i = 0; pose_controller_2_B.i < 3;
         pose_controller_2_B.i++) {
      pose_controller_2_B.Sum1_b[pose_controller_2_B.i] = 0.0;
      for (pose_controller_2_B.B_tmp = 0; pose_controller_2_B.B_tmp < 6;
           pose_controller_2_B.B_tmp++) {
        pose_controller_2_B.Sum1_b[pose_controller_2_B.i] +=
          pose_controller_2_B.Rt[3 * pose_controller_2_B.B_tmp +
          pose_controller_2_B.i] *
          pose_controller_2_B.LAMDA1_tmp[pose_controller_2_B.B_tmp];
      }
    }

    for (pose_controller_2_B.i = 0; pose_controller_2_B.i < 3;
         pose_controller_2_B.i++) {
      pose_controller_2_B.Rt[3 * pose_controller_2_B.i] =
        pose_controller_2_B.Rt_c[3 * pose_controller_2_B.i];
      pose_controller_2_B.Rt_tmp = 3 * (pose_controller_2_B.i + 3);
      pose_controller_2_B.Rt[pose_controller_2_B.Rt_tmp] =
        pose_controller_2_B.Rd[pose_controller_2_B.i];
      pose_controller_2_B.B_tmp = 3 * pose_controller_2_B.i + 1;
      pose_controller_2_B.Rt[pose_controller_2_B.B_tmp] =
        pose_controller_2_B.Rt_c[pose_controller_2_B.B_tmp];
      pose_controller_2_B.Rt[pose_controller_2_B.Rt_tmp + 1] =
        pose_controller_2_B.Rd[pose_controller_2_B.i + 3];
      pose_controller_2_B.B_tmp = 3 * pose_controller_2_B.i + 2;
      pose_controller_2_B.Rt[pose_controller_2_B.B_tmp] =
        pose_controller_2_B.Rt_c[pose_controller_2_B.B_tmp];
      pose_controller_2_B.Rt[pose_controller_2_B.Rt_tmp + 2] =
        pose_controller_2_B.Rd[pose_controller_2_B.i + 6];
    }

    for (pose_controller_2_B.i = 0; pose_controller_2_B.i < 3;
         pose_controller_2_B.i++) {
      pose_controller_2_B.ex[pose_controller_2_B.i] = 0.0;
      for (pose_controller_2_B.B_tmp = 0; pose_controller_2_B.B_tmp < 6;
           pose_controller_2_B.B_tmp++) {
        pose_controller_2_B.ex[pose_controller_2_B.i] += pose_controller_2_B.Rt
          [3 * pose_controller_2_B.B_tmp + pose_controller_2_B.i] *
          pose_controller_2_B.LAMDA1_tmp[pose_controller_2_B.B_tmp];
      }
    }

    for (pose_controller_2_B.i = 0; pose_controller_2_B.i < 3;
         pose_controller_2_B.i++) {
      pose_controller_2_B.Rt[3 * pose_controller_2_B.i] =
        pose_controller_2_B.Rt_c[3 * pose_controller_2_B.i];
      pose_controller_2_B.Rt_tmp = 3 * (pose_controller_2_B.i + 3);
      pose_controller_2_B.Rt[pose_controller_2_B.Rt_tmp] =
        pose_controller_2_B.R[pose_controller_2_B.i];
      pose_controller_2_B.B_tmp = 3 * pose_controller_2_B.i + 1;
      pose_controller_2_B.Rt[pose_controller_2_B.B_tmp] =
        pose_controller_2_B.Rt_c[pose_controller_2_B.B_tmp];
      pose_controller_2_B.Rt[pose_controller_2_B.Rt_tmp + 1] =
        pose_controller_2_B.R[pose_controller_2_B.i + 3];
      pose_controller_2_B.B_tmp = 3 * pose_controller_2_B.i + 2;
      pose_controller_2_B.Rt[pose_controller_2_B.B_tmp] =
        pose_controller_2_B.Rt_c[pose_controller_2_B.B_tmp];
      pose_controller_2_B.Rt[pose_controller_2_B.Rt_tmp + 2] =
        pose_controller_2_B.R[pose_controller_2_B.i + 6];
    }

    for (pose_controller_2_B.i = 0; pose_controller_2_B.i < 3;
         pose_controller_2_B.i++) {
      pose_controller_2_B.position_error[pose_controller_2_B.i] = 0.0;
      for (pose_controller_2_B.B_tmp = 0; pose_controller_2_B.B_tmp < 6;
           pose_controller_2_B.B_tmp++) {
        pose_controller_2_B.position_error[pose_controller_2_B.i] +=
          pose_controller_2_B.Rt[3 * pose_controller_2_B.B_tmp +
          pose_controller_2_B.i] *
          pose_controller_2_B.LAMDA1_tmp[pose_controller_2_B.B_tmp];
      }
    }

    if (rtmIsMajorTimeStep(pose_controller_2_M)) {
      // Outputs for Atomic SubSystem: '<S4>/Subscribe6'
      // MATLABSystem: '<S28>/SourceBlock' incorporates:
      //   Inport: '<S109>/In1'

      b_varargout_1 = Sub_pose_controller_2_2529.getLatestMessage
        (&pose_controller_2_B.b_varargout_2_o);

      // Outputs for Enabled SubSystem: '<S28>/Enabled Subsystem' incorporates:
      //   EnablePort: '<S109>/Enable'

      if (b_varargout_1) {
        pose_controller_2_B.In1_p = pose_controller_2_B.b_varargout_2_o;
      }

      // End of MATLABSystem: '<S28>/SourceBlock'
      // End of Outputs for SubSystem: '<S28>/Enabled Subsystem'
      // End of Outputs for SubSystem: '<S4>/Subscribe6'

      // Sum: '<S13>/Sum3'
      pose_controller_2_B.Sum3 = pose_controller_2_B.In1.Z +
        pose_controller_2_B.In1_p.Data;
    }

    // MATLAB Function: '<S13>/MATLAB Function1' incorporates:
    //   MATLAB Function: '<S13>/MATLAB Function'

    pose_controller_MATLABFunction1(pose_controller_2_B.In1_l.X,
      pose_controller_2_B.In1_l.Y, pose_controller_2_B.In1_l.Z,
      pose_controller_2_B.Sum1_b[0], pose_controller_2_B.Sum1_b[1],
      pose_controller_2_B.Sum1_b[2], pose_controller_2_B.Sum3,
      &pose_controller_2_B.q_bar, &pose_controller_2_B.Derivative_i,
      &pose_controller_2_B.Derivative, &pose_controller_2_B.sf_MATLABFunction1);

    // BusAssignment: '<S4>/Bus Assignment'
    pose_controller_2_B.BusAssignment.Data = pose_controller_2_B.q_bar;

    // Outputs for Atomic SubSystem: '<S4>/Publish'
    // MATLABSystem: '<S16>/SinkBlock'

    // kaidi wang code this line in 2021.11.17
    // thrust1 topic 
    if(pub.data == false)//mpc controller running
    {
      ROS_INFO_STREAM("gaoling ... ");
      // Pub_pose_controller_2_265.publish(&pose_controller_2_B.BusAssignment);
    }
    else
    {
      ROS_INFO_STREAM("kaidi ... ");
      Pub_pose_controller_2_265.publish(&pose_controller_2_B.BusAssignment);
    }
    // Pub_pose_controller_2_265.publish(&pose_controller_2_B.BusAssignment);

    // End of Outputs for SubSystem: '<S4>/Publish'

    // BusAssignment: '<S4>/Bus Assignment3'
    pose_controller_2_B.BusAssignment3.X = pose_controller_2_B.Derivative_i;
    pose_controller_2_B.BusAssignment3.Y = pose_controller_2_B.Derivative;
    pose_controller_2_B.BusAssignment3.Z = pose_controller_2_B.Sum3;

    // Outputs for Atomic SubSystem: '<S4>/Publish1'
    // MATLABSystem: '<S17>/SinkBlock'

    // kaidi wang code 2021.11.17
    // angle1
    if (pub.data == false)
    {
      /* code */
    }
    else
    {
      Pub_pose_controller_2_266.publish(&pose_controller_2_B.BusAssignment3);
    }
    // Pub_pose_controller_2_266.publish(&pose_controller_2_B.BusAssignment3);

    // End of Outputs for SubSystem: '<S4>/Publish1'
    if (rtmIsMajorTimeStep(pose_controller_2_M)) {
      // Outputs for Atomic SubSystem: '<S4>/Subscribe7'
      // MATLABSystem: '<S29>/SourceBlock' incorporates:
      //   Inport: '<S110>/In1'

      b_varargout_1 = Sub_pose_controller_2_2533.getLatestMessage
        (&pose_controller_2_B.b_varargout_2_o);

      // Outputs for Enabled SubSystem: '<S29>/Enabled Subsystem' incorporates:
      //   EnablePort: '<S110>/Enable'

      if (b_varargout_1) {
        pose_controller_2_B.In1_c = pose_controller_2_B.b_varargout_2_o;
      }

      // End of MATLABSystem: '<S29>/SourceBlock'
      // End of Outputs for SubSystem: '<S29>/Enabled Subsystem'
      // End of Outputs for SubSystem: '<S4>/Subscribe7'

      // Sum: '<S13>/Sum4'
      pose_controller_2_B.Sum4 = pose_controller_2_B.In1.Z +
        pose_controller_2_B.In1_c.Data;
    }

    // MATLAB Function: '<S13>/MATLAB Function2' incorporates:
    //   MATLAB Function: '<S13>/MATLAB Function'

    pose_controller_MATLABFunction1(pose_controller_2_B.In1_l.X,
      pose_controller_2_B.In1_l.Y, pose_controller_2_B.In1_l.Z,
      pose_controller_2_B.ex[0], pose_controller_2_B.ex[1],
      pose_controller_2_B.ex[2], pose_controller_2_B.Sum4,
      &pose_controller_2_B.q_bar, &pose_controller_2_B.Derivative_i,
      &pose_controller_2_B.Derivative, &pose_controller_2_B.sf_MATLABFunction2);

    // BusAssignment: '<S4>/Bus Assignment1'
    pose_controller_2_B.BusAssignment1.Data = pose_controller_2_B.q_bar;

    // Outputs for Atomic SubSystem: '<S4>/Publish2'
    // MATLABSystem: '<S18>/SinkBlock'

    // kaidi wang code this line in 2021.11.17
    // thrust2 topic 
    if(pub.data == false)
    {

    }
    else
    {
      Pub_pose_controller_2_267.publish(&pose_controller_2_B.BusAssignment1);
    }
    // Pub_pose_controller_2_267.publish(&pose_controller_2_B.BusAssignment1);

    // End of Outputs for SubSystem: '<S4>/Publish2'

    // BusAssignment: '<S4>/Bus Assignment4'
    pose_controller_2_B.BusAssignment4.X = pose_controller_2_B.Derivative_i;
    pose_controller_2_B.BusAssignment4.Y = pose_controller_2_B.Derivative;
    pose_controller_2_B.BusAssignment4.Z = pose_controller_2_B.Sum4;

    // Outputs for Atomic SubSystem: '<S4>/Publish3'
    // MATLABSystem: '<S19>/SinkBlock'

    // kaidi wang code 2021.11.17
    // angle2
    if (pub.data == false)
    {
      /* code */
    }
    else{
      Pub_pose_controller_2_268.publish(&pose_controller_2_B.BusAssignment4);
    }
    // Pub_pose_controller_2_268.publish(&pose_controller_2_B.BusAssignment4);

    // End of Outputs for SubSystem: '<S4>/Publish3'
    if (rtmIsMajorTimeStep(pose_controller_2_M)) {
      // Outputs for Atomic SubSystem: '<S4>/Subscribe8'
      // MATLABSystem: '<S30>/SourceBlock' incorporates:
      //   Inport: '<S111>/In1'

      b_varargout_1 = Sub_pose_controller_2_2536.getLatestMessage
        (&pose_controller_2_B.b_varargout_2_o);

      // Outputs for Enabled SubSystem: '<S30>/Enabled Subsystem' incorporates:
      //   EnablePort: '<S111>/Enable'

      if (b_varargout_1) {
        pose_controller_2_B.In1_g = pose_controller_2_B.b_varargout_2_o;
      }

      // End of MATLABSystem: '<S30>/SourceBlock'
      // End of Outputs for SubSystem: '<S30>/Enabled Subsystem'
      // End of Outputs for SubSystem: '<S4>/Subscribe8'

      // Sum: '<S13>/Sum1'
      pose_controller_2_B.Sum1 = pose_controller_2_B.In1.Z +
        pose_controller_2_B.In1_g.Data;
    }

    // MATLAB Function: '<S13>/MATLAB Function3' incorporates:
    //   MATLAB Function: '<S13>/MATLAB Function'

    pose_controller_MATLABFunction1(pose_controller_2_B.In1_l.X,
      pose_controller_2_B.In1_l.Y, pose_controller_2_B.In1_l.Z,
      pose_controller_2_B.position_error[0], pose_controller_2_B.position_error
      [1], pose_controller_2_B.position_error[2], pose_controller_2_B.Sum1,
      &pose_controller_2_B.Derivative, &pose_controller_2_B.Derivative_i,
      &pose_controller_2_B.q_bar, &pose_controller_2_B.sf_MATLABFunction3);

    // BusAssignment: '<S4>/Bus Assignment2'
    pose_controller_2_B.BusAssignment2.Data = pose_controller_2_B.Derivative;

    // Outputs for Atomic SubSystem: '<S4>/Publish4'
    // MATLABSystem: '<S20>/SinkBlock'

    // kaidi wang code 2021.11.17
    // thrust3
    if(pub.data == false)
    {

    }
    else
    {
      Pub_pose_controller_2_269.publish(&pose_controller_2_B.BusAssignment2);
    }
    // Pub_pose_controller_2_269.publish(&pose_controller_2_B.BusAssignment2);

    // End of Outputs for SubSystem: '<S4>/Publish4'

    // BusAssignment: '<S4>/Bus Assignment5'
    pose_controller_2_B.BusAssignment5.X = pose_controller_2_B.Derivative_i;
    pose_controller_2_B.BusAssignment5.Y = pose_controller_2_B.q_bar;
    pose_controller_2_B.BusAssignment5.Z = pose_controller_2_B.Sum1;

    // Outputs for Atomic SubSystem: '<S4>/Publish5'
    // MATLABSystem: '<S21>/SinkBlock'

    // kaidi wang code 2021.11.17
    // angle3
    if (pub.data == false)
    {
      /* code */
    }
    else
    {
      Pub_pose_controller_2_270.publish(&pose_controller_2_B.BusAssignment5);
    }
    // Pub_pose_controller_2_270.publish(&pose_controller_2_B.BusAssignment5);

    // End of Outputs for SubSystem: '<S4>/Publish5'

    // MATLAB Function: '<S54>/MATLAB Function3' incorporates:
    //   Constant: '<S54>/omega_w'

    pose_controll_MATLABFunction1_a(pose_controller_2_B.Integrator_jg,
      pose_controller_2_P.omega_w_Value, &pose_controller_2_B.q_bar);

    // MATLAB Function: '<S54>/MATLAB Function1' incorporates:
    //   Constant: '<S54>/omega_w'
    //   MATLAB Function: '<S39>/MATLAB Function'

    pose_controll_MATLABFunction1_a(pose_controller_2_B.epsilon_d_v[2],
      pose_controller_2_P.omega_w_Value, &pose_controller_2_B.Derivative_i);

    // Integrator: '<S54>/Integrator1'
    pose_controller_2_B.Integrator1 = pose_controller_2_X.Integrator1_CSTATE_k;

    // MATLAB Function: '<S54>/MATLAB Function2' incorporates:
    //   Constant: '<S54>/epsilon_w'
    //   Constant: '<S54>/omega_w'

    pose_controller_MATLABFunction2(pose_controller_2_B.Integrator1,
      pose_controller_2_P.omega_w_Value, pose_controller_2_P.epsilon_w_Value,
      &pose_controller_2_B.Derivative);

    // Sum: '<S54>/Sum'
    pose_controller_2_B.Sum = (pose_controller_2_B.Derivative_i -
      pose_controller_2_B.Derivative) - pose_controller_2_B.q_bar;

    // MATLAB Function: '<S53>/MATLAB Function3' incorporates:
    //   Constant: '<S53>/omega_v'

    pose_controll_MATLABFunction1_a(pose_controller_2_B.Integrator_a3,
      pose_controller_2_P.omega_v_Value, &pose_controller_2_B.q_bar);

    // MATLAB Function: '<S53>/MATLAB Function1' incorporates:
    //   Constant: '<S53>/omega_v'
    //   MATLAB Function: '<S39>/MATLAB Function'

    pose_controll_MATLABFunction1_a(pose_controller_2_B.epsilon_d_v[1],
      pose_controller_2_P.omega_v_Value, &pose_controller_2_B.Derivative_i);

    // Integrator: '<S53>/Integrator1'
    pose_controller_2_B.Integrator1_e = pose_controller_2_X.Integrator1_CSTATE_h;

    // MATLAB Function: '<S53>/MATLAB Function2' incorporates:
    //   Constant: '<S53>/epsilon_v'
    //   Constant: '<S53>/omega_v'

    pose_controller_MATLABFunction2(pose_controller_2_B.Integrator1_e,
      pose_controller_2_P.omega_v_Value, pose_controller_2_P.epsilon_v_Value,
      &pose_controller_2_B.Derivative);

    // Sum: '<S53>/Sum'
    pose_controller_2_B.Sum_n = (pose_controller_2_B.Derivative_i -
      pose_controller_2_B.Derivative) - pose_controller_2_B.q_bar;

    // MATLAB Function: '<S52>/MATLAB Function3' incorporates:
    //   Constant: '<S52>/omega_u'

    pose_controll_MATLABFunction1_a(pose_controller_2_B.Integrator_jx,
      pose_controller_2_P.omega_u_Value, &pose_controller_2_B.q_bar);

    // MATLAB Function: '<S52>/MATLAB Function1' incorporates:
    //   Constant: '<S52>/omega_u'
    //   MATLAB Function: '<S39>/MATLAB Function'

    pose_controll_MATLABFunction1_a(pose_controller_2_B.epsilon_d_v[0],
      pose_controller_2_P.omega_u_Value, &pose_controller_2_B.Derivative_i);

    // Integrator: '<S52>/Integrator1'
    pose_controller_2_B.Integrator1_h = pose_controller_2_X.Integrator1_CSTATE_g;

    // MATLAB Function: '<S52>/MATLAB Function2' incorporates:
    //   Constant: '<S52>/epsilon_u'
    //   Constant: '<S52>/omega_u'

    pose_controller_MATLABFunction2(pose_controller_2_B.Integrator1_h,
      pose_controller_2_P.omega_u_Value, pose_controller_2_P.epsilon_u_Value,
      &pose_controller_2_B.Derivative);

    // Sum: '<S52>/Sum'
    pose_controller_2_B.Sum_l = (pose_controller_2_B.Derivative_i -
      pose_controller_2_B.Derivative) - pose_controller_2_B.q_bar;

    // MATLAB Function: '<S51>/MATLAB Function3' incorporates:
    //   Constant: '<S51>/omega_r'

    pose_controll_MATLABFunction1_a(pose_controller_2_B.Integrator_h,
      pose_controller_2_P.omega_r_Value, &pose_controller_2_B.q_bar);

    // MATLAB Function: '<S51>/MATLAB Function1' incorporates:
    //   Constant: '<S51>/omega_r'
    //   MATLAB Function: '<S39>/MATLAB Function1'

    pose_controll_MATLABFunction1_a(pose_controller_2_B.vd[2],
      pose_controller_2_P.omega_r_Value, &pose_controller_2_B.Derivative_i);

    // Integrator: '<S51>/Integrator1'
    pose_controller_2_B.Integrator1_l = pose_controller_2_X.Integrator1_CSTATE_b;

    // MATLAB Function: '<S51>/MATLAB Function2' incorporates:
    //   Constant: '<S51>/epsilon_r'
    //   Constant: '<S51>/omega_r'

    pose_controller_MATLABFunction2(pose_controller_2_B.Integrator1_l,
      pose_controller_2_P.omega_r_Value, pose_controller_2_P.epsilon_r_Value,
      &pose_controller_2_B.Derivative);

    // Sum: '<S51>/Sum'
    pose_controller_2_B.Sum_j = (pose_controller_2_B.Derivative_i -
      pose_controller_2_B.Derivative) - pose_controller_2_B.q_bar;

    // MATLAB Function: '<S50>/MATLAB Function3' incorporates:
    //   Constant: '<S50>/omega_q'

    pose_controll_MATLABFunction1_a(pose_controller_2_B.Integrator_j,
      pose_controller_2_P.omega_q_Value, &pose_controller_2_B.q_bar);

    // MATLAB Function: '<S50>/MATLAB Function1' incorporates:
    //   Constant: '<S50>/omega_q'
    //   MATLAB Function: '<S39>/MATLAB Function1'

    pose_controll_MATLABFunction1_a(pose_controller_2_B.vd[1],
      pose_controller_2_P.omega_q_Value, &pose_controller_2_B.Derivative_i);

    // Integrator: '<S50>/Integrator1'
    pose_controller_2_B.Integrator1_ln =
      pose_controller_2_X.Integrator1_CSTATE_d;

    // MATLAB Function: '<S50>/MATLAB Function2' incorporates:
    //   Constant: '<S50>/epsilon_q'
    //   Constant: '<S50>/omega_q'

    pose_controller_MATLABFunction2(pose_controller_2_B.Integrator1_ln,
      pose_controller_2_P.omega_q_Value, pose_controller_2_P.epsilon_q_Value,
      &pose_controller_2_B.Derivative);

    // Sum: '<S50>/Sum'
    pose_controller_2_B.Sum_k = (pose_controller_2_B.Derivative_i -
      pose_controller_2_B.Derivative) - pose_controller_2_B.q_bar;

    // MATLAB Function: '<S49>/MATLAB Function3' incorporates:
    //   Constant: '<S49>/omega_p'

    pose_controll_MATLABFunction1_a(pose_controller_2_B.Integrator_aj,
      pose_controller_2_P.omega_p_Value, &pose_controller_2_B.q_bar);

    // MATLAB Function: '<S49>/MATLAB Function1' incorporates:
    //   Constant: '<S49>/omega_p'
    //   MATLAB Function: '<S39>/MATLAB Function1'

    pose_controll_MATLABFunction1_a(pose_controller_2_B.vd[0],
      pose_controller_2_P.omega_p_Value, &pose_controller_2_B.Derivative_i);

    // Integrator: '<S49>/Integrator1'
    pose_controller_2_B.Integrator1_n = pose_controller_2_X.Integrator1_CSTATE_n;

    // MATLAB Function: '<S49>/MATLAB Function2' incorporates:
    //   Constant: '<S49>/epsilon_p'
    //   Constant: '<S49>/omega_p'

    pose_controller_MATLABFunction2(pose_controller_2_B.Integrator1_n,
      pose_controller_2_P.omega_p_Value, pose_controller_2_P.epsilon_p_Value,
      &pose_controller_2_B.Derivative);

    // Sum: '<S49>/Sum'
    pose_controller_2_B.Sum_ng = (pose_controller_2_B.Derivative_i -
      pose_controller_2_B.Derivative) - pose_controller_2_B.q_bar;

    // MATLAB Function: '<S71>/MATLAB Function3' incorporates:
    //   Constant: '<S71>/omega_z'

    pose_controll_MATLABFunction1_a(pose_controller_2_B.Integrator_a,
      pose_controller_2_P.omega_z_Value, &pose_controller_2_B.q_bar);
    if (rtmIsMajorTimeStep(pose_controller_2_M)) {
      // MATLAB Function: '<S71>/MATLAB Function1' incorporates:
      //   Constant: '<S71>/omega_z'

      pose_controll_MATLABFunction1_a(pose_controller_2_B.In1_k.Z,
        pose_controller_2_P.omega_z_Value, &pose_controller_2_B.y);
    }

    // Integrator: '<S71>/Integrator1'
    pose_controller_2_B.Integrator1_k =
      pose_controller_2_X.Integrator1_CSTATE_pj;

    // MATLAB Function: '<S71>/MATLAB Function2' incorporates:
    //   Constant: '<S71>/epsilon_z'
    //   Constant: '<S71>/omega_z'

    pose_controller_MATLABFunction2(pose_controller_2_B.Integrator1_k,
      pose_controller_2_P.omega_z_Value, pose_controller_2_P.epsilon_z_Value,
      &pose_controller_2_B.Derivative);

    // Sum: '<S71>/Sum'
    pose_controller_2_B.Sum_o = (pose_controller_2_B.y -
      pose_controller_2_B.Derivative) - pose_controller_2_B.q_bar;

    // MATLAB Function: '<S70>/MATLAB Function3' incorporates:
    //   Constant: '<S70>/omega_y'

    pose_controll_MATLABFunction1_a(pose_controller_2_B.Integrator_n,
      pose_controller_2_P.omega_y_Value, &pose_controller_2_B.q_bar);
    if (rtmIsMajorTimeStep(pose_controller_2_M)) {
      // MATLAB Function: '<S70>/MATLAB Function1' incorporates:
      //   Constant: '<S70>/omega_y'

      pose_controll_MATLABFunction1_a(pose_controller_2_B.In1_k.Y,
        pose_controller_2_P.omega_y_Value, &pose_controller_2_B.y_a);
    }

    // Integrator: '<S70>/Integrator1'
    pose_controller_2_B.Integrator1_d =
      pose_controller_2_X.Integrator1_CSTATE_po;

    // MATLAB Function: '<S70>/MATLAB Function2' incorporates:
    //   Constant: '<S70>/epsilon_y'
    //   Constant: '<S70>/omega_y'

    pose_controller_MATLABFunction2(pose_controller_2_B.Integrator1_d,
      pose_controller_2_P.omega_y_Value, pose_controller_2_P.epsilon_y_Value,
      &pose_controller_2_B.Derivative);

    // Sum: '<S70>/Sum'
    pose_controller_2_B.Sum_ns = (pose_controller_2_B.y_a -
      pose_controller_2_B.Derivative) - pose_controller_2_B.q_bar;

    // MATLAB Function: '<S69>/MATLAB Function3' incorporates:
    //   Constant: '<S69>/omega_x'

    pose_controll_MATLABFunction1_a(pose_controller_2_B.Integrator_k,
      pose_controller_2_P.omega_x_Value, &pose_controller_2_B.q_bar);
    if (rtmIsMajorTimeStep(pose_controller_2_M)) {
      // MATLAB Function: '<S69>/MATLAB Function1' incorporates:
      //   Constant: '<S69>/omega_x'

      pose_controll_MATLABFunction1_a(pose_controller_2_B.In1_k.X,
        pose_controller_2_P.omega_x_Value, &pose_controller_2_B.y_f);
    }

    // Integrator: '<S69>/Integrator1'
    pose_controller_2_B.Integrator1_i =
      pose_controller_2_X.Integrator1_CSTATE_b2;

    // MATLAB Function: '<S69>/MATLAB Function2' incorporates:
    //   Constant: '<S69>/epsilon_x'
    //   Constant: '<S69>/omega_x'

    pose_controller_MATLABFunction2(pose_controller_2_B.Integrator1_i,
      pose_controller_2_P.omega_x_Value, pose_controller_2_P.epsilon_x_Value,
      &pose_controller_2_B.Derivative);

    // Sum: '<S69>/Sum'
    pose_controller_2_B.Sum_e = (pose_controller_2_B.y_f -
      pose_controller_2_B.Derivative) - pose_controller_2_B.q_bar;

    // MATLAB Function: '<S57>/MATLAB Function3' incorporates:
    //   Constant: '<S57>/omega_psi'

    pose_controll_MATLABFunction1_a(pose_controller_2_B.Integrator_i,
      pose_controller_2_P.omega_psi_Value, &pose_controller_2_B.q_bar);
    if (rtmIsMajorTimeStep(pose_controller_2_M)) {
      // MATLAB Function: '<S57>/MATLAB Function1' incorporates:
      //   Constant: '<S57>/omega_psi'

      pose_controll_MATLABFunction1_a(pose_controller_2_B.In1.Z,
        pose_controller_2_P.omega_psi_Value, &pose_controller_2_B.y_j);
    }

    // Integrator: '<S57>/Integrator1'
    pose_controller_2_B.Integrator1_g = pose_controller_2_X.Integrator1_CSTATE_j;

    // MATLAB Function: '<S57>/MATLAB Function2' incorporates:
    //   Constant: '<S57>/epsilon_psi'
    //   Constant: '<S57>/omega_psi'

    pose_controller_MATLABFunction2(pose_controller_2_B.Integrator1_g,
      pose_controller_2_P.omega_psi_Value, pose_controller_2_P.epsilon_psi_Value,
      &pose_controller_2_B.Derivative);

    // Sum: '<S57>/Sum'
    pose_controller_2_B.Sum_p = (pose_controller_2_B.y_j -
      pose_controller_2_B.Derivative) - pose_controller_2_B.q_bar;

    // MATLAB Function: '<S58>/MATLAB Function3' incorporates:
    //   Constant: '<S58>/omega_theta'

    pose_controll_MATLABFunction1_a(pose_controller_2_B.Integrator_e,
      pose_controller_2_P.omega_theta_Value, &pose_controller_2_B.q_bar);
    if (rtmIsMajorTimeStep(pose_controller_2_M)) {
      // MATLAB Function: '<S58>/MATLAB Function1' incorporates:
      //   Constant: '<S58>/omega_theta'

      pose_controll_MATLABFunction1_a(pose_controller_2_B.In1.Y,
        pose_controller_2_P.omega_theta_Value, &pose_controller_2_B.y_n);
    }

    // Integrator: '<S58>/Integrator1'
    pose_controller_2_B.Integrator1_a =
      pose_controller_2_X.Integrator1_CSTATE_n2;

    // MATLAB Function: '<S58>/MATLAB Function2' incorporates:
    //   Constant: '<S58>/epsilon_theta'
    //   Constant: '<S58>/omega_theta'

    pose_controller_MATLABFunction2(pose_controller_2_B.Integrator1_a,
      pose_controller_2_P.omega_theta_Value,
      pose_controller_2_P.epsilon_theta_Value, &pose_controller_2_B.Derivative);

    // Sum: '<S58>/Sum'
    pose_controller_2_B.Sum_c = (pose_controller_2_B.y_n -
      pose_controller_2_B.Derivative) - pose_controller_2_B.q_bar;

    // MATLAB Function: '<S56>/MATLAB Function3' incorporates:
    //   Constant: '<S56>/omega_phi'

    pose_controll_MATLABFunction1_a(pose_controller_2_B.Integrator,
      pose_controller_2_P.omega_phi_Value, &pose_controller_2_B.q_bar);
    if (rtmIsMajorTimeStep(pose_controller_2_M)) {
      // MATLAB Function: '<S56>/MATLAB Function1' incorporates:
      //   Constant: '<S56>/omega_phi'

      pose_controll_MATLABFunction1_a(pose_controller_2_B.In1.X,
        pose_controller_2_P.omega_phi_Value, &pose_controller_2_B.y_l);
    }

    // Integrator: '<S56>/Integrator1'
    pose_controller_2_B.Integrator1_f = pose_controller_2_X.Integrator1_CSTATE_i;

    // MATLAB Function: '<S56>/MATLAB Function2' incorporates:
    //   Constant: '<S56>/epsilon_phi'
    //   Constant: '<S56>/omega_phi'

    pose_controller_MATLABFunction2(pose_controller_2_B.Integrator1_f,
      pose_controller_2_P.omega_phi_Value, pose_controller_2_P.epsilon_phi_Value,
      &pose_controller_2_B.Derivative);

    // Sum: '<S56>/Sum'
    pose_controller_2_B.Sum_g = (pose_controller_2_B.y_l -
      pose_controller_2_B.Derivative) - pose_controller_2_B.q_bar;
  }

  // End of Outputs for SubSystem: '<Root>/Subsystem'

  // Update for Enabled SubSystem: '<Root>/Subsystem' incorporates:
  //   EnablePort: '<S4>/Enable'

  if (pose_controller_2_DW.Subsystem_MODE) {
    // Update for Integrator: '<S56>/Integrator'
    pose_controller_2_DW.Integrator_IWORK = 0;

    // Update for Derivative: '<S56>/Derivative'
    if (pose_controller_2_DW.TimeStampA == (rtInf)) {
      pose_controller_2_DW.TimeStampA = pose_controller_2_M->Timing.t[0];
      lastU = &pose_controller_2_DW.LastUAtTimeA;
    } else if (pose_controller_2_DW.TimeStampB == (rtInf)) {
      pose_controller_2_DW.TimeStampB = pose_controller_2_M->Timing.t[0];
      lastU = &pose_controller_2_DW.LastUAtTimeB;
    } else if (pose_controller_2_DW.TimeStampA < pose_controller_2_DW.TimeStampB)
    {
      pose_controller_2_DW.TimeStampA = pose_controller_2_M->Timing.t[0];
      lastU = &pose_controller_2_DW.LastUAtTimeA;
    } else {
      pose_controller_2_DW.TimeStampB = pose_controller_2_M->Timing.t[0];
      lastU = &pose_controller_2_DW.LastUAtTimeB;
    }

    *lastU = pose_controller_2_B.Integrator;

    // End of Update for Derivative: '<S56>/Derivative'

    // Update for Integrator: '<S58>/Integrator'
    pose_controller_2_DW.Integrator_IWORK_e = 0;

    // Update for Derivative: '<S58>/Derivative'
    if (pose_controller_2_DW.TimeStampA_n == (rtInf)) {
      pose_controller_2_DW.TimeStampA_n = pose_controller_2_M->Timing.t[0];
      lastU = &pose_controller_2_DW.LastUAtTimeA_l;
    } else if (pose_controller_2_DW.TimeStampB_g == (rtInf)) {
      pose_controller_2_DW.TimeStampB_g = pose_controller_2_M->Timing.t[0];
      lastU = &pose_controller_2_DW.LastUAtTimeB_h;
    } else if (pose_controller_2_DW.TimeStampA_n <
               pose_controller_2_DW.TimeStampB_g) {
      pose_controller_2_DW.TimeStampA_n = pose_controller_2_M->Timing.t[0];
      lastU = &pose_controller_2_DW.LastUAtTimeA_l;
    } else {
      pose_controller_2_DW.TimeStampB_g = pose_controller_2_M->Timing.t[0];
      lastU = &pose_controller_2_DW.LastUAtTimeB_h;
    }

    *lastU = pose_controller_2_B.Integrator_e;

    // End of Update for Derivative: '<S58>/Derivative'

    // Update for Integrator: '<S57>/Integrator'
    pose_controller_2_DW.Integrator_IWORK_l = 0;

    // Update for Derivative: '<S57>/Derivative'
    if (pose_controller_2_DW.TimeStampA_nq == (rtInf)) {
      pose_controller_2_DW.TimeStampA_nq = pose_controller_2_M->Timing.t[0];
      lastU = &pose_controller_2_DW.LastUAtTimeA_e;
    } else if (pose_controller_2_DW.TimeStampB_e == (rtInf)) {
      pose_controller_2_DW.TimeStampB_e = pose_controller_2_M->Timing.t[0];
      lastU = &pose_controller_2_DW.LastUAtTimeB_n;
    } else if (pose_controller_2_DW.TimeStampA_nq <
               pose_controller_2_DW.TimeStampB_e) {
      pose_controller_2_DW.TimeStampA_nq = pose_controller_2_M->Timing.t[0];
      lastU = &pose_controller_2_DW.LastUAtTimeA_e;
    } else {
      pose_controller_2_DW.TimeStampB_e = pose_controller_2_M->Timing.t[0];
      lastU = &pose_controller_2_DW.LastUAtTimeB_n;
    }

    *lastU = pose_controller_2_B.Integrator_i;

    // End of Update for Derivative: '<S57>/Derivative'

    // Update for Integrator: '<S69>/Integrator'
    pose_controller_2_DW.Integrator_IWORK_g = 0;

    // Update for Derivative: '<S69>/Derivative'
    if (pose_controller_2_DW.TimeStampA_f == (rtInf)) {
      pose_controller_2_DW.TimeStampA_f = pose_controller_2_M->Timing.t[0];
      lastU = &pose_controller_2_DW.LastUAtTimeA_d;
    } else if (pose_controller_2_DW.TimeStampB_f == (rtInf)) {
      pose_controller_2_DW.TimeStampB_f = pose_controller_2_M->Timing.t[0];
      lastU = &pose_controller_2_DW.LastUAtTimeB_f;
    } else if (pose_controller_2_DW.TimeStampA_f <
               pose_controller_2_DW.TimeStampB_f) {
      pose_controller_2_DW.TimeStampA_f = pose_controller_2_M->Timing.t[0];
      lastU = &pose_controller_2_DW.LastUAtTimeA_d;
    } else {
      pose_controller_2_DW.TimeStampB_f = pose_controller_2_M->Timing.t[0];
      lastU = &pose_controller_2_DW.LastUAtTimeB_f;
    }

    *lastU = pose_controller_2_B.Integrator_k;

    // End of Update for Derivative: '<S69>/Derivative'

    // Update for Integrator: '<S70>/Integrator'
    pose_controller_2_DW.Integrator_IWORK_k = 0;

    // Update for Derivative: '<S70>/Derivative'
    if (pose_controller_2_DW.TimeStampA_j == (rtInf)) {
      pose_controller_2_DW.TimeStampA_j = pose_controller_2_M->Timing.t[0];
      lastU = &pose_controller_2_DW.LastUAtTimeA_n;
    } else if (pose_controller_2_DW.TimeStampB_f4 == (rtInf)) {
      pose_controller_2_DW.TimeStampB_f4 = pose_controller_2_M->Timing.t[0];
      lastU = &pose_controller_2_DW.LastUAtTimeB_l;
    } else if (pose_controller_2_DW.TimeStampA_j <
               pose_controller_2_DW.TimeStampB_f4) {
      pose_controller_2_DW.TimeStampA_j = pose_controller_2_M->Timing.t[0];
      lastU = &pose_controller_2_DW.LastUAtTimeA_n;
    } else {
      pose_controller_2_DW.TimeStampB_f4 = pose_controller_2_M->Timing.t[0];
      lastU = &pose_controller_2_DW.LastUAtTimeB_l;
    }

    *lastU = pose_controller_2_B.Integrator_n;

    // End of Update for Derivative: '<S70>/Derivative'

    // Update for Integrator: '<S71>/Integrator'
    pose_controller_2_DW.Integrator_IWORK_d = 0;

    // Update for Derivative: '<S71>/Derivative'
    if (pose_controller_2_DW.TimeStampA_m == (rtInf)) {
      pose_controller_2_DW.TimeStampA_m = pose_controller_2_M->Timing.t[0];
      lastU = &pose_controller_2_DW.LastUAtTimeA_j;
    } else if (pose_controller_2_DW.TimeStampB_l == (rtInf)) {
      pose_controller_2_DW.TimeStampB_l = pose_controller_2_M->Timing.t[0];
      lastU = &pose_controller_2_DW.LastUAtTimeB_b;
    } else if (pose_controller_2_DW.TimeStampA_m <
               pose_controller_2_DW.TimeStampB_l) {
      pose_controller_2_DW.TimeStampA_m = pose_controller_2_M->Timing.t[0];
      lastU = &pose_controller_2_DW.LastUAtTimeA_j;
    } else {
      pose_controller_2_DW.TimeStampB_l = pose_controller_2_M->Timing.t[0];
      lastU = &pose_controller_2_DW.LastUAtTimeB_b;
    }

    *lastU = pose_controller_2_B.Integrator_a;

    // End of Update for Derivative: '<S71>/Derivative'

    // Update for Derivative: '<S49>/Derivative'
    if (pose_controller_2_DW.TimeStampA_fq == (rtInf)) {
      pose_controller_2_DW.TimeStampA_fq = pose_controller_2_M->Timing.t[0];
      lastU = &pose_controller_2_DW.LastUAtTimeA_et;
    } else if (pose_controller_2_DW.TimeStampB_m == (rtInf)) {
      pose_controller_2_DW.TimeStampB_m = pose_controller_2_M->Timing.t[0];
      lastU = &pose_controller_2_DW.LastUAtTimeB_ba;
    } else if (pose_controller_2_DW.TimeStampA_fq <
               pose_controller_2_DW.TimeStampB_m) {
      pose_controller_2_DW.TimeStampA_fq = pose_controller_2_M->Timing.t[0];
      lastU = &pose_controller_2_DW.LastUAtTimeA_et;
    } else {
      pose_controller_2_DW.TimeStampB_m = pose_controller_2_M->Timing.t[0];
      lastU = &pose_controller_2_DW.LastUAtTimeB_ba;
    }

    *lastU = pose_controller_2_B.Integrator_aj;

    // End of Update for Derivative: '<S49>/Derivative'

    // Update for Derivative: '<S50>/Derivative'
    if (pose_controller_2_DW.TimeStampA_a == (rtInf)) {
      pose_controller_2_DW.TimeStampA_a = pose_controller_2_M->Timing.t[0];
      lastU = &pose_controller_2_DW.LastUAtTimeA_p;
    } else if (pose_controller_2_DW.TimeStampB_gd == (rtInf)) {
      pose_controller_2_DW.TimeStampB_gd = pose_controller_2_M->Timing.t[0];
      lastU = &pose_controller_2_DW.LastUAtTimeB_d;
    } else if (pose_controller_2_DW.TimeStampA_a <
               pose_controller_2_DW.TimeStampB_gd) {
      pose_controller_2_DW.TimeStampA_a = pose_controller_2_M->Timing.t[0];
      lastU = &pose_controller_2_DW.LastUAtTimeA_p;
    } else {
      pose_controller_2_DW.TimeStampB_gd = pose_controller_2_M->Timing.t[0];
      lastU = &pose_controller_2_DW.LastUAtTimeB_d;
    }

    *lastU = pose_controller_2_B.Integrator_j;

    // End of Update for Derivative: '<S50>/Derivative'

    // Update for Derivative: '<S51>/Derivative'
    if (pose_controller_2_DW.TimeStampA_l == (rtInf)) {
      pose_controller_2_DW.TimeStampA_l = pose_controller_2_M->Timing.t[0];
      lastU = &pose_controller_2_DW.LastUAtTimeA_g;
    } else if (pose_controller_2_DW.TimeStampB_p == (rtInf)) {
      pose_controller_2_DW.TimeStampB_p = pose_controller_2_M->Timing.t[0];
      lastU = &pose_controller_2_DW.LastUAtTimeB_lt;
    } else if (pose_controller_2_DW.TimeStampA_l <
               pose_controller_2_DW.TimeStampB_p) {
      pose_controller_2_DW.TimeStampA_l = pose_controller_2_M->Timing.t[0];
      lastU = &pose_controller_2_DW.LastUAtTimeA_g;
    } else {
      pose_controller_2_DW.TimeStampB_p = pose_controller_2_M->Timing.t[0];
      lastU = &pose_controller_2_DW.LastUAtTimeB_lt;
    }

    *lastU = pose_controller_2_B.Integrator_h;

    // End of Update for Derivative: '<S51>/Derivative'

    // Update for Integrator: '<S44>/Integrator'
    pose_controller_2_DW.Integrator_IWORK_lt = 0;

    // Update for Integrator: '<S44>/Integrator1'
    pose_controller_2_DW.Integrator1_IWORK = 0;

    // Update for Integrator: '<S44>/Integrator2'
    pose_controller_2_DW.Integrator2_IWORK = 0;

    // Update for Derivative: '<S52>/Derivative'
    if (pose_controller_2_DW.TimeStampA_h == (rtInf)) {
      pose_controller_2_DW.TimeStampA_h = pose_controller_2_M->Timing.t[0];
      lastU = &pose_controller_2_DW.LastUAtTimeA_b;
    } else if (pose_controller_2_DW.TimeStampB_eu == (rtInf)) {
      pose_controller_2_DW.TimeStampB_eu = pose_controller_2_M->Timing.t[0];
      lastU = &pose_controller_2_DW.LastUAtTimeB_hq;
    } else if (pose_controller_2_DW.TimeStampA_h <
               pose_controller_2_DW.TimeStampB_eu) {
      pose_controller_2_DW.TimeStampA_h = pose_controller_2_M->Timing.t[0];
      lastU = &pose_controller_2_DW.LastUAtTimeA_b;
    } else {
      pose_controller_2_DW.TimeStampB_eu = pose_controller_2_M->Timing.t[0];
      lastU = &pose_controller_2_DW.LastUAtTimeB_hq;
    }

    *lastU = pose_controller_2_B.Integrator_jx;

    // End of Update for Derivative: '<S52>/Derivative'

    // Update for Derivative: '<S53>/Derivative'
    if (pose_controller_2_DW.TimeStampA_hg == (rtInf)) {
      pose_controller_2_DW.TimeStampA_hg = pose_controller_2_M->Timing.t[0];
      lastU = &pose_controller_2_DW.LastUAtTimeA_o;
    } else if (pose_controller_2_DW.TimeStampB_h == (rtInf)) {
      pose_controller_2_DW.TimeStampB_h = pose_controller_2_M->Timing.t[0];
      lastU = &pose_controller_2_DW.LastUAtTimeB_g;
    } else if (pose_controller_2_DW.TimeStampA_hg <
               pose_controller_2_DW.TimeStampB_h) {
      pose_controller_2_DW.TimeStampA_hg = pose_controller_2_M->Timing.t[0];
      lastU = &pose_controller_2_DW.LastUAtTimeA_o;
    } else {
      pose_controller_2_DW.TimeStampB_h = pose_controller_2_M->Timing.t[0];
      lastU = &pose_controller_2_DW.LastUAtTimeB_g;
    }

    *lastU = pose_controller_2_B.Integrator_a3;

    // End of Update for Derivative: '<S53>/Derivative'

    // Update for Derivative: '<S54>/Derivative'
    if (pose_controller_2_DW.TimeStampA_p == (rtInf)) {
      pose_controller_2_DW.TimeStampA_p = pose_controller_2_M->Timing.t[0];
      lastU = &pose_controller_2_DW.LastUAtTimeA_m;
    } else if (pose_controller_2_DW.TimeStampB_gm == (rtInf)) {
      pose_controller_2_DW.TimeStampB_gm = pose_controller_2_M->Timing.t[0];
      lastU = &pose_controller_2_DW.LastUAtTimeB_fn;
    } else if (pose_controller_2_DW.TimeStampA_p <
               pose_controller_2_DW.TimeStampB_gm) {
      pose_controller_2_DW.TimeStampA_p = pose_controller_2_M->Timing.t[0];
      lastU = &pose_controller_2_DW.LastUAtTimeA_m;
    } else {
      pose_controller_2_DW.TimeStampB_gm = pose_controller_2_M->Timing.t[0];
      lastU = &pose_controller_2_DW.LastUAtTimeB_fn;
    }

    *lastU = pose_controller_2_B.Integrator_jg;

    // End of Update for Derivative: '<S54>/Derivative'

    // Update for Integrator: '<S42>/Integrator'
    pose_controller_2_DW.Integrator_IWORK_df = 0;

    // Update for Integrator: '<S42>/Integrator1'
    pose_controller_2_DW.Integrator1_IWORK_g = 0;

    // Update for Integrator: '<S42>/Integrator2'
    pose_controller_2_DW.Integrator2_IWORK_m = 0;
  }

  // End of Update for SubSystem: '<Root>/Subsystem'
  if (rtmIsMajorTimeStep(pose_controller_2_M)) {
    rt_ertODEUpdateContinuousStates(&pose_controller_2_M->solverInfo);

    // Update absolute time for base rate
    // The "clockTick0" counts the number of times the code of this task has
    //  been executed. The absolute time is the multiplication of "clockTick0"
    //  and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
    //  overflow during the application lifespan selected.

    ++pose_controller_2_M->Timing.clockTick0;
    pose_controller_2_M->Timing.t[0] = rtsiGetSolverStopTime
      (&pose_controller_2_M->solverInfo);

    {
      // Update absolute timer for sample time: [0.0125s, 0.0s]
      // The "clockTick1" counts the number of times the code of this task has
      //  been executed. The resolution of this integer timer is 0.0125, which is the step size
      //  of the task. Size of "clockTick1" ensures timer will not overflow during the
      //  application lifespan selected.

      pose_controller_2_M->Timing.clockTick1++;
    }
  }                                    // end MajorTimeStep
}

// Derivatives for root system: '<Root>'
void pose_controller_2_derivatives(void)
{
  XDot_pose_controller_2_T *_rtXdot;
  _rtXdot = ((XDot_pose_controller_2_T *) pose_controller_2_M->derivs);

  // Derivatives for Enabled SubSystem: '<Root>/Subsystem'
  if (pose_controller_2_DW.Subsystem_MODE) {
    // Derivatives for Integrator: '<S56>/Integrator'
    _rtXdot->Integrator_CSTATE = pose_controller_2_B.Integrator1_f;

    // Derivatives for Integrator: '<S58>/Integrator'
    _rtXdot->Integrator_CSTATE_k = pose_controller_2_B.Integrator1_a;

    // Derivatives for Integrator: '<S57>/Integrator'
    _rtXdot->Integrator_CSTATE_j = pose_controller_2_B.Integrator1_g;

    // Derivatives for Integrator: '<S69>/Integrator'
    _rtXdot->Integrator_CSTATE_k3 = pose_controller_2_B.Integrator1_i;

    // Derivatives for Integrator: '<S70>/Integrator'
    _rtXdot->Integrator_CSTATE_c = pose_controller_2_B.Integrator1_d;

    // Derivatives for Integrator: '<S71>/Integrator'
    _rtXdot->Integrator_CSTATE_i = pose_controller_2_B.Integrator1_k;

    // Derivatives for Integrator: '<S49>/Integrator'
    _rtXdot->Integrator_CSTATE_cv = pose_controller_2_B.Integrator1_n;

    // Derivatives for Integrator: '<S50>/Integrator'
    _rtXdot->Integrator_CSTATE_b = pose_controller_2_B.Integrator1_ln;

    // Derivatives for Integrator: '<S51>/Integrator'
    _rtXdot->Integrator_CSTATE_o = pose_controller_2_B.Integrator1_l;

    // Derivatives for Integrator: '<S44>/Integrator'
    _rtXdot->Integrator_CSTATE_a = pose_controller_2_B.phi_error;

    // Derivatives for Integrator: '<S44>/Integrator1'
    _rtXdot->Integrator1_CSTATE = pose_controller_2_B.theta_error;

    // Derivatives for Integrator: '<S44>/Integrator2'
    _rtXdot->Integrator2_CSTATE = pose_controller_2_B.psi_error;

    // Derivatives for Integrator: '<S52>/Integrator'
    _rtXdot->Integrator_CSTATE_jf = pose_controller_2_B.Integrator1_h;

    // Derivatives for Integrator: '<S53>/Integrator'
    _rtXdot->Integrator_CSTATE_p = pose_controller_2_B.Integrator1_e;

    // Derivatives for Integrator: '<S54>/Integrator'
    _rtXdot->Integrator_CSTATE_m = pose_controller_2_B.Integrator1;

    // Derivatives for Integrator: '<S42>/Integrator'
    _rtXdot->Integrator_CSTATE_e = pose_controller_2_B.x_error;

    // Derivatives for Integrator: '<S42>/Integrator1'
    _rtXdot->Integrator1_CSTATE_p = pose_controller_2_B.y_error;

    // Derivatives for Integrator: '<S42>/Integrator2'
    _rtXdot->Integrator2_CSTATE_c = pose_controller_2_B.z_error;

    // Derivatives for Integrator: '<S54>/Integrator1'
    _rtXdot->Integrator1_CSTATE_k = pose_controller_2_B.Sum;

    // Derivatives for Integrator: '<S53>/Integrator1'
    _rtXdot->Integrator1_CSTATE_h = pose_controller_2_B.Sum_n;

    // Derivatives for Integrator: '<S52>/Integrator1'
    _rtXdot->Integrator1_CSTATE_g = pose_controller_2_B.Sum_l;

    // Derivatives for Integrator: '<S51>/Integrator1'
    _rtXdot->Integrator1_CSTATE_b = pose_controller_2_B.Sum_j;

    // Derivatives for Integrator: '<S50>/Integrator1'
    _rtXdot->Integrator1_CSTATE_d = pose_controller_2_B.Sum_k;

    // Derivatives for Integrator: '<S49>/Integrator1'
    _rtXdot->Integrator1_CSTATE_n = pose_controller_2_B.Sum_ng;

    // Derivatives for Integrator: '<S71>/Integrator1'
    _rtXdot->Integrator1_CSTATE_pj = pose_controller_2_B.Sum_o;

    // Derivatives for Integrator: '<S70>/Integrator1'
    _rtXdot->Integrator1_CSTATE_po = pose_controller_2_B.Sum_ns;

    // Derivatives for Integrator: '<S69>/Integrator1'
    _rtXdot->Integrator1_CSTATE_b2 = pose_controller_2_B.Sum_e;

    // Derivatives for Integrator: '<S57>/Integrator1'
    _rtXdot->Integrator1_CSTATE_j = pose_controller_2_B.Sum_p;

    // Derivatives for Integrator: '<S58>/Integrator1'
    _rtXdot->Integrator1_CSTATE_n2 = pose_controller_2_B.Sum_c;

    // Derivatives for Integrator: '<S56>/Integrator1'
    _rtXdot->Integrator1_CSTATE_i = pose_controller_2_B.Sum_g;
  } else {
    {
      real_T *dx;
      int_T i;
      dx = &(((XDot_pose_controller_2_T *) pose_controller_2_M->derivs)
             ->Integrator_CSTATE);
      for (i=0; i < 30; i++) {
        dx[i] = 0.0;
      }
    }
  }

  // End of Derivatives for SubSystem: '<Root>/Subsystem'
}

// Model initialize function
void pose_controller_2_initialize(void)
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  {
    // Setup solver object
    rtsiSetSimTimeStepPtr(&pose_controller_2_M->solverInfo,
                          &pose_controller_2_M->Timing.simTimeStep);
    rtsiSetTPtr(&pose_controller_2_M->solverInfo, &rtmGetTPtr
                (pose_controller_2_M));
    rtsiSetStepSizePtr(&pose_controller_2_M->solverInfo,
                       &pose_controller_2_M->Timing.stepSize0);
    rtsiSetdXPtr(&pose_controller_2_M->solverInfo, &pose_controller_2_M->derivs);
    rtsiSetContStatesPtr(&pose_controller_2_M->solverInfo, (real_T **)
                         &pose_controller_2_M->contStates);
    rtsiSetNumContStatesPtr(&pose_controller_2_M->solverInfo,
      &pose_controller_2_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&pose_controller_2_M->solverInfo,
      &pose_controller_2_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&pose_controller_2_M->solverInfo,
      &pose_controller_2_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&pose_controller_2_M->solverInfo,
      &pose_controller_2_M->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&pose_controller_2_M->solverInfo, (&rtmGetErrorStatus
      (pose_controller_2_M)));
    rtsiSetRTModelPtr(&pose_controller_2_M->solverInfo, pose_controller_2_M);
  }

  rtsiSetSimTimeStep(&pose_controller_2_M->solverInfo, MAJOR_TIME_STEP);
  pose_controller_2_M->intgData.y = pose_controller_2_M->odeY;
  pose_controller_2_M->intgData.f[0] = pose_controller_2_M->odeF[0];
  pose_controller_2_M->intgData.f[1] = pose_controller_2_M->odeF[1];
  pose_controller_2_M->intgData.f[2] = pose_controller_2_M->odeF[2];
  pose_controller_2_M->contStates = ((X_pose_controller_2_T *)
    &pose_controller_2_X);
  rtsiSetSolverData(&pose_controller_2_M->solverInfo, static_cast<void *>
                    (&pose_controller_2_M->intgData));
  rtsiSetSolverName(&pose_controller_2_M->solverInfo,"ode3");
  rtmSetTPtr(pose_controller_2_M, &pose_controller_2_M->Timing.tArray[0]);
  pose_controller_2_M->Timing.stepSize0 = 0.0125;
  rtmSetFirstInitCond(pose_controller_2_M, 1);

  {
    static const char_T tmp_0[22] = { '/', 'i', 'n', 'i', 't', '_', 'e', 'u',
      'l', 'e', 'r', '_', 'a', 'n', 'g', 'l', 'e', 's', '_', 'c', 'm', 'd' };

    static const char_T tmp_4[21] = { '/', 'n', 'o', 'm', 'i', 'n', 'a', 'l',
      '_', 'e', 'u', 'l', 'e', 'r', '_', 'a', 'n', 'g', 'l', 'e', 's' };

    static const char_T tmp[18] = { '/', 'i', 'n', 'i', 't', '_', 'p', 'o', 's',
      'i', 't', 'i', 'o', 'n', '_', 'c', 'm', 'd' };

    static const char_T tmp_3[18] = { '/', 'm', 'a', 'i', 'n', '_', 'e', 'u',
      'l', 'e', 'r', '_', 'a', 'n', 'g', 'l', 'e', 's' };

    static const char_T tmp_2[17] = { '/', 'n', 'o', 'm', 'i', 'n', 'a', 'l',
      '_', 'p', 'o', 's', 'i', 't', 'i', 'o', 'n' };

    static const char_T tmp_6[16] = { '/', 'm', 'a', 'i', 'n', '_', 'b', 'o',
      'd', 'y', '_', 'r', 'a', 't', 'e', 's' };

    static const char_T tmp_1[14] = { '/', 'm', 'a', 'i', 'n', '_', 'p', 'o',
      's', 'i', 't', 'i', 'o', 'n' };

    static const char_T tmp_5[14] = { '/', 'm', 'a', 'i', 'n', '_', 'v', 'e',
      'l', 'o', 'c', 'i', 't', 'y' };

    static const char_T tmp_7[11] = { '/', 'p', 's', 'i', '_', 'b', 'i', 'a',
      's', '_', '1' };

    static const char_T tmp_a[11] = { '/', 'p', 's', 'i', '_', 'b', 'i', 'a',
      's', '_', '2' };

    static const char_T tmp_d[11] = { '/', 'p', 's', 'i', '_', 'b', 'i', 'a',
      's', '_', '3' };

    static const char_T tmp_8[8] = { '/', 't', 'h', 'r', 'u', 's', 't', '1' };

    static const char_T tmp_b[8] = { '/', 't', 'h', 'r', 'u', 's', 't', '2' };

    static const char_T tmp_e[8] = { '/', 't', 'h', 'r', 'u', 's', 't', '3' };

    static const char_T tmp_9[7] = { '/', 'a', 'n', 'g', 'l', 'e', '1' };

    static const char_T tmp_c[7] = { '/', 'a', 'n', 'g', 'l', 'e', '2' };

    static const char_T tmp_f[7] = { '/', 'a', 'n', 'g', 'l', 'e', '3' };

    int32_T i;
    char_T b_zeroDelimTopic[15];
    char_T b_zeroDelimTopic_0[12];
    char_T b_zeroDelimTopic_1[9];
    char_T b_zeroDelimTopic_2[8];

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe9'
    // SystemInitialize for Enabled SubSystem: '<S3>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S6>/Out1' incorporates:
    //   Inport: '<S6>/In1'

    pose_controller_2_B.In1_a = pose_controller_2_P.Out1_Y0_p;

    // End of SystemInitialize for SubSystem: '<S3>/Enabled Subsystem'

    // Start for MATLABSystem: '<S3>/SourceBlock'
    pose_controller_2_DW.obj_j.matlabCodegenIsDeleted = false;
    pose_controller_2_DW.obj_j.isInitialized = 1;
    for (i = 0; i < 18; i++) {
      pose_controller_2_B.b_zeroDelimTopic_g[i] = tmp[i];
    }

    pose_controller_2_B.b_zeroDelimTopic_g[18] = '\x00';
    Sub_pose_controller_2_2544.createSubscriber
      (&pose_controller_2_B.b_zeroDelimTopic_g[0], 1);
    pose_controller_2_DW.obj_j.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S3>/SourceBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe9'

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe10'
    // SystemInitialize for Enabled SubSystem: '<S2>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S5>/Out1' incorporates:
    //   Inport: '<S5>/In1'

    pose_controller_2_B.In1_fj = pose_controller_2_P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S2>/Enabled Subsystem'

    // Start for MATLABSystem: '<S2>/SourceBlock'
    pose_controller_2_DW.obj_lc.matlabCodegenIsDeleted = false;
    pose_controller_2_DW.obj_lc.isInitialized = 1;
    for (i = 0; i < 22; i++) {
      pose_controller_2_B.b_zeroDelimTopic[i] = tmp_0[i];
    }

    pose_controller_2_B.b_zeroDelimTopic[22] = '\x00';
    Sub_pose_controller_2_2548.createSubscriber
      (&pose_controller_2_B.b_zeroDelimTopic[0], 1);
    pose_controller_2_DW.obj_lc.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S2>/SourceBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe10'

    // SystemInitialize for Enabled SubSystem: '<Root>/Subsystem'
    // InitializeConditions for Integrator: '<S56>/Integrator' incorporates:
    //   Integrator: '<S58>/Integrator'

    if (rtmIsFirstInitCond(pose_controller_2_M)) {
      pose_controller_2_X.Integrator_CSTATE = 0.0;
      pose_controller_2_X.Integrator_CSTATE_k = 0.0;
    }

    pose_controller_2_DW.Integrator_IWORK = 1;

    // End of InitializeConditions for Integrator: '<S56>/Integrator'

    // InitializeConditions for Derivative: '<S56>/Derivative'
    pose_controller_2_DW.TimeStampA = (rtInf);
    pose_controller_2_DW.TimeStampB = (rtInf);

    // InitializeConditions for Integrator: '<S58>/Integrator'
    pose_controller_2_DW.Integrator_IWORK_e = 1;

    // InitializeConditions for Derivative: '<S58>/Derivative'
    pose_controller_2_DW.TimeStampA_n = (rtInf);
    pose_controller_2_DW.TimeStampB_g = (rtInf);

    // InitializeConditions for Integrator: '<S57>/Integrator' incorporates:
    //   Integrator: '<S69>/Integrator'

    if (rtmIsFirstInitCond(pose_controller_2_M)) {
      pose_controller_2_X.Integrator_CSTATE_j = 0.0;
      pose_controller_2_X.Integrator_CSTATE_k3 = 0.0;
    }

    pose_controller_2_DW.Integrator_IWORK_l = 1;

    // End of InitializeConditions for Integrator: '<S57>/Integrator'

    // InitializeConditions for Derivative: '<S57>/Derivative'
    pose_controller_2_DW.TimeStampA_nq = (rtInf);
    pose_controller_2_DW.TimeStampB_e = (rtInf);

    // InitializeConditions for Integrator: '<S69>/Integrator'
    pose_controller_2_DW.Integrator_IWORK_g = 1;

    // InitializeConditions for Derivative: '<S69>/Derivative'
    pose_controller_2_DW.TimeStampA_f = (rtInf);
    pose_controller_2_DW.TimeStampB_f = (rtInf);

    // InitializeConditions for Integrator: '<S70>/Integrator' incorporates:
    //   Integrator: '<S71>/Integrator'

    if (rtmIsFirstInitCond(pose_controller_2_M)) {
      pose_controller_2_X.Integrator_CSTATE_c = 0.0;
      pose_controller_2_X.Integrator_CSTATE_i = 0.0;
    }

    pose_controller_2_DW.Integrator_IWORK_k = 1;

    // End of InitializeConditions for Integrator: '<S70>/Integrator'

    // InitializeConditions for Derivative: '<S70>/Derivative'
    pose_controller_2_DW.TimeStampA_j = (rtInf);
    pose_controller_2_DW.TimeStampB_f4 = (rtInf);

    // InitializeConditions for Integrator: '<S71>/Integrator'
    pose_controller_2_DW.Integrator_IWORK_d = 1;

    // InitializeConditions for Derivative: '<S71>/Derivative'
    pose_controller_2_DW.TimeStampA_m = (rtInf);
    pose_controller_2_DW.TimeStampB_l = (rtInf);

    // InitializeConditions for Integrator: '<S49>/Integrator'
    pose_controller_2_X.Integrator_CSTATE_cv = pose_controller_2_P.Integrator_IC;

    // InitializeConditions for Derivative: '<S49>/Derivative'
    pose_controller_2_DW.TimeStampA_fq = (rtInf);
    pose_controller_2_DW.TimeStampB_m = (rtInf);

    // InitializeConditions for Integrator: '<S50>/Integrator'
    pose_controller_2_X.Integrator_CSTATE_b =
      pose_controller_2_P.Integrator_IC_d;

    // InitializeConditions for Derivative: '<S50>/Derivative'
    pose_controller_2_DW.TimeStampA_a = (rtInf);
    pose_controller_2_DW.TimeStampB_gd = (rtInf);

    // InitializeConditions for Integrator: '<S51>/Integrator'
    pose_controller_2_X.Integrator_CSTATE_o =
      pose_controller_2_P.Integrator_IC_a;

    // InitializeConditions for Derivative: '<S51>/Derivative'
    pose_controller_2_DW.TimeStampA_l = (rtInf);
    pose_controller_2_DW.TimeStampB_p = (rtInf);

    // InitializeConditions for Integrator: '<S44>/Integrator' incorporates:
    //   Integrator: '<S44>/Integrator1'

    if (rtmIsFirstInitCond(pose_controller_2_M)) {
      pose_controller_2_X.Integrator_CSTATE_a = 0.0;
      pose_controller_2_X.Integrator1_CSTATE = 0.0;
    }

    pose_controller_2_DW.Integrator_IWORK_lt = 1;

    // End of InitializeConditions for Integrator: '<S44>/Integrator'

    // InitializeConditions for Integrator: '<S44>/Integrator1'
    pose_controller_2_DW.Integrator1_IWORK = 1;

    // InitializeConditions for Integrator: '<S44>/Integrator2' incorporates:
    //   Integrator: '<S42>/Integrator'

    if (rtmIsFirstInitCond(pose_controller_2_M)) {
      pose_controller_2_X.Integrator2_CSTATE = 0.0;
      pose_controller_2_X.Integrator_CSTATE_e = 0.0;
    }

    pose_controller_2_DW.Integrator2_IWORK = 1;

    // End of InitializeConditions for Integrator: '<S44>/Integrator2'

    // InitializeConditions for Integrator: '<S52>/Integrator'
    pose_controller_2_X.Integrator_CSTATE_jf =
      pose_controller_2_P.Integrator_IC_g;

    // InitializeConditions for Derivative: '<S52>/Derivative'
    pose_controller_2_DW.TimeStampA_h = (rtInf);
    pose_controller_2_DW.TimeStampB_eu = (rtInf);

    // InitializeConditions for Integrator: '<S53>/Integrator'
    pose_controller_2_X.Integrator_CSTATE_p =
      pose_controller_2_P.Integrator_IC_b;

    // InitializeConditions for Derivative: '<S53>/Derivative'
    pose_controller_2_DW.TimeStampA_hg = (rtInf);
    pose_controller_2_DW.TimeStampB_h = (rtInf);

    // InitializeConditions for Integrator: '<S54>/Integrator'
    pose_controller_2_X.Integrator_CSTATE_m =
      pose_controller_2_P.Integrator_IC_ad;

    // InitializeConditions for Derivative: '<S54>/Derivative'
    pose_controller_2_DW.TimeStampA_p = (rtInf);
    pose_controller_2_DW.TimeStampB_gm = (rtInf);

    // InitializeConditions for Integrator: '<S42>/Integrator'
    pose_controller_2_DW.Integrator_IWORK_df = 1;

    // InitializeConditions for Integrator: '<S42>/Integrator1' incorporates:
    //   Integrator: '<S42>/Integrator2'

    if (rtmIsFirstInitCond(pose_controller_2_M)) {
      pose_controller_2_X.Integrator1_CSTATE_p = 0.0;
      pose_controller_2_X.Integrator2_CSTATE_c = 0.0;
    }

    pose_controller_2_DW.Integrator1_IWORK_g = 1;

    // End of InitializeConditions for Integrator: '<S42>/Integrator1'

    // InitializeConditions for Integrator: '<S42>/Integrator2'
    pose_controller_2_DW.Integrator2_IWORK_m = 1;

    // InitializeConditions for Integrator: '<S54>/Integrator1'
    pose_controller_2_X.Integrator1_CSTATE_k =
      pose_controller_2_P.Integrator1_IC;

    // InitializeConditions for Integrator: '<S53>/Integrator1'
    pose_controller_2_X.Integrator1_CSTATE_h =
      pose_controller_2_P.Integrator1_IC_j;

    // InitializeConditions for Integrator: '<S52>/Integrator1'
    pose_controller_2_X.Integrator1_CSTATE_g =
      pose_controller_2_P.Integrator1_IC_p;

    // InitializeConditions for Integrator: '<S51>/Integrator1'
    pose_controller_2_X.Integrator1_CSTATE_b =
      pose_controller_2_P.Integrator1_IC_b;

    // InitializeConditions for Integrator: '<S50>/Integrator1'
    pose_controller_2_X.Integrator1_CSTATE_d =
      pose_controller_2_P.Integrator1_IC_c;

    // InitializeConditions for Integrator: '<S49>/Integrator1'
    pose_controller_2_X.Integrator1_CSTATE_n =
      pose_controller_2_P.Integrator1_IC_g;

    // InitializeConditions for Integrator: '<S71>/Integrator1'
    pose_controller_2_X.Integrator1_CSTATE_pj =
      pose_controller_2_P.Integrator1_IC_h;

    // InitializeConditions for Integrator: '<S70>/Integrator1'
    pose_controller_2_X.Integrator1_CSTATE_po =
      pose_controller_2_P.Integrator1_IC_pm;

    // InitializeConditions for Integrator: '<S69>/Integrator1'
    pose_controller_2_X.Integrator1_CSTATE_b2 =
      pose_controller_2_P.Integrator1_IC_a;

    // InitializeConditions for Integrator: '<S57>/Integrator1'
    pose_controller_2_X.Integrator1_CSTATE_j =
      pose_controller_2_P.Integrator1_IC_bk;

    // InitializeConditions for Integrator: '<S58>/Integrator1'
    pose_controller_2_X.Integrator1_CSTATE_n2 =
      pose_controller_2_P.Integrator1_IC_e;

    // InitializeConditions for Integrator: '<S56>/Integrator1'
    pose_controller_2_X.Integrator1_CSTATE_i =
      pose_controller_2_P.Integrator1_IC_m;

    // SystemInitialize for Atomic SubSystem: '<S4>/Subscribe3'
    // SystemInitialize for Enabled SubSystem: '<S25>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S106>/Out1' incorporates:
    //   Inport: '<S106>/In1'

    pose_controller_2_B.In1_e = pose_controller_2_P.Out1_Y0_m;

    // End of SystemInitialize for SubSystem: '<S25>/Enabled Subsystem'

    // Start for MATLABSystem: '<S25>/SourceBlock'
    pose_controller_2_DW.obj_o.matlabCodegenIsDeleted = false;
    pose_controller_2_DW.obj_o.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      b_zeroDelimTopic[i] = tmp_1[i];
    }

    b_zeroDelimTopic[14] = '\x00';
    Sub_pose_controller_2_232.createSubscriber(&b_zeroDelimTopic[0], 1);
    pose_controller_2_DW.obj_o.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S25>/SourceBlock'
    // End of SystemInitialize for SubSystem: '<S4>/Subscribe3'

    // SystemInitialize for Atomic SubSystem: '<S4>/Subscribe4'
    // SystemInitialize for Enabled SubSystem: '<S26>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S107>/Out1' incorporates:
    //   Inport: '<S107>/In1'

    pose_controller_2_B.In1_k = pose_controller_2_P.Out1_Y0_h;

    // End of SystemInitialize for SubSystem: '<S26>/Enabled Subsystem'

    // Start for MATLABSystem: '<S26>/SourceBlock'
    pose_controller_2_DW.obj_a.matlabCodegenIsDeleted = false;
    pose_controller_2_DW.obj_a.isInitialized = 1;
    for (i = 0; i < 17; i++) {
      pose_controller_2_B.b_zeroDelimTopic_g1[i] = tmp_2[i];
    }

    pose_controller_2_B.b_zeroDelimTopic_g1[17] = '\x00';
    Sub_pose_controller_2_235.createSubscriber
      (&pose_controller_2_B.b_zeroDelimTopic_g1[0], 1);
    pose_controller_2_DW.obj_a.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S26>/SourceBlock'
    // End of SystemInitialize for SubSystem: '<S4>/Subscribe4'

    // SystemInitialize for Atomic SubSystem: '<S4>/Subscribe'
    // SystemInitialize for Enabled SubSystem: '<S22>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S103>/Out1' incorporates:
    //   Inport: '<S103>/In1'

    pose_controller_2_B.In1_l = pose_controller_2_P.Out1_Y0_p4;

    // End of SystemInitialize for SubSystem: '<S22>/Enabled Subsystem'

    // Start for MATLABSystem: '<S22>/SourceBlock'
    pose_controller_2_DW.obj_mx.matlabCodegenIsDeleted = false;
    pose_controller_2_DW.obj_mx.isInitialized = 1;
    for (i = 0; i < 18; i++) {
      pose_controller_2_B.b_zeroDelimTopic_g[i] = tmp_3[i];
    }

    pose_controller_2_B.b_zeroDelimTopic_g[18] = '\x00';
    Sub_pose_controller_2_222.createSubscriber
      (&pose_controller_2_B.b_zeroDelimTopic_g[0], 1);
    pose_controller_2_DW.obj_mx.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S22>/SourceBlock'
    // End of SystemInitialize for SubSystem: '<S4>/Subscribe'

    // SystemInitialize for Atomic SubSystem: '<S4>/Subscribe5'
    // SystemInitialize for Enabled SubSystem: '<S27>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S108>/Out1' incorporates:
    //   Inport: '<S108>/In1'

    pose_controller_2_B.In1 = pose_controller_2_P.Out1_Y0_k;

    // End of SystemInitialize for SubSystem: '<S27>/Enabled Subsystem'

    // Start for MATLABSystem: '<S27>/SourceBlock'
    pose_controller_2_DW.obj_m.matlabCodegenIsDeleted = false;
    pose_controller_2_DW.obj_m.isInitialized = 1;
    for (i = 0; i < 21; i++) {
      pose_controller_2_B.b_zeroDelimTopic_f[i] = tmp_4[i];
    }

    pose_controller_2_B.b_zeroDelimTopic_f[21] = '\x00';
    Sub_pose_controller_2_238.createSubscriber
      (&pose_controller_2_B.b_zeroDelimTopic_f[0], 1);
    pose_controller_2_DW.obj_m.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S27>/SourceBlock'
    // End of SystemInitialize for SubSystem: '<S4>/Subscribe5'

    // SystemInitialize for Atomic SubSystem: '<S4>/Subscribe2'
    // SystemInitialize for Enabled SubSystem: '<S24>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S105>/Out1' incorporates:
    //   Inport: '<S105>/In1'

    pose_controller_2_B.In1_h = pose_controller_2_P.Out1_Y0_f;

    // End of SystemInitialize for SubSystem: '<S24>/Enabled Subsystem'

    // Start for MATLABSystem: '<S24>/SourceBlock'
    pose_controller_2_DW.obj_ou.matlabCodegenIsDeleted = false;
    pose_controller_2_DW.obj_ou.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      b_zeroDelimTopic[i] = tmp_5[i];
    }

    b_zeroDelimTopic[14] = '\x00';
    Sub_pose_controller_2_229.createSubscriber(&b_zeroDelimTopic[0], 1);
    pose_controller_2_DW.obj_ou.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S24>/SourceBlock'
    // End of SystemInitialize for SubSystem: '<S4>/Subscribe2'

    // SystemInitialize for Atomic SubSystem: '<S4>/Subscribe1'
    // SystemInitialize for Enabled SubSystem: '<S23>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S104>/Out1' incorporates:
    //   Inport: '<S104>/In1'

    pose_controller_2_B.In1_f = pose_controller_2_P.Out1_Y0_d;

    // End of SystemInitialize for SubSystem: '<S23>/Enabled Subsystem'

    // Start for MATLABSystem: '<S23>/SourceBlock'
    pose_controller_2_DW.obj_l.matlabCodegenIsDeleted = false;
    pose_controller_2_DW.obj_l.isInitialized = 1;
    for (i = 0; i < 16; i++) {
      pose_controller_2_B.b_zeroDelimTopic_m[i] = tmp_6[i];
    }

    pose_controller_2_B.b_zeroDelimTopic_m[16] = '\x00';
    Sub_pose_controller_2_225.createSubscriber
      (&pose_controller_2_B.b_zeroDelimTopic_m[0], 1);
    pose_controller_2_DW.obj_l.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S23>/SourceBlock'
    // End of SystemInitialize for SubSystem: '<S4>/Subscribe1'

    // SystemInitialize for Atomic SubSystem: '<S4>/Subscribe6'
    // SystemInitialize for Enabled SubSystem: '<S28>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S109>/Out1' incorporates:
    //   Inport: '<S109>/In1'

    pose_controller_2_B.In1_p = pose_controller_2_P.Out1_Y0_p3;

    // End of SystemInitialize for SubSystem: '<S28>/Enabled Subsystem'

    // Start for MATLABSystem: '<S28>/SourceBlock'
    pose_controller_2_DW.obj_f.matlabCodegenIsDeleted = false;
    pose_controller_2_DW.obj_f.isInitialized = 1;
    for (i = 0; i < 11; i++) {
      b_zeroDelimTopic_0[i] = tmp_7[i];
    }

    b_zeroDelimTopic_0[11] = '\x00';
    Sub_pose_controller_2_2529.createSubscriber(&b_zeroDelimTopic_0[0], 1);
    pose_controller_2_DW.obj_f.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S28>/SourceBlock'
    // End of SystemInitialize for SubSystem: '<S4>/Subscribe6'

    // SystemInitialize for Atomic SubSystem: '<S4>/Publish'
    // Start for MATLABSystem: '<S16>/SinkBlock'
    pose_controller_2_DW.obj_gb.matlabCodegenIsDeleted = false;
    pose_controller_2_DW.obj_gb.isInitialized = 1;
    for (i = 0; i < 8; i++) {
      b_zeroDelimTopic_1[i] = tmp_8[i];
    }

    b_zeroDelimTopic_1[8] = '\x00';
    Pub_pose_controller_2_265.createPublisher(&b_zeroDelimTopic_1[0], 1);
    pose_controller_2_DW.obj_gb.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S16>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<S4>/Publish'

    // SystemInitialize for Atomic SubSystem: '<S4>/Publish1'
    // Start for MATLABSystem: '<S17>/SinkBlock'
    pose_controller_2_DW.obj_e.matlabCodegenIsDeleted = false;
    pose_controller_2_DW.obj_e.isInitialized = 1;
    for (i = 0; i < 7; i++) {
      b_zeroDelimTopic_2[i] = tmp_9[i];
    }

    b_zeroDelimTopic_2[7] = '\x00';
    Pub_pose_controller_2_266.createPublisher(&b_zeroDelimTopic_2[0], 1);
    pose_controller_2_DW.obj_e.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S17>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<S4>/Publish1'

    // SystemInitialize for Atomic SubSystem: '<S4>/Subscribe7'
    // SystemInitialize for Enabled SubSystem: '<S29>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S110>/Out1' incorporates:
    //   Inport: '<S110>/In1'

    pose_controller_2_B.In1_c = pose_controller_2_P.Out1_Y0_n;

    // End of SystemInitialize for SubSystem: '<S29>/Enabled Subsystem'

    // Start for MATLABSystem: '<S29>/SourceBlock'
    pose_controller_2_DW.obj_g.matlabCodegenIsDeleted = false;
    pose_controller_2_DW.obj_g.isInitialized = 1;
    for (i = 0; i < 11; i++) {
      b_zeroDelimTopic_0[i] = tmp_a[i];
    }

    b_zeroDelimTopic_0[11] = '\x00';
    Sub_pose_controller_2_2533.createSubscriber(&b_zeroDelimTopic_0[0], 1);
    pose_controller_2_DW.obj_g.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S29>/SourceBlock'
    // End of SystemInitialize for SubSystem: '<S4>/Subscribe7'

    // SystemInitialize for Atomic SubSystem: '<S4>/Publish2'
    // Start for MATLABSystem: '<S18>/SinkBlock'
    pose_controller_2_DW.obj_k.matlabCodegenIsDeleted = false;
    pose_controller_2_DW.obj_k.isInitialized = 1;
    for (i = 0; i < 8; i++) {
      b_zeroDelimTopic_1[i] = tmp_b[i];
    }

    b_zeroDelimTopic_1[8] = '\x00';
    Pub_pose_controller_2_267.createPublisher(&b_zeroDelimTopic_1[0], 1);
    pose_controller_2_DW.obj_k.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S18>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<S4>/Publish2'

    // SystemInitialize for Atomic SubSystem: '<S4>/Publish3'
    // Start for MATLABSystem: '<S19>/SinkBlock'
    pose_controller_2_DW.obj_b.matlabCodegenIsDeleted = false;
    pose_controller_2_DW.obj_b.isInitialized = 1;
    for (i = 0; i < 7; i++) {
      b_zeroDelimTopic_2[i] = tmp_c[i];
    }

    b_zeroDelimTopic_2[7] = '\x00';
    Pub_pose_controller_2_268.createPublisher(&b_zeroDelimTopic_2[0], 1);
    pose_controller_2_DW.obj_b.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S19>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<S4>/Publish3'

    // SystemInitialize for Atomic SubSystem: '<S4>/Subscribe8'
    // SystemInitialize for Enabled SubSystem: '<S30>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S111>/Out1' incorporates:
    //   Inport: '<S111>/In1'

    pose_controller_2_B.In1_g = pose_controller_2_P.Out1_Y0_kk;

    // End of SystemInitialize for SubSystem: '<S30>/Enabled Subsystem'

    // Start for MATLABSystem: '<S30>/SourceBlock'
    pose_controller_2_DW.obj.matlabCodegenIsDeleted = false;
    pose_controller_2_DW.obj.isInitialized = 1;
    for (i = 0; i < 11; i++) {
      b_zeroDelimTopic_0[i] = tmp_d[i];
    }

    b_zeroDelimTopic_0[11] = '\x00';
    Sub_pose_controller_2_2536.createSubscriber(&b_zeroDelimTopic_0[0], 1);
    pose_controller_2_DW.obj.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S30>/SourceBlock'
    // End of SystemInitialize for SubSystem: '<S4>/Subscribe8'

    // SystemInitialize for Atomic SubSystem: '<S4>/Publish4'
    // Start for MATLABSystem: '<S20>/SinkBlock'
    pose_controller_2_DW.obj_of.matlabCodegenIsDeleted = false;
    pose_controller_2_DW.obj_of.isInitialized = 1;
    for (i = 0; i < 8; i++) {
      b_zeroDelimTopic_1[i] = tmp_e[i];
    }

    b_zeroDelimTopic_1[8] = '\x00';
    Pub_pose_controller_2_269.createPublisher(&b_zeroDelimTopic_1[0], 1);
    pose_controller_2_DW.obj_of.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S20>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<S4>/Publish4'

    // SystemInitialize for Atomic SubSystem: '<S4>/Publish5'
    // Start for MATLABSystem: '<S21>/SinkBlock'
    pose_controller_2_DW.obj_j4.matlabCodegenIsDeleted = false;
    pose_controller_2_DW.obj_j4.isInitialized = 1;
    for (i = 0; i < 7; i++) {
      b_zeroDelimTopic_2[i] = tmp_f[i];
    }

    b_zeroDelimTopic_2[7] = '\x00';
    Pub_pose_controller_2_270.createPublisher(&b_zeroDelimTopic_2[0], 1);
    pose_controller_2_DW.obj_j4.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S21>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<S4>/Publish5'
    // End of SystemInitialize for SubSystem: '<Root>/Subsystem'
  }

  // set "at time zero" to false
  if (rtmIsFirstInitCond(pose_controller_2_M)) {
    rtmSetFirstInitCond(pose_controller_2_M, 0);
  }
}

// Model terminate function
void pose_controller_2_terminate(void)
{
  // Terminate for Atomic SubSystem: '<Root>/Subscribe9'
  // Terminate for MATLABSystem: '<S3>/SourceBlock'
  if (!pose_controller_2_DW.obj_j.matlabCodegenIsDeleted) {
    pose_controller_2_DW.obj_j.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S3>/SourceBlock'
  // End of Terminate for SubSystem: '<Root>/Subscribe9'

  // Terminate for Atomic SubSystem: '<Root>/Subscribe10'
  // Terminate for MATLABSystem: '<S2>/SourceBlock'
  if (!pose_controller_2_DW.obj_lc.matlabCodegenIsDeleted) {
    pose_controller_2_DW.obj_lc.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S2>/SourceBlock'
  // End of Terminate for SubSystem: '<Root>/Subscribe10'

  // Terminate for Enabled SubSystem: '<Root>/Subsystem'
  // Terminate for Atomic SubSystem: '<S4>/Subscribe3'
  // Terminate for MATLABSystem: '<S25>/SourceBlock'
  if (!pose_controller_2_DW.obj_o.matlabCodegenIsDeleted) {
    pose_controller_2_DW.obj_o.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S25>/SourceBlock'
  // End of Terminate for SubSystem: '<S4>/Subscribe3'

  // Terminate for Atomic SubSystem: '<S4>/Subscribe4'
  // Terminate for MATLABSystem: '<S26>/SourceBlock'
  if (!pose_controller_2_DW.obj_a.matlabCodegenIsDeleted) {
    pose_controller_2_DW.obj_a.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S26>/SourceBlock'
  // End of Terminate for SubSystem: '<S4>/Subscribe4'

  // Terminate for Atomic SubSystem: '<S4>/Subscribe'
  // Terminate for MATLABSystem: '<S22>/SourceBlock'
  if (!pose_controller_2_DW.obj_mx.matlabCodegenIsDeleted) {
    pose_controller_2_DW.obj_mx.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S22>/SourceBlock'
  // End of Terminate for SubSystem: '<S4>/Subscribe'

  // Terminate for Atomic SubSystem: '<S4>/Subscribe5'
  // Terminate for MATLABSystem: '<S27>/SourceBlock'
  if (!pose_controller_2_DW.obj_m.matlabCodegenIsDeleted) {
    pose_controller_2_DW.obj_m.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S27>/SourceBlock'
  // End of Terminate for SubSystem: '<S4>/Subscribe5'

  // Terminate for Atomic SubSystem: '<S4>/Subscribe2'
  // Terminate for MATLABSystem: '<S24>/SourceBlock'
  if (!pose_controller_2_DW.obj_ou.matlabCodegenIsDeleted) {
    pose_controller_2_DW.obj_ou.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S24>/SourceBlock'
  // End of Terminate for SubSystem: '<S4>/Subscribe2'

  // Terminate for Atomic SubSystem: '<S4>/Subscribe1'
  // Terminate for MATLABSystem: '<S23>/SourceBlock'
  if (!pose_controller_2_DW.obj_l.matlabCodegenIsDeleted) {
    pose_controller_2_DW.obj_l.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S23>/SourceBlock'
  // End of Terminate for SubSystem: '<S4>/Subscribe1'

  // Terminate for Atomic SubSystem: '<S4>/Subscribe6'
  // Terminate for MATLABSystem: '<S28>/SourceBlock'
  if (!pose_controller_2_DW.obj_f.matlabCodegenIsDeleted) {
    pose_controller_2_DW.obj_f.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S28>/SourceBlock'
  // End of Terminate for SubSystem: '<S4>/Subscribe6'

  // Terminate for Atomic SubSystem: '<S4>/Publish'
  // Terminate for MATLABSystem: '<S16>/SinkBlock'
  if (!pose_controller_2_DW.obj_gb.matlabCodegenIsDeleted) {
    pose_controller_2_DW.obj_gb.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S16>/SinkBlock'
  // End of Terminate for SubSystem: '<S4>/Publish'

  // Terminate for Atomic SubSystem: '<S4>/Publish1'
  // Terminate for MATLABSystem: '<S17>/SinkBlock'
  if (!pose_controller_2_DW.obj_e.matlabCodegenIsDeleted) {
    pose_controller_2_DW.obj_e.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S17>/SinkBlock'
  // End of Terminate for SubSystem: '<S4>/Publish1'

  // Terminate for Atomic SubSystem: '<S4>/Subscribe7'
  // Terminate for MATLABSystem: '<S29>/SourceBlock'
  if (!pose_controller_2_DW.obj_g.matlabCodegenIsDeleted) {
    pose_controller_2_DW.obj_g.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S29>/SourceBlock'
  // End of Terminate for SubSystem: '<S4>/Subscribe7'

  // Terminate for Atomic SubSystem: '<S4>/Publish2'
  // Terminate for MATLABSystem: '<S18>/SinkBlock'
  if (!pose_controller_2_DW.obj_k.matlabCodegenIsDeleted) {
    pose_controller_2_DW.obj_k.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S18>/SinkBlock'
  // End of Terminate for SubSystem: '<S4>/Publish2'

  // Terminate for Atomic SubSystem: '<S4>/Publish3'
  // Terminate for MATLABSystem: '<S19>/SinkBlock'
  if (!pose_controller_2_DW.obj_b.matlabCodegenIsDeleted) {
    pose_controller_2_DW.obj_b.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S19>/SinkBlock'
  // End of Terminate for SubSystem: '<S4>/Publish3'

  // Terminate for Atomic SubSystem: '<S4>/Subscribe8'
  // Terminate for MATLABSystem: '<S30>/SourceBlock'
  if (!pose_controller_2_DW.obj.matlabCodegenIsDeleted) {
    pose_controller_2_DW.obj.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S30>/SourceBlock'
  // End of Terminate for SubSystem: '<S4>/Subscribe8'

  // Terminate for Atomic SubSystem: '<S4>/Publish4'
  // Terminate for MATLABSystem: '<S20>/SinkBlock'
  if (!pose_controller_2_DW.obj_of.matlabCodegenIsDeleted) {
    pose_controller_2_DW.obj_of.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S20>/SinkBlock'
  // End of Terminate for SubSystem: '<S4>/Publish4'

  // Terminate for Atomic SubSystem: '<S4>/Publish5'
  // Terminate for MATLABSystem: '<S21>/SinkBlock'
  if (!pose_controller_2_DW.obj_j4.matlabCodegenIsDeleted) {
    pose_controller_2_DW.obj_j4.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S21>/SinkBlock'
  // End of Terminate for SubSystem: '<S4>/Publish5'
  // End of Terminate for SubSystem: '<Root>/Subsystem'
}

//
// File trailer for generated code.
//
// [EOF]
//
