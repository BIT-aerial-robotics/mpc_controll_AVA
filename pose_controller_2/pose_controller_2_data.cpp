//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: pose_controller_2_data.cpp
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

// Block parameters (default storage)
P_pose_controller_2_T pose_controller_2_P = {
  // Computed Parameter: Out1_Y0
  //  Referenced by: '<S5>/Out1'

  {
    0.0,                               // X
    0.0,                               // Y
    0.0                                // Z
  },

  // Computed Parameter: Constant_Value
  //  Referenced by: '<S2>/Constant'

  {
    0.0,                               // X
    0.0,                               // Y
    0.0                                // Z
  },

  // Computed Parameter: Out1_Y0_p
  //  Referenced by: '<S6>/Out1'

  {
    0.0,                               // X
    0.0,                               // Y
    0.0                                // Z
  },

  // Computed Parameter: Constant_Value_i
  //  Referenced by: '<S3>/Constant'

  {
    0.0,                               // X
    0.0,                               // Y
    0.0                                // Z
  },

  // Computed Parameter: Constant_Value_o
  //  Referenced by: '<S7>/Constant'

  {
    0.0,                               // X
    0.0,                               // Y
    0.0                                // Z
  },

  // Computed Parameter: Constant_Value_b
  //  Referenced by: '<S8>/Constant'

  {
    0.0,                               // X
    0.0,                               // Y
    0.0                                // Z
  },

  // Computed Parameter: Constant_Value_a
  //  Referenced by: '<S9>/Constant'

  {
    0.0,                               // X
    0.0,                               // Y
    0.0                                // Z
  },

  // Computed Parameter: Out1_Y0_p4
  //  Referenced by: '<S103>/Out1'

  {
    0.0,                               // X
    0.0,                               // Y
    0.0                                // Z
  },

  // Computed Parameter: Constant_Value_ii
  //  Referenced by: '<S22>/Constant'

  {
    0.0,                               // X
    0.0,                               // Y
    0.0                                // Z
  },

  // Computed Parameter: Out1_Y0_d
  //  Referenced by: '<S104>/Out1'

  {
    0.0,                               // X
    0.0,                               // Y
    0.0                                // Z
  },

  // Computed Parameter: Constant_Value_c
  //  Referenced by: '<S23>/Constant'

  {
    0.0,                               // X
    0.0,                               // Y
    0.0                                // Z
  },

  // Computed Parameter: Out1_Y0_f
  //  Referenced by: '<S105>/Out1'

  {
    0.0,                               // X
    0.0,                               // Y
    0.0                                // Z
  },

  // Computed Parameter: Constant_Value_d
  //  Referenced by: '<S24>/Constant'

  {
    0.0,                               // X
    0.0,                               // Y
    0.0                                // Z
  },

  // Computed Parameter: Out1_Y0_m
  //  Referenced by: '<S106>/Out1'

  {
    0.0,                               // X
    0.0,                               // Y
    0.0                                // Z
  },

  // Computed Parameter: Constant_Value_cf
  //  Referenced by: '<S25>/Constant'

  {
    0.0,                               // X
    0.0,                               // Y
    0.0                                // Z
  },

  // Computed Parameter: Out1_Y0_h
  //  Referenced by: '<S107>/Out1'

  {
    0.0,                               // X
    0.0,                               // Y
    0.0                                // Z
  },

  // Computed Parameter: Constant_Value_b1
  //  Referenced by: '<S26>/Constant'

  {
    0.0,                               // X
    0.0,                               // Y
    0.0                                // Z
  },

  // Computed Parameter: Out1_Y0_k
  //  Referenced by: '<S108>/Out1'

  {
    0.0,                               // X
    0.0,                               // Y
    0.0                                // Z
  },

  // Computed Parameter: Constant_Value_h
  //  Referenced by: '<S27>/Constant'

  {
    0.0,                               // X
    0.0,                               // Y
    0.0                                // Z
  },

  // Computed Parameter: Constant_Value_cn
  //  Referenced by: '<S10>/Constant'

  {
    0.0                                // Data
  },

  // Computed Parameter: Constant_Value_n
  //  Referenced by: '<S11>/Constant'

  {
    0.0                                // Data
  },

  // Computed Parameter: Constant_Value_g
  //  Referenced by: '<S12>/Constant'

  {
    0.0                                // Data
  },

  // Computed Parameter: Out1_Y0_p3
  //  Referenced by: '<S109>/Out1'

  {
    0.0                                // Data
  },

  // Computed Parameter: Constant_Value_j
  //  Referenced by: '<S28>/Constant'

  {
    0.0                                // Data
  },

  // Computed Parameter: Out1_Y0_n
  //  Referenced by: '<S110>/Out1'

  {
    0.0                                // Data
  },

  // Computed Parameter: Constant_Value_aw
  //  Referenced by: '<S29>/Constant'

  {
    0.0                                // Data
  },

  // Computed Parameter: Out1_Y0_kk
  //  Referenced by: '<S111>/Out1'

  {
    0.0                                // Data
  },

  // Computed Parameter: Constant_Value_l
  //  Referenced by: '<S30>/Constant'

  {
    0.0                                // Data
  },

  // Expression: 0
  //  Referenced by: '<S31>/tool_x'

  0.0,

  // Expression: 0
  //  Referenced by: '<S31>/tool_y'

  0.0,

  // Expression: 0
  //  Referenced by: '<S31>/tool_z'

  0.0,

  // Expression: 0
  //  Referenced by: '<S49>/Integrator'

  0.0,

  // Expression: 0
  //  Referenced by: '<S50>/Integrator'

  0.0,

  // Expression: 0
  //  Referenced by: '<S51>/Integrator'

  0.0,

  // Expression: 5
  //  Referenced by: '<S40>/kp_x'

  5.0*2.5,

  // Expression: 5
  //  Referenced by: '<S40>/kp_y'

  5.0*2.5,

  // Expression: 5
  //  Referenced by: '<S40>/kp_z'

  5.0*2.5,

  // Expression: 3
  //  Referenced by: '<S40>/kp_phi'

  3.0*2.5,

  // Expression: 3
  //  Referenced by: '<S40>/kp_theta'

  3.0*2.5,

  // Expression: 5
  //  Referenced by: '<S40>/kp_psi'

  5.0*2.5,

  // Expression: 0.6737
  //  Referenced by: '<S31>/drone1_x'

  0.6737,

  // Expression: 0
  //  Referenced by: '<S31>/drone1_y'

  0.0,

  // Expression: -0.12468
  //  Referenced by: '<S31>/drone1_z'

  -0.12468,

  // Expression: -0.3368
  //  Referenced by: '<S31>/drone2_x'

  -0.3368,

  // Expression: 0.5834
  //  Referenced by: '<S31>/drone2_y'

  0.5834,

  // Expression: -0.12468
  //  Referenced by: '<S31>/drone2_z'

  -0.12468,

  // Expression: -0.3368
  //  Referenced by: '<S31>/drone3_x'

  -0.3368,

  // Expression: -0.5834
  //  Referenced by: '<S31>/drone3_y'

  -0.5834,

  // Expression: -0.12468
  //  Referenced by: '<S31>/drone3_z'

  -0.12468,

  // Expression: 2.5
  //  Referenced by: '<S32>/center_mass'

  //2.97,
  1.607,

  // Expression: 2.93
  //  Referenced by: '<S32>/drone1_mass'
  /*
  fly1_mass = 1.597;//
  fly2_mass = 1.604;
  fly3_mass = 1.583;
  center_mass = 1.607;
  */
  //2.75,
  1.597,

  // Expression: 2.975
  //  Referenced by: '<S32>/drone2_mass'

  //2.69,
  1.604,

  // Expression: 2.85
  //  Referenced by: '<S32>/drone3_mass'

  //2.75,
  1.583,

  // Expression: 0.06250784647
  //  Referenced by: '<S101>/Ixx'
  //0.06250784647,
  0.056804546,

  // Expression: 0.06280979995
  //  Referenced by: '<S101>/Iyy'
  //0.06280979995,
  0.057436539,

  // Expression: 0.11503523247
  //  Referenced by: '<S101>/Izz'
  //0.11503523247,
  0.108610089,

  // Expression: 9
  //  Referenced by: '<S38>/kd_phi'

  9.0,

  // Expression: 9
  //  Referenced by: '<S38>/kd_theta'

  9.0,

  // Expression: 15
  //  Referenced by: '<S38>/kd_psi'

  15.0,

  // Expression: 1.4
  //  Referenced by: '<S38>/ki_phi'

   2.0,

  // Expression: 1.4
  //  Referenced by: '<S38>/ki_theta'

  2.0,

  // Expression: 2
  //  Referenced by: '<S38>/ki_psi'

  2.5,

  // Expression: 0
  //  Referenced by: '<S52>/Integrator'

  0.0,

  // Expression: 0
  //  Referenced by: '<S53>/Integrator'

  0.0,

  // Expression: 0
  //  Referenced by: '<S54>/Integrator'

  0.0,

  // Expression: 15
  //  Referenced by: '<S37>/kd_x'

  15.0*3,

  // Expression: 15
  //  Referenced by: '<S37>/kd_y'

  15.0*3,

  // Expression: 15
  //  Referenced by: '<S37>/kd_z'

  15.0*3,

  // Expression: 2
  //  Referenced by: '<S37>/ki_x'

  3.0,

  // Expression: 2
  //  Referenced by: '<S37>/ki_y'

  3.0,

  // Expression: 2
  //  Referenced by: '<S37>/ki_z'

  3.0,

  // Expression: 10
  //  Referenced by: '<S54>/omega_w'

  10.0,

  // Expression: 0
  //  Referenced by: '<S54>/Integrator1'

  0.0,

  // Expression: 1.414
  //  Referenced by: '<S54>/epsilon_w'

  1.414,

  // Expression: 24
  //  Referenced by: '<S53>/omega_v'

  24.0,

  // Expression: 0
  //  Referenced by: '<S53>/Integrator1'

  0.0,

  // Expression: 1.414
  //  Referenced by: '<S53>/epsilon_v'

  1.414,

  // Expression: 15
  //  Referenced by: '<S52>/omega_u'

  15.0,

  // Expression: 0
  //  Referenced by: '<S52>/Integrator1'

  0.0,

  // Expression: 1.414
  //  Referenced by: '<S52>/epsilon_u'

  1.414,

  // Expression: 20
  //  Referenced by: '<S51>/omega_r'

  20.0,

  // Expression: 0
  //  Referenced by: '<S51>/Integrator1'

  0.0,

  // Expression: 1.414
  //  Referenced by: '<S51>/epsilon_r'

  1.414,

  // Expression: 20
  //  Referenced by: '<S50>/omega_q'

  20.0,

  // Expression: 0
  //  Referenced by: '<S50>/Integrator1'

  0.0,

  // Expression: 1.414
  //  Referenced by: '<S50>/epsilon_q'

  1.414,

  // Expression: 24
  //  Referenced by: '<S49>/omega_p'

  24.0,

  // Expression: 0
  //  Referenced by: '<S49>/Integrator1'

  0.0,

  // Expression: 1.414
  //  Referenced by: '<S49>/epsilon_p'

  1.414,

  // Expression: 5
  //  Referenced by: '<S71>/omega_z'

  5.0,

  // Expression: 0
  //  Referenced by: '<S71>/Integrator1'

  0.0,

  // Expression: 1.414
  //  Referenced by: '<S71>/epsilon_z'

  1.414,

  // Expression: 8
  //  Referenced by: '<S70>/omega_y'

  8.0,

  // Expression: 0
  //  Referenced by: '<S70>/Integrator1'

  0.0,

  // Expression: 1.414
  //  Referenced by: '<S70>/epsilon_y'

  1.414,

  // Expression: 5
  //  Referenced by: '<S69>/omega_x'

  5.0,

  // Expression: 0
  //  Referenced by: '<S69>/Integrator1'

  0.0,

  // Expression: 1.414
  //  Referenced by: '<S69>/epsilon_x'

  1.414,

  // Expression: 10
  //  Referenced by: '<S57>/omega_psi'

  10.0,

  // Expression: 0
  //  Referenced by: '<S57>/Integrator1'

  0.0,

  // Expression: 1.414
  //  Referenced by: '<S57>/epsilon_psi'

  1.414,

  // Expression: 6.9
  //  Referenced by: '<S58>/omega_theta'

  6.9,

  // Expression: 0
  //  Referenced by: '<S58>/Integrator1'

  0.0,

  // Expression: 1.414
  //  Referenced by: '<S58>/epsilon_theta'

  1.414,

  // Expression: 8
  //  Referenced by: '<S56>/omega_phi'

  8.0,

  // Expression: 0
  //  Referenced by: '<S56>/Integrator1'

  0.0,

  // Expression: 1.414
  //  Referenced by: '<S56>/epsilon_phi'

  1.414
};

//
// File trailer for generated code.
//
// [EOF]
//
