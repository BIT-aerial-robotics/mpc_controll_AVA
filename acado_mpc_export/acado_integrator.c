/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"


real_t rk_dim24_swap;

/** Column vector of size: 24 */
real_t rk_dim24_bPerm[ 24 ];

/** Column vector of size: 217 */
real_t auxVar[ 217 ];

real_t rk_ttt;

/** Row vector of size: 18 */
real_t rk_xxx[ 18 ];

/** Matrix of size: 12 x 2 (row major format) */
real_t rk_kkk[ 24 ];

/** Matrix of size: 24 x 24 (row major format) */
real_t rk_A[ 576 ];

/** Column vector of size: 24 */
real_t rk_b[ 24 ];

/** Row vector of size: 24 */
int rk_dim24_perm[ 24 ];

/** Column vector of size: 12 */
real_t rk_rhsTemp[ 12 ];

/** Matrix of size: 2 x 216 (row major format) */
real_t rk_diffsTemp2[ 432 ];

/** Matrix of size: 12 x 2 (row major format) */
real_t rk_diffK[ 24 ];

/** Matrix of size: 12 x 18 (row major format) */
real_t rk_diffsNew2[ 216 ];

#pragma omp threadprivate( auxVar, rk_ttt, rk_xxx, rk_kkk, rk_diffK, rk_rhsTemp, rk_dim24_perm, rk_A, rk_b, rk_diffsNew2, rk_diffsTemp2, rk_dim24_swap, rk_dim24_bPerm )

void acado_rhs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 12;
/* Vector of auxiliary variables; number of elements: 53. */
real_t* a = auxVar;

/* Compute intermediate quantities: */
a[0] = (cos(xd[4]));
a[1] = (cos(xd[5]));
a[2] = (a[0]*a[1]);
a[3] = (sin(xd[3]));
a[4] = (sin(xd[4]));
a[5] = (cos(xd[5]));
a[6] = (cos(xd[3]));
a[7] = (sin(xd[5]));
a[8] = (((a[3]*a[4])*a[5])-(a[6]*a[7]));
a[9] = (cos(xd[3]));
a[10] = (sin(xd[4]));
a[11] = (cos(xd[5]));
a[12] = (sin(xd[3]));
a[13] = (sin(xd[5]));
a[14] = (((a[9]*a[10])*a[11])+(a[12]*a[13]));
a[15] = ((real_t)(1.5647003598810827e-01)*(((a[2]*u[0])+(a[8]*u[1]))+(a[14]*u[2])));
a[16] = (cos(xd[4]));
a[17] = (sin(xd[5]));
a[18] = (a[16]*a[17]);
a[19] = (sin(xd[3]));
a[20] = (sin(xd[4]));
a[21] = (sin(xd[5]));
a[22] = (cos(xd[3]));
a[23] = (cos(xd[5]));
a[24] = (((a[19]*a[20])*a[21])+(a[22]*a[23]));
a[25] = (cos(xd[3]));
a[26] = (sin(xd[4]));
a[27] = (sin(xd[5]));
a[28] = (sin(xd[3]));
a[29] = (cos(xd[5]));
a[30] = (((a[25]*a[26])*a[27])-(a[28]*a[29]));
a[31] = ((real_t)(1.5647003598810827e-01)*(((a[18]*u[0])+(a[24]*u[1]))+(a[30]*u[2])));
a[32] = (sin(xd[4]));
a[33] = ((real_t)(0.0000000000000000e+00)-a[32]);
a[34] = (sin(xd[3]));
a[35] = (cos(xd[4]));
a[36] = (a[34]*a[35]);
a[37] = (cos(xd[3]));
a[38] = (cos(xd[4]));
a[39] = (a[37]*a[38]);
a[40] = (((real_t)(1.5647003598810827e-01)*(((a[33]*u[0])+(a[36]*u[1]))+(a[39]*u[2])))-(real_t)(9.8000000000000007e+00));
a[41] = ((real_t)(1.3606214523315430e+00)*xd[11]);
a[42] = ((real_t)(0.0000000000000000e+00)-a[41]);
a[43] = ((real_t)(7.1065181493759155e-01)*xd[10]);
a[44] = ((a[42]*xd[10])+(a[43]*xd[11]));
a[45] = ((real_t)(1.4881833791732788e+00)*(u[3]+a[44]));
a[46] = ((real_t)(6.7196023464202881e-01)*xd[9]);
a[47] = ((real_t)(0.0000000000000000e+00)-a[46]);
a[48] = ((a[41]*xd[9])+(a[47]*xd[11]));
a[49] = ((real_t)(1.4071588516235352e+00)*(u[4]+a[48]));
a[50] = ((real_t)(0.0000000000000000e+00)-a[43]);
a[51] = ((a[50]*xd[9])+(a[46]*xd[10]));
a[52] = ((real_t)(7.3495829105377197e-01)*(u[5]+a[51]));

/* Compute outputs: */
out[0] = xd[6];
out[1] = xd[7];
out[2] = xd[8];
out[3] = xd[9];
out[4] = xd[10];
out[5] = xd[11];
out[6] = a[15];
out[7] = a[31];
out[8] = a[40];
out[9] = a[45];
out[10] = a[49];
out[11] = a[52];
}



void acado_diffs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 12;
/* Vector of auxiliary variables; number of elements: 217. */
real_t* a = auxVar;

/* Compute intermediate quantities: */
a[0] = (real_t)(0.0000000000000000e+00);
a[1] = (real_t)(0.0000000000000000e+00);
a[2] = (real_t)(0.0000000000000000e+00);
a[3] = (cos(xd[3]));
a[4] = (sin(xd[4]));
a[5] = (cos(xd[5]));
a[6] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[3])));
a[7] = (sin(xd[5]));
a[8] = (((a[3]*a[4])*a[5])-(a[6]*a[7]));
a[9] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[3])));
a[10] = (sin(xd[4]));
a[11] = (cos(xd[5]));
a[12] = (cos(xd[3]));
a[13] = (sin(xd[5]));
a[14] = (((a[9]*a[10])*a[11])+(a[12]*a[13]));
a[15] = ((real_t)(1.5647003598810827e-01)*((a[8]*u[1])+(a[14]*u[2])));
a[16] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[4])));
a[17] = (cos(xd[5]));
a[18] = (a[16]*a[17]);
a[19] = (sin(xd[3]));
a[20] = (cos(xd[4]));
a[21] = ((a[19]*a[20])*a[5]);
a[22] = (cos(xd[3]));
a[23] = (cos(xd[4]));
a[24] = ((a[22]*a[23])*a[11]);
a[25] = ((real_t)(1.5647003598810827e-01)*(((a[18]*u[0])+(a[21]*u[1]))+(a[24]*u[2])));
a[26] = (cos(xd[4]));
a[27] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[5])));
a[28] = (a[26]*a[27]);
a[29] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[5])));
a[30] = (cos(xd[3]));
a[31] = (cos(xd[5]));
a[32] = (((a[19]*a[4])*a[29])-(a[30]*a[31]));
a[33] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[5])));
a[34] = (sin(xd[3]));
a[35] = (cos(xd[5]));
a[36] = (((a[22]*a[10])*a[33])+(a[34]*a[35]));
a[37] = ((real_t)(1.5647003598810827e-01)*(((a[28]*u[0])+(a[32]*u[1]))+(a[36]*u[2])));
a[38] = (real_t)(0.0000000000000000e+00);
a[39] = (real_t)(0.0000000000000000e+00);
a[40] = (real_t)(0.0000000000000000e+00);
a[41] = (real_t)(0.0000000000000000e+00);
a[42] = (real_t)(0.0000000000000000e+00);
a[43] = (real_t)(0.0000000000000000e+00);
a[44] = (a[26]*a[17]);
a[45] = ((real_t)(1.5647003598810827e-01)*a[44]);
a[46] = (((a[19]*a[4])*a[5])-(a[30]*a[7]));
a[47] = ((real_t)(1.5647003598810827e-01)*a[46]);
a[48] = (((a[22]*a[10])*a[11])+(a[34]*a[13]));
a[49] = ((real_t)(1.5647003598810827e-01)*a[48]);
a[50] = (real_t)(0.0000000000000000e+00);
a[51] = (real_t)(0.0000000000000000e+00);
a[52] = (real_t)(0.0000000000000000e+00);
a[53] = (real_t)(0.0000000000000000e+00);
a[54] = (real_t)(0.0000000000000000e+00);
a[55] = (real_t)(0.0000000000000000e+00);
a[56] = (cos(xd[3]));
a[57] = (sin(xd[4]));
a[58] = (sin(xd[5]));
a[59] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[3])));
a[60] = (cos(xd[5]));
a[61] = (((a[56]*a[57])*a[58])+(a[59]*a[60]));
a[62] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[3])));
a[63] = (sin(xd[4]));
a[64] = (sin(xd[5]));
a[65] = (cos(xd[3]));
a[66] = (cos(xd[5]));
a[67] = (((a[62]*a[63])*a[64])-(a[65]*a[66]));
a[68] = ((real_t)(1.5647003598810827e-01)*((a[61]*u[1])+(a[67]*u[2])));
a[69] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[4])));
a[70] = (sin(xd[5]));
a[71] = (a[69]*a[70]);
a[72] = (sin(xd[3]));
a[73] = (cos(xd[4]));
a[74] = ((a[72]*a[73])*a[58]);
a[75] = (cos(xd[3]));
a[76] = (cos(xd[4]));
a[77] = ((a[75]*a[76])*a[64]);
a[78] = ((real_t)(1.5647003598810827e-01)*(((a[71]*u[0])+(a[74]*u[1]))+(a[77]*u[2])));
a[79] = (cos(xd[4]));
a[80] = (cos(xd[5]));
a[81] = (a[79]*a[80]);
a[82] = (cos(xd[5]));
a[83] = (cos(xd[3]));
a[84] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[5])));
a[85] = (((a[72]*a[57])*a[82])+(a[83]*a[84]));
a[86] = (cos(xd[5]));
a[87] = (sin(xd[3]));
a[88] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[5])));
a[89] = (((a[75]*a[63])*a[86])-(a[87]*a[88]));
a[90] = ((real_t)(1.5647003598810827e-01)*(((a[81]*u[0])+(a[85]*u[1]))+(a[89]*u[2])));
a[91] = (real_t)(0.0000000000000000e+00);
a[92] = (real_t)(0.0000000000000000e+00);
a[93] = (real_t)(0.0000000000000000e+00);
a[94] = (real_t)(0.0000000000000000e+00);
a[95] = (real_t)(0.0000000000000000e+00);
a[96] = (real_t)(0.0000000000000000e+00);
a[97] = (a[79]*a[70]);
a[98] = ((real_t)(1.5647003598810827e-01)*a[97]);
a[99] = (((a[72]*a[57])*a[58])+(a[83]*a[60]));
a[100] = ((real_t)(1.5647003598810827e-01)*a[99]);
a[101] = (((a[75]*a[63])*a[64])-(a[87]*a[66]));
a[102] = ((real_t)(1.5647003598810827e-01)*a[101]);
a[103] = (real_t)(0.0000000000000000e+00);
a[104] = (real_t)(0.0000000000000000e+00);
a[105] = (real_t)(0.0000000000000000e+00);
a[106] = (real_t)(0.0000000000000000e+00);
a[107] = (real_t)(0.0000000000000000e+00);
a[108] = (real_t)(0.0000000000000000e+00);
a[109] = (cos(xd[3]));
a[110] = (cos(xd[4]));
a[111] = (a[109]*a[110]);
a[112] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[3])));
a[113] = (cos(xd[4]));
a[114] = (a[112]*a[113]);
a[115] = ((real_t)(1.5647003598810827e-01)*((a[111]*u[1])+(a[114]*u[2])));
a[116] = (cos(xd[4]));
a[117] = ((real_t)(0.0000000000000000e+00)-a[116]);
a[118] = (sin(xd[3]));
a[119] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[4])));
a[120] = (a[118]*a[119]);
a[121] = (cos(xd[3]));
a[122] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[4])));
a[123] = (a[121]*a[122]);
a[124] = ((real_t)(1.5647003598810827e-01)*(((a[117]*u[0])+(a[120]*u[1]))+(a[123]*u[2])));
a[125] = (real_t)(0.0000000000000000e+00);
a[126] = (real_t)(0.0000000000000000e+00);
a[127] = (real_t)(0.0000000000000000e+00);
a[128] = (real_t)(0.0000000000000000e+00);
a[129] = (real_t)(0.0000000000000000e+00);
a[130] = (real_t)(0.0000000000000000e+00);
a[131] = (real_t)(0.0000000000000000e+00);
a[132] = (sin(xd[4]));
a[133] = ((real_t)(0.0000000000000000e+00)-a[132]);
a[134] = ((real_t)(1.5647003598810827e-01)*a[133]);
a[135] = (a[118]*a[110]);
a[136] = ((real_t)(1.5647003598810827e-01)*a[135]);
a[137] = (a[121]*a[113]);
a[138] = ((real_t)(1.5647003598810827e-01)*a[137]);
a[139] = (real_t)(0.0000000000000000e+00);
a[140] = (real_t)(0.0000000000000000e+00);
a[141] = (real_t)(0.0000000000000000e+00);
a[142] = (real_t)(0.0000000000000000e+00);
a[143] = (real_t)(0.0000000000000000e+00);
a[144] = (real_t)(0.0000000000000000e+00);
a[145] = (real_t)(0.0000000000000000e+00);
a[146] = (real_t)(0.0000000000000000e+00);
a[147] = (real_t)(0.0000000000000000e+00);
a[148] = (real_t)(0.0000000000000000e+00);
a[149] = (real_t)(0.0000000000000000e+00);
a[150] = (real_t)(0.0000000000000000e+00);
a[151] = (real_t)(0.0000000000000000e+00);
a[152] = ((real_t)(1.3606214523315430e+00)*xd[11]);
a[153] = ((real_t)(0.0000000000000000e+00)-a[152]);
a[154] = (real_t)(7.1065181493759155e-01);
a[155] = (a[153]+(a[154]*xd[11]));
a[156] = ((real_t)(1.4881833791732788e+00)*a[155]);
a[157] = (real_t)(1.3606214523315430e+00);
a[158] = ((real_t)(0.0000000000000000e+00)-a[157]);
a[159] = ((real_t)(7.1065181493759155e-01)*xd[10]);
a[160] = ((a[158]*xd[10])+a[159]);
a[161] = ((real_t)(1.4881833791732788e+00)*a[160]);
a[162] = (real_t)(0.0000000000000000e+00);
a[163] = (real_t)(0.0000000000000000e+00);
a[164] = (real_t)(0.0000000000000000e+00);
a[165] = (real_t)(1.4881833791732788e+00);
a[166] = (real_t)(0.0000000000000000e+00);
a[167] = (real_t)(0.0000000000000000e+00);
a[168] = (real_t)(0.0000000000000000e+00);
a[169] = (real_t)(0.0000000000000000e+00);
a[170] = (real_t)(0.0000000000000000e+00);
a[171] = (real_t)(0.0000000000000000e+00);
a[172] = (real_t)(0.0000000000000000e+00);
a[173] = (real_t)(0.0000000000000000e+00);
a[174] = (real_t)(0.0000000000000000e+00);
a[175] = (real_t)(0.0000000000000000e+00);
a[176] = (real_t)(0.0000000000000000e+00);
a[177] = (real_t)(6.7196023464202881e-01);
a[178] = ((real_t)(0.0000000000000000e+00)-a[177]);
a[179] = (a[152]+(a[178]*xd[11]));
a[180] = ((real_t)(1.4071588516235352e+00)*a[179]);
a[181] = (real_t)(0.0000000000000000e+00);
a[182] = (real_t)(1.3606214523315430e+00);
a[183] = ((real_t)(6.7196023464202881e-01)*xd[9]);
a[184] = ((real_t)(0.0000000000000000e+00)-a[183]);
a[185] = ((a[182]*xd[9])+a[184]);
a[186] = ((real_t)(1.4071588516235352e+00)*a[185]);
a[187] = (real_t)(0.0000000000000000e+00);
a[188] = (real_t)(0.0000000000000000e+00);
a[189] = (real_t)(0.0000000000000000e+00);
a[190] = (real_t)(0.0000000000000000e+00);
a[191] = (real_t)(1.4071588516235352e+00);
a[192] = (real_t)(0.0000000000000000e+00);
a[193] = (real_t)(0.0000000000000000e+00);
a[194] = (real_t)(0.0000000000000000e+00);
a[195] = (real_t)(0.0000000000000000e+00);
a[196] = (real_t)(0.0000000000000000e+00);
a[197] = (real_t)(0.0000000000000000e+00);
a[198] = (real_t)(0.0000000000000000e+00);
a[199] = (real_t)(0.0000000000000000e+00);
a[200] = (real_t)(0.0000000000000000e+00);
a[201] = (real_t)(0.0000000000000000e+00);
a[202] = ((real_t)(0.0000000000000000e+00)-a[159]);
a[203] = (real_t)(6.7196023464202881e-01);
a[204] = (a[202]+(a[203]*xd[10]));
a[205] = ((real_t)(7.3495829105377197e-01)*a[204]);
a[206] = (real_t)(7.1065181493759155e-01);
a[207] = ((real_t)(0.0000000000000000e+00)-a[206]);
a[208] = ((a[207]*xd[9])+a[183]);
a[209] = ((real_t)(7.3495829105377197e-01)*a[208]);
a[210] = (real_t)(0.0000000000000000e+00);
a[211] = (real_t)(0.0000000000000000e+00);
a[212] = (real_t)(0.0000000000000000e+00);
a[213] = (real_t)(0.0000000000000000e+00);
a[214] = (real_t)(0.0000000000000000e+00);
a[215] = (real_t)(0.0000000000000000e+00);
a[216] = (real_t)(7.3495829105377197e-01);

/* Compute outputs: */
out[0] = (real_t)(0.0000000000000000e+00);
out[1] = (real_t)(0.0000000000000000e+00);
out[2] = (real_t)(0.0000000000000000e+00);
out[3] = (real_t)(0.0000000000000000e+00);
out[4] = (real_t)(0.0000000000000000e+00);
out[5] = (real_t)(0.0000000000000000e+00);
out[6] = (real_t)(1.0000000000000000e+00);
out[7] = (real_t)(0.0000000000000000e+00);
out[8] = (real_t)(0.0000000000000000e+00);
out[9] = (real_t)(0.0000000000000000e+00);
out[10] = (real_t)(0.0000000000000000e+00);
out[11] = (real_t)(0.0000000000000000e+00);
out[12] = (real_t)(0.0000000000000000e+00);
out[13] = (real_t)(0.0000000000000000e+00);
out[14] = (real_t)(0.0000000000000000e+00);
out[15] = (real_t)(0.0000000000000000e+00);
out[16] = (real_t)(0.0000000000000000e+00);
out[17] = (real_t)(0.0000000000000000e+00);
out[18] = (real_t)(0.0000000000000000e+00);
out[19] = (real_t)(0.0000000000000000e+00);
out[20] = (real_t)(0.0000000000000000e+00);
out[21] = (real_t)(0.0000000000000000e+00);
out[22] = (real_t)(0.0000000000000000e+00);
out[23] = (real_t)(0.0000000000000000e+00);
out[24] = (real_t)(0.0000000000000000e+00);
out[25] = (real_t)(1.0000000000000000e+00);
out[26] = (real_t)(0.0000000000000000e+00);
out[27] = (real_t)(0.0000000000000000e+00);
out[28] = (real_t)(0.0000000000000000e+00);
out[29] = (real_t)(0.0000000000000000e+00);
out[30] = (real_t)(0.0000000000000000e+00);
out[31] = (real_t)(0.0000000000000000e+00);
out[32] = (real_t)(0.0000000000000000e+00);
out[33] = (real_t)(0.0000000000000000e+00);
out[34] = (real_t)(0.0000000000000000e+00);
out[35] = (real_t)(0.0000000000000000e+00);
out[36] = (real_t)(0.0000000000000000e+00);
out[37] = (real_t)(0.0000000000000000e+00);
out[38] = (real_t)(0.0000000000000000e+00);
out[39] = (real_t)(0.0000000000000000e+00);
out[40] = (real_t)(0.0000000000000000e+00);
out[41] = (real_t)(0.0000000000000000e+00);
out[42] = (real_t)(0.0000000000000000e+00);
out[43] = (real_t)(0.0000000000000000e+00);
out[44] = (real_t)(1.0000000000000000e+00);
out[45] = (real_t)(0.0000000000000000e+00);
out[46] = (real_t)(0.0000000000000000e+00);
out[47] = (real_t)(0.0000000000000000e+00);
out[48] = (real_t)(0.0000000000000000e+00);
out[49] = (real_t)(0.0000000000000000e+00);
out[50] = (real_t)(0.0000000000000000e+00);
out[51] = (real_t)(0.0000000000000000e+00);
out[52] = (real_t)(0.0000000000000000e+00);
out[53] = (real_t)(0.0000000000000000e+00);
out[54] = (real_t)(0.0000000000000000e+00);
out[55] = (real_t)(0.0000000000000000e+00);
out[56] = (real_t)(0.0000000000000000e+00);
out[57] = (real_t)(0.0000000000000000e+00);
out[58] = (real_t)(0.0000000000000000e+00);
out[59] = (real_t)(0.0000000000000000e+00);
out[60] = (real_t)(0.0000000000000000e+00);
out[61] = (real_t)(0.0000000000000000e+00);
out[62] = (real_t)(0.0000000000000000e+00);
out[63] = (real_t)(1.0000000000000000e+00);
out[64] = (real_t)(0.0000000000000000e+00);
out[65] = (real_t)(0.0000000000000000e+00);
out[66] = (real_t)(0.0000000000000000e+00);
out[67] = (real_t)(0.0000000000000000e+00);
out[68] = (real_t)(0.0000000000000000e+00);
out[69] = (real_t)(0.0000000000000000e+00);
out[70] = (real_t)(0.0000000000000000e+00);
out[71] = (real_t)(0.0000000000000000e+00);
out[72] = (real_t)(0.0000000000000000e+00);
out[73] = (real_t)(0.0000000000000000e+00);
out[74] = (real_t)(0.0000000000000000e+00);
out[75] = (real_t)(0.0000000000000000e+00);
out[76] = (real_t)(0.0000000000000000e+00);
out[77] = (real_t)(0.0000000000000000e+00);
out[78] = (real_t)(0.0000000000000000e+00);
out[79] = (real_t)(0.0000000000000000e+00);
out[80] = (real_t)(0.0000000000000000e+00);
out[81] = (real_t)(0.0000000000000000e+00);
out[82] = (real_t)(1.0000000000000000e+00);
out[83] = (real_t)(0.0000000000000000e+00);
out[84] = (real_t)(0.0000000000000000e+00);
out[85] = (real_t)(0.0000000000000000e+00);
out[86] = (real_t)(0.0000000000000000e+00);
out[87] = (real_t)(0.0000000000000000e+00);
out[88] = (real_t)(0.0000000000000000e+00);
out[89] = (real_t)(0.0000000000000000e+00);
out[90] = (real_t)(0.0000000000000000e+00);
out[91] = (real_t)(0.0000000000000000e+00);
out[92] = (real_t)(0.0000000000000000e+00);
out[93] = (real_t)(0.0000000000000000e+00);
out[94] = (real_t)(0.0000000000000000e+00);
out[95] = (real_t)(0.0000000000000000e+00);
out[96] = (real_t)(0.0000000000000000e+00);
out[97] = (real_t)(0.0000000000000000e+00);
out[98] = (real_t)(0.0000000000000000e+00);
out[99] = (real_t)(0.0000000000000000e+00);
out[100] = (real_t)(0.0000000000000000e+00);
out[101] = (real_t)(1.0000000000000000e+00);
out[102] = (real_t)(0.0000000000000000e+00);
out[103] = (real_t)(0.0000000000000000e+00);
out[104] = (real_t)(0.0000000000000000e+00);
out[105] = (real_t)(0.0000000000000000e+00);
out[106] = (real_t)(0.0000000000000000e+00);
out[107] = (real_t)(0.0000000000000000e+00);
out[108] = a[0];
out[109] = a[1];
out[110] = a[2];
out[111] = a[15];
out[112] = a[25];
out[113] = a[37];
out[114] = a[38];
out[115] = a[39];
out[116] = a[40];
out[117] = a[41];
out[118] = a[42];
out[119] = a[43];
out[120] = a[45];
out[121] = a[47];
out[122] = a[49];
out[123] = a[50];
out[124] = a[51];
out[125] = a[52];
out[126] = a[53];
out[127] = a[54];
out[128] = a[55];
out[129] = a[68];
out[130] = a[78];
out[131] = a[90];
out[132] = a[91];
out[133] = a[92];
out[134] = a[93];
out[135] = a[94];
out[136] = a[95];
out[137] = a[96];
out[138] = a[98];
out[139] = a[100];
out[140] = a[102];
out[141] = a[103];
out[142] = a[104];
out[143] = a[105];
out[144] = a[106];
out[145] = a[107];
out[146] = a[108];
out[147] = a[115];
out[148] = a[124];
out[149] = a[125];
out[150] = a[126];
out[151] = a[127];
out[152] = a[128];
out[153] = a[129];
out[154] = a[130];
out[155] = a[131];
out[156] = a[134];
out[157] = a[136];
out[158] = a[138];
out[159] = a[139];
out[160] = a[140];
out[161] = a[141];
out[162] = a[142];
out[163] = a[143];
out[164] = a[144];
out[165] = a[145];
out[166] = a[146];
out[167] = a[147];
out[168] = a[148];
out[169] = a[149];
out[170] = a[150];
out[171] = a[151];
out[172] = a[156];
out[173] = a[161];
out[174] = a[162];
out[175] = a[163];
out[176] = a[164];
out[177] = a[165];
out[178] = a[166];
out[179] = a[167];
out[180] = a[168];
out[181] = a[169];
out[182] = a[170];
out[183] = a[171];
out[184] = a[172];
out[185] = a[173];
out[186] = a[174];
out[187] = a[175];
out[188] = a[176];
out[189] = a[180];
out[190] = a[181];
out[191] = a[186];
out[192] = a[187];
out[193] = a[188];
out[194] = a[189];
out[195] = a[190];
out[196] = a[191];
out[197] = a[192];
out[198] = a[193];
out[199] = a[194];
out[200] = a[195];
out[201] = a[196];
out[202] = a[197];
out[203] = a[198];
out[204] = a[199];
out[205] = a[200];
out[206] = a[201];
out[207] = a[205];
out[208] = a[209];
out[209] = a[210];
out[210] = a[211];
out[211] = a[212];
out[212] = a[213];
out[213] = a[214];
out[214] = a[215];
out[215] = a[216];
}



void acado_solve_dim24_triangular( real_t* const A, real_t* const b )
{

b[23] = b[23]/A[575];
b[22] -= + A[551]*b[23];
b[22] = b[22]/A[550];
b[21] -= + A[527]*b[23];
b[21] -= + A[526]*b[22];
b[21] = b[21]/A[525];
b[20] -= + A[503]*b[23];
b[20] -= + A[502]*b[22];
b[20] -= + A[501]*b[21];
b[20] = b[20]/A[500];
b[19] -= + A[479]*b[23];
b[19] -= + A[478]*b[22];
b[19] -= + A[477]*b[21];
b[19] -= + A[476]*b[20];
b[19] = b[19]/A[475];
b[18] -= + A[455]*b[23];
b[18] -= + A[454]*b[22];
b[18] -= + A[453]*b[21];
b[18] -= + A[452]*b[20];
b[18] -= + A[451]*b[19];
b[18] = b[18]/A[450];
b[17] -= + A[431]*b[23];
b[17] -= + A[430]*b[22];
b[17] -= + A[429]*b[21];
b[17] -= + A[428]*b[20];
b[17] -= + A[427]*b[19];
b[17] -= + A[426]*b[18];
b[17] = b[17]/A[425];
b[16] -= + A[407]*b[23];
b[16] -= + A[406]*b[22];
b[16] -= + A[405]*b[21];
b[16] -= + A[404]*b[20];
b[16] -= + A[403]*b[19];
b[16] -= + A[402]*b[18];
b[16] -= + A[401]*b[17];
b[16] = b[16]/A[400];
b[15] -= + A[383]*b[23];
b[15] -= + A[382]*b[22];
b[15] -= + A[381]*b[21];
b[15] -= + A[380]*b[20];
b[15] -= + A[379]*b[19];
b[15] -= + A[378]*b[18];
b[15] -= + A[377]*b[17];
b[15] -= + A[376]*b[16];
b[15] = b[15]/A[375];
b[14] -= + A[359]*b[23];
b[14] -= + A[358]*b[22];
b[14] -= + A[357]*b[21];
b[14] -= + A[356]*b[20];
b[14] -= + A[355]*b[19];
b[14] -= + A[354]*b[18];
b[14] -= + A[353]*b[17];
b[14] -= + A[352]*b[16];
b[14] -= + A[351]*b[15];
b[14] = b[14]/A[350];
b[13] -= + A[335]*b[23];
b[13] -= + A[334]*b[22];
b[13] -= + A[333]*b[21];
b[13] -= + A[332]*b[20];
b[13] -= + A[331]*b[19];
b[13] -= + A[330]*b[18];
b[13] -= + A[329]*b[17];
b[13] -= + A[328]*b[16];
b[13] -= + A[327]*b[15];
b[13] -= + A[326]*b[14];
b[13] = b[13]/A[325];
b[12] -= + A[311]*b[23];
b[12] -= + A[310]*b[22];
b[12] -= + A[309]*b[21];
b[12] -= + A[308]*b[20];
b[12] -= + A[307]*b[19];
b[12] -= + A[306]*b[18];
b[12] -= + A[305]*b[17];
b[12] -= + A[304]*b[16];
b[12] -= + A[303]*b[15];
b[12] -= + A[302]*b[14];
b[12] -= + A[301]*b[13];
b[12] = b[12]/A[300];
b[11] -= + A[287]*b[23];
b[11] -= + A[286]*b[22];
b[11] -= + A[285]*b[21];
b[11] -= + A[284]*b[20];
b[11] -= + A[283]*b[19];
b[11] -= + A[282]*b[18];
b[11] -= + A[281]*b[17];
b[11] -= + A[280]*b[16];
b[11] -= + A[279]*b[15];
b[11] -= + A[278]*b[14];
b[11] -= + A[277]*b[13];
b[11] -= + A[276]*b[12];
b[11] = b[11]/A[275];
b[10] -= + A[263]*b[23];
b[10] -= + A[262]*b[22];
b[10] -= + A[261]*b[21];
b[10] -= + A[260]*b[20];
b[10] -= + A[259]*b[19];
b[10] -= + A[258]*b[18];
b[10] -= + A[257]*b[17];
b[10] -= + A[256]*b[16];
b[10] -= + A[255]*b[15];
b[10] -= + A[254]*b[14];
b[10] -= + A[253]*b[13];
b[10] -= + A[252]*b[12];
b[10] -= + A[251]*b[11];
b[10] = b[10]/A[250];
b[9] -= + A[239]*b[23];
b[9] -= + A[238]*b[22];
b[9] -= + A[237]*b[21];
b[9] -= + A[236]*b[20];
b[9] -= + A[235]*b[19];
b[9] -= + A[234]*b[18];
b[9] -= + A[233]*b[17];
b[9] -= + A[232]*b[16];
b[9] -= + A[231]*b[15];
b[9] -= + A[230]*b[14];
b[9] -= + A[229]*b[13];
b[9] -= + A[228]*b[12];
b[9] -= + A[227]*b[11];
b[9] -= + A[226]*b[10];
b[9] = b[9]/A[225];
b[8] -= + A[215]*b[23];
b[8] -= + A[214]*b[22];
b[8] -= + A[213]*b[21];
b[8] -= + A[212]*b[20];
b[8] -= + A[211]*b[19];
b[8] -= + A[210]*b[18];
b[8] -= + A[209]*b[17];
b[8] -= + A[208]*b[16];
b[8] -= + A[207]*b[15];
b[8] -= + A[206]*b[14];
b[8] -= + A[205]*b[13];
b[8] -= + A[204]*b[12];
b[8] -= + A[203]*b[11];
b[8] -= + A[202]*b[10];
b[8] -= + A[201]*b[9];
b[8] = b[8]/A[200];
b[7] -= + A[191]*b[23];
b[7] -= + A[190]*b[22];
b[7] -= + A[189]*b[21];
b[7] -= + A[188]*b[20];
b[7] -= + A[187]*b[19];
b[7] -= + A[186]*b[18];
b[7] -= + A[185]*b[17];
b[7] -= + A[184]*b[16];
b[7] -= + A[183]*b[15];
b[7] -= + A[182]*b[14];
b[7] -= + A[181]*b[13];
b[7] -= + A[180]*b[12];
b[7] -= + A[179]*b[11];
b[7] -= + A[178]*b[10];
b[7] -= + A[177]*b[9];
b[7] -= + A[176]*b[8];
b[7] = b[7]/A[175];
b[6] -= + A[167]*b[23];
b[6] -= + A[166]*b[22];
b[6] -= + A[165]*b[21];
b[6] -= + A[164]*b[20];
b[6] -= + A[163]*b[19];
b[6] -= + A[162]*b[18];
b[6] -= + A[161]*b[17];
b[6] -= + A[160]*b[16];
b[6] -= + A[159]*b[15];
b[6] -= + A[158]*b[14];
b[6] -= + A[157]*b[13];
b[6] -= + A[156]*b[12];
b[6] -= + A[155]*b[11];
b[6] -= + A[154]*b[10];
b[6] -= + A[153]*b[9];
b[6] -= + A[152]*b[8];
b[6] -= + A[151]*b[7];
b[6] = b[6]/A[150];
b[5] -= + A[143]*b[23];
b[5] -= + A[142]*b[22];
b[5] -= + A[141]*b[21];
b[5] -= + A[140]*b[20];
b[5] -= + A[139]*b[19];
b[5] -= + A[138]*b[18];
b[5] -= + A[137]*b[17];
b[5] -= + A[136]*b[16];
b[5] -= + A[135]*b[15];
b[5] -= + A[134]*b[14];
b[5] -= + A[133]*b[13];
b[5] -= + A[132]*b[12];
b[5] -= + A[131]*b[11];
b[5] -= + A[130]*b[10];
b[5] -= + A[129]*b[9];
b[5] -= + A[128]*b[8];
b[5] -= + A[127]*b[7];
b[5] -= + A[126]*b[6];
b[5] = b[5]/A[125];
b[4] -= + A[119]*b[23];
b[4] -= + A[118]*b[22];
b[4] -= + A[117]*b[21];
b[4] -= + A[116]*b[20];
b[4] -= + A[115]*b[19];
b[4] -= + A[114]*b[18];
b[4] -= + A[113]*b[17];
b[4] -= + A[112]*b[16];
b[4] -= + A[111]*b[15];
b[4] -= + A[110]*b[14];
b[4] -= + A[109]*b[13];
b[4] -= + A[108]*b[12];
b[4] -= + A[107]*b[11];
b[4] -= + A[106]*b[10];
b[4] -= + A[105]*b[9];
b[4] -= + A[104]*b[8];
b[4] -= + A[103]*b[7];
b[4] -= + A[102]*b[6];
b[4] -= + A[101]*b[5];
b[4] = b[4]/A[100];
b[3] -= + A[95]*b[23];
b[3] -= + A[94]*b[22];
b[3] -= + A[93]*b[21];
b[3] -= + A[92]*b[20];
b[3] -= + A[91]*b[19];
b[3] -= + A[90]*b[18];
b[3] -= + A[89]*b[17];
b[3] -= + A[88]*b[16];
b[3] -= + A[87]*b[15];
b[3] -= + A[86]*b[14];
b[3] -= + A[85]*b[13];
b[3] -= + A[84]*b[12];
b[3] -= + A[83]*b[11];
b[3] -= + A[82]*b[10];
b[3] -= + A[81]*b[9];
b[3] -= + A[80]*b[8];
b[3] -= + A[79]*b[7];
b[3] -= + A[78]*b[6];
b[3] -= + A[77]*b[5];
b[3] -= + A[76]*b[4];
b[3] = b[3]/A[75];
b[2] -= + A[71]*b[23];
b[2] -= + A[70]*b[22];
b[2] -= + A[69]*b[21];
b[2] -= + A[68]*b[20];
b[2] -= + A[67]*b[19];
b[2] -= + A[66]*b[18];
b[2] -= + A[65]*b[17];
b[2] -= + A[64]*b[16];
b[2] -= + A[63]*b[15];
b[2] -= + A[62]*b[14];
b[2] -= + A[61]*b[13];
b[2] -= + A[60]*b[12];
b[2] -= + A[59]*b[11];
b[2] -= + A[58]*b[10];
b[2] -= + A[57]*b[9];
b[2] -= + A[56]*b[8];
b[2] -= + A[55]*b[7];
b[2] -= + A[54]*b[6];
b[2] -= + A[53]*b[5];
b[2] -= + A[52]*b[4];
b[2] -= + A[51]*b[3];
b[2] = b[2]/A[50];
b[1] -= + A[47]*b[23];
b[1] -= + A[46]*b[22];
b[1] -= + A[45]*b[21];
b[1] -= + A[44]*b[20];
b[1] -= + A[43]*b[19];
b[1] -= + A[42]*b[18];
b[1] -= + A[41]*b[17];
b[1] -= + A[40]*b[16];
b[1] -= + A[39]*b[15];
b[1] -= + A[38]*b[14];
b[1] -= + A[37]*b[13];
b[1] -= + A[36]*b[12];
b[1] -= + A[35]*b[11];
b[1] -= + A[34]*b[10];
b[1] -= + A[33]*b[9];
b[1] -= + A[32]*b[8];
b[1] -= + A[31]*b[7];
b[1] -= + A[30]*b[6];
b[1] -= + A[29]*b[5];
b[1] -= + A[28]*b[4];
b[1] -= + A[27]*b[3];
b[1] -= + A[26]*b[2];
b[1] = b[1]/A[25];
b[0] -= + A[23]*b[23];
b[0] -= + A[22]*b[22];
b[0] -= + A[21]*b[21];
b[0] -= + A[20]*b[20];
b[0] -= + A[19]*b[19];
b[0] -= + A[18]*b[18];
b[0] -= + A[17]*b[17];
b[0] -= + A[16]*b[16];
b[0] -= + A[15]*b[15];
b[0] -= + A[14]*b[14];
b[0] -= + A[13]*b[13];
b[0] -= + A[12]*b[12];
b[0] -= + A[11]*b[11];
b[0] -= + A[10]*b[10];
b[0] -= + A[9]*b[9];
b[0] -= + A[8]*b[8];
b[0] -= + A[7]*b[7];
b[0] -= + A[6]*b[6];
b[0] -= + A[5]*b[5];
b[0] -= + A[4]*b[4];
b[0] -= + A[3]*b[3];
b[0] -= + A[2]*b[2];
b[0] -= + A[1]*b[1];
b[0] = b[0]/A[0];
}

real_t acado_solve_dim24_system( real_t* const A, real_t* const b, int* const rk_perm )
{
real_t det;

int i;
int j;
int k;

int indexMax;

int intSwap;

real_t valueMax;

real_t temp;

for (i = 0; i < 24; ++i)
{
rk_perm[i] = i;
}
det = 1.0000000000000000e+00;
for( i=0; i < (23); i++ ) {
	indexMax = i;
	valueMax = fabs(A[i*24+i]);
	for( j=(i+1); j < 24; j++ ) {
		temp = fabs(A[j*24+i]);
		if( temp > valueMax ) {
			indexMax = j;
			valueMax = temp;
		}
	}
	if( indexMax > i ) {
for (k = 0; k < 24; ++k)
{
	rk_dim24_swap = A[i*24+k];
	A[i*24+k] = A[indexMax*24+k];
	A[indexMax*24+k] = rk_dim24_swap;
}
	rk_dim24_swap = b[i];
	b[i] = b[indexMax];
	b[indexMax] = rk_dim24_swap;
	intSwap = rk_perm[i];
	rk_perm[i] = rk_perm[indexMax];
	rk_perm[indexMax] = intSwap;
	}
	det *= A[i*24+i];
	for( j=i+1; j < 24; j++ ) {
		A[j*24+i] = -A[j*24+i]/A[i*24+i];
		for( k=i+1; k < 24; k++ ) {
			A[j*24+k] += A[j*24+i] * A[i*24+k];
		}
		b[j] += A[j*24+i] * b[i];
	}
}
det *= A[575];
det = fabs(det);
acado_solve_dim24_triangular( A, b );
return det;
}

void acado_solve_dim24_system_reuse( real_t* const A, real_t* const b, int* const rk_perm )
{

rk_dim24_bPerm[0] = b[rk_perm[0]];
rk_dim24_bPerm[1] = b[rk_perm[1]];
rk_dim24_bPerm[2] = b[rk_perm[2]];
rk_dim24_bPerm[3] = b[rk_perm[3]];
rk_dim24_bPerm[4] = b[rk_perm[4]];
rk_dim24_bPerm[5] = b[rk_perm[5]];
rk_dim24_bPerm[6] = b[rk_perm[6]];
rk_dim24_bPerm[7] = b[rk_perm[7]];
rk_dim24_bPerm[8] = b[rk_perm[8]];
rk_dim24_bPerm[9] = b[rk_perm[9]];
rk_dim24_bPerm[10] = b[rk_perm[10]];
rk_dim24_bPerm[11] = b[rk_perm[11]];
rk_dim24_bPerm[12] = b[rk_perm[12]];
rk_dim24_bPerm[13] = b[rk_perm[13]];
rk_dim24_bPerm[14] = b[rk_perm[14]];
rk_dim24_bPerm[15] = b[rk_perm[15]];
rk_dim24_bPerm[16] = b[rk_perm[16]];
rk_dim24_bPerm[17] = b[rk_perm[17]];
rk_dim24_bPerm[18] = b[rk_perm[18]];
rk_dim24_bPerm[19] = b[rk_perm[19]];
rk_dim24_bPerm[20] = b[rk_perm[20]];
rk_dim24_bPerm[21] = b[rk_perm[21]];
rk_dim24_bPerm[22] = b[rk_perm[22]];
rk_dim24_bPerm[23] = b[rk_perm[23]];
rk_dim24_bPerm[1] += A[24]*rk_dim24_bPerm[0];

rk_dim24_bPerm[2] += A[48]*rk_dim24_bPerm[0];
rk_dim24_bPerm[2] += A[49]*rk_dim24_bPerm[1];

rk_dim24_bPerm[3] += A[72]*rk_dim24_bPerm[0];
rk_dim24_bPerm[3] += A[73]*rk_dim24_bPerm[1];
rk_dim24_bPerm[3] += A[74]*rk_dim24_bPerm[2];

rk_dim24_bPerm[4] += A[96]*rk_dim24_bPerm[0];
rk_dim24_bPerm[4] += A[97]*rk_dim24_bPerm[1];
rk_dim24_bPerm[4] += A[98]*rk_dim24_bPerm[2];
rk_dim24_bPerm[4] += A[99]*rk_dim24_bPerm[3];

rk_dim24_bPerm[5] += A[120]*rk_dim24_bPerm[0];
rk_dim24_bPerm[5] += A[121]*rk_dim24_bPerm[1];
rk_dim24_bPerm[5] += A[122]*rk_dim24_bPerm[2];
rk_dim24_bPerm[5] += A[123]*rk_dim24_bPerm[3];
rk_dim24_bPerm[5] += A[124]*rk_dim24_bPerm[4];

rk_dim24_bPerm[6] += A[144]*rk_dim24_bPerm[0];
rk_dim24_bPerm[6] += A[145]*rk_dim24_bPerm[1];
rk_dim24_bPerm[6] += A[146]*rk_dim24_bPerm[2];
rk_dim24_bPerm[6] += A[147]*rk_dim24_bPerm[3];
rk_dim24_bPerm[6] += A[148]*rk_dim24_bPerm[4];
rk_dim24_bPerm[6] += A[149]*rk_dim24_bPerm[5];

rk_dim24_bPerm[7] += A[168]*rk_dim24_bPerm[0];
rk_dim24_bPerm[7] += A[169]*rk_dim24_bPerm[1];
rk_dim24_bPerm[7] += A[170]*rk_dim24_bPerm[2];
rk_dim24_bPerm[7] += A[171]*rk_dim24_bPerm[3];
rk_dim24_bPerm[7] += A[172]*rk_dim24_bPerm[4];
rk_dim24_bPerm[7] += A[173]*rk_dim24_bPerm[5];
rk_dim24_bPerm[7] += A[174]*rk_dim24_bPerm[6];

rk_dim24_bPerm[8] += A[192]*rk_dim24_bPerm[0];
rk_dim24_bPerm[8] += A[193]*rk_dim24_bPerm[1];
rk_dim24_bPerm[8] += A[194]*rk_dim24_bPerm[2];
rk_dim24_bPerm[8] += A[195]*rk_dim24_bPerm[3];
rk_dim24_bPerm[8] += A[196]*rk_dim24_bPerm[4];
rk_dim24_bPerm[8] += A[197]*rk_dim24_bPerm[5];
rk_dim24_bPerm[8] += A[198]*rk_dim24_bPerm[6];
rk_dim24_bPerm[8] += A[199]*rk_dim24_bPerm[7];

rk_dim24_bPerm[9] += A[216]*rk_dim24_bPerm[0];
rk_dim24_bPerm[9] += A[217]*rk_dim24_bPerm[1];
rk_dim24_bPerm[9] += A[218]*rk_dim24_bPerm[2];
rk_dim24_bPerm[9] += A[219]*rk_dim24_bPerm[3];
rk_dim24_bPerm[9] += A[220]*rk_dim24_bPerm[4];
rk_dim24_bPerm[9] += A[221]*rk_dim24_bPerm[5];
rk_dim24_bPerm[9] += A[222]*rk_dim24_bPerm[6];
rk_dim24_bPerm[9] += A[223]*rk_dim24_bPerm[7];
rk_dim24_bPerm[9] += A[224]*rk_dim24_bPerm[8];

rk_dim24_bPerm[10] += A[240]*rk_dim24_bPerm[0];
rk_dim24_bPerm[10] += A[241]*rk_dim24_bPerm[1];
rk_dim24_bPerm[10] += A[242]*rk_dim24_bPerm[2];
rk_dim24_bPerm[10] += A[243]*rk_dim24_bPerm[3];
rk_dim24_bPerm[10] += A[244]*rk_dim24_bPerm[4];
rk_dim24_bPerm[10] += A[245]*rk_dim24_bPerm[5];
rk_dim24_bPerm[10] += A[246]*rk_dim24_bPerm[6];
rk_dim24_bPerm[10] += A[247]*rk_dim24_bPerm[7];
rk_dim24_bPerm[10] += A[248]*rk_dim24_bPerm[8];
rk_dim24_bPerm[10] += A[249]*rk_dim24_bPerm[9];

rk_dim24_bPerm[11] += A[264]*rk_dim24_bPerm[0];
rk_dim24_bPerm[11] += A[265]*rk_dim24_bPerm[1];
rk_dim24_bPerm[11] += A[266]*rk_dim24_bPerm[2];
rk_dim24_bPerm[11] += A[267]*rk_dim24_bPerm[3];
rk_dim24_bPerm[11] += A[268]*rk_dim24_bPerm[4];
rk_dim24_bPerm[11] += A[269]*rk_dim24_bPerm[5];
rk_dim24_bPerm[11] += A[270]*rk_dim24_bPerm[6];
rk_dim24_bPerm[11] += A[271]*rk_dim24_bPerm[7];
rk_dim24_bPerm[11] += A[272]*rk_dim24_bPerm[8];
rk_dim24_bPerm[11] += A[273]*rk_dim24_bPerm[9];
rk_dim24_bPerm[11] += A[274]*rk_dim24_bPerm[10];

rk_dim24_bPerm[12] += A[288]*rk_dim24_bPerm[0];
rk_dim24_bPerm[12] += A[289]*rk_dim24_bPerm[1];
rk_dim24_bPerm[12] += A[290]*rk_dim24_bPerm[2];
rk_dim24_bPerm[12] += A[291]*rk_dim24_bPerm[3];
rk_dim24_bPerm[12] += A[292]*rk_dim24_bPerm[4];
rk_dim24_bPerm[12] += A[293]*rk_dim24_bPerm[5];
rk_dim24_bPerm[12] += A[294]*rk_dim24_bPerm[6];
rk_dim24_bPerm[12] += A[295]*rk_dim24_bPerm[7];
rk_dim24_bPerm[12] += A[296]*rk_dim24_bPerm[8];
rk_dim24_bPerm[12] += A[297]*rk_dim24_bPerm[9];
rk_dim24_bPerm[12] += A[298]*rk_dim24_bPerm[10];
rk_dim24_bPerm[12] += A[299]*rk_dim24_bPerm[11];

rk_dim24_bPerm[13] += A[312]*rk_dim24_bPerm[0];
rk_dim24_bPerm[13] += A[313]*rk_dim24_bPerm[1];
rk_dim24_bPerm[13] += A[314]*rk_dim24_bPerm[2];
rk_dim24_bPerm[13] += A[315]*rk_dim24_bPerm[3];
rk_dim24_bPerm[13] += A[316]*rk_dim24_bPerm[4];
rk_dim24_bPerm[13] += A[317]*rk_dim24_bPerm[5];
rk_dim24_bPerm[13] += A[318]*rk_dim24_bPerm[6];
rk_dim24_bPerm[13] += A[319]*rk_dim24_bPerm[7];
rk_dim24_bPerm[13] += A[320]*rk_dim24_bPerm[8];
rk_dim24_bPerm[13] += A[321]*rk_dim24_bPerm[9];
rk_dim24_bPerm[13] += A[322]*rk_dim24_bPerm[10];
rk_dim24_bPerm[13] += A[323]*rk_dim24_bPerm[11];
rk_dim24_bPerm[13] += A[324]*rk_dim24_bPerm[12];

rk_dim24_bPerm[14] += A[336]*rk_dim24_bPerm[0];
rk_dim24_bPerm[14] += A[337]*rk_dim24_bPerm[1];
rk_dim24_bPerm[14] += A[338]*rk_dim24_bPerm[2];
rk_dim24_bPerm[14] += A[339]*rk_dim24_bPerm[3];
rk_dim24_bPerm[14] += A[340]*rk_dim24_bPerm[4];
rk_dim24_bPerm[14] += A[341]*rk_dim24_bPerm[5];
rk_dim24_bPerm[14] += A[342]*rk_dim24_bPerm[6];
rk_dim24_bPerm[14] += A[343]*rk_dim24_bPerm[7];
rk_dim24_bPerm[14] += A[344]*rk_dim24_bPerm[8];
rk_dim24_bPerm[14] += A[345]*rk_dim24_bPerm[9];
rk_dim24_bPerm[14] += A[346]*rk_dim24_bPerm[10];
rk_dim24_bPerm[14] += A[347]*rk_dim24_bPerm[11];
rk_dim24_bPerm[14] += A[348]*rk_dim24_bPerm[12];
rk_dim24_bPerm[14] += A[349]*rk_dim24_bPerm[13];

rk_dim24_bPerm[15] += A[360]*rk_dim24_bPerm[0];
rk_dim24_bPerm[15] += A[361]*rk_dim24_bPerm[1];
rk_dim24_bPerm[15] += A[362]*rk_dim24_bPerm[2];
rk_dim24_bPerm[15] += A[363]*rk_dim24_bPerm[3];
rk_dim24_bPerm[15] += A[364]*rk_dim24_bPerm[4];
rk_dim24_bPerm[15] += A[365]*rk_dim24_bPerm[5];
rk_dim24_bPerm[15] += A[366]*rk_dim24_bPerm[6];
rk_dim24_bPerm[15] += A[367]*rk_dim24_bPerm[7];
rk_dim24_bPerm[15] += A[368]*rk_dim24_bPerm[8];
rk_dim24_bPerm[15] += A[369]*rk_dim24_bPerm[9];
rk_dim24_bPerm[15] += A[370]*rk_dim24_bPerm[10];
rk_dim24_bPerm[15] += A[371]*rk_dim24_bPerm[11];
rk_dim24_bPerm[15] += A[372]*rk_dim24_bPerm[12];
rk_dim24_bPerm[15] += A[373]*rk_dim24_bPerm[13];
rk_dim24_bPerm[15] += A[374]*rk_dim24_bPerm[14];

rk_dim24_bPerm[16] += A[384]*rk_dim24_bPerm[0];
rk_dim24_bPerm[16] += A[385]*rk_dim24_bPerm[1];
rk_dim24_bPerm[16] += A[386]*rk_dim24_bPerm[2];
rk_dim24_bPerm[16] += A[387]*rk_dim24_bPerm[3];
rk_dim24_bPerm[16] += A[388]*rk_dim24_bPerm[4];
rk_dim24_bPerm[16] += A[389]*rk_dim24_bPerm[5];
rk_dim24_bPerm[16] += A[390]*rk_dim24_bPerm[6];
rk_dim24_bPerm[16] += A[391]*rk_dim24_bPerm[7];
rk_dim24_bPerm[16] += A[392]*rk_dim24_bPerm[8];
rk_dim24_bPerm[16] += A[393]*rk_dim24_bPerm[9];
rk_dim24_bPerm[16] += A[394]*rk_dim24_bPerm[10];
rk_dim24_bPerm[16] += A[395]*rk_dim24_bPerm[11];
rk_dim24_bPerm[16] += A[396]*rk_dim24_bPerm[12];
rk_dim24_bPerm[16] += A[397]*rk_dim24_bPerm[13];
rk_dim24_bPerm[16] += A[398]*rk_dim24_bPerm[14];
rk_dim24_bPerm[16] += A[399]*rk_dim24_bPerm[15];

rk_dim24_bPerm[17] += A[408]*rk_dim24_bPerm[0];
rk_dim24_bPerm[17] += A[409]*rk_dim24_bPerm[1];
rk_dim24_bPerm[17] += A[410]*rk_dim24_bPerm[2];
rk_dim24_bPerm[17] += A[411]*rk_dim24_bPerm[3];
rk_dim24_bPerm[17] += A[412]*rk_dim24_bPerm[4];
rk_dim24_bPerm[17] += A[413]*rk_dim24_bPerm[5];
rk_dim24_bPerm[17] += A[414]*rk_dim24_bPerm[6];
rk_dim24_bPerm[17] += A[415]*rk_dim24_bPerm[7];
rk_dim24_bPerm[17] += A[416]*rk_dim24_bPerm[8];
rk_dim24_bPerm[17] += A[417]*rk_dim24_bPerm[9];
rk_dim24_bPerm[17] += A[418]*rk_dim24_bPerm[10];
rk_dim24_bPerm[17] += A[419]*rk_dim24_bPerm[11];
rk_dim24_bPerm[17] += A[420]*rk_dim24_bPerm[12];
rk_dim24_bPerm[17] += A[421]*rk_dim24_bPerm[13];
rk_dim24_bPerm[17] += A[422]*rk_dim24_bPerm[14];
rk_dim24_bPerm[17] += A[423]*rk_dim24_bPerm[15];
rk_dim24_bPerm[17] += A[424]*rk_dim24_bPerm[16];

rk_dim24_bPerm[18] += A[432]*rk_dim24_bPerm[0];
rk_dim24_bPerm[18] += A[433]*rk_dim24_bPerm[1];
rk_dim24_bPerm[18] += A[434]*rk_dim24_bPerm[2];
rk_dim24_bPerm[18] += A[435]*rk_dim24_bPerm[3];
rk_dim24_bPerm[18] += A[436]*rk_dim24_bPerm[4];
rk_dim24_bPerm[18] += A[437]*rk_dim24_bPerm[5];
rk_dim24_bPerm[18] += A[438]*rk_dim24_bPerm[6];
rk_dim24_bPerm[18] += A[439]*rk_dim24_bPerm[7];
rk_dim24_bPerm[18] += A[440]*rk_dim24_bPerm[8];
rk_dim24_bPerm[18] += A[441]*rk_dim24_bPerm[9];
rk_dim24_bPerm[18] += A[442]*rk_dim24_bPerm[10];
rk_dim24_bPerm[18] += A[443]*rk_dim24_bPerm[11];
rk_dim24_bPerm[18] += A[444]*rk_dim24_bPerm[12];
rk_dim24_bPerm[18] += A[445]*rk_dim24_bPerm[13];
rk_dim24_bPerm[18] += A[446]*rk_dim24_bPerm[14];
rk_dim24_bPerm[18] += A[447]*rk_dim24_bPerm[15];
rk_dim24_bPerm[18] += A[448]*rk_dim24_bPerm[16];
rk_dim24_bPerm[18] += A[449]*rk_dim24_bPerm[17];

rk_dim24_bPerm[19] += A[456]*rk_dim24_bPerm[0];
rk_dim24_bPerm[19] += A[457]*rk_dim24_bPerm[1];
rk_dim24_bPerm[19] += A[458]*rk_dim24_bPerm[2];
rk_dim24_bPerm[19] += A[459]*rk_dim24_bPerm[3];
rk_dim24_bPerm[19] += A[460]*rk_dim24_bPerm[4];
rk_dim24_bPerm[19] += A[461]*rk_dim24_bPerm[5];
rk_dim24_bPerm[19] += A[462]*rk_dim24_bPerm[6];
rk_dim24_bPerm[19] += A[463]*rk_dim24_bPerm[7];
rk_dim24_bPerm[19] += A[464]*rk_dim24_bPerm[8];
rk_dim24_bPerm[19] += A[465]*rk_dim24_bPerm[9];
rk_dim24_bPerm[19] += A[466]*rk_dim24_bPerm[10];
rk_dim24_bPerm[19] += A[467]*rk_dim24_bPerm[11];
rk_dim24_bPerm[19] += A[468]*rk_dim24_bPerm[12];
rk_dim24_bPerm[19] += A[469]*rk_dim24_bPerm[13];
rk_dim24_bPerm[19] += A[470]*rk_dim24_bPerm[14];
rk_dim24_bPerm[19] += A[471]*rk_dim24_bPerm[15];
rk_dim24_bPerm[19] += A[472]*rk_dim24_bPerm[16];
rk_dim24_bPerm[19] += A[473]*rk_dim24_bPerm[17];
rk_dim24_bPerm[19] += A[474]*rk_dim24_bPerm[18];

rk_dim24_bPerm[20] += A[480]*rk_dim24_bPerm[0];
rk_dim24_bPerm[20] += A[481]*rk_dim24_bPerm[1];
rk_dim24_bPerm[20] += A[482]*rk_dim24_bPerm[2];
rk_dim24_bPerm[20] += A[483]*rk_dim24_bPerm[3];
rk_dim24_bPerm[20] += A[484]*rk_dim24_bPerm[4];
rk_dim24_bPerm[20] += A[485]*rk_dim24_bPerm[5];
rk_dim24_bPerm[20] += A[486]*rk_dim24_bPerm[6];
rk_dim24_bPerm[20] += A[487]*rk_dim24_bPerm[7];
rk_dim24_bPerm[20] += A[488]*rk_dim24_bPerm[8];
rk_dim24_bPerm[20] += A[489]*rk_dim24_bPerm[9];
rk_dim24_bPerm[20] += A[490]*rk_dim24_bPerm[10];
rk_dim24_bPerm[20] += A[491]*rk_dim24_bPerm[11];
rk_dim24_bPerm[20] += A[492]*rk_dim24_bPerm[12];
rk_dim24_bPerm[20] += A[493]*rk_dim24_bPerm[13];
rk_dim24_bPerm[20] += A[494]*rk_dim24_bPerm[14];
rk_dim24_bPerm[20] += A[495]*rk_dim24_bPerm[15];
rk_dim24_bPerm[20] += A[496]*rk_dim24_bPerm[16];
rk_dim24_bPerm[20] += A[497]*rk_dim24_bPerm[17];
rk_dim24_bPerm[20] += A[498]*rk_dim24_bPerm[18];
rk_dim24_bPerm[20] += A[499]*rk_dim24_bPerm[19];

rk_dim24_bPerm[21] += A[504]*rk_dim24_bPerm[0];
rk_dim24_bPerm[21] += A[505]*rk_dim24_bPerm[1];
rk_dim24_bPerm[21] += A[506]*rk_dim24_bPerm[2];
rk_dim24_bPerm[21] += A[507]*rk_dim24_bPerm[3];
rk_dim24_bPerm[21] += A[508]*rk_dim24_bPerm[4];
rk_dim24_bPerm[21] += A[509]*rk_dim24_bPerm[5];
rk_dim24_bPerm[21] += A[510]*rk_dim24_bPerm[6];
rk_dim24_bPerm[21] += A[511]*rk_dim24_bPerm[7];
rk_dim24_bPerm[21] += A[512]*rk_dim24_bPerm[8];
rk_dim24_bPerm[21] += A[513]*rk_dim24_bPerm[9];
rk_dim24_bPerm[21] += A[514]*rk_dim24_bPerm[10];
rk_dim24_bPerm[21] += A[515]*rk_dim24_bPerm[11];
rk_dim24_bPerm[21] += A[516]*rk_dim24_bPerm[12];
rk_dim24_bPerm[21] += A[517]*rk_dim24_bPerm[13];
rk_dim24_bPerm[21] += A[518]*rk_dim24_bPerm[14];
rk_dim24_bPerm[21] += A[519]*rk_dim24_bPerm[15];
rk_dim24_bPerm[21] += A[520]*rk_dim24_bPerm[16];
rk_dim24_bPerm[21] += A[521]*rk_dim24_bPerm[17];
rk_dim24_bPerm[21] += A[522]*rk_dim24_bPerm[18];
rk_dim24_bPerm[21] += A[523]*rk_dim24_bPerm[19];
rk_dim24_bPerm[21] += A[524]*rk_dim24_bPerm[20];

rk_dim24_bPerm[22] += A[528]*rk_dim24_bPerm[0];
rk_dim24_bPerm[22] += A[529]*rk_dim24_bPerm[1];
rk_dim24_bPerm[22] += A[530]*rk_dim24_bPerm[2];
rk_dim24_bPerm[22] += A[531]*rk_dim24_bPerm[3];
rk_dim24_bPerm[22] += A[532]*rk_dim24_bPerm[4];
rk_dim24_bPerm[22] += A[533]*rk_dim24_bPerm[5];
rk_dim24_bPerm[22] += A[534]*rk_dim24_bPerm[6];
rk_dim24_bPerm[22] += A[535]*rk_dim24_bPerm[7];
rk_dim24_bPerm[22] += A[536]*rk_dim24_bPerm[8];
rk_dim24_bPerm[22] += A[537]*rk_dim24_bPerm[9];
rk_dim24_bPerm[22] += A[538]*rk_dim24_bPerm[10];
rk_dim24_bPerm[22] += A[539]*rk_dim24_bPerm[11];
rk_dim24_bPerm[22] += A[540]*rk_dim24_bPerm[12];
rk_dim24_bPerm[22] += A[541]*rk_dim24_bPerm[13];
rk_dim24_bPerm[22] += A[542]*rk_dim24_bPerm[14];
rk_dim24_bPerm[22] += A[543]*rk_dim24_bPerm[15];
rk_dim24_bPerm[22] += A[544]*rk_dim24_bPerm[16];
rk_dim24_bPerm[22] += A[545]*rk_dim24_bPerm[17];
rk_dim24_bPerm[22] += A[546]*rk_dim24_bPerm[18];
rk_dim24_bPerm[22] += A[547]*rk_dim24_bPerm[19];
rk_dim24_bPerm[22] += A[548]*rk_dim24_bPerm[20];
rk_dim24_bPerm[22] += A[549]*rk_dim24_bPerm[21];

rk_dim24_bPerm[23] += A[552]*rk_dim24_bPerm[0];
rk_dim24_bPerm[23] += A[553]*rk_dim24_bPerm[1];
rk_dim24_bPerm[23] += A[554]*rk_dim24_bPerm[2];
rk_dim24_bPerm[23] += A[555]*rk_dim24_bPerm[3];
rk_dim24_bPerm[23] += A[556]*rk_dim24_bPerm[4];
rk_dim24_bPerm[23] += A[557]*rk_dim24_bPerm[5];
rk_dim24_bPerm[23] += A[558]*rk_dim24_bPerm[6];
rk_dim24_bPerm[23] += A[559]*rk_dim24_bPerm[7];
rk_dim24_bPerm[23] += A[560]*rk_dim24_bPerm[8];
rk_dim24_bPerm[23] += A[561]*rk_dim24_bPerm[9];
rk_dim24_bPerm[23] += A[562]*rk_dim24_bPerm[10];
rk_dim24_bPerm[23] += A[563]*rk_dim24_bPerm[11];
rk_dim24_bPerm[23] += A[564]*rk_dim24_bPerm[12];
rk_dim24_bPerm[23] += A[565]*rk_dim24_bPerm[13];
rk_dim24_bPerm[23] += A[566]*rk_dim24_bPerm[14];
rk_dim24_bPerm[23] += A[567]*rk_dim24_bPerm[15];
rk_dim24_bPerm[23] += A[568]*rk_dim24_bPerm[16];
rk_dim24_bPerm[23] += A[569]*rk_dim24_bPerm[17];
rk_dim24_bPerm[23] += A[570]*rk_dim24_bPerm[18];
rk_dim24_bPerm[23] += A[571]*rk_dim24_bPerm[19];
rk_dim24_bPerm[23] += A[572]*rk_dim24_bPerm[20];
rk_dim24_bPerm[23] += A[573]*rk_dim24_bPerm[21];
rk_dim24_bPerm[23] += A[574]*rk_dim24_bPerm[22];


acado_solve_dim24_triangular( A, rk_dim24_bPerm );
b[0] = rk_dim24_bPerm[0];
b[1] = rk_dim24_bPerm[1];
b[2] = rk_dim24_bPerm[2];
b[3] = rk_dim24_bPerm[3];
b[4] = rk_dim24_bPerm[4];
b[5] = rk_dim24_bPerm[5];
b[6] = rk_dim24_bPerm[6];
b[7] = rk_dim24_bPerm[7];
b[8] = rk_dim24_bPerm[8];
b[9] = rk_dim24_bPerm[9];
b[10] = rk_dim24_bPerm[10];
b[11] = rk_dim24_bPerm[11];
b[12] = rk_dim24_bPerm[12];
b[13] = rk_dim24_bPerm[13];
b[14] = rk_dim24_bPerm[14];
b[15] = rk_dim24_bPerm[15];
b[16] = rk_dim24_bPerm[16];
b[17] = rk_dim24_bPerm[17];
b[18] = rk_dim24_bPerm[18];
b[19] = rk_dim24_bPerm[19];
b[20] = rk_dim24_bPerm[20];
b[21] = rk_dim24_bPerm[21];
b[22] = rk_dim24_bPerm[22];
b[23] = rk_dim24_bPerm[23];
}



/** Matrix of size: 2 x 2 (row major format) */
static const real_t acado_Ah_mat[ 4 ] = 
{ 1.2500000000000001e-02, 2.6933756729740646e-02, 
-1.9337567297406434e-03, 1.2500000000000001e-02 };


/* Fixed step size:0.05 */
int acado_integrate( real_t* const rk_eta, int resetIntegrator )
{
int error;

int i;
int j;
int k;
int run;
int run1;
int tmp_index1;
int tmp_index2;

real_t det;

rk_ttt = 0.0000000000000000e+00;
rk_xxx[12] = rk_eta[228];
rk_xxx[13] = rk_eta[229];
rk_xxx[14] = rk_eta[230];
rk_xxx[15] = rk_eta[231];
rk_xxx[16] = rk_eta[232];
rk_xxx[17] = rk_eta[233];

for (run = 0; run < 1; ++run)
{
if( resetIntegrator ) {
for (i = 0; i < 1; ++i)
{
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 12; ++j)
{
rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
rk_xxx[j] += + acado_Ah_mat[run1 * 2]*rk_kkk[tmp_index1 * 2];
rk_xxx[j] += + acado_Ah_mat[run1 * 2 + 1]*rk_kkk[tmp_index1 * 2 + 1];
}
acado_diffs( rk_xxx, &(rk_diffsTemp2[ run1 * 216 ]) );
for (j = 0; j < 12; ++j)
{
tmp_index1 = (run1 * 12) + (j);
rk_A[tmp_index1 * 24] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 216) + (j * 18)];
rk_A[tmp_index1 * 24 + 1] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 216) + (j * 18 + 1)];
rk_A[tmp_index1 * 24 + 2] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 216) + (j * 18 + 2)];
rk_A[tmp_index1 * 24 + 3] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 216) + (j * 18 + 3)];
rk_A[tmp_index1 * 24 + 4] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 216) + (j * 18 + 4)];
rk_A[tmp_index1 * 24 + 5] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 216) + (j * 18 + 5)];
rk_A[tmp_index1 * 24 + 6] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 216) + (j * 18 + 6)];
rk_A[tmp_index1 * 24 + 7] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 216) + (j * 18 + 7)];
rk_A[tmp_index1 * 24 + 8] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 216) + (j * 18 + 8)];
rk_A[tmp_index1 * 24 + 9] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 216) + (j * 18 + 9)];
rk_A[tmp_index1 * 24 + 10] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 216) + (j * 18 + 10)];
rk_A[tmp_index1 * 24 + 11] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 216) + (j * 18 + 11)];
if( 0 == run1 ) rk_A[(tmp_index1 * 24) + (j)] -= 1.0000000000000000e+00;
rk_A[tmp_index1 * 24 + 12] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 216) + (j * 18)];
rk_A[tmp_index1 * 24 + 13] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 216) + (j * 18 + 1)];
rk_A[tmp_index1 * 24 + 14] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 216) + (j * 18 + 2)];
rk_A[tmp_index1 * 24 + 15] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 216) + (j * 18 + 3)];
rk_A[tmp_index1 * 24 + 16] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 216) + (j * 18 + 4)];
rk_A[tmp_index1 * 24 + 17] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 216) + (j * 18 + 5)];
rk_A[tmp_index1 * 24 + 18] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 216) + (j * 18 + 6)];
rk_A[tmp_index1 * 24 + 19] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 216) + (j * 18 + 7)];
rk_A[tmp_index1 * 24 + 20] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 216) + (j * 18 + 8)];
rk_A[tmp_index1 * 24 + 21] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 216) + (j * 18 + 9)];
rk_A[tmp_index1 * 24 + 22] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 216) + (j * 18 + 10)];
rk_A[tmp_index1 * 24 + 23] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 216) + (j * 18 + 11)];
if( 1 == run1 ) rk_A[(tmp_index1 * 24) + (j + 12)] -= 1.0000000000000000e+00;
}
acado_rhs( rk_xxx, rk_rhsTemp );
rk_b[run1 * 12] = rk_kkk[run1] - rk_rhsTemp[0];
rk_b[run1 * 12 + 1] = rk_kkk[run1 + 2] - rk_rhsTemp[1];
rk_b[run1 * 12 + 2] = rk_kkk[run1 + 4] - rk_rhsTemp[2];
rk_b[run1 * 12 + 3] = rk_kkk[run1 + 6] - rk_rhsTemp[3];
rk_b[run1 * 12 + 4] = rk_kkk[run1 + 8] - rk_rhsTemp[4];
rk_b[run1 * 12 + 5] = rk_kkk[run1 + 10] - rk_rhsTemp[5];
rk_b[run1 * 12 + 6] = rk_kkk[run1 + 12] - rk_rhsTemp[6];
rk_b[run1 * 12 + 7] = rk_kkk[run1 + 14] - rk_rhsTemp[7];
rk_b[run1 * 12 + 8] = rk_kkk[run1 + 16] - rk_rhsTemp[8];
rk_b[run1 * 12 + 9] = rk_kkk[run1 + 18] - rk_rhsTemp[9];
rk_b[run1 * 12 + 10] = rk_kkk[run1 + 20] - rk_rhsTemp[10];
rk_b[run1 * 12 + 11] = rk_kkk[run1 + 22] - rk_rhsTemp[11];
}
det = acado_solve_dim24_system( rk_A, rk_b, rk_dim24_perm );
for (j = 0; j < 2; ++j)
{
rk_kkk[j] += rk_b[j * 12];
rk_kkk[j + 2] += rk_b[j * 12 + 1];
rk_kkk[j + 4] += rk_b[j * 12 + 2];
rk_kkk[j + 6] += rk_b[j * 12 + 3];
rk_kkk[j + 8] += rk_b[j * 12 + 4];
rk_kkk[j + 10] += rk_b[j * 12 + 5];
rk_kkk[j + 12] += rk_b[j * 12 + 6];
rk_kkk[j + 14] += rk_b[j * 12 + 7];
rk_kkk[j + 16] += rk_b[j * 12 + 8];
rk_kkk[j + 18] += rk_b[j * 12 + 9];
rk_kkk[j + 20] += rk_b[j * 12 + 10];
rk_kkk[j + 22] += rk_b[j * 12 + 11];
}
}
}
for (i = 0; i < 5; ++i)
{
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 12; ++j)
{
rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
rk_xxx[j] += + acado_Ah_mat[run1 * 2]*rk_kkk[tmp_index1 * 2];
rk_xxx[j] += + acado_Ah_mat[run1 * 2 + 1]*rk_kkk[tmp_index1 * 2 + 1];
}
acado_rhs( rk_xxx, rk_rhsTemp );
rk_b[run1 * 12] = rk_kkk[run1] - rk_rhsTemp[0];
rk_b[run1 * 12 + 1] = rk_kkk[run1 + 2] - rk_rhsTemp[1];
rk_b[run1 * 12 + 2] = rk_kkk[run1 + 4] - rk_rhsTemp[2];
rk_b[run1 * 12 + 3] = rk_kkk[run1 + 6] - rk_rhsTemp[3];
rk_b[run1 * 12 + 4] = rk_kkk[run1 + 8] - rk_rhsTemp[4];
rk_b[run1 * 12 + 5] = rk_kkk[run1 + 10] - rk_rhsTemp[5];
rk_b[run1 * 12 + 6] = rk_kkk[run1 + 12] - rk_rhsTemp[6];
rk_b[run1 * 12 + 7] = rk_kkk[run1 + 14] - rk_rhsTemp[7];
rk_b[run1 * 12 + 8] = rk_kkk[run1 + 16] - rk_rhsTemp[8];
rk_b[run1 * 12 + 9] = rk_kkk[run1 + 18] - rk_rhsTemp[9];
rk_b[run1 * 12 + 10] = rk_kkk[run1 + 20] - rk_rhsTemp[10];
rk_b[run1 * 12 + 11] = rk_kkk[run1 + 22] - rk_rhsTemp[11];
}
acado_solve_dim24_system_reuse( rk_A, rk_b, rk_dim24_perm );
for (j = 0; j < 2; ++j)
{
rk_kkk[j] += rk_b[j * 12];
rk_kkk[j + 2] += rk_b[j * 12 + 1];
rk_kkk[j + 4] += rk_b[j * 12 + 2];
rk_kkk[j + 6] += rk_b[j * 12 + 3];
rk_kkk[j + 8] += rk_b[j * 12 + 4];
rk_kkk[j + 10] += rk_b[j * 12 + 5];
rk_kkk[j + 12] += rk_b[j * 12 + 6];
rk_kkk[j + 14] += rk_b[j * 12 + 7];
rk_kkk[j + 16] += rk_b[j * 12 + 8];
rk_kkk[j + 18] += rk_b[j * 12 + 9];
rk_kkk[j + 20] += rk_b[j * 12 + 10];
rk_kkk[j + 22] += rk_b[j * 12 + 11];
}
}
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 12; ++j)
{
rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
rk_xxx[j] += + acado_Ah_mat[run1 * 2]*rk_kkk[tmp_index1 * 2];
rk_xxx[j] += + acado_Ah_mat[run1 * 2 + 1]*rk_kkk[tmp_index1 * 2 + 1];
}
acado_diffs( rk_xxx, &(rk_diffsTemp2[ run1 * 216 ]) );
for (j = 0; j < 12; ++j)
{
tmp_index1 = (run1 * 12) + (j);
rk_A[tmp_index1 * 24] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 216) + (j * 18)];
rk_A[tmp_index1 * 24 + 1] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 216) + (j * 18 + 1)];
rk_A[tmp_index1 * 24 + 2] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 216) + (j * 18 + 2)];
rk_A[tmp_index1 * 24 + 3] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 216) + (j * 18 + 3)];
rk_A[tmp_index1 * 24 + 4] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 216) + (j * 18 + 4)];
rk_A[tmp_index1 * 24 + 5] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 216) + (j * 18 + 5)];
rk_A[tmp_index1 * 24 + 6] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 216) + (j * 18 + 6)];
rk_A[tmp_index1 * 24 + 7] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 216) + (j * 18 + 7)];
rk_A[tmp_index1 * 24 + 8] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 216) + (j * 18 + 8)];
rk_A[tmp_index1 * 24 + 9] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 216) + (j * 18 + 9)];
rk_A[tmp_index1 * 24 + 10] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 216) + (j * 18 + 10)];
rk_A[tmp_index1 * 24 + 11] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 216) + (j * 18 + 11)];
if( 0 == run1 ) rk_A[(tmp_index1 * 24) + (j)] -= 1.0000000000000000e+00;
rk_A[tmp_index1 * 24 + 12] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 216) + (j * 18)];
rk_A[tmp_index1 * 24 + 13] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 216) + (j * 18 + 1)];
rk_A[tmp_index1 * 24 + 14] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 216) + (j * 18 + 2)];
rk_A[tmp_index1 * 24 + 15] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 216) + (j * 18 + 3)];
rk_A[tmp_index1 * 24 + 16] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 216) + (j * 18 + 4)];
rk_A[tmp_index1 * 24 + 17] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 216) + (j * 18 + 5)];
rk_A[tmp_index1 * 24 + 18] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 216) + (j * 18 + 6)];
rk_A[tmp_index1 * 24 + 19] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 216) + (j * 18 + 7)];
rk_A[tmp_index1 * 24 + 20] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 216) + (j * 18 + 8)];
rk_A[tmp_index1 * 24 + 21] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 216) + (j * 18 + 9)];
rk_A[tmp_index1 * 24 + 22] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 216) + (j * 18 + 10)];
rk_A[tmp_index1 * 24 + 23] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 216) + (j * 18 + 11)];
if( 1 == run1 ) rk_A[(tmp_index1 * 24) + (j + 12)] -= 1.0000000000000000e+00;
}
}
for (run1 = 0; run1 < 12; ++run1)
{
for (i = 0; i < 2; ++i)
{
rk_b[i * 12] = - rk_diffsTemp2[(i * 216) + (run1)];
rk_b[i * 12 + 1] = - rk_diffsTemp2[(i * 216) + (run1 + 18)];
rk_b[i * 12 + 2] = - rk_diffsTemp2[(i * 216) + (run1 + 36)];
rk_b[i * 12 + 3] = - rk_diffsTemp2[(i * 216) + (run1 + 54)];
rk_b[i * 12 + 4] = - rk_diffsTemp2[(i * 216) + (run1 + 72)];
rk_b[i * 12 + 5] = - rk_diffsTemp2[(i * 216) + (run1 + 90)];
rk_b[i * 12 + 6] = - rk_diffsTemp2[(i * 216) + (run1 + 108)];
rk_b[i * 12 + 7] = - rk_diffsTemp2[(i * 216) + (run1 + 126)];
rk_b[i * 12 + 8] = - rk_diffsTemp2[(i * 216) + (run1 + 144)];
rk_b[i * 12 + 9] = - rk_diffsTemp2[(i * 216) + (run1 + 162)];
rk_b[i * 12 + 10] = - rk_diffsTemp2[(i * 216) + (run1 + 180)];
rk_b[i * 12 + 11] = - rk_diffsTemp2[(i * 216) + (run1 + 198)];
}
if( 0 == run1 ) {
det = acado_solve_dim24_system( rk_A, rk_b, rk_dim24_perm );
}
 else {
acado_solve_dim24_system_reuse( rk_A, rk_b, rk_dim24_perm );
}
for (i = 0; i < 2; ++i)
{
rk_diffK[i] = rk_b[i * 12];
rk_diffK[i + 2] = rk_b[i * 12 + 1];
rk_diffK[i + 4] = rk_b[i * 12 + 2];
rk_diffK[i + 6] = rk_b[i * 12 + 3];
rk_diffK[i + 8] = rk_b[i * 12 + 4];
rk_diffK[i + 10] = rk_b[i * 12 + 5];
rk_diffK[i + 12] = rk_b[i * 12 + 6];
rk_diffK[i + 14] = rk_b[i * 12 + 7];
rk_diffK[i + 16] = rk_b[i * 12 + 8];
rk_diffK[i + 18] = rk_b[i * 12 + 9];
rk_diffK[i + 20] = rk_b[i * 12 + 10];
rk_diffK[i + 22] = rk_b[i * 12 + 11];
}
for (i = 0; i < 12; ++i)
{
rk_diffsNew2[(i * 18) + (run1)] = (i == run1-0);
rk_diffsNew2[(i * 18) + (run1)] += + rk_diffK[i * 2]*(real_t)2.5000000000000001e-02 + rk_diffK[i * 2 + 1]*(real_t)2.5000000000000001e-02;
}
}
for (run1 = 0; run1 < 6; ++run1)
{
for (i = 0; i < 2; ++i)
{
for (j = 0; j < 12; ++j)
{
tmp_index1 = (i * 12) + (j);
tmp_index2 = (run1) + (j * 18);
rk_b[tmp_index1] = - rk_diffsTemp2[(i * 216) + (tmp_index2 + 12)];
}
}
acado_solve_dim24_system_reuse( rk_A, rk_b, rk_dim24_perm );
for (i = 0; i < 2; ++i)
{
rk_diffK[i] = rk_b[i * 12];
rk_diffK[i + 2] = rk_b[i * 12 + 1];
rk_diffK[i + 4] = rk_b[i * 12 + 2];
rk_diffK[i + 6] = rk_b[i * 12 + 3];
rk_diffK[i + 8] = rk_b[i * 12 + 4];
rk_diffK[i + 10] = rk_b[i * 12 + 5];
rk_diffK[i + 12] = rk_b[i * 12 + 6];
rk_diffK[i + 14] = rk_b[i * 12 + 7];
rk_diffK[i + 16] = rk_b[i * 12 + 8];
rk_diffK[i + 18] = rk_b[i * 12 + 9];
rk_diffK[i + 20] = rk_b[i * 12 + 10];
rk_diffK[i + 22] = rk_b[i * 12 + 11];
}
for (i = 0; i < 12; ++i)
{
rk_diffsNew2[(i * 18) + (run1 + 12)] = + rk_diffK[i * 2]*(real_t)2.5000000000000001e-02 + rk_diffK[i * 2 + 1]*(real_t)2.5000000000000001e-02;
}
}
rk_eta[0] += + rk_kkk[0]*(real_t)2.5000000000000001e-02 + rk_kkk[1]*(real_t)2.5000000000000001e-02;
rk_eta[1] += + rk_kkk[2]*(real_t)2.5000000000000001e-02 + rk_kkk[3]*(real_t)2.5000000000000001e-02;
rk_eta[2] += + rk_kkk[4]*(real_t)2.5000000000000001e-02 + rk_kkk[5]*(real_t)2.5000000000000001e-02;
rk_eta[3] += + rk_kkk[6]*(real_t)2.5000000000000001e-02 + rk_kkk[7]*(real_t)2.5000000000000001e-02;
rk_eta[4] += + rk_kkk[8]*(real_t)2.5000000000000001e-02 + rk_kkk[9]*(real_t)2.5000000000000001e-02;
rk_eta[5] += + rk_kkk[10]*(real_t)2.5000000000000001e-02 + rk_kkk[11]*(real_t)2.5000000000000001e-02;
rk_eta[6] += + rk_kkk[12]*(real_t)2.5000000000000001e-02 + rk_kkk[13]*(real_t)2.5000000000000001e-02;
rk_eta[7] += + rk_kkk[14]*(real_t)2.5000000000000001e-02 + rk_kkk[15]*(real_t)2.5000000000000001e-02;
rk_eta[8] += + rk_kkk[16]*(real_t)2.5000000000000001e-02 + rk_kkk[17]*(real_t)2.5000000000000001e-02;
rk_eta[9] += + rk_kkk[18]*(real_t)2.5000000000000001e-02 + rk_kkk[19]*(real_t)2.5000000000000001e-02;
rk_eta[10] += + rk_kkk[20]*(real_t)2.5000000000000001e-02 + rk_kkk[21]*(real_t)2.5000000000000001e-02;
rk_eta[11] += + rk_kkk[22]*(real_t)2.5000000000000001e-02 + rk_kkk[23]*(real_t)2.5000000000000001e-02;
for (i = 0; i < 12; ++i)
{
for (j = 0; j < 12; ++j)
{
tmp_index2 = (j) + (i * 12);
rk_eta[tmp_index2 + 12] = rk_diffsNew2[(i * 18) + (j)];
}
for (j = 0; j < 6; ++j)
{
tmp_index2 = (j) + (i * 6);
rk_eta[tmp_index2 + 156] = rk_diffsNew2[(i * 18) + (j + 12)];
}
}
resetIntegrator = 0;
rk_ttt += 1.0000000000000000e+00;
}
for (i = 0; i < 12; ++i)
{
}
if( det < 1e-12 ) {
error = 2;
} else if( det < 1e-6 ) {
error = 1;
} else {
error = 0;
}
return error;
}



