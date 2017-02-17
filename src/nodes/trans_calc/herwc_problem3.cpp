/* 
   This file was automatically generated on May 26, 2015 23:44:10
   by GpoSolver, (c) 2013--2014, Jan Heller, hellej1@cmp.felk.cvut.cz,
   Czech Technical University, Prague, Czech Republic 

   This solver takes:
      24 residual parameters: [ra1, ra4, ra7, ra2, ra5, ra8, ra3, ra6, ra9, ta1, ta2, ta3, rb1, rb4, rb7, rb2, rb5, rb8, rb3, rb6, rb9, tb1, tb2, tb3]
       2 problem parameters : [n, m]
*/

#include "herwc_problem3.h"

using namespace GpoSolver;
using namespace Eigen;


void HerwcProblem::updateAuxiliaries(const double *pars, double *aux) const {
  aux[0] = pars[0];
  aux[1] = pars[1];
}

double HerwcProblem::evalPolyObjective(const double *rpars, const double *ppars, const double *vars) const {
  double t3 = vars[0]*vars[0];
  double t4 = vars[1]*vars[1];
  double t5 = vars[2]*vars[2];
  double t6 = vars[3]*vars[3];
  double t7 = t3+t4-t5-t6;
  double t8 = vars[7]*vars[7];
  double t9 = vars[8]*vars[8];
  double t10 = vars[9]*vars[9];
  double t11 = vars[10]*vars[10];
  double t12 = vars[0]*vars[3]*2.0;
  double t13 = vars[1]*vars[2]*2.0;
  double t14 = t12+t13;
  double t15 = vars[0]*vars[2]*2.0;
  double t16 = vars[1]*vars[3]*2.0;
  double t17 = t15-t16;
  double t18 = vars[7]*vars[10]*2.0;
  double t19 = vars[8]*vars[9]*2.0;
  double t21 = vars[7]*vars[9]*2.0;
  double t22 = vars[8]*vars[10]*2.0;
  double t26 = t8+t9-t10-t11;
  double t27 = t18-t19;
  double t28 = t21+t22;
  double t2 = rpars[0]*t7+rpars[3]*t14-rpars[6]*t17-rpars[12]*t26+rpars[13]*t27-rpars[14]*t28;
  double t23 = vars[7]*vars[8]*2.0;
  double t24 = vars[9]*vars[10]*2.0;
  double t31 = t8-t9+t10-t11;
  double t36 = t18+t19;
  double t37 = t23-t24;
  double t20 = -rpars[1]*t7-rpars[4]*t14+rpars[7]*t17+rpars[13]*t31+rpars[12]*t36-rpars[14]*t37;
  double t39 = t8-t9-t10+t11;
  double t40 = t21-t22;
  double t41 = t23+t24;
  double t25 = -rpars[2]*t7-rpars[5]*t14+rpars[8]*t17-rpars[12]*t40+rpars[14]*t39+rpars[13]*t41;
  double t30 = t3-t4+t5-t6;
  double t32 = t12-t13;
  double t33 = vars[0]*vars[1]*2.0;
  double t34 = vars[2]*vars[3]*2.0;
  double t35 = t33+t34;
  double t29 = rpars[0]*t32-rpars[3]*t30-rpars[6]*t35+rpars[15]*t26-rpars[16]*t27+rpars[17]*t28;
  double t38 = rpars[1]*t32-rpars[4]*t30-rpars[7]*t35+rpars[16]*t31+rpars[15]*t36-rpars[17]*t37;
  double t42 = rpars[2]*t32-rpars[5]*t30-rpars[8]*t35-rpars[15]*t40+rpars[17]*t39+rpars[16]*t41;
  double t44 = t3-t4-t5+t6;
  double t45 = t15+t16;
  double t46 = t33-t34;
  double t43 = -rpars[18]*t26+rpars[0]*t45+rpars[19]*t27-rpars[20]*t28-rpars[3]*t46+rpars[6]*t44;
  double t47 = rpars[1]*t45-rpars[4]*t46-rpars[19]*t31+rpars[7]*t44-rpars[18]*t36+rpars[20]*t37;
  double t48 = rpars[2]*t45-rpars[5]*t46+rpars[8]*t44+rpars[18]*t40-rpars[20]*t39-rpars[19]*t41;
  double t49 = rpars[9]-vars[11]-rpars[21]*t26+rpars[22]*t27-rpars[23]*t28+rpars[0]*vars[4]+rpars[3]*vars[5]+rpars[6]*vars[6];
  double t50 = rpars[10]-vars[12]-rpars[22]*t31-rpars[21]*t36+rpars[23]*t37+rpars[1]*vars[4]+rpars[4]*vars[5]+rpars[7]*vars[6];
  double t51 = rpars[11]-vars[13]+rpars[21]*t40-rpars[23]*t39-rpars[22]*t41+rpars[2]*vars[4]+rpars[5]*vars[5]+rpars[8]*vars[6];
  double t0 = t2*t2+t20*t20+t25*t25+t29*t29+t38*t38+t42*t42+t43*t43+t47*t47+t48*t48+t49*t49+t50*t50+t51*t51;

  return t0;
}

void HerwcProblem::evalPolyConstraints(const double *vars, const double *pars, double *cons) const {

  double aux[3];

  updateAuxiliaries(pars, aux);

  double t2 = vars[0]*vars[0];
  double t3 = vars[1]*vars[1];
  double t4 = vars[2]*vars[2];
  double t5 = vars[3]*vars[3];
  double t6 = vars[7]*vars[7];
  double t7 = vars[8]*vars[8];
  double t8 = vars[9]*vars[9];
  double t9 = vars[10]*vars[10];
  cons[0] = t2+t3+t4+t5-1.0;
  cons[1] = -t2-t3-t4-t5+1.0;
  cons[2] = vars[0];
  cons[3] = aux[0]-vars[4]*vars[4]-vars[5]*vars[5]-vars[6]*vars[6];
  cons[4] = t6+t7+t8+t9-1.0;
  cons[5] = -t6-t7-t8-t9+1.0;
  cons[6] = vars[7];
  cons[7] = aux[1]-vars[11]*vars[11]-vars[12]*vars[12]-vars[13]*vars[13];
}

void HerwcProblem::evalSdpObjectiveVariables(const double *rpars, const double *ppars, double **vars) const {
  double t2 = rpars[9]*rpars[22]*4.0;
  double t3 = rpars[11]*rpars[21]*4.0;
  double t4 = rpars[10]*rpars[22]*2.0;
  double t5 = rpars[11]*rpars[23]*2.0;
  double t6 = rpars[10]*rpars[23]*4.0;
  double t7 = rpars[9]*rpars[21]*2.0;
  double t8 = rpars[0]*rpars[22]*4.0;
  double t9 = rpars[2]*rpars[21]*4.0;
  double t10 = rpars[1]*rpars[22]*2.0;
  double t11 = rpars[2]*rpars[23]*2.0;
  double t12 = rpars[1]*rpars[23]*4.0;
  double t13 = rpars[0]*rpars[21]*2.0;
  double t14 = rpars[3]*rpars[22]*4.0;
  double t15 = rpars[5]*rpars[21]*4.0;
  double t16 = rpars[4]*rpars[22]*2.0;
  double t17 = rpars[5]*rpars[23]*2.0;
  double t18 = rpars[4]*rpars[23]*4.0;
  double t19 = rpars[3]*rpars[21]*2.0;
  double t20 = rpars[6]*rpars[22]*4.0;
  double t21 = rpars[8]*rpars[21]*4.0;
  double t22 = rpars[7]*rpars[22]*2.0;
  double t23 = rpars[8]*rpars[23]*2.0;
  double t24 = rpars[7]*rpars[23]*4.0;
  double t25 = rpars[6]*rpars[21]*2.0;
  double t26 = rpars[22]*4.0;
  double t27 = rpars[21]*2.0;
  double t28 = rpars[22]*2.0;
  double t29 = rpars[23]*2.0;
  double t30 = rpars[21]*4.0;
  double t31 = rpars[23]*4.0;
  double t32 = rpars[0]*rpars[0];
  double t33 = rpars[1]*rpars[1];
  double t34 = rpars[2]*rpars[2];
  double t35 = rpars[3]*rpars[3];
  double t36 = rpars[4]*rpars[4];
  double t37 = rpars[5]*rpars[5];
  double t38 = rpars[6]*rpars[6];
  double t39 = rpars[7]*rpars[7];
  double t40 = rpars[8]*rpars[8];
  double t41 = t32*2.0;
  double t42 = t33*2.0;
  double t43 = t34*2.0;
  double t44 = t35*2.0;
  double t45 = t36*2.0;
  double t46 = t37*2.0;
  double t47 = t38*2.0;
  double t48 = t39*2.0;
  double t49 = t40*2.0;
  double t50 = t41+t42+t43+t44+t45+t46+t47+t48+t49;
  double t51 = rpars[0]*rpars[13]*4.0;
  double t52 = rpars[3]*rpars[16]*4.0;
  double t53 = rpars[6]*rpars[19]*4.0;
  double t54 = rpars[2]*rpars[12]*4.0;
  double t55 = rpars[5]*rpars[15]*4.0;
  double t56 = rpars[8]*rpars[18]*4.0;
  double t57 = rpars[1]*rpars[13]*2.0;
  double t58 = rpars[2]*rpars[14]*2.0;
  double t59 = rpars[4]*rpars[16]*2.0;
  double t60 = rpars[5]*rpars[17]*2.0;
  double t61 = rpars[7]*rpars[19]*2.0;
  double t62 = rpars[8]*rpars[20]*2.0;
  double t63 = rpars[1]*rpars[14]*4.0;
  double t64 = rpars[4]*rpars[17]*4.0;
  double t65 = rpars[7]*rpars[20]*4.0;
  double t66 = rpars[0]*rpars[12]*2.0;
  double t67 = rpars[3]*rpars[15]*2.0;
  double t68 = rpars[6]*rpars[18]*2.0;
  double t69 = rpars[3]*rpars[18]*4.0;
  double t70 = rpars[4]*rpars[19]*4.0;
  double t71 = rpars[5]*rpars[20]*4.0;
  double t72 = rpars[4]*rpars[18]*8.0;
  double t73 = rpars[6]*rpars[16]*8.0;
  double t74 = rpars[3]*rpars[20]*8.0;
  double t75 = rpars[8]*rpars[15]*8.0;
  double t76 = rpars[7]*rpars[16]*4.0;
  double t77 = rpars[8]*rpars[17]*4.0;
  double t78 = rpars[5]*rpars[19]*8.0;
  double t79 = rpars[7]*rpars[17]*8.0;
  double t80 = rpars[6]*rpars[15]*4.0;
  double t81 = rpars[6]*rpars[12]*4.0;
  double t82 = rpars[7]*rpars[13]*4.0;
  double t83 = rpars[8]*rpars[14]*4.0;
  double t84 = rpars[0]*rpars[19]*8.0;
  double t85 = rpars[7]*rpars[12]*8.0;
  double t86 = rpars[2]*rpars[18]*8.0;
  double t87 = rpars[6]*rpars[14]*8.0;
  double t88 = rpars[1]*rpars[19]*4.0;
  double t89 = rpars[2]*rpars[20]*4.0;
  double t90 = rpars[1]*rpars[20]*8.0;
  double t91 = rpars[8]*rpars[13]*8.0;
  double t92 = rpars[0]*rpars[18]*4.0;
  double t93 = rpars[0]*rpars[15]*4.0;
  double t94 = rpars[1]*rpars[16]*4.0;
  double t95 = rpars[2]*rpars[17]*4.0;
  double t96 = rpars[1]*rpars[15]*8.0;
  double t97 = rpars[3]*rpars[13]*8.0;
  double t98 = rpars[0]*rpars[17]*8.0;
  double t99 = rpars[5]*rpars[12]*8.0;
  double t100 = rpars[4]*rpars[13]*4.0;
  double t101 = rpars[5]*rpars[14]*4.0;
  double t102 = rpars[2]*rpars[16]*8.0;
  double t103 = rpars[4]*rpars[14]*8.0;
  double t104 = rpars[3]*rpars[12]*4.0;
  double t105 = t32+t33+t34+t35+t36+t37+t38+t39+t40;
  double t106 = rpars[4]*rpars[15]*4.0;
  double t107 = rpars[7]*rpars[18]*4.0;
  double t108 = rpars[3]*rpars[17]*4.0;
  double t109 = rpars[6]*rpars[20]*4.0;
  double t110 = rpars[5]*rpars[16]*4.0;
  double t111 = rpars[8]*rpars[19]*4.0;
  double t112 = rpars[1]*rpars[17]*8.0;
  double t113 = rpars[2]*rpars[15]*8.0;
  double t114 = rpars[0]*rpars[16]*8.0;
  double t115 = rpars[7]*rpars[14]*8.0;
  double t116 = rpars[8]*rpars[12]*8.0;
  double t117 = rpars[6]*rpars[13]*8.0;
  double t118 = rpars[1]*rpars[12]*4.0;
  double t119 = rpars[0]*rpars[14]*4.0;
  double t120 = rpars[2]*rpars[13]*4.0;
  double t121 = rpars[4]*rpars[20]*8.0;
  double t122 = rpars[5]*rpars[18]*8.0;
  double t123 = rpars[3]*rpars[19]*8.0;
  double t124 = rpars[12]*rpars[12];
  double t125 = rpars[13]*rpars[13];
  double t126 = rpars[14]*rpars[14];
  double t127 = rpars[15]*rpars[15];
  double t128 = rpars[16]*rpars[16];
  double t129 = rpars[17]*rpars[17];
  double t130 = rpars[18]*rpars[18];
  double t131 = rpars[19]*rpars[19];
  double t132 = rpars[20]*rpars[20];
  double t133 = rpars[21]*rpars[21];
  double t134 = rpars[22]*rpars[22];
  double t135 = rpars[23]*rpars[23];
  double t136 = t124*2.0;
  double t137 = t125*2.0;
  double t138 = t126*2.0;
  double t139 = t127*2.0;
  double t140 = t128*2.0;
  double t141 = t129*2.0;
  double t142 = t130*2.0;
  double t143 = t131*2.0;
  double t144 = t132*2.0;
  double t145 = t133*2.0;
  double t146 = t134*2.0;
  double t147 = t135*2.0;
  double t148 = t136+t137+t138+t139+t140+t141+t142+t143+t144+t145+t146+t147;
  double t149 = t124+t125+t126+t127+t128+t129+t130+t131+t132+t133+t134+t135;
  *(vars[0]) = rpars[9]*rpars[9]+rpars[10]*rpars[10]+rpars[11]*rpars[11];
  *(vars[1]) = rpars[0]*rpars[9]*2.0+rpars[1]*rpars[10]*2.0+rpars[2]*rpars[11]*2.0;
  *(vars[2]) = rpars[3]*rpars[9]*2.0+rpars[4]*rpars[10]*2.0+rpars[5]*rpars[11]*2.0;
  *(vars[3]) = rpars[6]*rpars[9]*2.0+rpars[7]*rpars[10]*2.0+rpars[8]*rpars[11]*2.0;
  *(vars[4]) = rpars[9]*-2.0;
  *(vars[5]) = rpars[10]*-2.0;
  *(vars[6]) = rpars[11]*-2.0;
  *(vars[7]) = t32+t33+t34;
  *(vars[8]) = rpars[0]*rpars[3]*2.0+rpars[1]*rpars[4]*2.0+rpars[2]*rpars[5]*2.0;
  *(vars[9]) = rpars[0]*rpars[6]*2.0+rpars[1]*rpars[7]*2.0+rpars[2]*rpars[8]*2.0;
  *(vars[10]) = rpars[0]*-2.0;
  *(vars[11]) = rpars[1]*-2.0;
  *(vars[12]) = rpars[2]*-2.0;
  *(vars[13]) = t35+t36+t37;
  *(vars[14]) = rpars[3]*rpars[6]*2.0+rpars[4]*rpars[7]*2.0+rpars[5]*rpars[8]*2.0;
  *(vars[15]) = rpars[3]*-2.0;
  *(vars[16]) = rpars[4]*-2.0;
  *(vars[17]) = rpars[5]*-2.0;
  *(vars[18]) = t38+t39+t40;
  *(vars[19]) = rpars[6]*-2.0;
  *(vars[20]) = rpars[7]*-2.0;
  *(vars[21]) = rpars[8]*-2.0;
  *(vars[22]) = rpars[9]*rpars[21]*-2.0-rpars[10]*rpars[22]*2.0-rpars[11]*rpars[23]*2.0;
  *(vars[23]) = t6-rpars[11]*rpars[22]*4.0;
  *(vars[24]) = t3-rpars[9]*rpars[23]*4.0;
  *(vars[25]) = t2-rpars[10]*rpars[21]*4.0;
  *(vars[26]) = t4+t5-rpars[9]*rpars[21]*2.0;
  *(vars[27]) = -t2-rpars[10]*rpars[21]*4.0;
  *(vars[28]) = -t3-rpars[9]*rpars[23]*4.0;
  *(vars[29]) = -t4+t5+t7;
  *(vars[30]) = -t6-rpars[11]*rpars[22]*4.0;
  *(vars[31]) = t4-t5+t7;
  *(vars[32]) = rpars[0]*rpars[21]*-2.0-rpars[1]*rpars[22]*2.0-rpars[2]*rpars[23]*2.0;
  *(vars[33]) = t12-rpars[2]*rpars[22]*4.0;
  *(vars[34]) = t9-rpars[0]*rpars[23]*4.0;
  *(vars[35]) = t8-rpars[1]*rpars[21]*4.0;
  *(vars[36]) = t10+t11-rpars[0]*rpars[21]*2.0;
  *(vars[37]) = -t8-rpars[1]*rpars[21]*4.0;
  *(vars[38]) = -t9-rpars[0]*rpars[23]*4.0;
  *(vars[39]) = -t10+t11+t13;
  *(vars[40]) = -t12-rpars[2]*rpars[22]*4.0;
  *(vars[41]) = t10-t11+t13;
  *(vars[42]) = rpars[3]*rpars[21]*-2.0-rpars[4]*rpars[22]*2.0-rpars[5]*rpars[23]*2.0;
  *(vars[43]) = t18-rpars[5]*rpars[22]*4.0;
  *(vars[44]) = t15-rpars[3]*rpars[23]*4.0;
  *(vars[45]) = t14-rpars[4]*rpars[21]*4.0;
  *(vars[46]) = t16+t17-rpars[3]*rpars[21]*2.0;
  *(vars[47]) = -t14-rpars[4]*rpars[21]*4.0;
  *(vars[48]) = -t15-rpars[3]*rpars[23]*4.0;
  *(vars[49]) = -t16+t17+t19;
  *(vars[50]) = -t18-rpars[5]*rpars[22]*4.0;
  *(vars[51]) = t16-t17+t19;
  *(vars[52]) = rpars[6]*rpars[21]*-2.0-rpars[7]*rpars[22]*2.0-rpars[8]*rpars[23]*2.0;
  *(vars[53]) = t24-rpars[8]*rpars[22]*4.0;
  *(vars[54]) = t21-rpars[6]*rpars[23]*4.0;
  *(vars[55]) = t20-rpars[7]*rpars[21]*4.0;
  *(vars[56]) = t22+t23-rpars[6]*rpars[21]*2.0;
  *(vars[57]) = -t20-rpars[7]*rpars[21]*4.0;
  *(vars[58]) = -t21-rpars[6]*rpars[23]*4.0;
  *(vars[59]) = -t22+t23+t25;
  *(vars[60]) = -t24-rpars[8]*rpars[22]*4.0;
  *(vars[61]) = t22-t23+t25;
  *(vars[62]) = t27;
  *(vars[63]) = t28;
  *(vars[64]) = t29;
  *(vars[65]) = rpars[23]*-4.0;
  *(vars[66]) = t26;
  *(vars[67]) = t31;
  *(vars[68]) = rpars[21]*-4.0;
  *(vars[69]) = -t26;
  *(vars[70]) = t30;
  *(vars[71]) = t27;
  *(vars[72]) = -t28;
  *(vars[73]) = -t29;
  *(vars[74]) = t26;
  *(vars[75]) = t30;
  *(vars[76]) = t31;
  *(vars[77]) = t30;
  *(vars[78]) = -t27;
  *(vars[79]) = t28;
  *(vars[80]) = -t29;
  *(vars[81]) = t31;
  *(vars[82]) = t26;
  *(vars[83]) = -t27;
  *(vars[84]) = -t28;
  *(vars[85]) = t29;
  *(vars[86]) = t105;
  *(vars[87]) = t50;
  *(vars[88]) = t50;
  *(vars[89]) = t50;
  *(vars[90]) = rpars[0]*rpars[12]*-2.0-rpars[1]*rpars[13]*2.0-rpars[2]*rpars[14]*2.0-rpars[3]*
         rpars[15]*2.0-rpars[4]*rpars[16]*2.0-rpars[5]*rpars[17]*2.0-rpars[6]*rpars[18]*2.0-rpars[7]*
         rpars[19]*2.0-rpars[8]*rpars[20]*2.0;
  *(vars[91]) = t63+t64+t65-rpars[2]*rpars[13]*4.0-rpars[5]*rpars[16]*4.0-rpars[8]*rpars[19]*4.0;
  *(vars[92]) = t54+t55+t56-rpars[0]*rpars[14]*4.0-rpars[3]*rpars[17]*4.0-rpars[6]*rpars[20]*4.0;
  *(vars[93]) = t51+t52+t53-rpars[1]*rpars[12]*4.0-rpars[4]*rpars[15]*4.0-rpars[7]*rpars[18]*4.0;
  *(vars[94]) = t57+t58+t59+t60+t61+t62-rpars[0]*rpars[12]*2.0-rpars[3]*rpars[15]*2.0-rpars[6]*rpars[18]*2.0;
  *(vars[95]) = -t51-t52-t53-rpars[1]*rpars[12]*4.0-rpars[4]*rpars[15]*4.0-rpars[7]*rpars[18]*4.0;
  *(vars[96]) = -t54-t55-t56-rpars[0]*rpars[14]*4.0-rpars[3]*rpars[17]*4.0-rpars[6]*rpars[20]*4.0;
  *(vars[97]) = -t57+t58-t59+t60-t61+t62+t66+t67+t68;
  *(vars[98]) = -t63-t64-t65-rpars[2]*rpars[13]*4.0-rpars[5]*rpars[16]*4.0-rpars[8]*rpars[19]*4.0;
  *(vars[99]) = t57-t58+t59-t60+t61-t62+t66+t67+t68;
  *(vars[100]) = t69+t70+t71-rpars[6]*rpars[15]*4.0-rpars[7]*rpars[16]*4.0-rpars[8]*rpars[17]*4.0;
  *(vars[101]) = t78+t79-rpars[4]*rpars[20]*8.0-rpars[8]*rpars[16]*8.0;
  *(vars[102]) = t74+t75-rpars[5]*rpars[18]*8.0-rpars[6]*rpars[17]*8.0;
  *(vars[103]) = t72+t73-rpars[3]*rpars[19]*8.0-rpars[7]*rpars[15]*8.0;
  *(vars[104]) = t69-t70-t71+t76+t77-rpars[6]*rpars[15]*4.0;
  *(vars[105]) = t72-t73+t123-rpars[7]*rpars[15]*8.0;
  *(vars[106]) = t74-t75+t122-rpars[6]*rpars[17]*8.0;
  *(vars[107]) = -t69+t70-t71-t76+t77+t80;
  *(vars[108]) = t78-t79+t121-rpars[8]*rpars[16]*8.0;
  *(vars[109]) = -t69-t70+t71+t76-t77+t80;
  *(vars[110]) = t81+t82+t83-rpars[0]*rpars[18]*4.0-rpars[1]*rpars[19]*4.0-rpars[2]*rpars[20]*4.0;
  *(vars[111]) = t90+t91-rpars[2]*rpars[19]*8.0-rpars[7]*rpars[14]*8.0;
  *(vars[112]) = t86+t87-rpars[0]*rpars[20]*8.0-rpars[8]*rpars[12]*8.0;
  *(vars[113]) = t84+t85-rpars[1]*rpars[18]*8.0-rpars[6]*rpars[13]*8.0;
  *(vars[114]) = t81-t82-t83+t88+t89-rpars[0]*rpars[18]*4.0;
  *(vars[115]) = -t84+t85+t117-rpars[1]*rpars[18]*8.0;
  *(vars[116]) = -t86+t87+t116-rpars[0]*rpars[20]*8.0;
  *(vars[117]) = -t81+t82-t83-t88+t89+t92;
  *(vars[118]) = -t90+t91+t115-rpars[2]*rpars[19]*8.0;
  *(vars[119]) = -t81-t82+t83+t88-t89+t92;
  *(vars[120]) = t93+t94+t95-rpars[3]*rpars[12]*4.0-rpars[4]*rpars[13]*4.0-rpars[5]*rpars[14]*4.0;
  *(vars[121]) = t102+t103-rpars[1]*rpars[17]*8.0-rpars[5]*rpars[13]*8.0;
  *(vars[122]) = t98+t99-rpars[2]*rpars[15]*8.0-rpars[3]*rpars[14]*8.0;
  *(vars[123]) = t96+t97-rpars[0]*rpars[16]*8.0-rpars[4]*rpars[12]*8.0;
  *(vars[124]) = t93-t94-t95+t100+t101-rpars[3]*rpars[12]*4.0;
  *(vars[125]) = t96-t97+t114-rpars[4]*rpars[12]*8.0;
  *(vars[126]) = t98-t99+t113-rpars[3]*rpars[14]*8.0;
  *(vars[127]) = -t93+t94-t95-t100+t101+t104;
  *(vars[128]) = t102-t103+t112-rpars[5]*rpars[13]*8.0;
  *(vars[129]) = -t93-t94+t95+t100-t101+t104;
  *(vars[130]) = t105;
  *(vars[131]) = t50;
  *(vars[132]) = t50;
  *(vars[133]) = -t57-t58+t59+t60+t61+t62-t66+t67+t68;
  *(vars[134]) = t63-t64-t65+t110+t111-rpars[2]*rpars[13]*4.0;
  *(vars[135]) = t54-t55-t56+t108+t109-rpars[0]*rpars[14]*4.0;
  *(vars[136]) = t51-t52-t53+t106+t107-rpars[1]*rpars[12]*4.0;
  *(vars[137]) = t57+t58-t59-t60-t61-t62-t66+t67+t68;
  *(vars[138]) = -t51+t52+t53+t106+t107-rpars[1]*rpars[12]*4.0;
  *(vars[139]) = -t54+t55+t56+t108+t109-rpars[0]*rpars[14]*4.0;
  *(vars[140]) = -t57+t58+t59-t60+t61-t62+t66-t67-t68;
  *(vars[141]) = -t63+t64+t65+t110+t111-rpars[2]*rpars[13]*4.0;
  *(vars[142]) = t57-t58-t59+t60-t61+t62+t66-t67-t68;
  *(vars[143]) = -t93-t94-t95-t100-t101-t104;
  *(vars[144]) = -t102+t103+t112-rpars[5]*rpars[13]*8.0;
  *(vars[145]) = -t98+t99+t113-rpars[3]*rpars[14]*8.0;
  *(vars[146]) = -t96+t97+t114-rpars[4]*rpars[12]*8.0;
  *(vars[147]) = -t93+t94+t95+t100+t101-t104;
  *(vars[148]) = -t96-t97-t114-rpars[4]*rpars[12]*8.0;
  *(vars[149]) = -t98-t99-t113-rpars[3]*rpars[14]*8.0;
  *(vars[150]) = t93-t94+t95-t100+t101+t104;
  *(vars[151]) = -t102-t103-t112-rpars[5]*rpars[13]*8.0;
  *(vars[152]) = t93+t94-t95+t100-t101+t104;
  *(vars[153]) = -t81-t82-t83-t88-t89-t92;
  *(vars[154]) = t90-t91+t115-rpars[2]*rpars[19]*8.0;
  *(vars[155]) = t86-t87+t116-rpars[0]*rpars[20]*8.0;
  *(vars[156]) = t84-t85+t117-rpars[1]*rpars[18]*8.0;
  *(vars[157]) = -t81+t82+t83+t88+t89-t92;
  *(vars[158]) = -t84-t85-t117-rpars[1]*rpars[18]*8.0;
  *(vars[159]) = -t86-t87-t116-rpars[0]*rpars[20]*8.0;
  *(vars[160]) = t81-t82+t83-t88+t89+t92;
  *(vars[161]) = -t90-t91-t115-rpars[2]*rpars[19]*8.0;
  *(vars[162]) = t81+t82-t83+t88-t89+t92;
  *(vars[163]) = t105;
  *(vars[164]) = t50;
  *(vars[165]) = t57+t58-t59-t60+t61+t62+t66-t67+t68;
  *(vars[166]) = -t63+t64-t65-t110+t111+t120;
  *(vars[167]) = -t54+t55-t56-t108+t109+t119;
  *(vars[168]) = -t51+t52-t53-t106+t107+t118;
  *(vars[169]) = -t57-t58+t59+t60-t61-t62+t66-t67+t68;
  *(vars[170]) = t51-t52+t53-t106+t107+t118;
  *(vars[171]) = t54-t55+t56-t108+t109+t119;
  *(vars[172]) = t57-t58-t59+t60+t61-t62-t66+t67-t68;
  *(vars[173]) = t63-t64+t65-t110+t111+t120;
  *(vars[174]) = -t57+t58+t59-t60-t61+t62-t66+t67-t68;
  *(vars[175]) = -t69-t70-t71-t76-t77-t80;
  *(vars[176]) = -t78+t79+t121-rpars[8]*rpars[16]*8.0;
  *(vars[177]) = -t74+t75+t122-rpars[6]*rpars[17]*8.0;
  *(vars[178]) = -t72+t73+t123-rpars[7]*rpars[15]*8.0;
  *(vars[179]) = -t69+t70+t71+t76+t77-t80;
  *(vars[180]) = -t72-t73-t123-rpars[7]*rpars[15]*8.0;
  *(vars[181]) = -t74-t75-t122-rpars[6]*rpars[17]*8.0;
  *(vars[182]) = t69-t70+t71-t76+t77+t80;
  *(vars[183]) = -t78-t79-t121-rpars[8]*rpars[16]*8.0;
  *(vars[184]) = t69+t70-t71+t76-t77+t80;
  *(vars[185]) = t105;
  *(vars[186]) = t57+t58+t59+t60-t61-t62+t66+t67-t68;
  *(vars[187]) = -t63-t64+t65+t110-t111+t120;
  *(vars[188]) = -t54-t55+t56+t108-t109+t119;
  *(vars[189]) = -t51-t52+t53+t106-t107+t118;
  *(vars[190]) = -t57-t58-t59-t60+t61+t62+t66+t67-t68;
  *(vars[191]) = t51+t52-t53+t106-t107+t118;
  *(vars[192]) = t54+t55-t56+t108-t109+t119;
  *(vars[193]) = t57-t58+t59-t60-t61+t62-t66-t67+t68;
  *(vars[194]) = t63+t64-t65+t110-t111+t120;
  *(vars[195]) = -t57+t58-t59+t60+t61-t62-t66-t67+t68;
  *(vars[196]) = t149;
  *(vars[197]) = t148;
  *(vars[198]) = t148;
  *(vars[199]) = t148;
  *(vars[200]) = t149;
  *(vars[201]) = t148;
  *(vars[202]) = t148;
  *(vars[203]) = t149;
  *(vars[204]) = t148;
  *(vars[205]) = t149;
}

void HerwcProblem::evalSdpConstraintsVariables(const double *pars, const double *vmul, double **vars) const {

  double aux[3];

  updateAuxiliaries(pars, aux);

  *(vars[0]) = vmul[0] * (aux[0]);  *(vars[1]) = vmul[1] * (aux[0]);  *(vars[2]) = vmul[2] * (aux[0]);  *(vars[3]) = vmul[3] * (aux[0]);
  *(vars[4]) = vmul[4] * (aux[0]);  *(vars[5]) = vmul[5] * (aux[0]);  *(vars[6]) = vmul[6] * (aux[0]);  *(vars[7]) = vmul[7] * (aux[0]);
  *(vars[8]) = vmul[8] * (aux[0]);  *(vars[9]) = vmul[9] * (aux[0]);  *(vars[10]) = vmul[10] * (aux[0]);  *(vars[11]) = vmul[11] * (aux[0]);
  *(vars[12]) = vmul[12] * (aux[0]);  *(vars[13]) = vmul[13] * (aux[0]);  *(vars[14]) = vmul[14] * (aux[0]);  *(vars[15]) = vmul[15] * (aux[0]);
  *(vars[16]) = vmul[16] * (aux[0]);  *(vars[17]) = vmul[17] * (aux[0]);  *(vars[18]) = vmul[18] * (aux[0]);  *(vars[19]) = vmul[19] * (aux[0]);
  *(vars[20]) = vmul[20] * (aux[0]);  *(vars[21]) = vmul[21] * (aux[0]);  *(vars[22]) = vmul[22] * (aux[0]);  *(vars[23]) = vmul[23] * (aux[0]);
  *(vars[24]) = vmul[24] * (aux[0]);  *(vars[25]) = vmul[25] * (aux[0]);  *(vars[26]) = vmul[26] * (aux[0]);  *(vars[27]) = vmul[27] * (aux[0]);
  *(vars[28]) = vmul[28] * (aux[0]);  *(vars[29]) = vmul[29] * (aux[0]);  *(vars[30]) = vmul[30] * (aux[0]);  *(vars[31]) = vmul[31] * (aux[0]);
  *(vars[32]) = vmul[32] * (aux[0]);  *(vars[33]) = vmul[33] * (aux[0]);  *(vars[34]) = vmul[34] * (aux[0]);  *(vars[35]) = vmul[35] * (aux[0]);
  *(vars[36]) = vmul[36] * (aux[0]);  *(vars[37]) = vmul[37] * (aux[0]);  *(vars[38]) = vmul[38] * (aux[0]);  *(vars[39]) = vmul[39] * (aux[0]);
  *(vars[40]) = vmul[40] * (aux[0]);  *(vars[41]) = vmul[41] * (aux[0]);  *(vars[42]) = vmul[42] * (aux[0]);  *(vars[43]) = vmul[43] * (aux[0]);
  *(vars[44]) = vmul[44] * (aux[0]);  *(vars[45]) = vmul[45] * (aux[0]);  *(vars[46]) = vmul[46] * (aux[0]);  *(vars[47]) = vmul[47] * (aux[0]);
  *(vars[48]) = vmul[48] * (aux[0]);  *(vars[49]) = vmul[49] * (aux[0]);  *(vars[50]) = vmul[50] * (aux[0]);  *(vars[51]) = vmul[51] * (aux[0]);
  *(vars[52]) = vmul[52] * (aux[0]);  *(vars[53]) = vmul[53] * (aux[0]);  *(vars[54]) = vmul[54] * (aux[0]);  *(vars[55]) = vmul[55] * (aux[0]);
  *(vars[56]) = vmul[56] * (aux[0]);  *(vars[57]) = vmul[57] * (aux[0]);  *(vars[58]) = vmul[58] * (aux[0]);  *(vars[59]) = vmul[59] * (aux[0]);
  *(vars[60]) = vmul[60] * (aux[0]);  *(vars[61]) = vmul[61] * (aux[0]);  *(vars[62]) = vmul[62] * (aux[0]);  *(vars[63]) = vmul[63] * (aux[0]);
  *(vars[64]) = vmul[64] * (aux[0]);  *(vars[65]) = vmul[65] * (aux[0]);  *(vars[66]) = vmul[66] * (aux[0]);  *(vars[67]) = vmul[67] * (aux[0]);
  *(vars[68]) = vmul[68] * (aux[0]);  *(vars[69]) = vmul[69] * (aux[0]);  *(vars[70]) = vmul[70] * (aux[0]);  *(vars[71]) = vmul[71] * (aux[0]);
  *(vars[72]) = vmul[72] * (aux[0]);  *(vars[73]) = vmul[73] * (aux[0]);  *(vars[74]) = vmul[74] * (aux[0]);  *(vars[75]) = vmul[75] * (aux[0]);
  *(vars[76]) = vmul[76] * (aux[0]);  *(vars[77]) = vmul[77] * (aux[0]);  *(vars[78]) = vmul[78] * (aux[0]);  *(vars[79]) = vmul[79] * (aux[0]);
  *(vars[80]) = vmul[80] * (aux[0]);  *(vars[81]) = vmul[81] * (aux[0]);  *(vars[82]) = vmul[82] * (aux[0]);  *(vars[83]) = vmul[83] * (aux[0]);
  *(vars[84]) = vmul[84] * (aux[0]);  *(vars[85]) = vmul[85] * (aux[0]);  *(vars[86]) = vmul[86] * (aux[0]);  *(vars[87]) = vmul[87] * (aux[0]);
  *(vars[88]) = vmul[88] * (aux[0]);  *(vars[89]) = vmul[89] * (aux[0]);  *(vars[90]) = vmul[90] * (aux[0]);  *(vars[91]) = vmul[91] * (aux[0]);
  *(vars[92]) = vmul[92] * (aux[0]);  *(vars[93]) = vmul[93] * (aux[0]);  *(vars[94]) = vmul[94] * (aux[0]);  *(vars[95]) = vmul[95] * (aux[0]);
  *(vars[96]) = vmul[96] * (aux[0]);  *(vars[97]) = vmul[97] * (aux[0]);  *(vars[98]) = vmul[98] * (aux[0]);  *(vars[99]) = vmul[99] * (aux[0]);
  *(vars[100]) = vmul[100] * (aux[0]);  *(vars[101]) = vmul[101] * (aux[0]);  *(vars[102]) = vmul[102] * (aux[0]);  *(vars[103]) = vmul[103] * (aux[0]);
  *(vars[104]) = vmul[104] * (aux[0]);  *(vars[105]) = vmul[105] * (aux[0]);  *(vars[106]) = vmul[106] * (aux[0]);  *(vars[107]) = vmul[107] * (aux[0]);
  *(vars[108]) = vmul[108] * (aux[0]);  *(vars[109]) = vmul[109] * (aux[0]);  *(vars[110]) = vmul[110] * (aux[0]);  *(vars[111]) = vmul[111] * (aux[0]);
  *(vars[112]) = vmul[112] * (aux[0]);  *(vars[113]) = vmul[113] * (aux[0]);  *(vars[114]) = vmul[114] * (aux[0]);  *(vars[115]) = vmul[115] * (aux[0]);
  *(vars[116]) = vmul[116] * (aux[0]);  *(vars[117]) = vmul[117] * (aux[0]);  *(vars[118]) = vmul[118] * (aux[0]);  *(vars[119]) = vmul[119] * (aux[0]);
  *(vars[120]) = vmul[120] * (aux[1]);  *(vars[121]) = vmul[121] * (aux[1]);  *(vars[122]) = vmul[122] * (aux[1]);  *(vars[123]) = vmul[123] * (aux[1]);
  *(vars[124]) = vmul[124] * (aux[1]);  *(vars[125]) = vmul[125] * (aux[1]);  *(vars[126]) = vmul[126] * (aux[1]);  *(vars[127]) = vmul[127] * (aux[1]);
  *(vars[128]) = vmul[128] * (aux[1]);  *(vars[129]) = vmul[129] * (aux[1]);  *(vars[130]) = vmul[130] * (aux[1]);  *(vars[131]) = vmul[131] * (aux[1]);
  *(vars[132]) = vmul[132] * (aux[1]);  *(vars[133]) = vmul[133] * (aux[1]);  *(vars[134]) = vmul[134] * (aux[1]);  *(vars[135]) = vmul[135] * (aux[1]);
  *(vars[136]) = vmul[136] * (aux[1]);  *(vars[137]) = vmul[137] * (aux[1]);  *(vars[138]) = vmul[138] * (aux[1]);  *(vars[139]) = vmul[139] * (aux[1]);
  *(vars[140]) = vmul[140] * (aux[1]);  *(vars[141]) = vmul[141] * (aux[1]);  *(vars[142]) = vmul[142] * (aux[1]);  *(vars[143]) = vmul[143] * (aux[1]);
  *(vars[144]) = vmul[144] * (aux[1]);  *(vars[145]) = vmul[145] * (aux[1]);  *(vars[146]) = vmul[146] * (aux[1]);  *(vars[147]) = vmul[147] * (aux[1]);
  *(vars[148]) = vmul[148] * (aux[1]);  *(vars[149]) = vmul[149] * (aux[1]);  *(vars[150]) = vmul[150] * (aux[1]);  *(vars[151]) = vmul[151] * (aux[1]);
  *(vars[152]) = vmul[152] * (aux[1]);  *(vars[153]) = vmul[153] * (aux[1]);  *(vars[154]) = vmul[154] * (aux[1]);  *(vars[155]) = vmul[155] * (aux[1]);
  *(vars[156]) = vmul[156] * (aux[1]);  *(vars[157]) = vmul[157] * (aux[1]);  *(vars[158]) = vmul[158] * (aux[1]);  *(vars[159]) = vmul[159] * (aux[1]);
  *(vars[160]) = vmul[160] * (aux[1]);  *(vars[161]) = vmul[161] * (aux[1]);  *(vars[162]) = vmul[162] * (aux[1]);  *(vars[163]) = vmul[163] * (aux[1]);
  *(vars[164]) = vmul[164] * (aux[1]);  *(vars[165]) = vmul[165] * (aux[1]);  *(vars[166]) = vmul[166] * (aux[1]);  *(vars[167]) = vmul[167] * (aux[1]);
  *(vars[168]) = vmul[168] * (aux[1]);  *(vars[169]) = vmul[169] * (aux[1]);  *(vars[170]) = vmul[170] * (aux[1]);  *(vars[171]) = vmul[171] * (aux[1]);
  *(vars[172]) = vmul[172] * (aux[1]);  *(vars[173]) = vmul[173] * (aux[1]);  *(vars[174]) = vmul[174] * (aux[1]);  *(vars[175]) = vmul[175] * (aux[1]);
  *(vars[176]) = vmul[176] * (aux[1]);  *(vars[177]) = vmul[177] * (aux[1]);  *(vars[178]) = vmul[178] * (aux[1]);  *(vars[179]) = vmul[179] * (aux[1]);
  *(vars[180]) = vmul[180] * (aux[1]);  *(vars[181]) = vmul[181] * (aux[1]);  *(vars[182]) = vmul[182] * (aux[1]);  *(vars[183]) = vmul[183] * (aux[1]);
  *(vars[184]) = vmul[184] * (aux[1]);  *(vars[185]) = vmul[185] * (aux[1]);  *(vars[186]) = vmul[186] * (aux[1]);  *(vars[187]) = vmul[187] * (aux[1]);
  *(vars[188]) = vmul[188] * (aux[1]);  *(vars[189]) = vmul[189] * (aux[1]);  *(vars[190]) = vmul[190] * (aux[1]);  *(vars[191]) = vmul[191] * (aux[1]);
  *(vars[192]) = vmul[192] * (aux[1]);  *(vars[193]) = vmul[193] * (aux[1]);  *(vars[194]) = vmul[194] * (aux[1]);  *(vars[195]) = vmul[195] * (aux[1]);
  *(vars[196]) = vmul[196] * (aux[1]);  *(vars[197]) = vmul[197] * (aux[1]);  *(vars[198]) = vmul[198] * (aux[1]);  *(vars[199]) = vmul[199] * (aux[1]);
  *(vars[200]) = vmul[200] * (aux[1]);  *(vars[201]) = vmul[201] * (aux[1]);  *(vars[202]) = vmul[202] * (aux[1]);  *(vars[203]) = vmul[203] * (aux[1]);
  *(vars[204]) = vmul[204] * (aux[1]);  *(vars[205]) = vmul[205] * (aux[1]);  *(vars[206]) = vmul[206] * (aux[1]);  *(vars[207]) = vmul[207] * (aux[1]);
  *(vars[208]) = vmul[208] * (aux[1]);  *(vars[209]) = vmul[209] * (aux[1]);  *(vars[210]) = vmul[210] * (aux[1]);  *(vars[211]) = vmul[211] * (aux[1]);
  *(vars[212]) = vmul[212] * (aux[1]);  *(vars[213]) = vmul[213] * (aux[1]);  *(vars[214]) = vmul[214] * (aux[1]);  *(vars[215]) = vmul[215] * (aux[1]);
  *(vars[216]) = vmul[216] * (aux[1]);  *(vars[217]) = vmul[217] * (aux[1]);  *(vars[218]) = vmul[218] * (aux[1]);  *(vars[219]) = vmul[219] * (aux[1]);
  *(vars[220]) = vmul[220] * (aux[1]);  *(vars[221]) = vmul[221] * (aux[1]);  *(vars[222]) = vmul[222] * (aux[1]);  *(vars[223]) = vmul[223] * (aux[1]);
  *(vars[224]) = vmul[224] * (aux[1]);  *(vars[225]) = vmul[225] * (aux[1]);  *(vars[226]) = vmul[226] * (aux[1]);  *(vars[227]) = vmul[227] * (aux[1]);
  *(vars[228]) = vmul[228] * (aux[1]);  *(vars[229]) = vmul[229] * (aux[1]);  *(vars[230]) = vmul[230] * (aux[1]);  *(vars[231]) = vmul[231] * (aux[1]);
  *(vars[232]) = vmul[232] * (aux[1]);  *(vars[233]) = vmul[233] * (aux[1]);  *(vars[234]) = vmul[234] * (aux[1]);  *(vars[235]) = vmul[235] * (aux[1]);
  *(vars[236]) = vmul[236] * (aux[1]);  *(vars[237]) = vmul[237] * (aux[1]);  *(vars[238]) = vmul[238] * (aux[1]);  *(vars[239]) = vmul[239] * (aux[1]);
}

const int HerwcProblem::num_blocks = 9;
const int HerwcProblem::num_cons = 3059;
const int HerwcProblem::num_pcons = 8;
const int HerwcProblem::num_pvars = 14;
const int HerwcProblem::num_rpars = 24;
const int HerwcProblem::num_ppars = 2;
const int HerwcProblem::problem_size = 240;
const int HerwcProblem::relax_order = 2;
const int HerwcProblem::max_degree = 4;
const int HerwcProblem::rank_shift = 1;
const int HerwcProblem::block_sizes[] = {120, 15, 15, 15, 15, 15, 15, 15, 15};

const int HerwcProblem::num_obj_data = 209;
const int HerwcProblem::num_obj_vars = 206;
const HerwcProblem::objData  HerwcProblem::obj_data[] = {
  {0, 0.0},  {5, 0.0},  {6, 0.0},  {7, 0.0},  {12, 0.0},  {13, 0.0},
  {14, 0.0},  {65, 0.0},  {66, 0.0},  {67, 0.0},  {72, 0.0},  {73, 0.0},
  {74, 0.0},  {75, 0.0},  {76, 0.0},  {81, 0.0},  {82, 0.0},  {83, 0.0},
  {84, 0.0},  {89, 0.0},  {90, 0.0},  {91, 0.0},  {92, 0.0},  {93, 0.0},
  {94, 0.0},  {95, 0.0},  {99, 0.0},  {100, 0.0},  {101, 0.0},  {105, 0.0},
  {106, 0.0},  {110, 0.0},  {114, 1},  {117, 1},  {119, 1},  {487, 0.0},
  {488, 0.0},  {489, 0.0},  {490, 0.0},  {494, 0.0},  {495, 0.0},  {496, 0.0},
  {500, 0.0},  {501, 0.0},  {505, 0.0},  {532, 0.0},  {533, 0.0},  {534, 0.0},
  {535, 0.0},  {539, 0.0},  {540, 0.0},  {541, 0.0},  {545, 0.0},  {546, 0.0},
  {550, 0.0},  {568, 0.0},  {569, 0.0},  {570, 0.0},  {571, 0.0},  {575, 0.0},
  {576, 0.0},  {577, 0.0},  {581, 0.0},  {582, 0.0},  {586, 0.0},  {600, 0.0},
  {601, 0.0},  {602, 0.0},  {607, 0.0},  {608, 0.0},  {611, 0.0},  {613, 0.0},
  {615, 0.0},  {616, 0.0},  {627, 0.0},  {628, 0.0},  {629, 0.0},  {632, 0.0},
  {633, 0.0},  {636, 0.0},  {638, 0.0},  {647, 0.0},  {648, 0.0},  {649, 0.0},
  {652, 0.0},  {653, 0.0},  {661, 0.0},  {662, 0.0},  {663, 0.0},  {680, 0.0},
  {694, 0.0},  {707, 0.0},  {719, 0.0},  {757, 0.0},  {758, 0.0},  {759, 0.0},
  {760, 0.0},  {764, 0.0},  {765, 0.0},  {766, 0.0},  {770, 0.0},  {771, 0.0},
  {775, 0.0},  {848, 0.0},  {849, 0.0},  {850, 0.0},  {851, 0.0},  {855, 0.0},
  {856, 0.0},  {857, 0.0},  {861, 0.0},  {862, 0.0},  {866, 0.0},  {926, 0.0},
  {927, 0.0},  {928, 0.0},  {929, 0.0},  {933, 0.0},  {934, 0.0},  {935, 0.0},
  {939, 0.0},  {940, 0.0},  {944, 0.0},  {992, 0.0},  {993, 0.0},  {994, 0.0},
  {995, 0.0},  {999, 0.0},  {1000, 0.0},  {1001, 0.0},  {1005, 0.0},  {1006, 0.0},
  {1010, 0.0},  {1240, 0.0},  {1253, 0.0},  {1265, 0.0},  {1303, 0.0},  {1304, 0.0},
  {1305, 0.0},  {1306, 0.0},  {1310, 0.0},  {1311, 0.0},  {1312, 0.0},  {1316, 0.0},
  {1317, 0.0},  {1321, 0.0},  {1381, 0.0},  {1382, 0.0},  {1383, 0.0},  {1384, 0.0},
  {1388, 0.0},  {1389, 0.0},  {1390, 0.0},  {1394, 0.0},  {1395, 0.0},  {1399, 0.0},
  {1447, 0.0},  {1448, 0.0},  {1449, 0.0},  {1450, 0.0},  {1454, 0.0},  {1455, 0.0},
  {1456, 0.0},  {1460, 0.0},  {1461, 0.0},  {1465, 0.0},  {1695, 0.0},  {1707, 0.0},
  {1745, 0.0},  {1746, 0.0},  {1747, 0.0},  {1748, 0.0},  {1752, 0.0},  {1753, 0.0},
  {1754, 0.0},  {1758, 0.0},  {1759, 0.0},  {1763, 0.0},  {1811, 0.0},  {1812, 0.0},
  {1813, 0.0},  {1814, 0.0},  {1818, 0.0},  {1819, 0.0},  {1820, 0.0},  {1824, 0.0},
  {1825, 0.0},  {1829, 0.0},  {2059, 0.0},  {2097, 0.0},  {2098, 0.0},  {2099, 0.0},
  {2100, 0.0},  {2104, 0.0},  {2105, 0.0},  {2106, 0.0},  {2110, 0.0},  {2111, 0.0},
  {2115, 0.0},  {2850, 0.0},  {2857, 0.0},  {2863, 0.0},  {2868, 0.0},  {2934, 0.0},
  {2940, 0.0},  {2945, 0.0},  {2990, 0.0},  {2995, 0.0},  {3025, 0.0}
};
const HerwcProblem::objVar HerwcProblem::obj_vars[] = {
  {0},  {5},  {6},  {7},  {12},  {13},
  {14},  {65},  {66},  {67},  {72},  {73},
  {74},  {75},  {76},  {81},  {82},  {83},
  {84},  {89},  {90},  {91},  {92},  {93},
  {94},  {95},  {99},  {100},  {101},  {105},
  {106},  {110},  {487},  {488},  {489},  {490},
  {494},  {495},  {496},  {500},  {501},  {505},
  {532},  {533},  {534},  {535},  {539},  {540},
  {541},  {545},  {546},  {550},  {568},  {569},
  {570},  {571},  {575},  {576},  {577},  {581},
  {582},  {586},  {600},  {601},  {602},  {607},
  {608},  {611},  {613},  {615},  {616},  {627},
  {628},  {629},  {632},  {633},  {636},  {638},
  {647},  {648},  {649},  {652},  {653},  {661},
  {662},  {663},  {680},  {694},  {707},  {719},
  {757},  {758},  {759},  {760},  {764},  {765},
  {766},  {770},  {771},  {775},  {848},  {849},
  {850},  {851},  {855},  {856},  {857},  {861},
  {862},  {866},  {926},  {927},  {928},  {929},
  {933},  {934},  {935},  {939},  {940},  {944},
  {992},  {993},  {994},  {995},  {999},  {1000},
  {1001},  {1005},  {1006},  {1010},  {1240},  {1253},
  {1265},  {1303},  {1304},  {1305},  {1306},  {1310},
  {1311},  {1312},  {1316},  {1317},  {1321},  {1381},
  {1382},  {1383},  {1384},  {1388},  {1389},  {1390},
  {1394},  {1395},  {1399},  {1447},  {1448},  {1449},
  {1450},  {1454},  {1455},  {1456},  {1460},  {1461},
  {1465},  {1695},  {1707},  {1745},  {1746},  {1747},
  {1748},  {1752},  {1753},  {1754},  {1758},  {1759},
  {1763},  {1811},  {1812},  {1813},  {1814},  {1818},
  {1819},  {1820},  {1824},  {1825},  {1829},  {2059},
  {2097},  {2098},  {2099},  {2100},  {2104},  {2105},
  {2106},  {2110},  {2111},  {2115},  {2850},  {2857},
  {2863},  {2868},  {2934},  {2940},  {2945},  {2990},
  {2995},  {3025}
};

const int HerwcProblem::num_con_data = 10860;
const int HerwcProblem::num_con_vars = 240;
const HerwcProblem::conData HerwcProblem::con_data[] = {
  {0, 0, 0, 0, 1.0},  {0, 1, 0, 1, 1.0},  {0, 2, 0, 2, 1.0},  {0, 3, 0, 3, 1.0},  {0, 4, 0, 4, 1.0},
  {0, 5, 0, 5, 1.0},  {0, 6, 0, 6, 1.0},  {0, 7, 0, 7, 1.0},  {0, 8, 0, 8, 1.0},  {0, 9, 0, 9, 1.0},  {0, 10, 0, 10, 1.0},
  {0, 11, 0, 11, 1.0},  {0, 12, 0, 12, 1.0},  {0, 13, 0, 13, 1.0},  {0, 14, 0, 14, 1.0},  {0, 15, 0, 15, 1.0},  {0, 15, 1, 1, 1.0},
  {0, 16, 0, 16, 1.0},  {0, 16, 1, 2, 1.0},  {0, 17, 0, 17, 1.0},  {0, 17, 1, 3, 1.0},  {0, 18, 0, 18, 1.0},  {0, 18, 1, 4, 1.0},
  {0, 19, 0, 19, 1.0},  {0, 19, 1, 5, 1.0},  {0, 20, 0, 20, 1.0},  {0, 20, 1, 6, 1.0},  {0, 21, 0, 21, 1.0},  {0, 21, 1, 7, 1.0},
  {0, 22, 0, 22, 1.0},  {0, 22, 1, 8, 1.0},  {0, 23, 0, 23, 1.0},  {0, 23, 1, 9, 1.0},  {0, 24, 0, 24, 1.0},  {0, 24, 1, 10, 1.0},
  {0, 25, 0, 25, 1.0},  {0, 25, 1, 11, 1.0},  {0, 26, 0, 26, 1.0},  {0, 26, 1, 12, 1.0},  {0, 27, 0, 27, 1.0},  {0, 27, 1, 13, 1.0},
  {0, 28, 0, 28, 1.0},  {0, 28, 1, 14, 1.0},  {0, 29, 0, 29, 1.0},  {0, 29, 2, 2, 1.0},  {0, 30, 0, 30, 1.0},  {0, 30, 2, 3, 1.0},
  {0, 31, 0, 31, 1.0},  {0, 31, 2, 4, 1.0},  {0, 32, 0, 32, 1.0},  {0, 32, 2, 5, 1.0},  {0, 33, 0, 33, 1.0},  {0, 33, 2, 6, 1.0},
  {0, 34, 0, 34, 1.0},  {0, 34, 2, 7, 1.0},  {0, 35, 0, 35, 1.0},  {0, 35, 2, 8, 1.0},  {0, 36, 0, 36, 1.0},  {0, 36, 2, 9, 1.0},
  {0, 37, 0, 37, 1.0},  {0, 37, 2, 10, 1.0},  {0, 38, 0, 38, 1.0},  {0, 38, 2, 11, 1.0},  {0, 39, 0, 39, 1.0},  {0, 39, 2, 12, 1.0},
  {0, 40, 0, 40, 1.0},  {0, 40, 2, 13, 1.0},  {0, 41, 0, 41, 1.0},  {0, 41, 2, 14, 1.0},  {0, 42, 0, 42, 1.0},  {0, 42, 3, 3, 1.0},
  {0, 43, 0, 43, 1.0},  {0, 43, 3, 4, 1.0},  {0, 44, 0, 44, 1.0},  {0, 44, 3, 5, 1.0},  {0, 45, 0, 45, 1.0},  {0, 45, 3, 6, 1.0},
  {0, 46, 0, 46, 1.0},  {0, 46, 3, 7, 1.0},  {0, 47, 0, 47, 1.0},  {0, 47, 3, 8, 1.0},  {0, 48, 0, 48, 1.0},  {0, 48, 3, 9, 1.0},
  {0, 49, 0, 49, 1.0},  {0, 49, 3, 10, 1.0},  {0, 50, 0, 50, 1.0},  {0, 50, 3, 11, 1.0},  {0, 51, 0, 51, 1.0},  {0, 51, 3, 12, 1.0},
  {0, 52, 0, 52, 1.0},  {0, 52, 3, 13, 1.0},  {0, 53, 0, 53, 1.0},  {0, 53, 3, 14, 1.0},  {0, 54, 0, 54, 1.0},  {0, 54, 4, 4, 1.0},
  {0, 55, 0, 55, 1.0},  {0, 55, 4, 5, 1.0},  {0, 56, 0, 56, 1.0},  {0, 56, 4, 6, 1.0},  {0, 57, 0, 57, 1.0},  {0, 57, 4, 7, 1.0},
  {0, 58, 0, 58, 1.0},  {0, 58, 4, 8, 1.0},  {0, 59, 0, 59, 1.0},  {0, 59, 4, 9, 1.0},  {0, 60, 0, 60, 1.0},  {0, 60, 4, 10, 1.0},
  {0, 61, 0, 61, 1.0},  {0, 61, 4, 11, 1.0},  {0, 62, 0, 62, 1.0},  {0, 62, 4, 12, 1.0},  {0, 63, 0, 63, 1.0},  {0, 63, 4, 13, 1.0},
  {0, 64, 0, 64, 1.0},  {0, 64, 4, 14, 1.0},  {0, 65, 0, 65, 1.0},  {0, 65, 5, 5, 1.0},  {0, 66, 0, 66, 1.0},  {0, 66, 5, 6, 1.0},
  {0, 67, 0, 67, 1.0},  {0, 67, 5, 7, 1.0},  {0, 68, 0, 68, 1.0},  {0, 68, 5, 8, 1.0},  {0, 69, 0, 69, 1.0},  {0, 69, 5, 9, 1.0},
  {0, 70, 0, 70, 1.0},  {0, 70, 5, 10, 1.0},  {0, 71, 0, 71, 1.0},  {0, 71, 5, 11, 1.0},  {0, 72, 0, 72, 1.0},  {0, 72, 5, 12, 1.0},
  {0, 73, 0, 73, 1.0},  {0, 73, 5, 13, 1.0},  {0, 74, 0, 74, 1.0},  {0, 74, 5, 14, 1.0},  {0, 75, 0, 75, 1.0},  {0, 75, 6, 6, 1.0},
  {0, 76, 0, 76, 1.0},  {0, 76, 6, 7, 1.0},  {0, 77, 0, 77, 1.0},  {0, 77, 6, 8, 1.0},  {0, 78, 0, 78, 1.0},  {0, 78, 6, 9, 1.0},
  {0, 79, 0, 79, 1.0},  {0, 79, 6, 10, 1.0},  {0, 80, 0, 80, 1.0},  {0, 80, 6, 11, 1.0},  {0, 81, 0, 81, 1.0},  {0, 81, 6, 12, 1.0},
  {0, 82, 0, 82, 1.0},  {0, 82, 6, 13, 1.0},  {0, 83, 0, 83, 1.0},  {0, 83, 6, 14, 1.0},  {0, 84, 0, 84, 1.0},  {0, 84, 7, 7, 1.0},
  {0, 85, 0, 85, 1.0},  {0, 85, 7, 8, 1.0},  {0, 86, 0, 86, 1.0},  {0, 86, 7, 9, 1.0},  {0, 87, 0, 87, 1.0},  {0, 87, 7, 10, 1.0},
  {0, 88, 0, 88, 1.0},  {0, 88, 7, 11, 1.0},  {0, 89, 0, 89, 1.0},  {0, 89, 7, 12, 1.0},  {0, 90, 0, 90, 1.0},  {0, 90, 7, 13, 1.0},
  {0, 91, 0, 91, 1.0},  {0, 91, 7, 14, 1.0},  {0, 92, 0, 92, 1.0},  {0, 92, 8, 8, 1.0},  {0, 93, 0, 93, 1.0},  {0, 93, 8, 9, 1.0},
  {0, 94, 0, 94, 1.0},  {0, 94, 8, 10, 1.0},  {0, 95, 0, 95, 1.0},  {0, 95, 8, 11, 1.0},  {0, 96, 0, 96, 1.0},  {0, 96, 8, 12, 1.0},
  {0, 97, 0, 97, 1.0},  {0, 97, 8, 13, 1.0},  {0, 98, 0, 98, 1.0},  {0, 98, 8, 14, 1.0},  {0, 99, 0, 99, 1.0},  {0, 99, 9, 9, 1.0},
  {0, 100, 0, 100, 1.0},  {0, 100, 9, 10, 1.0},  {0, 101, 0, 101, 1.0},  {0, 101, 9, 11, 1.0},  {0, 102, 0, 102, 1.0},  {0, 102, 9, 12, 1.0},
  {0, 103, 0, 103, 1.0},  {0, 103, 9, 13, 1.0},  {0, 104, 0, 104, 1.0},  {0, 104, 9, 14, 1.0},  {0, 105, 0, 105, 1.0},  {0, 105, 10, 10, 1.0},
  {0, 106, 0, 106, 1.0},  {0, 106, 10, 11, 1.0},  {0, 107, 0, 107, 1.0},  {0, 107, 10, 12, 1.0},  {0, 108, 0, 108, 1.0},  {0, 108, 10, 13, 1.0},
  {0, 109, 0, 109, 1.0},  {0, 109, 10, 14, 1.0},  {0, 110, 0, 110, 1.0},  {0, 110, 11, 11, 1.0},  {0, 111, 0, 111, 1.0},  {0, 111, 11, 12, 1.0},
  {0, 112, 0, 112, 1.0},  {0, 112, 11, 13, 1.0},  {0, 113, 0, 113, 1.0},  {0, 113, 11, 14, 1.0},  {0, 114, 0, 114, 1.0},  {0, 114, 12, 12, 1.0},
  {0, 115, 0, 115, 1.0},  {0, 115, 12, 13, 1.0},  {0, 116, 0, 116, 1.0},  {0, 116, 12, 14, 1.0},  {0, 117, 0, 117, 1.0},  {0, 117, 13, 13, 1.0},
  {0, 118, 0, 118, 1.0},  {0, 118, 13, 14, 1.0},  {0, 119, 0, 119, 1.0},  {0, 119, 14, 14, 1.0},  {0, 120, 1, 15, 1.0},  {0, 121, 1, 16, 1.0},
  {0, 121, 2, 15, 1.0},  {0, 122, 1, 17, 1.0},  {0, 122, 3, 15, 1.0},  {0, 123, 1, 18, 1.0},  {0, 123, 4, 15, 1.0},  {0, 124, 1, 19, 1.0},
  {0, 124, 5, 15, 1.0},  {0, 125, 1, 20, 1.0},  {0, 125, 6, 15, 1.0},  {0, 126, 1, 21, 1.0},  {0, 126, 7, 15, 1.0},  {0, 127, 1, 22, 1.0},
  {0, 127, 8, 15, 1.0},  {0, 128, 1, 23, 1.0},  {0, 128, 9, 15, 1.0},  {0, 129, 1, 24, 1.0},  {0, 129, 10, 15, 1.0},  {0, 130, 1, 25, 1.0},
  {0, 130, 11, 15, 1.0},  {0, 131, 1, 26, 1.0},  {0, 131, 12, 15, 1.0},  {0, 132, 1, 27, 1.0},  {0, 132, 13, 15, 1.0},  {0, 133, 1, 28, 1.0},
  {0, 133, 14, 15, 1.0},  {0, 134, 1, 29, 1.0},  {0, 134, 2, 16, 1.0},  {0, 135, 1, 30, 1.0},  {0, 135, 2, 17, 1.0},  {0, 135, 3, 16, 1.0},
  {0, 136, 1, 31, 1.0},  {0, 136, 2, 18, 1.0},  {0, 136, 4, 16, 1.0},  {0, 137, 1, 32, 1.0},  {0, 137, 2, 19, 1.0},  {0, 137, 5, 16, 1.0},
  {0, 138, 1, 33, 1.0},  {0, 138, 2, 20, 1.0},  {0, 138, 6, 16, 1.0},  {0, 139, 1, 34, 1.0},  {0, 139, 2, 21, 1.0},  {0, 139, 7, 16, 1.0},
  {0, 140, 1, 35, 1.0},  {0, 140, 2, 22, 1.0},  {0, 140, 8, 16, 1.0},  {0, 141, 1, 36, 1.0},  {0, 141, 2, 23, 1.0},  {0, 141, 9, 16, 1.0},
  {0, 142, 1, 37, 1.0},  {0, 142, 2, 24, 1.0},  {0, 142, 10, 16, 1.0},  {0, 143, 1, 38, 1.0},  {0, 143, 2, 25, 1.0},  {0, 143, 11, 16, 1.0},
  {0, 144, 1, 39, 1.0},  {0, 144, 2, 26, 1.0},  {0, 144, 12, 16, 1.0},  {0, 145, 1, 40, 1.0},  {0, 145, 2, 27, 1.0},  {0, 145, 13, 16, 1.0},
  {0, 146, 1, 41, 1.0},  {0, 146, 2, 28, 1.0},  {0, 146, 14, 16, 1.0},  {0, 147, 1, 42, 1.0},  {0, 147, 3, 17, 1.0},  {0, 148, 1, 43, 1.0},
  {0, 148, 3, 18, 1.0},  {0, 148, 4, 17, 1.0},  {0, 149, 1, 44, 1.0},  {0, 149, 3, 19, 1.0},  {0, 149, 5, 17, 1.0},  {0, 150, 1, 45, 1.0},
  {0, 150, 3, 20, 1.0},  {0, 150, 6, 17, 1.0},  {0, 151, 1, 46, 1.0},  {0, 151, 3, 21, 1.0},  {0, 151, 7, 17, 1.0},  {0, 152, 1, 47, 1.0},
  {0, 152, 3, 22, 1.0},  {0, 152, 8, 17, 1.0},  {0, 153, 1, 48, 1.0},  {0, 153, 3, 23, 1.0},  {0, 153, 9, 17, 1.0},  {0, 154, 1, 49, 1.0},
  {0, 154, 3, 24, 1.0},  {0, 154, 10, 17, 1.0},  {0, 155, 1, 50, 1.0},  {0, 155, 3, 25, 1.0},  {0, 155, 11, 17, 1.0},  {0, 156, 1, 51, 1.0},
  {0, 156, 3, 26, 1.0},  {0, 156, 12, 17, 1.0},  {0, 157, 1, 52, 1.0},  {0, 157, 3, 27, 1.0},  {0, 157, 13, 17, 1.0},  {0, 158, 1, 53, 1.0},
  {0, 158, 3, 28, 1.0},  {0, 158, 14, 17, 1.0},  {0, 159, 1, 54, 1.0},  {0, 159, 4, 18, 1.0},  {0, 160, 1, 55, 1.0},  {0, 160, 4, 19, 1.0},
  {0, 160, 5, 18, 1.0},  {0, 161, 1, 56, 1.0},  {0, 161, 4, 20, 1.0},  {0, 161, 6, 18, 1.0},  {0, 162, 1, 57, 1.0},  {0, 162, 4, 21, 1.0},
  {0, 162, 7, 18, 1.0},  {0, 163, 1, 58, 1.0},  {0, 163, 4, 22, 1.0},  {0, 163, 8, 18, 1.0},  {0, 164, 1, 59, 1.0},  {0, 164, 4, 23, 1.0},
  {0, 164, 9, 18, 1.0},  {0, 165, 1, 60, 1.0},  {0, 165, 4, 24, 1.0},  {0, 165, 10, 18, 1.0},  {0, 166, 1, 61, 1.0},  {0, 166, 4, 25, 1.0},
  {0, 166, 11, 18, 1.0},  {0, 167, 1, 62, 1.0},  {0, 167, 4, 26, 1.0},  {0, 167, 12, 18, 1.0},  {0, 168, 1, 63, 1.0},  {0, 168, 4, 27, 1.0},
  {0, 168, 13, 18, 1.0},  {0, 169, 1, 64, 1.0},  {0, 169, 4, 28, 1.0},  {0, 169, 14, 18, 1.0},  {0, 170, 1, 65, 1.0},  {0, 170, 5, 19, 1.0},
  {0, 171, 1, 66, 1.0},  {0, 171, 5, 20, 1.0},  {0, 171, 6, 19, 1.0},  {0, 172, 1, 67, 1.0},  {0, 172, 5, 21, 1.0},  {0, 172, 7, 19, 1.0},
  {0, 173, 1, 68, 1.0},  {0, 173, 5, 22, 1.0},  {0, 173, 8, 19, 1.0},  {0, 174, 1, 69, 1.0},  {0, 174, 5, 23, 1.0},  {0, 174, 9, 19, 1.0},
  {0, 175, 1, 70, 1.0},  {0, 175, 5, 24, 1.0},  {0, 175, 10, 19, 1.0},  {0, 176, 1, 71, 1.0},  {0, 176, 5, 25, 1.0},  {0, 176, 11, 19, 1.0},
  {0, 177, 1, 72, 1.0},  {0, 177, 5, 26, 1.0},  {0, 177, 12, 19, 1.0},  {0, 178, 1, 73, 1.0},  {0, 178, 5, 27, 1.0},  {0, 178, 13, 19, 1.0},
  {0, 179, 1, 74, 1.0},  {0, 179, 5, 28, 1.0},  {0, 179, 14, 19, 1.0},  {0, 180, 1, 75, 1.0},  {0, 180, 6, 20, 1.0},  {0, 181, 1, 76, 1.0},
  {0, 181, 6, 21, 1.0},  {0, 181, 7, 20, 1.0},  {0, 182, 1, 77, 1.0},  {0, 182, 6, 22, 1.0},  {0, 182, 8, 20, 1.0},  {0, 183, 1, 78, 1.0},
  {0, 183, 6, 23, 1.0},  {0, 183, 9, 20, 1.0},  {0, 184, 1, 79, 1.0},  {0, 184, 6, 24, 1.0},  {0, 184, 10, 20, 1.0},  {0, 185, 1, 80, 1.0},
  {0, 185, 6, 25, 1.0},  {0, 185, 11, 20, 1.0},  {0, 186, 1, 81, 1.0},  {0, 186, 6, 26, 1.0},  {0, 186, 12, 20, 1.0},  {0, 187, 1, 82, 1.0},
  {0, 187, 6, 27, 1.0},  {0, 187, 13, 20, 1.0},  {0, 188, 1, 83, 1.0},  {0, 188, 6, 28, 1.0},  {0, 188, 14, 20, 1.0},  {0, 189, 1, 84, 1.0},
  {0, 189, 7, 21, 1.0},  {0, 190, 1, 85, 1.0},  {0, 190, 7, 22, 1.0},  {0, 190, 8, 21, 1.0},  {0, 191, 1, 86, 1.0},  {0, 191, 7, 23, 1.0},
  {0, 191, 9, 21, 1.0},  {0, 192, 1, 87, 1.0},  {0, 192, 7, 24, 1.0},  {0, 192, 10, 21, 1.0},  {0, 193, 1, 88, 1.0},  {0, 193, 7, 25, 1.0},
  {0, 193, 11, 21, 1.0},  {0, 194, 1, 89, 1.0},  {0, 194, 7, 26, 1.0},  {0, 194, 12, 21, 1.0},  {0, 195, 1, 90, 1.0},  {0, 195, 7, 27, 1.0},
  {0, 195, 13, 21, 1.0},  {0, 196, 1, 91, 1.0},  {0, 196, 7, 28, 1.0},  {0, 196, 14, 21, 1.0},  {0, 197, 1, 92, 1.0},  {0, 197, 8, 22, 1.0},
  {0, 198, 1, 93, 1.0},  {0, 198, 8, 23, 1.0},  {0, 198, 9, 22, 1.0},  {0, 199, 1, 94, 1.0},  {0, 199, 8, 24, 1.0},  {0, 199, 10, 22, 1.0},
  {0, 200, 1, 95, 1.0},  {0, 200, 8, 25, 1.0},  {0, 200, 11, 22, 1.0},  {0, 201, 1, 96, 1.0},  {0, 201, 8, 26, 1.0},  {0, 201, 12, 22, 1.0},
  {0, 202, 1, 97, 1.0},  {0, 202, 8, 27, 1.0},  {0, 202, 13, 22, 1.0},  {0, 203, 1, 98, 1.0},  {0, 203, 8, 28, 1.0},  {0, 203, 14, 22, 1.0},
  {0, 204, 1, 99, 1.0},  {0, 204, 9, 23, 1.0},  {0, 205, 1, 100, 1.0},  {0, 205, 9, 24, 1.0},  {0, 205, 10, 23, 1.0},  {0, 206, 1, 101, 1.0},
  {0, 206, 9, 25, 1.0},  {0, 206, 11, 23, 1.0},  {0, 207, 1, 102, 1.0},  {0, 207, 9, 26, 1.0},  {0, 207, 12, 23, 1.0},  {0, 208, 1, 103, 1.0},
  {0, 208, 9, 27, 1.0},  {0, 208, 13, 23, 1.0},  {0, 209, 1, 104, 1.0},  {0, 209, 9, 28, 1.0},  {0, 209, 14, 23, 1.0},  {0, 210, 1, 105, 1.0},
  {0, 210, 10, 24, 1.0},  {0, 211, 1, 106, 1.0},  {0, 211, 10, 25, 1.0},  {0, 211, 11, 24, 1.0},  {0, 212, 1, 107, 1.0},  {0, 212, 10, 26, 1.0},
  {0, 212, 12, 24, 1.0},  {0, 213, 1, 108, 1.0},  {0, 213, 10, 27, 1.0},  {0, 213, 13, 24, 1.0},  {0, 214, 1, 109, 1.0},  {0, 214, 10, 28, 1.0},
  {0, 214, 14, 24, 1.0},  {0, 215, 1, 110, 1.0},  {0, 215, 11, 25, 1.0},  {0, 216, 1, 111, 1.0},  {0, 216, 11, 26, 1.0},  {0, 216, 12, 25, 1.0},
  {0, 217, 1, 112, 1.0},  {0, 217, 11, 27, 1.0},  {0, 217, 13, 25, 1.0},  {0, 218, 1, 113, 1.0},  {0, 218, 11, 28, 1.0},  {0, 218, 14, 25, 1.0},
  {0, 219, 1, 114, 1.0},  {0, 219, 12, 26, 1.0},  {0, 220, 1, 115, 1.0},  {0, 220, 12, 27, 1.0},  {0, 220, 13, 26, 1.0},  {0, 221, 1, 116, 1.0},
  {0, 221, 12, 28, 1.0},  {0, 221, 14, 26, 1.0},  {0, 222, 1, 117, 1.0},  {0, 222, 13, 27, 1.0},  {0, 223, 1, 118, 1.0},  {0, 223, 13, 28, 1.0},
  {0, 223, 14, 27, 1.0},  {0, 224, 1, 119, 1.0},  {0, 224, 14, 28, 1.0},  {0, 225, 2, 29, 1.0},  {0, 226, 2, 30, 1.0},  {0, 226, 3, 29, 1.0},
  {0, 227, 2, 31, 1.0},  {0, 227, 4, 29, 1.0},  {0, 228, 2, 32, 1.0},  {0, 228, 5, 29, 1.0},  {0, 229, 2, 33, 1.0},  {0, 229, 6, 29, 1.0},
  {0, 230, 2, 34, 1.0},  {0, 230, 7, 29, 1.0},  {0, 231, 2, 35, 1.0},  {0, 231, 8, 29, 1.0},  {0, 232, 2, 36, 1.0},  {0, 232, 9, 29, 1.0},
  {0, 233, 2, 37, 1.0},  {0, 233, 10, 29, 1.0},  {0, 234, 2, 38, 1.0},  {0, 234, 11, 29, 1.0},  {0, 235, 2, 39, 1.0},  {0, 235, 12, 29, 1.0},
  {0, 236, 2, 40, 1.0},  {0, 236, 13, 29, 1.0},  {0, 237, 2, 41, 1.0},  {0, 237, 14, 29, 1.0},  {0, 238, 2, 42, 1.0},  {0, 238, 3, 30, 1.0},
  {0, 239, 2, 43, 1.0},  {0, 239, 3, 31, 1.0},  {0, 239, 4, 30, 1.0},  {0, 240, 2, 44, 1.0},  {0, 240, 3, 32, 1.0},  {0, 240, 5, 30, 1.0},
  {0, 241, 2, 45, 1.0},  {0, 241, 3, 33, 1.0},  {0, 241, 6, 30, 1.0},  {0, 242, 2, 46, 1.0},  {0, 242, 3, 34, 1.0},  {0, 242, 7, 30, 1.0},
  {0, 243, 2, 47, 1.0},  {0, 243, 3, 35, 1.0},  {0, 243, 8, 30, 1.0},  {0, 244, 2, 48, 1.0},  {0, 244, 3, 36, 1.0},  {0, 244, 9, 30, 1.0},
  {0, 245, 2, 49, 1.0},  {0, 245, 3, 37, 1.0},  {0, 245, 10, 30, 1.0},  {0, 246, 2, 50, 1.0},  {0, 246, 3, 38, 1.0},  {0, 246, 11, 30, 1.0},
  {0, 247, 2, 51, 1.0},  {0, 247, 3, 39, 1.0},  {0, 247, 12, 30, 1.0},  {0, 248, 2, 52, 1.0},  {0, 248, 3, 40, 1.0},  {0, 248, 13, 30, 1.0},
  {0, 249, 2, 53, 1.0},  {0, 249, 3, 41, 1.0},  {0, 249, 14, 30, 1.0},  {0, 250, 2, 54, 1.0},  {0, 250, 4, 31, 1.0},  {0, 251, 2, 55, 1.0},
  {0, 251, 4, 32, 1.0},  {0, 251, 5, 31, 1.0},  {0, 252, 2, 56, 1.0},  {0, 252, 4, 33, 1.0},  {0, 252, 6, 31, 1.0},  {0, 253, 2, 57, 1.0},
  {0, 253, 4, 34, 1.0},  {0, 253, 7, 31, 1.0},  {0, 254, 2, 58, 1.0},  {0, 254, 4, 35, 1.0},  {0, 254, 8, 31, 1.0},  {0, 255, 2, 59, 1.0},
  {0, 255, 4, 36, 1.0},  {0, 255, 9, 31, 1.0},  {0, 256, 2, 60, 1.0},  {0, 256, 4, 37, 1.0},  {0, 256, 10, 31, 1.0},  {0, 257, 2, 61, 1.0},
  {0, 257, 4, 38, 1.0},  {0, 257, 11, 31, 1.0},  {0, 258, 2, 62, 1.0},  {0, 258, 4, 39, 1.0},  {0, 258, 12, 31, 1.0},  {0, 259, 2, 63, 1.0},
  {0, 259, 4, 40, 1.0},  {0, 259, 13, 31, 1.0},  {0, 260, 2, 64, 1.0},  {0, 260, 4, 41, 1.0},  {0, 260, 14, 31, 1.0},  {0, 261, 2, 65, 1.0},
  {0, 261, 5, 32, 1.0},  {0, 262, 2, 66, 1.0},  {0, 262, 5, 33, 1.0},  {0, 262, 6, 32, 1.0},  {0, 263, 2, 67, 1.0},  {0, 263, 5, 34, 1.0},
  {0, 263, 7, 32, 1.0},  {0, 264, 2, 68, 1.0},  {0, 264, 5, 35, 1.0},  {0, 264, 8, 32, 1.0},  {0, 265, 2, 69, 1.0},  {0, 265, 5, 36, 1.0},
  {0, 265, 9, 32, 1.0},  {0, 266, 2, 70, 1.0},  {0, 266, 5, 37, 1.0},  {0, 266, 10, 32, 1.0},  {0, 267, 2, 71, 1.0},  {0, 267, 5, 38, 1.0},
  {0, 267, 11, 32, 1.0},  {0, 268, 2, 72, 1.0},  {0, 268, 5, 39, 1.0},  {0, 268, 12, 32, 1.0},  {0, 269, 2, 73, 1.0},  {0, 269, 5, 40, 1.0},
  {0, 269, 13, 32, 1.0},  {0, 270, 2, 74, 1.0},  {0, 270, 5, 41, 1.0},  {0, 270, 14, 32, 1.0},  {0, 271, 2, 75, 1.0},  {0, 271, 6, 33, 1.0},
  {0, 272, 2, 76, 1.0},  {0, 272, 6, 34, 1.0},  {0, 272, 7, 33, 1.0},  {0, 273, 2, 77, 1.0},  {0, 273, 6, 35, 1.0},  {0, 273, 8, 33, 1.0},
  {0, 274, 2, 78, 1.0},  {0, 274, 6, 36, 1.0},  {0, 274, 9, 33, 1.0},  {0, 275, 2, 79, 1.0},  {0, 275, 6, 37, 1.0},  {0, 275, 10, 33, 1.0},
  {0, 276, 2, 80, 1.0},  {0, 276, 6, 38, 1.0},  {0, 276, 11, 33, 1.0},  {0, 277, 2, 81, 1.0},  {0, 277, 6, 39, 1.0},  {0, 277, 12, 33, 1.0},
  {0, 278, 2, 82, 1.0},  {0, 278, 6, 40, 1.0},  {0, 278, 13, 33, 1.0},  {0, 279, 2, 83, 1.0},  {0, 279, 6, 41, 1.0},  {0, 279, 14, 33, 1.0},
  {0, 280, 2, 84, 1.0},  {0, 280, 7, 34, 1.0},  {0, 281, 2, 85, 1.0},  {0, 281, 7, 35, 1.0},  {0, 281, 8, 34, 1.0},  {0, 282, 2, 86, 1.0},
  {0, 282, 7, 36, 1.0},  {0, 282, 9, 34, 1.0},  {0, 283, 2, 87, 1.0},  {0, 283, 7, 37, 1.0},  {0, 283, 10, 34, 1.0},  {0, 284, 2, 88, 1.0},
  {0, 284, 7, 38, 1.0},  {0, 284, 11, 34, 1.0},  {0, 285, 2, 89, 1.0},  {0, 285, 7, 39, 1.0},  {0, 285, 12, 34, 1.0},  {0, 286, 2, 90, 1.0},
  {0, 286, 7, 40, 1.0},  {0, 286, 13, 34, 1.0},  {0, 287, 2, 91, 1.0},  {0, 287, 7, 41, 1.0},  {0, 287, 14, 34, 1.0},  {0, 288, 2, 92, 1.0},
  {0, 288, 8, 35, 1.0},  {0, 289, 2, 93, 1.0},  {0, 289, 8, 36, 1.0},  {0, 289, 9, 35, 1.0},  {0, 290, 2, 94, 1.0},  {0, 290, 8, 37, 1.0},
  {0, 290, 10, 35, 1.0},  {0, 291, 2, 95, 1.0},  {0, 291, 8, 38, 1.0},  {0, 291, 11, 35, 1.0},  {0, 292, 2, 96, 1.0},  {0, 292, 8, 39, 1.0},
  {0, 292, 12, 35, 1.0},  {0, 293, 2, 97, 1.0},  {0, 293, 8, 40, 1.0},  {0, 293, 13, 35, 1.0},  {0, 294, 2, 98, 1.0},  {0, 294, 8, 41, 1.0},
  {0, 294, 14, 35, 1.0},  {0, 295, 2, 99, 1.0},  {0, 295, 9, 36, 1.0},  {0, 296, 2, 100, 1.0},  {0, 296, 9, 37, 1.0},  {0, 296, 10, 36, 1.0},
  {0, 297, 2, 101, 1.0},  {0, 297, 9, 38, 1.0},  {0, 297, 11, 36, 1.0},  {0, 298, 2, 102, 1.0},  {0, 298, 9, 39, 1.0},  {0, 298, 12, 36, 1.0},
  {0, 299, 2, 103, 1.0},  {0, 299, 9, 40, 1.0},  {0, 299, 13, 36, 1.0},  {0, 300, 2, 104, 1.0},  {0, 300, 9, 41, 1.0},  {0, 300, 14, 36, 1.0},
  {0, 301, 2, 105, 1.0},  {0, 301, 10, 37, 1.0},  {0, 302, 2, 106, 1.0},  {0, 302, 10, 38, 1.0},  {0, 302, 11, 37, 1.0},  {0, 303, 2, 107, 1.0},
  {0, 303, 10, 39, 1.0},  {0, 303, 12, 37, 1.0},  {0, 304, 2, 108, 1.0},  {0, 304, 10, 40, 1.0},  {0, 304, 13, 37, 1.0},  {0, 305, 2, 109, 1.0},
  {0, 305, 10, 41, 1.0},  {0, 305, 14, 37, 1.0},  {0, 306, 2, 110, 1.0},  {0, 306, 11, 38, 1.0},  {0, 307, 2, 111, 1.0},  {0, 307, 11, 39, 1.0},
  {0, 307, 12, 38, 1.0},  {0, 308, 2, 112, 1.0},  {0, 308, 11, 40, 1.0},  {0, 308, 13, 38, 1.0},  {0, 309, 2, 113, 1.0},  {0, 309, 11, 41, 1.0},
  {0, 309, 14, 38, 1.0},  {0, 310, 2, 114, 1.0},  {0, 310, 12, 39, 1.0},  {0, 311, 2, 115, 1.0},  {0, 311, 12, 40, 1.0},  {0, 311, 13, 39, 1.0},
  {0, 312, 2, 116, 1.0},  {0, 312, 12, 41, 1.0},  {0, 312, 14, 39, 1.0},  {0, 313, 2, 117, 1.0},  {0, 313, 13, 40, 1.0},  {0, 314, 2, 118, 1.0},
  {0, 314, 13, 41, 1.0},  {0, 314, 14, 40, 1.0},  {0, 315, 2, 119, 1.0},  {0, 315, 14, 41, 1.0},  {0, 316, 3, 42, 1.0},  {0, 317, 3, 43, 1.0},
  {0, 317, 4, 42, 1.0},  {0, 318, 3, 44, 1.0},  {0, 318, 5, 42, 1.0},  {0, 319, 3, 45, 1.0},  {0, 319, 6, 42, 1.0},  {0, 320, 3, 46, 1.0},
  {0, 320, 7, 42, 1.0},  {0, 321, 3, 47, 1.0},  {0, 321, 8, 42, 1.0},  {0, 322, 3, 48, 1.0},  {0, 322, 9, 42, 1.0},  {0, 323, 3, 49, 1.0},
  {0, 323, 10, 42, 1.0},  {0, 324, 3, 50, 1.0},  {0, 324, 11, 42, 1.0},  {0, 325, 3, 51, 1.0},  {0, 325, 12, 42, 1.0},  {0, 326, 3, 52, 1.0},
  {0, 326, 13, 42, 1.0},  {0, 327, 3, 53, 1.0},  {0, 327, 14, 42, 1.0},  {0, 328, 3, 54, 1.0},  {0, 328, 4, 43, 1.0},  {0, 329, 3, 55, 1.0},
  {0, 329, 4, 44, 1.0},  {0, 329, 5, 43, 1.0},  {0, 330, 3, 56, 1.0},  {0, 330, 4, 45, 1.0},  {0, 330, 6, 43, 1.0},  {0, 331, 3, 57, 1.0},
  {0, 331, 4, 46, 1.0},  {0, 331, 7, 43, 1.0},  {0, 332, 3, 58, 1.0},  {0, 332, 4, 47, 1.0},  {0, 332, 8, 43, 1.0},  {0, 333, 3, 59, 1.0},
  {0, 333, 4, 48, 1.0},  {0, 333, 9, 43, 1.0},  {0, 334, 3, 60, 1.0},  {0, 334, 4, 49, 1.0},  {0, 334, 10, 43, 1.0},  {0, 335, 3, 61, 1.0},
  {0, 335, 4, 50, 1.0},  {0, 335, 11, 43, 1.0},  {0, 336, 3, 62, 1.0},  {0, 336, 4, 51, 1.0},  {0, 336, 12, 43, 1.0},  {0, 337, 3, 63, 1.0},
  {0, 337, 4, 52, 1.0},  {0, 337, 13, 43, 1.0},  {0, 338, 3, 64, 1.0},  {0, 338, 4, 53, 1.0},  {0, 338, 14, 43, 1.0},  {0, 339, 3, 65, 1.0},
  {0, 339, 5, 44, 1.0},  {0, 340, 3, 66, 1.0},  {0, 340, 5, 45, 1.0},  {0, 340, 6, 44, 1.0},  {0, 341, 3, 67, 1.0},  {0, 341, 5, 46, 1.0},
  {0, 341, 7, 44, 1.0},  {0, 342, 3, 68, 1.0},  {0, 342, 5, 47, 1.0},  {0, 342, 8, 44, 1.0},  {0, 343, 3, 69, 1.0},  {0, 343, 5, 48, 1.0},
  {0, 343, 9, 44, 1.0},  {0, 344, 3, 70, 1.0},  {0, 344, 5, 49, 1.0},  {0, 344, 10, 44, 1.0},  {0, 345, 3, 71, 1.0},  {0, 345, 5, 50, 1.0},
  {0, 345, 11, 44, 1.0},  {0, 346, 3, 72, 1.0},  {0, 346, 5, 51, 1.0},  {0, 346, 12, 44, 1.0},  {0, 347, 3, 73, 1.0},  {0, 347, 5, 52, 1.0},
  {0, 347, 13, 44, 1.0},  {0, 348, 3, 74, 1.0},  {0, 348, 5, 53, 1.0},  {0, 348, 14, 44, 1.0},  {0, 349, 3, 75, 1.0},  {0, 349, 6, 45, 1.0},
  {0, 350, 3, 76, 1.0},  {0, 350, 6, 46, 1.0},  {0, 350, 7, 45, 1.0},  {0, 351, 3, 77, 1.0},  {0, 351, 6, 47, 1.0},  {0, 351, 8, 45, 1.0},
  {0, 352, 3, 78, 1.0},  {0, 352, 6, 48, 1.0},  {0, 352, 9, 45, 1.0},  {0, 353, 3, 79, 1.0},  {0, 353, 6, 49, 1.0},  {0, 353, 10, 45, 1.0},
  {0, 354, 3, 80, 1.0},  {0, 354, 6, 50, 1.0},  {0, 354, 11, 45, 1.0},  {0, 355, 3, 81, 1.0},  {0, 355, 6, 51, 1.0},  {0, 355, 12, 45, 1.0},
  {0, 356, 3, 82, 1.0},  {0, 356, 6, 52, 1.0},  {0, 356, 13, 45, 1.0},  {0, 357, 3, 83, 1.0},  {0, 357, 6, 53, 1.0},  {0, 357, 14, 45, 1.0},
  {0, 358, 3, 84, 1.0},  {0, 358, 7, 46, 1.0},  {0, 359, 3, 85, 1.0},  {0, 359, 7, 47, 1.0},  {0, 359, 8, 46, 1.0},  {0, 360, 3, 86, 1.0},
  {0, 360, 7, 48, 1.0},  {0, 360, 9, 46, 1.0},  {0, 361, 3, 87, 1.0},  {0, 361, 7, 49, 1.0},  {0, 361, 10, 46, 1.0},  {0, 362, 3, 88, 1.0},
  {0, 362, 7, 50, 1.0},  {0, 362, 11, 46, 1.0},  {0, 363, 3, 89, 1.0},  {0, 363, 7, 51, 1.0},  {0, 363, 12, 46, 1.0},  {0, 364, 3, 90, 1.0},
  {0, 364, 7, 52, 1.0},  {0, 364, 13, 46, 1.0},  {0, 365, 3, 91, 1.0},  {0, 365, 7, 53, 1.0},  {0, 365, 14, 46, 1.0},  {0, 366, 3, 92, 1.0},
  {0, 366, 8, 47, 1.0},  {0, 367, 3, 93, 1.0},  {0, 367, 8, 48, 1.0},  {0, 367, 9, 47, 1.0},  {0, 368, 3, 94, 1.0},  {0, 368, 8, 49, 1.0},
  {0, 368, 10, 47, 1.0},  {0, 369, 3, 95, 1.0},  {0, 369, 8, 50, 1.0},  {0, 369, 11, 47, 1.0},  {0, 370, 3, 96, 1.0},  {0, 370, 8, 51, 1.0},
  {0, 370, 12, 47, 1.0},  {0, 371, 3, 97, 1.0},  {0, 371, 8, 52, 1.0},  {0, 371, 13, 47, 1.0},  {0, 372, 3, 98, 1.0},  {0, 372, 8, 53, 1.0},
  {0, 372, 14, 47, 1.0},  {0, 373, 3, 99, 1.0},  {0, 373, 9, 48, 1.0},  {0, 374, 3, 100, 1.0},  {0, 374, 9, 49, 1.0},  {0, 374, 10, 48, 1.0},
  {0, 375, 3, 101, 1.0},  {0, 375, 9, 50, 1.0},  {0, 375, 11, 48, 1.0},  {0, 376, 3, 102, 1.0},  {0, 376, 9, 51, 1.0},  {0, 376, 12, 48, 1.0},
  {0, 377, 3, 103, 1.0},  {0, 377, 9, 52, 1.0},  {0, 377, 13, 48, 1.0},  {0, 378, 3, 104, 1.0},  {0, 378, 9, 53, 1.0},  {0, 378, 14, 48, 1.0},
  {0, 379, 3, 105, 1.0},  {0, 379, 10, 49, 1.0},  {0, 380, 3, 106, 1.0},  {0, 380, 10, 50, 1.0},  {0, 380, 11, 49, 1.0},  {0, 381, 3, 107, 1.0},
  {0, 381, 10, 51, 1.0},  {0, 381, 12, 49, 1.0},  {0, 382, 3, 108, 1.0},  {0, 382, 10, 52, 1.0},  {0, 382, 13, 49, 1.0},  {0, 383, 3, 109, 1.0},
  {0, 383, 10, 53, 1.0},  {0, 383, 14, 49, 1.0},  {0, 384, 3, 110, 1.0},  {0, 384, 11, 50, 1.0},  {0, 385, 3, 111, 1.0},  {0, 385, 11, 51, 1.0},
  {0, 385, 12, 50, 1.0},  {0, 386, 3, 112, 1.0},  {0, 386, 11, 52, 1.0},  {0, 386, 13, 50, 1.0},  {0, 387, 3, 113, 1.0},  {0, 387, 11, 53, 1.0},
  {0, 387, 14, 50, 1.0},  {0, 388, 3, 114, 1.0},  {0, 388, 12, 51, 1.0},  {0, 389, 3, 115, 1.0},  {0, 389, 12, 52, 1.0},  {0, 389, 13, 51, 1.0},
  {0, 390, 3, 116, 1.0},  {0, 390, 12, 53, 1.0},  {0, 390, 14, 51, 1.0},  {0, 391, 3, 117, 1.0},  {0, 391, 13, 52, 1.0},  {0, 392, 3, 118, 1.0},
  {0, 392, 13, 53, 1.0},  {0, 392, 14, 52, 1.0},  {0, 393, 3, 119, 1.0},  {0, 393, 14, 53, 1.0},  {0, 394, 4, 54, 1.0},  {0, 395, 4, 55, 1.0},
  {0, 395, 5, 54, 1.0},  {0, 396, 4, 56, 1.0},  {0, 396, 6, 54, 1.0},  {0, 397, 4, 57, 1.0},  {0, 397, 7, 54, 1.0},  {0, 398, 4, 58, 1.0},
  {0, 398, 8, 54, 1.0},  {0, 399, 4, 59, 1.0},  {0, 399, 9, 54, 1.0},  {0, 400, 4, 60, 1.0},  {0, 400, 10, 54, 1.0},  {0, 401, 4, 61, 1.0},
  {0, 401, 11, 54, 1.0},  {0, 402, 4, 62, 1.0},  {0, 402, 12, 54, 1.0},  {0, 403, 4, 63, 1.0},  {0, 403, 13, 54, 1.0},  {0, 404, 4, 64, 1.0},
  {0, 404, 14, 54, 1.0},  {0, 405, 4, 65, 1.0},  {0, 405, 5, 55, 1.0},  {0, 406, 4, 66, 1.0},  {0, 406, 5, 56, 1.0},  {0, 406, 6, 55, 1.0},
  {0, 407, 4, 67, 1.0},  {0, 407, 5, 57, 1.0},  {0, 407, 7, 55, 1.0},  {0, 408, 4, 68, 1.0},  {0, 408, 5, 58, 1.0},  {0, 408, 8, 55, 1.0},
  {0, 409, 4, 69, 1.0},  {0, 409, 5, 59, 1.0},  {0, 409, 9, 55, 1.0},  {0, 410, 4, 70, 1.0},  {0, 410, 5, 60, 1.0},  {0, 410, 10, 55, 1.0},
  {0, 411, 4, 71, 1.0},  {0, 411, 5, 61, 1.0},  {0, 411, 11, 55, 1.0},  {0, 412, 4, 72, 1.0},  {0, 412, 5, 62, 1.0},  {0, 412, 12, 55, 1.0},
  {0, 413, 4, 73, 1.0},  {0, 413, 5, 63, 1.0},  {0, 413, 13, 55, 1.0},  {0, 414, 4, 74, 1.0},  {0, 414, 5, 64, 1.0},  {0, 414, 14, 55, 1.0},
  {0, 415, 4, 75, 1.0},  {0, 415, 6, 56, 1.0},  {0, 416, 4, 76, 1.0},  {0, 416, 6, 57, 1.0},  {0, 416, 7, 56, 1.0},  {0, 417, 4, 77, 1.0},
  {0, 417, 6, 58, 1.0},  {0, 417, 8, 56, 1.0},  {0, 418, 4, 78, 1.0},  {0, 418, 6, 59, 1.0},  {0, 418, 9, 56, 1.0},  {0, 419, 4, 79, 1.0},
  {0, 419, 6, 60, 1.0},  {0, 419, 10, 56, 1.0},  {0, 420, 4, 80, 1.0},  {0, 420, 6, 61, 1.0},  {0, 420, 11, 56, 1.0},  {0, 421, 4, 81, 1.0},
  {0, 421, 6, 62, 1.0},  {0, 421, 12, 56, 1.0},  {0, 422, 4, 82, 1.0},  {0, 422, 6, 63, 1.0},  {0, 422, 13, 56, 1.0},  {0, 423, 4, 83, 1.0},
  {0, 423, 6, 64, 1.0},  {0, 423, 14, 56, 1.0},  {0, 424, 4, 84, 1.0},  {0, 424, 7, 57, 1.0},  {0, 425, 4, 85, 1.0},  {0, 425, 7, 58, 1.0},
  {0, 425, 8, 57, 1.0},  {0, 426, 4, 86, 1.0},  {0, 426, 7, 59, 1.0},  {0, 426, 9, 57, 1.0},  {0, 427, 4, 87, 1.0},  {0, 427, 7, 60, 1.0},
  {0, 427, 10, 57, 1.0},  {0, 428, 4, 88, 1.0},  {0, 428, 7, 61, 1.0},  {0, 428, 11, 57, 1.0},  {0, 429, 4, 89, 1.0},  {0, 429, 7, 62, 1.0},
  {0, 429, 12, 57, 1.0},  {0, 430, 4, 90, 1.0},  {0, 430, 7, 63, 1.0},  {0, 430, 13, 57, 1.0},  {0, 431, 4, 91, 1.0},  {0, 431, 7, 64, 1.0},
  {0, 431, 14, 57, 1.0},  {0, 432, 4, 92, 1.0},  {0, 432, 8, 58, 1.0},  {0, 433, 4, 93, 1.0},  {0, 433, 8, 59, 1.0},  {0, 433, 9, 58, 1.0},
  {0, 434, 4, 94, 1.0},  {0, 434, 8, 60, 1.0},  {0, 434, 10, 58, 1.0},  {0, 435, 4, 95, 1.0},  {0, 435, 8, 61, 1.0},  {0, 435, 11, 58, 1.0},
  {0, 436, 4, 96, 1.0},  {0, 436, 8, 62, 1.0},  {0, 436, 12, 58, 1.0},  {0, 437, 4, 97, 1.0},  {0, 437, 8, 63, 1.0},  {0, 437, 13, 58, 1.0},
  {0, 438, 4, 98, 1.0},  {0, 438, 8, 64, 1.0},  {0, 438, 14, 58, 1.0},  {0, 439, 4, 99, 1.0},  {0, 439, 9, 59, 1.0},  {0, 440, 4, 100, 1.0},
  {0, 440, 9, 60, 1.0},  {0, 440, 10, 59, 1.0},  {0, 441, 4, 101, 1.0},  {0, 441, 9, 61, 1.0},  {0, 441, 11, 59, 1.0},  {0, 442, 4, 102, 1.0},
  {0, 442, 9, 62, 1.0},  {0, 442, 12, 59, 1.0},  {0, 443, 4, 103, 1.0},  {0, 443, 9, 63, 1.0},  {0, 443, 13, 59, 1.0},  {0, 444, 4, 104, 1.0},
  {0, 444, 9, 64, 1.0},  {0, 444, 14, 59, 1.0},  {0, 445, 4, 105, 1.0},  {0, 445, 10, 60, 1.0},  {0, 446, 4, 106, 1.0},  {0, 446, 10, 61, 1.0},
  {0, 446, 11, 60, 1.0},  {0, 447, 4, 107, 1.0},  {0, 447, 10, 62, 1.0},  {0, 447, 12, 60, 1.0},  {0, 448, 4, 108, 1.0},  {0, 448, 10, 63, 1.0},
  {0, 448, 13, 60, 1.0},  {0, 449, 4, 109, 1.0},  {0, 449, 10, 64, 1.0},  {0, 449, 14, 60, 1.0},  {0, 450, 4, 110, 1.0},  {0, 450, 11, 61, 1.0},
  {0, 451, 4, 111, 1.0},  {0, 451, 11, 62, 1.0},  {0, 451, 12, 61, 1.0},  {0, 452, 4, 112, 1.0},  {0, 452, 11, 63, 1.0},  {0, 452, 13, 61, 1.0},
  {0, 453, 4, 113, 1.0},  {0, 453, 11, 64, 1.0},  {0, 453, 14, 61, 1.0},  {0, 454, 4, 114, 1.0},  {0, 454, 12, 62, 1.0},  {0, 455, 4, 115, 1.0},
  {0, 455, 12, 63, 1.0},  {0, 455, 13, 62, 1.0},  {0, 456, 4, 116, 1.0},  {0, 456, 12, 64, 1.0},  {0, 456, 14, 62, 1.0},  {0, 457, 4, 117, 1.0},
  {0, 457, 13, 63, 1.0},  {0, 458, 4, 118, 1.0},  {0, 458, 13, 64, 1.0},  {0, 458, 14, 63, 1.0},  {0, 459, 4, 119, 1.0},  {0, 459, 14, 64, 1.0},
  {0, 460, 5, 65, 1.0},  {0, 461, 5, 66, 1.0},  {0, 461, 6, 65, 1.0},  {0, 462, 5, 67, 1.0},  {0, 462, 7, 65, 1.0},  {0, 463, 5, 68, 1.0},
  {0, 463, 8, 65, 1.0},  {0, 464, 5, 69, 1.0},  {0, 464, 9, 65, 1.0},  {0, 465, 5, 70, 1.0},  {0, 465, 10, 65, 1.0},  {0, 466, 5, 71, 1.0},
  {0, 466, 11, 65, 1.0},  {0, 467, 5, 72, 1.0},  {0, 467, 12, 65, 1.0},  {0, 468, 5, 73, 1.0},  {0, 468, 13, 65, 1.0},  {0, 469, 5, 74, 1.0},
  {0, 469, 14, 65, 1.0},  {0, 470, 5, 75, 1.0},  {0, 470, 6, 66, 1.0},  {0, 471, 5, 76, 1.0},  {0, 471, 6, 67, 1.0},  {0, 471, 7, 66, 1.0},
  {0, 472, 5, 77, 1.0},  {0, 472, 6, 68, 1.0},  {0, 472, 8, 66, 1.0},  {0, 473, 5, 78, 1.0},  {0, 473, 6, 69, 1.0},  {0, 473, 9, 66, 1.0},
  {0, 474, 5, 79, 1.0},  {0, 474, 6, 70, 1.0},  {0, 474, 10, 66, 1.0},  {0, 475, 5, 80, 1.0},  {0, 475, 6, 71, 1.0},  {0, 475, 11, 66, 1.0},
  {0, 476, 5, 81, 1.0},  {0, 476, 6, 72, 1.0},  {0, 476, 12, 66, 1.0},  {0, 477, 5, 82, 1.0},  {0, 477, 6, 73, 1.0},  {0, 477, 13, 66, 1.0},
  {0, 478, 5, 83, 1.0},  {0, 478, 6, 74, 1.0},  {0, 478, 14, 66, 1.0},  {0, 479, 5, 84, 1.0},  {0, 479, 7, 67, 1.0},  {0, 480, 5, 85, 1.0},
  {0, 480, 7, 68, 1.0},  {0, 480, 8, 67, 1.0},  {0, 481, 5, 86, 1.0},  {0, 481, 7, 69, 1.0},  {0, 481, 9, 67, 1.0},  {0, 482, 5, 87, 1.0},
  {0, 482, 7, 70, 1.0},  {0, 482, 10, 67, 1.0},  {0, 483, 5, 88, 1.0},  {0, 483, 7, 71, 1.0},  {0, 483, 11, 67, 1.0},  {0, 484, 5, 89, 1.0},
  {0, 484, 7, 72, 1.0},  {0, 484, 12, 67, 1.0},  {0, 485, 5, 90, 1.0},  {0, 485, 7, 73, 1.0},  {0, 485, 13, 67, 1.0},  {0, 486, 5, 91, 1.0},
  {0, 486, 7, 74, 1.0},  {0, 486, 14, 67, 1.0},  {0, 487, 5, 92, 1.0},  {0, 487, 8, 68, 1.0},  {0, 488, 5, 93, 1.0},  {0, 488, 8, 69, 1.0},
  {0, 488, 9, 68, 1.0},  {0, 489, 5, 94, 1.0},  {0, 489, 8, 70, 1.0},  {0, 489, 10, 68, 1.0},  {0, 490, 5, 95, 1.0},  {0, 490, 8, 71, 1.0},
  {0, 490, 11, 68, 1.0},  {0, 491, 5, 96, 1.0},  {0, 491, 8, 72, 1.0},  {0, 491, 12, 68, 1.0},  {0, 492, 5, 97, 1.0},  {0, 492, 8, 73, 1.0},
  {0, 492, 13, 68, 1.0},  {0, 493, 5, 98, 1.0},  {0, 493, 8, 74, 1.0},  {0, 493, 14, 68, 1.0},  {0, 494, 5, 99, 1.0},  {0, 494, 9, 69, 1.0},
  {0, 495, 5, 100, 1.0},  {0, 495, 9, 70, 1.0},  {0, 495, 10, 69, 1.0},  {0, 496, 5, 101, 1.0},  {0, 496, 9, 71, 1.0},  {0, 496, 11, 69, 1.0},
  {0, 497, 5, 102, 1.0},  {0, 497, 9, 72, 1.0},  {0, 497, 12, 69, 1.0},  {0, 498, 5, 103, 1.0},  {0, 498, 9, 73, 1.0},  {0, 498, 13, 69, 1.0},
  {0, 499, 5, 104, 1.0},  {0, 499, 9, 74, 1.0},  {0, 499, 14, 69, 1.0},  {0, 500, 5, 105, 1.0},  {0, 500, 10, 70, 1.0},  {0, 501, 5, 106, 1.0},
  {0, 501, 10, 71, 1.0},  {0, 501, 11, 70, 1.0},  {0, 502, 5, 107, 1.0},  {0, 502, 10, 72, 1.0},  {0, 502, 12, 70, 1.0},  {0, 503, 5, 108, 1.0},
  {0, 503, 10, 73, 1.0},  {0, 503, 13, 70, 1.0},  {0, 504, 5, 109, 1.0},  {0, 504, 10, 74, 1.0},  {0, 504, 14, 70, 1.0},  {0, 505, 5, 110, 1.0},
  {0, 505, 11, 71, 1.0},  {0, 506, 5, 111, 1.0},  {0, 506, 11, 72, 1.0},  {0, 506, 12, 71, 1.0},  {0, 507, 5, 112, 1.0},  {0, 507, 11, 73, 1.0},
  {0, 507, 13, 71, 1.0},  {0, 508, 5, 113, 1.0},  {0, 508, 11, 74, 1.0},  {0, 508, 14, 71, 1.0},  {0, 509, 5, 114, 1.0},  {0, 509, 12, 72, 1.0},
  {0, 510, 5, 115, 1.0},  {0, 510, 12, 73, 1.0},  {0, 510, 13, 72, 1.0},  {0, 511, 5, 116, 1.0},  {0, 511, 12, 74, 1.0},  {0, 511, 14, 72, 1.0},
  {0, 512, 5, 117, 1.0},  {0, 512, 13, 73, 1.0},  {0, 513, 5, 118, 1.0},  {0, 513, 13, 74, 1.0},  {0, 513, 14, 73, 1.0},  {0, 514, 5, 119, 1.0},
  {0, 514, 14, 74, 1.0},  {0, 515, 6, 75, 1.0},  {0, 516, 6, 76, 1.0},  {0, 516, 7, 75, 1.0},  {0, 517, 6, 77, 1.0},  {0, 517, 8, 75, 1.0},
  {0, 518, 6, 78, 1.0},  {0, 518, 9, 75, 1.0},  {0, 519, 6, 79, 1.0},  {0, 519, 10, 75, 1.0},  {0, 520, 6, 80, 1.0},  {0, 520, 11, 75, 1.0},
  {0, 521, 6, 81, 1.0},  {0, 521, 12, 75, 1.0},  {0, 522, 6, 82, 1.0},  {0, 522, 13, 75, 1.0},  {0, 523, 6, 83, 1.0},  {0, 523, 14, 75, 1.0},
  {0, 524, 6, 84, 1.0},  {0, 524, 7, 76, 1.0},  {0, 525, 6, 85, 1.0},  {0, 525, 7, 77, 1.0},  {0, 525, 8, 76, 1.0},  {0, 526, 6, 86, 1.0},
  {0, 526, 7, 78, 1.0},  {0, 526, 9, 76, 1.0},  {0, 527, 6, 87, 1.0},  {0, 527, 7, 79, 1.0},  {0, 527, 10, 76, 1.0},  {0, 528, 6, 88, 1.0},
  {0, 528, 7, 80, 1.0},  {0, 528, 11, 76, 1.0},  {0, 529, 6, 89, 1.0},  {0, 529, 7, 81, 1.0},  {0, 529, 12, 76, 1.0},  {0, 530, 6, 90, 1.0},
  {0, 530, 7, 82, 1.0},  {0, 530, 13, 76, 1.0},  {0, 531, 6, 91, 1.0},  {0, 531, 7, 83, 1.0},  {0, 531, 14, 76, 1.0},  {0, 532, 6, 92, 1.0},
  {0, 532, 8, 77, 1.0},  {0, 533, 6, 93, 1.0},  {0, 533, 8, 78, 1.0},  {0, 533, 9, 77, 1.0},  {0, 534, 6, 94, 1.0},  {0, 534, 8, 79, 1.0},
  {0, 534, 10, 77, 1.0},  {0, 535, 6, 95, 1.0},  {0, 535, 8, 80, 1.0},  {0, 535, 11, 77, 1.0},  {0, 536, 6, 96, 1.0},  {0, 536, 8, 81, 1.0},
  {0, 536, 12, 77, 1.0},  {0, 537, 6, 97, 1.0},  {0, 537, 8, 82, 1.0},  {0, 537, 13, 77, 1.0},  {0, 538, 6, 98, 1.0},  {0, 538, 8, 83, 1.0},
  {0, 538, 14, 77, 1.0},  {0, 539, 6, 99, 1.0},  {0, 539, 9, 78, 1.0},  {0, 540, 6, 100, 1.0},  {0, 540, 9, 79, 1.0},  {0, 540, 10, 78, 1.0},
  {0, 541, 6, 101, 1.0},  {0, 541, 9, 80, 1.0},  {0, 541, 11, 78, 1.0},  {0, 542, 6, 102, 1.0},  {0, 542, 9, 81, 1.0},  {0, 542, 12, 78, 1.0},
  {0, 543, 6, 103, 1.0},  {0, 543, 9, 82, 1.0},  {0, 543, 13, 78, 1.0},  {0, 544, 6, 104, 1.0},  {0, 544, 9, 83, 1.0},  {0, 544, 14, 78, 1.0},
  {0, 545, 6, 105, 1.0},  {0, 545, 10, 79, 1.0},  {0, 546, 6, 106, 1.0},  {0, 546, 10, 80, 1.0},  {0, 546, 11, 79, 1.0},  {0, 547, 6, 107, 1.0},
  {0, 547, 10, 81, 1.0},  {0, 547, 12, 79, 1.0},  {0, 548, 6, 108, 1.0},  {0, 548, 10, 82, 1.0},  {0, 548, 13, 79, 1.0},  {0, 549, 6, 109, 1.0},
  {0, 549, 10, 83, 1.0},  {0, 549, 14, 79, 1.0},  {0, 550, 6, 110, 1.0},  {0, 550, 11, 80, 1.0},  {0, 551, 6, 111, 1.0},  {0, 551, 11, 81, 1.0},
  {0, 551, 12, 80, 1.0},  {0, 552, 6, 112, 1.0},  {0, 552, 11, 82, 1.0},  {0, 552, 13, 80, 1.0},  {0, 553, 6, 113, 1.0},  {0, 553, 11, 83, 1.0},
  {0, 553, 14, 80, 1.0},  {0, 554, 6, 114, 1.0},  {0, 554, 12, 81, 1.0},  {0, 555, 6, 115, 1.0},  {0, 555, 12, 82, 1.0},  {0, 555, 13, 81, 1.0},
  {0, 556, 6, 116, 1.0},  {0, 556, 12, 83, 1.0},  {0, 556, 14, 81, 1.0},  {0, 557, 6, 117, 1.0},  {0, 557, 13, 82, 1.0},  {0, 558, 6, 118, 1.0},
  {0, 558, 13, 83, 1.0},  {0, 558, 14, 82, 1.0},  {0, 559, 6, 119, 1.0},  {0, 559, 14, 83, 1.0},  {0, 560, 7, 84, 1.0},  {0, 561, 7, 85, 1.0},
  {0, 561, 8, 84, 1.0},  {0, 562, 7, 86, 1.0},  {0, 562, 9, 84, 1.0},  {0, 563, 7, 87, 1.0},  {0, 563, 10, 84, 1.0},  {0, 564, 7, 88, 1.0},
  {0, 564, 11, 84, 1.0},  {0, 565, 7, 89, 1.0},  {0, 565, 12, 84, 1.0},  {0, 566, 7, 90, 1.0},  {0, 566, 13, 84, 1.0},  {0, 567, 7, 91, 1.0},
  {0, 567, 14, 84, 1.0},  {0, 568, 7, 92, 1.0},  {0, 568, 8, 85, 1.0},  {0, 569, 7, 93, 1.0},  {0, 569, 8, 86, 1.0},  {0, 569, 9, 85, 1.0},
  {0, 570, 7, 94, 1.0},  {0, 570, 8, 87, 1.0},  {0, 570, 10, 85, 1.0},  {0, 571, 7, 95, 1.0},  {0, 571, 8, 88, 1.0},  {0, 571, 11, 85, 1.0},
  {0, 572, 7, 96, 1.0},  {0, 572, 8, 89, 1.0},  {0, 572, 12, 85, 1.0},  {0, 573, 7, 97, 1.0},  {0, 573, 8, 90, 1.0},  {0, 573, 13, 85, 1.0},
  {0, 574, 7, 98, 1.0},  {0, 574, 8, 91, 1.0},  {0, 574, 14, 85, 1.0},  {0, 575, 7, 99, 1.0},  {0, 575, 9, 86, 1.0},  {0, 576, 7, 100, 1.0},
  {0, 576, 9, 87, 1.0},  {0, 576, 10, 86, 1.0},  {0, 577, 7, 101, 1.0},  {0, 577, 9, 88, 1.0},  {0, 577, 11, 86, 1.0},  {0, 578, 7, 102, 1.0},
  {0, 578, 9, 89, 1.0},  {0, 578, 12, 86, 1.0},  {0, 579, 7, 103, 1.0},  {0, 579, 9, 90, 1.0},  {0, 579, 13, 86, 1.0},  {0, 580, 7, 104, 1.0},
  {0, 580, 9, 91, 1.0},  {0, 580, 14, 86, 1.0},  {0, 581, 7, 105, 1.0},  {0, 581, 10, 87, 1.0},  {0, 582, 7, 106, 1.0},  {0, 582, 10, 88, 1.0},
  {0, 582, 11, 87, 1.0},  {0, 583, 7, 107, 1.0},  {0, 583, 10, 89, 1.0},  {0, 583, 12, 87, 1.0},  {0, 584, 7, 108, 1.0},  {0, 584, 10, 90, 1.0},
  {0, 584, 13, 87, 1.0},  {0, 585, 7, 109, 1.0},  {0, 585, 10, 91, 1.0},  {0, 585, 14, 87, 1.0},  {0, 586, 7, 110, 1.0},  {0, 586, 11, 88, 1.0},
  {0, 587, 7, 111, 1.0},  {0, 587, 11, 89, 1.0},  {0, 587, 12, 88, 1.0},  {0, 588, 7, 112, 1.0},  {0, 588, 11, 90, 1.0},  {0, 588, 13, 88, 1.0},
  {0, 589, 7, 113, 1.0},  {0, 589, 11, 91, 1.0},  {0, 589, 14, 88, 1.0},  {0, 590, 7, 114, 1.0},  {0, 590, 12, 89, 1.0},  {0, 591, 7, 115, 1.0},
  {0, 591, 12, 90, 1.0},  {0, 591, 13, 89, 1.0},  {0, 592, 7, 116, 1.0},  {0, 592, 12, 91, 1.0},  {0, 592, 14, 89, 1.0},  {0, 593, 7, 117, 1.0},
  {0, 593, 13, 90, 1.0},  {0, 594, 7, 118, 1.0},  {0, 594, 13, 91, 1.0},  {0, 594, 14, 90, 1.0},  {0, 595, 7, 119, 1.0},  {0, 595, 14, 91, 1.0},
  {0, 596, 8, 92, 1.0},  {0, 597, 8, 93, 1.0},  {0, 597, 9, 92, 1.0},  {0, 598, 8, 94, 1.0},  {0, 598, 10, 92, 1.0},  {0, 599, 8, 95, 1.0},
  {0, 599, 11, 92, 1.0},  {0, 600, 8, 96, 1.0},  {0, 600, 12, 92, 1.0},  {0, 601, 8, 97, 1.0},  {0, 601, 13, 92, 1.0},  {0, 602, 8, 98, 1.0},
  {0, 602, 14, 92, 1.0},  {0, 603, 8, 99, 1.0},  {0, 603, 9, 93, 1.0},  {0, 604, 8, 100, 1.0},  {0, 604, 9, 94, 1.0},  {0, 604, 10, 93, 1.0},
  {0, 605, 8, 101, 1.0},  {0, 605, 9, 95, 1.0},  {0, 605, 11, 93, 1.0},  {0, 606, 8, 102, 1.0},  {0, 606, 9, 96, 1.0},  {0, 606, 12, 93, 1.0},
  {0, 607, 8, 103, 1.0},  {0, 607, 9, 97, 1.0},  {0, 607, 13, 93, 1.0},  {0, 608, 8, 104, 1.0},  {0, 608, 9, 98, 1.0},  {0, 608, 14, 93, 1.0},
  {0, 609, 8, 105, 1.0},  {0, 609, 10, 94, 1.0},  {0, 610, 8, 106, 1.0},  {0, 610, 10, 95, 1.0},  {0, 610, 11, 94, 1.0},  {0, 611, 8, 107, 1.0},
  {0, 611, 10, 96, 1.0},  {0, 611, 12, 94, 1.0},  {0, 612, 8, 108, 1.0},  {0, 612, 10, 97, 1.0},  {0, 612, 13, 94, 1.0},  {0, 613, 8, 109, 1.0},
  {0, 613, 10, 98, 1.0},  {0, 613, 14, 94, 1.0},  {0, 614, 8, 110, 1.0},  {0, 614, 11, 95, 1.0},  {0, 615, 8, 111, 1.0},  {0, 615, 11, 96, 1.0},
  {0, 615, 12, 95, 1.0},  {0, 616, 8, 112, 1.0},  {0, 616, 11, 97, 1.0},  {0, 616, 13, 95, 1.0},  {0, 617, 8, 113, 1.0},  {0, 617, 11, 98, 1.0},
  {0, 617, 14, 95, 1.0},  {0, 618, 8, 114, 1.0},  {0, 618, 12, 96, 1.0},  {0, 619, 8, 115, 1.0},  {0, 619, 12, 97, 1.0},  {0, 619, 13, 96, 1.0},
  {0, 620, 8, 116, 1.0},  {0, 620, 12, 98, 1.0},  {0, 620, 14, 96, 1.0},  {0, 621, 8, 117, 1.0},  {0, 621, 13, 97, 1.0},  {0, 622, 8, 118, 1.0},
  {0, 622, 13, 98, 1.0},  {0, 622, 14, 97, 1.0},  {0, 623, 8, 119, 1.0},  {0, 623, 14, 98, 1.0},  {0, 624, 9, 99, 1.0},  {0, 625, 9, 100, 1.0},
  {0, 625, 10, 99, 1.0},  {0, 626, 9, 101, 1.0},  {0, 626, 11, 99, 1.0},  {0, 627, 9, 102, 1.0},  {0, 627, 12, 99, 1.0},  {0, 628, 9, 103, 1.0},
  {0, 628, 13, 99, 1.0},  {0, 629, 9, 104, 1.0},  {0, 629, 14, 99, 1.0},  {0, 630, 9, 105, 1.0},  {0, 630, 10, 100, 1.0},  {0, 631, 9, 106, 1.0},
  {0, 631, 10, 101, 1.0},  {0, 631, 11, 100, 1.0},  {0, 632, 9, 107, 1.0},  {0, 632, 10, 102, 1.0},  {0, 632, 12, 100, 1.0},  {0, 633, 9, 108, 1.0},
  {0, 633, 10, 103, 1.0},  {0, 633, 13, 100, 1.0},  {0, 634, 9, 109, 1.0},  {0, 634, 10, 104, 1.0},  {0, 634, 14, 100, 1.0},  {0, 635, 9, 110, 1.0},
  {0, 635, 11, 101, 1.0},  {0, 636, 9, 111, 1.0},  {0, 636, 11, 102, 1.0},  {0, 636, 12, 101, 1.0},  {0, 637, 9, 112, 1.0},  {0, 637, 11, 103, 1.0},
  {0, 637, 13, 101, 1.0},  {0, 638, 9, 113, 1.0},  {0, 638, 11, 104, 1.0},  {0, 638, 14, 101, 1.0},  {0, 639, 9, 114, 1.0},  {0, 639, 12, 102, 1.0},
  {0, 640, 9, 115, 1.0},  {0, 640, 12, 103, 1.0},  {0, 640, 13, 102, 1.0},  {0, 641, 9, 116, 1.0},  {0, 641, 12, 104, 1.0},  {0, 641, 14, 102, 1.0},
  {0, 642, 9, 117, 1.0},  {0, 642, 13, 103, 1.0},  {0, 643, 9, 118, 1.0},  {0, 643, 13, 104, 1.0},  {0, 643, 14, 103, 1.0},  {0, 644, 9, 119, 1.0},
  {0, 644, 14, 104, 1.0},  {0, 645, 10, 105, 1.0},  {0, 646, 10, 106, 1.0},  {0, 646, 11, 105, 1.0},  {0, 647, 10, 107, 1.0},  {0, 647, 12, 105, 1.0},
  {0, 648, 10, 108, 1.0},  {0, 648, 13, 105, 1.0},  {0, 649, 10, 109, 1.0},  {0, 649, 14, 105, 1.0},  {0, 650, 10, 110, 1.0},  {0, 650, 11, 106, 1.0},
  {0, 651, 10, 111, 1.0},  {0, 651, 11, 107, 1.0},  {0, 651, 12, 106, 1.0},  {0, 652, 10, 112, 1.0},  {0, 652, 11, 108, 1.0},  {0, 652, 13, 106, 1.0},
  {0, 653, 10, 113, 1.0},  {0, 653, 11, 109, 1.0},  {0, 653, 14, 106, 1.0},  {0, 654, 10, 114, 1.0},  {0, 654, 12, 107, 1.0},  {0, 655, 10, 115, 1.0},
  {0, 655, 12, 108, 1.0},  {0, 655, 13, 107, 1.0},  {0, 656, 10, 116, 1.0},  {0, 656, 12, 109, 1.0},  {0, 656, 14, 107, 1.0},  {0, 657, 10, 117, 1.0},
  {0, 657, 13, 108, 1.0},  {0, 658, 10, 118, 1.0},  {0, 658, 13, 109, 1.0},  {0, 658, 14, 108, 1.0},  {0, 659, 10, 119, 1.0},  {0, 659, 14, 109, 1.0},
  {0, 660, 11, 110, 1.0},  {0, 661, 11, 111, 1.0},  {0, 661, 12, 110, 1.0},  {0, 662, 11, 112, 1.0},  {0, 662, 13, 110, 1.0},  {0, 663, 11, 113, 1.0},
  {0, 663, 14, 110, 1.0},  {0, 664, 11, 114, 1.0},  {0, 664, 12, 111, 1.0},  {0, 665, 11, 115, 1.0},  {0, 665, 12, 112, 1.0},  {0, 665, 13, 111, 1.0},
  {0, 666, 11, 116, 1.0},  {0, 666, 12, 113, 1.0},  {0, 666, 14, 111, 1.0},  {0, 667, 11, 117, 1.0},  {0, 667, 13, 112, 1.0},  {0, 668, 11, 118, 1.0},
  {0, 668, 13, 113, 1.0},  {0, 668, 14, 112, 1.0},  {0, 669, 11, 119, 1.0},  {0, 669, 14, 113, 1.0},  {0, 670, 12, 114, 1.0},  {0, 671, 12, 115, 1.0},
  {0, 671, 13, 114, 1.0},  {0, 672, 12, 116, 1.0},  {0, 672, 14, 114, 1.0},  {0, 673, 12, 117, 1.0},  {0, 673, 13, 115, 1.0},  {0, 674, 12, 118, 1.0},
  {0, 674, 13, 116, 1.0},  {0, 674, 14, 115, 1.0},  {0, 675, 12, 119, 1.0},  {0, 675, 14, 116, 1.0},  {0, 676, 13, 117, 1.0},  {0, 677, 13, 118, 1.0},
  {0, 677, 14, 117, 1.0},  {0, 678, 13, 119, 1.0},  {0, 678, 14, 118, 1.0},  {0, 679, 14, 119, 1.0},  {0, 680, 15, 15, 1.0},  {0, 681, 15, 16, 1.0},
  {0, 682, 15, 17, 1.0},  {0, 683, 15, 18, 1.0},  {0, 684, 15, 19, 1.0},  {0, 685, 15, 20, 1.0},  {0, 686, 15, 21, 1.0},  {0, 687, 15, 22, 1.0},
  {0, 688, 15, 23, 1.0},  {0, 689, 15, 24, 1.0},  {0, 690, 15, 25, 1.0},  {0, 691, 15, 26, 1.0},  {0, 692, 15, 27, 1.0},  {0, 693, 15, 28, 1.0},
  {0, 694, 15, 29, 1.0},  {0, 694, 16, 16, 1.0},  {0, 695, 15, 30, 1.0},  {0, 695, 16, 17, 1.0},  {0, 696, 15, 31, 1.0},  {0, 696, 16, 18, 1.0},
  {0, 697, 15, 32, 1.0},  {0, 697, 16, 19, 1.0},  {0, 698, 15, 33, 1.0},  {0, 698, 16, 20, 1.0},  {0, 699, 15, 34, 1.0},  {0, 699, 16, 21, 1.0},
  {0, 700, 15, 35, 1.0},  {0, 700, 16, 22, 1.0},  {0, 701, 15, 36, 1.0},  {0, 701, 16, 23, 1.0},  {0, 702, 15, 37, 1.0},  {0, 702, 16, 24, 1.0},
  {0, 703, 15, 38, 1.0},  {0, 703, 16, 25, 1.0},  {0, 704, 15, 39, 1.0},  {0, 704, 16, 26, 1.0},  {0, 705, 15, 40, 1.0},  {0, 705, 16, 27, 1.0},
  {0, 706, 15, 41, 1.0},  {0, 706, 16, 28, 1.0},  {0, 707, 15, 42, 1.0},  {0, 707, 17, 17, 1.0},  {0, 708, 15, 43, 1.0},  {0, 708, 17, 18, 1.0},
  {0, 709, 15, 44, 1.0},  {0, 709, 17, 19, 1.0},  {0, 710, 15, 45, 1.0},  {0, 710, 17, 20, 1.0},  {0, 711, 15, 46, 1.0},  {0, 711, 17, 21, 1.0},
  {0, 712, 15, 47, 1.0},  {0, 712, 17, 22, 1.0},  {0, 713, 15, 48, 1.0},  {0, 713, 17, 23, 1.0},  {0, 714, 15, 49, 1.0},  {0, 714, 17, 24, 1.0},
  {0, 715, 15, 50, 1.0},  {0, 715, 17, 25, 1.0},  {0, 716, 15, 51, 1.0},  {0, 716, 17, 26, 1.0},  {0, 717, 15, 52, 1.0},  {0, 717, 17, 27, 1.0},
  {0, 718, 15, 53, 1.0},  {0, 718, 17, 28, 1.0},  {0, 719, 15, 54, 1.0},  {0, 719, 18, 18, 1.0},  {0, 720, 15, 55, 1.0},  {0, 720, 18, 19, 1.0},
  {0, 721, 15, 56, 1.0},  {0, 721, 18, 20, 1.0},  {0, 722, 15, 57, 1.0},  {0, 722, 18, 21, 1.0},  {0, 723, 15, 58, 1.0},  {0, 723, 18, 22, 1.0},
  {0, 724, 15, 59, 1.0},  {0, 724, 18, 23, 1.0},  {0, 725, 15, 60, 1.0},  {0, 725, 18, 24, 1.0},  {0, 726, 15, 61, 1.0},  {0, 726, 18, 25, 1.0},
  {0, 727, 15, 62, 1.0},  {0, 727, 18, 26, 1.0},  {0, 728, 15, 63, 1.0},  {0, 728, 18, 27, 1.0},  {0, 729, 15, 64, 1.0},  {0, 729, 18, 28, 1.0},
  {0, 730, 15, 65, 1.0},  {0, 730, 19, 19, 1.0},  {0, 731, 15, 66, 1.0},  {0, 731, 19, 20, 1.0},  {0, 732, 15, 67, 1.0},  {0, 732, 19, 21, 1.0},
  {0, 733, 15, 68, 1.0},  {0, 733, 19, 22, 1.0},  {0, 734, 15, 69, 1.0},  {0, 734, 19, 23, 1.0},  {0, 735, 15, 70, 1.0},  {0, 735, 19, 24, 1.0},
  {0, 736, 15, 71, 1.0},  {0, 736, 19, 25, 1.0},  {0, 737, 15, 72, 1.0},  {0, 737, 19, 26, 1.0},  {0, 738, 15, 73, 1.0},  {0, 738, 19, 27, 1.0},
  {0, 739, 15, 74, 1.0},  {0, 739, 19, 28, 1.0},  {0, 740, 15, 75, 1.0},  {0, 740, 20, 20, 1.0},  {0, 741, 15, 76, 1.0},  {0, 741, 20, 21, 1.0},
  {0, 742, 15, 77, 1.0},  {0, 742, 20, 22, 1.0},  {0, 743, 15, 78, 1.0},  {0, 743, 20, 23, 1.0},  {0, 744, 15, 79, 1.0},  {0, 744, 20, 24, 1.0},
  {0, 745, 15, 80, 1.0},  {0, 745, 20, 25, 1.0},  {0, 746, 15, 81, 1.0},  {0, 746, 20, 26, 1.0},  {0, 747, 15, 82, 1.0},  {0, 747, 20, 27, 1.0},
  {0, 748, 15, 83, 1.0},  {0, 748, 20, 28, 1.0},  {0, 749, 15, 84, 1.0},  {0, 749, 21, 21, 1.0},  {0, 750, 15, 85, 1.0},  {0, 750, 21, 22, 1.0},
  {0, 751, 15, 86, 1.0},  {0, 751, 21, 23, 1.0},  {0, 752, 15, 87, 1.0},  {0, 752, 21, 24, 1.0},  {0, 753, 15, 88, 1.0},  {0, 753, 21, 25, 1.0},
  {0, 754, 15, 89, 1.0},  {0, 754, 21, 26, 1.0},  {0, 755, 15, 90, 1.0},  {0, 755, 21, 27, 1.0},  {0, 756, 15, 91, 1.0},  {0, 756, 21, 28, 1.0},
  {0, 757, 15, 92, 1.0},  {0, 757, 22, 22, 1.0},  {0, 758, 15, 93, 1.0},  {0, 758, 22, 23, 1.0},  {0, 759, 15, 94, 1.0},  {0, 759, 22, 24, 1.0},
  {0, 760, 15, 95, 1.0},  {0, 760, 22, 25, 1.0},  {0, 761, 15, 96, 1.0},  {0, 761, 22, 26, 1.0},  {0, 762, 15, 97, 1.0},  {0, 762, 22, 27, 1.0},
  {0, 763, 15, 98, 1.0},  {0, 763, 22, 28, 1.0},  {0, 764, 15, 99, 1.0},  {0, 764, 23, 23, 1.0},  {0, 765, 15, 100, 1.0},  {0, 765, 23, 24, 1.0},
  {0, 766, 15, 101, 1.0},  {0, 766, 23, 25, 1.0},  {0, 767, 15, 102, 1.0},  {0, 767, 23, 26, 1.0},  {0, 768, 15, 103, 1.0},  {0, 768, 23, 27, 1.0},
  {0, 769, 15, 104, 1.0},  {0, 769, 23, 28, 1.0},  {0, 770, 15, 105, 1.0},  {0, 770, 24, 24, 1.0},  {0, 771, 15, 106, 1.0},  {0, 771, 24, 25, 1.0},
  {0, 772, 15, 107, 1.0},  {0, 772, 24, 26, 1.0},  {0, 773, 15, 108, 1.0},  {0, 773, 24, 27, 1.0},  {0, 774, 15, 109, 1.0},  {0, 774, 24, 28, 1.0},
  {0, 775, 15, 110, 1.0},  {0, 775, 25, 25, 1.0},  {0, 776, 15, 111, 1.0},  {0, 776, 25, 26, 1.0},  {0, 777, 15, 112, 1.0},  {0, 777, 25, 27, 1.0},
  {0, 778, 15, 113, 1.0},  {0, 778, 25, 28, 1.0},  {0, 779, 15, 114, 1.0},  {0, 779, 26, 26, 1.0},  {0, 780, 15, 115, 1.0},  {0, 780, 26, 27, 1.0},
  {0, 781, 15, 116, 1.0},  {0, 781, 26, 28, 1.0},  {0, 782, 15, 117, 1.0},  {0, 782, 27, 27, 1.0},  {0, 783, 15, 118, 1.0},  {0, 783, 27, 28, 1.0},
  {0, 784, 15, 119, 1.0},  {0, 784, 28, 28, 1.0},  {0, 785, 16, 29, 1.0},  {0, 786, 16, 30, 1.0},  {0, 786, 17, 29, 1.0},  {0, 787, 16, 31, 1.0},
  {0, 787, 18, 29, 1.0},  {0, 788, 16, 32, 1.0},  {0, 788, 19, 29, 1.0},  {0, 789, 16, 33, 1.0},  {0, 789, 20, 29, 1.0},  {0, 790, 16, 34, 1.0},
  {0, 790, 21, 29, 1.0},  {0, 791, 16, 35, 1.0},  {0, 791, 22, 29, 1.0},  {0, 792, 16, 36, 1.0},  {0, 792, 23, 29, 1.0},  {0, 793, 16, 37, 1.0},
  {0, 793, 24, 29, 1.0},  {0, 794, 16, 38, 1.0},  {0, 794, 25, 29, 1.0},  {0, 795, 16, 39, 1.0},  {0, 795, 26, 29, 1.0},  {0, 796, 16, 40, 1.0},
  {0, 796, 27, 29, 1.0},  {0, 797, 16, 41, 1.0},  {0, 797, 28, 29, 1.0},  {0, 798, 16, 42, 1.0},  {0, 798, 17, 30, 1.0},  {0, 799, 16, 43, 1.0},
  {0, 799, 17, 31, 1.0},  {0, 799, 18, 30, 1.0},  {0, 800, 16, 44, 1.0},  {0, 800, 17, 32, 1.0},  {0, 800, 19, 30, 1.0},  {0, 801, 16, 45, 1.0},
  {0, 801, 17, 33, 1.0},  {0, 801, 20, 30, 1.0},  {0, 802, 16, 46, 1.0},  {0, 802, 17, 34, 1.0},  {0, 802, 21, 30, 1.0},  {0, 803, 16, 47, 1.0},
  {0, 803, 17, 35, 1.0},  {0, 803, 22, 30, 1.0},  {0, 804, 16, 48, 1.0},  {0, 804, 17, 36, 1.0},  {0, 804, 23, 30, 1.0},  {0, 805, 16, 49, 1.0},
  {0, 805, 17, 37, 1.0},  {0, 805, 24, 30, 1.0},  {0, 806, 16, 50, 1.0},  {0, 806, 17, 38, 1.0},  {0, 806, 25, 30, 1.0},  {0, 807, 16, 51, 1.0},
  {0, 807, 17, 39, 1.0},  {0, 807, 26, 30, 1.0},  {0, 808, 16, 52, 1.0},  {0, 808, 17, 40, 1.0},  {0, 808, 27, 30, 1.0},  {0, 809, 16, 53, 1.0},
  {0, 809, 17, 41, 1.0},  {0, 809, 28, 30, 1.0},  {0, 810, 16, 54, 1.0},  {0, 810, 18, 31, 1.0},  {0, 811, 16, 55, 1.0},  {0, 811, 18, 32, 1.0},
  {0, 811, 19, 31, 1.0},  {0, 812, 16, 56, 1.0},  {0, 812, 18, 33, 1.0},  {0, 812, 20, 31, 1.0},  {0, 813, 16, 57, 1.0},  {0, 813, 18, 34, 1.0},
  {0, 813, 21, 31, 1.0},  {0, 814, 16, 58, 1.0},  {0, 814, 18, 35, 1.0},  {0, 814, 22, 31, 1.0},  {0, 815, 16, 59, 1.0},  {0, 815, 18, 36, 1.0},
  {0, 815, 23, 31, 1.0},  {0, 816, 16, 60, 1.0},  {0, 816, 18, 37, 1.0},  {0, 816, 24, 31, 1.0},  {0, 817, 16, 61, 1.0},  {0, 817, 18, 38, 1.0},
  {0, 817, 25, 31, 1.0},  {0, 818, 16, 62, 1.0},  {0, 818, 18, 39, 1.0},  {0, 818, 26, 31, 1.0},  {0, 819, 16, 63, 1.0},  {0, 819, 18, 40, 1.0},
  {0, 819, 27, 31, 1.0},  {0, 820, 16, 64, 1.0},  {0, 820, 18, 41, 1.0},  {0, 820, 28, 31, 1.0},  {0, 821, 16, 65, 1.0},  {0, 821, 19, 32, 1.0},
  {0, 822, 16, 66, 1.0},  {0, 822, 19, 33, 1.0},  {0, 822, 20, 32, 1.0},  {0, 823, 16, 67, 1.0},  {0, 823, 19, 34, 1.0},  {0, 823, 21, 32, 1.0},
  {0, 824, 16, 68, 1.0},  {0, 824, 19, 35, 1.0},  {0, 824, 22, 32, 1.0},  {0, 825, 16, 69, 1.0},  {0, 825, 19, 36, 1.0},  {0, 825, 23, 32, 1.0},
  {0, 826, 16, 70, 1.0},  {0, 826, 19, 37, 1.0},  {0, 826, 24, 32, 1.0},  {0, 827, 16, 71, 1.0},  {0, 827, 19, 38, 1.0},  {0, 827, 25, 32, 1.0},
  {0, 828, 16, 72, 1.0},  {0, 828, 19, 39, 1.0},  {0, 828, 26, 32, 1.0},  {0, 829, 16, 73, 1.0},  {0, 829, 19, 40, 1.0},  {0, 829, 27, 32, 1.0},
  {0, 830, 16, 74, 1.0},  {0, 830, 19, 41, 1.0},  {0, 830, 28, 32, 1.0},  {0, 831, 16, 75, 1.0},  {0, 831, 20, 33, 1.0},  {0, 832, 16, 76, 1.0},
  {0, 832, 20, 34, 1.0},  {0, 832, 21, 33, 1.0},  {0, 833, 16, 77, 1.0},  {0, 833, 20, 35, 1.0},  {0, 833, 22, 33, 1.0},  {0, 834, 16, 78, 1.0},
  {0, 834, 20, 36, 1.0},  {0, 834, 23, 33, 1.0},  {0, 835, 16, 79, 1.0},  {0, 835, 20, 37, 1.0},  {0, 835, 24, 33, 1.0},  {0, 836, 16, 80, 1.0},
  {0, 836, 20, 38, 1.0},  {0, 836, 25, 33, 1.0},  {0, 837, 16, 81, 1.0},  {0, 837, 20, 39, 1.0},  {0, 837, 26, 33, 1.0},  {0, 838, 16, 82, 1.0},
  {0, 838, 20, 40, 1.0},  {0, 838, 27, 33, 1.0},  {0, 839, 16, 83, 1.0},  {0, 839, 20, 41, 1.0},  {0, 839, 28, 33, 1.0},  {0, 840, 16, 84, 1.0},
  {0, 840, 21, 34, 1.0},  {0, 841, 16, 85, 1.0},  {0, 841, 21, 35, 1.0},  {0, 841, 22, 34, 1.0},  {0, 842, 16, 86, 1.0},  {0, 842, 21, 36, 1.0},
  {0, 842, 23, 34, 1.0},  {0, 843, 16, 87, 1.0},  {0, 843, 21, 37, 1.0},  {0, 843, 24, 34, 1.0},  {0, 844, 16, 88, 1.0},  {0, 844, 21, 38, 1.0},
  {0, 844, 25, 34, 1.0},  {0, 845, 16, 89, 1.0},  {0, 845, 21, 39, 1.0},  {0, 845, 26, 34, 1.0},  {0, 846, 16, 90, 1.0},  {0, 846, 21, 40, 1.0},
  {0, 846, 27, 34, 1.0},  {0, 847, 16, 91, 1.0},  {0, 847, 21, 41, 1.0},  {0, 847, 28, 34, 1.0},  {0, 848, 16, 92, 1.0},  {0, 848, 22, 35, 1.0},
  {0, 849, 16, 93, 1.0},  {0, 849, 22, 36, 1.0},  {0, 849, 23, 35, 1.0},  {0, 850, 16, 94, 1.0},  {0, 850, 22, 37, 1.0},  {0, 850, 24, 35, 1.0},
  {0, 851, 16, 95, 1.0},  {0, 851, 22, 38, 1.0},  {0, 851, 25, 35, 1.0},  {0, 852, 16, 96, 1.0},  {0, 852, 22, 39, 1.0},  {0, 852, 26, 35, 1.0},
  {0, 853, 16, 97, 1.0},  {0, 853, 22, 40, 1.0},  {0, 853, 27, 35, 1.0},  {0, 854, 16, 98, 1.0},  {0, 854, 22, 41, 1.0},  {0, 854, 28, 35, 1.0},
  {0, 855, 16, 99, 1.0},  {0, 855, 23, 36, 1.0},  {0, 856, 16, 100, 1.0},  {0, 856, 23, 37, 1.0},  {0, 856, 24, 36, 1.0},  {0, 857, 16, 101, 1.0},
  {0, 857, 23, 38, 1.0},  {0, 857, 25, 36, 1.0},  {0, 858, 16, 102, 1.0},  {0, 858, 23, 39, 1.0},  {0, 858, 26, 36, 1.0},  {0, 859, 16, 103, 1.0},
  {0, 859, 23, 40, 1.0},  {0, 859, 27, 36, 1.0},  {0, 860, 16, 104, 1.0},  {0, 860, 23, 41, 1.0},  {0, 860, 28, 36, 1.0},  {0, 861, 16, 105, 1.0},
  {0, 861, 24, 37, 1.0},  {0, 862, 16, 106, 1.0},  {0, 862, 24, 38, 1.0},  {0, 862, 25, 37, 1.0},  {0, 863, 16, 107, 1.0},  {0, 863, 24, 39, 1.0},
  {0, 863, 26, 37, 1.0},  {0, 864, 16, 108, 1.0},  {0, 864, 24, 40, 1.0},  {0, 864, 27, 37, 1.0},  {0, 865, 16, 109, 1.0},  {0, 865, 24, 41, 1.0},
  {0, 865, 28, 37, 1.0},  {0, 866, 16, 110, 1.0},  {0, 866, 25, 38, 1.0},  {0, 867, 16, 111, 1.0},  {0, 867, 25, 39, 1.0},  {0, 867, 26, 38, 1.0},
  {0, 868, 16, 112, 1.0},  {0, 868, 25, 40, 1.0},  {0, 868, 27, 38, 1.0},  {0, 869, 16, 113, 1.0},  {0, 869, 25, 41, 1.0},  {0, 869, 28, 38, 1.0},
  {0, 870, 16, 114, 1.0},  {0, 870, 26, 39, 1.0},  {0, 871, 16, 115, 1.0},  {0, 871, 26, 40, 1.0},  {0, 871, 27, 39, 1.0},  {0, 872, 16, 116, 1.0},
  {0, 872, 26, 41, 1.0},  {0, 872, 28, 39, 1.0},  {0, 873, 16, 117, 1.0},  {0, 873, 27, 40, 1.0},  {0, 874, 16, 118, 1.0},  {0, 874, 27, 41, 1.0},
  {0, 874, 28, 40, 1.0},  {0, 875, 16, 119, 1.0},  {0, 875, 28, 41, 1.0},  {0, 876, 17, 42, 1.0},  {0, 877, 17, 43, 1.0},  {0, 877, 18, 42, 1.0},
  {0, 878, 17, 44, 1.0},  {0, 878, 19, 42, 1.0},  {0, 879, 17, 45, 1.0},  {0, 879, 20, 42, 1.0},  {0, 880, 17, 46, 1.0},  {0, 880, 21, 42, 1.0},
  {0, 881, 17, 47, 1.0},  {0, 881, 22, 42, 1.0},  {0, 882, 17, 48, 1.0},  {0, 882, 23, 42, 1.0},  {0, 883, 17, 49, 1.0},  {0, 883, 24, 42, 1.0},
  {0, 884, 17, 50, 1.0},  {0, 884, 25, 42, 1.0},  {0, 885, 17, 51, 1.0},  {0, 885, 26, 42, 1.0},  {0, 886, 17, 52, 1.0},  {0, 886, 27, 42, 1.0},
  {0, 887, 17, 53, 1.0},  {0, 887, 28, 42, 1.0},  {0, 888, 17, 54, 1.0},  {0, 888, 18, 43, 1.0},  {0, 889, 17, 55, 1.0},  {0, 889, 18, 44, 1.0},
  {0, 889, 19, 43, 1.0},  {0, 890, 17, 56, 1.0},  {0, 890, 18, 45, 1.0},  {0, 890, 20, 43, 1.0},  {0, 891, 17, 57, 1.0},  {0, 891, 18, 46, 1.0},
  {0, 891, 21, 43, 1.0},  {0, 892, 17, 58, 1.0},  {0, 892, 18, 47, 1.0},  {0, 892, 22, 43, 1.0},  {0, 893, 17, 59, 1.0},  {0, 893, 18, 48, 1.0},
  {0, 893, 23, 43, 1.0},  {0, 894, 17, 60, 1.0},  {0, 894, 18, 49, 1.0},  {0, 894, 24, 43, 1.0},  {0, 895, 17, 61, 1.0},  {0, 895, 18, 50, 1.0},
  {0, 895, 25, 43, 1.0},  {0, 896, 17, 62, 1.0},  {0, 896, 18, 51, 1.0},  {0, 896, 26, 43, 1.0},  {0, 897, 17, 63, 1.0},  {0, 897, 18, 52, 1.0},
  {0, 897, 27, 43, 1.0},  {0, 898, 17, 64, 1.0},  {0, 898, 18, 53, 1.0},  {0, 898, 28, 43, 1.0},  {0, 899, 17, 65, 1.0},  {0, 899, 19, 44, 1.0},
  {0, 900, 17, 66, 1.0},  {0, 900, 19, 45, 1.0},  {0, 900, 20, 44, 1.0},  {0, 901, 17, 67, 1.0},  {0, 901, 19, 46, 1.0},  {0, 901, 21, 44, 1.0},
  {0, 902, 17, 68, 1.0},  {0, 902, 19, 47, 1.0},  {0, 902, 22, 44, 1.0},  {0, 903, 17, 69, 1.0},  {0, 903, 19, 48, 1.0},  {0, 903, 23, 44, 1.0},
  {0, 904, 17, 70, 1.0},  {0, 904, 19, 49, 1.0},  {0, 904, 24, 44, 1.0},  {0, 905, 17, 71, 1.0},  {0, 905, 19, 50, 1.0},  {0, 905, 25, 44, 1.0},
  {0, 906, 17, 72, 1.0},  {0, 906, 19, 51, 1.0},  {0, 906, 26, 44, 1.0},  {0, 907, 17, 73, 1.0},  {0, 907, 19, 52, 1.0},  {0, 907, 27, 44, 1.0},
  {0, 908, 17, 74, 1.0},  {0, 908, 19, 53, 1.0},  {0, 908, 28, 44, 1.0},  {0, 909, 17, 75, 1.0},  {0, 909, 20, 45, 1.0},  {0, 910, 17, 76, 1.0},
  {0, 910, 20, 46, 1.0},  {0, 910, 21, 45, 1.0},  {0, 911, 17, 77, 1.0},  {0, 911, 20, 47, 1.0},  {0, 911, 22, 45, 1.0},  {0, 912, 17, 78, 1.0},
  {0, 912, 20, 48, 1.0},  {0, 912, 23, 45, 1.0},  {0, 913, 17, 79, 1.0},  {0, 913, 20, 49, 1.0},  {0, 913, 24, 45, 1.0},  {0, 914, 17, 80, 1.0},
  {0, 914, 20, 50, 1.0},  {0, 914, 25, 45, 1.0},  {0, 915, 17, 81, 1.0},  {0, 915, 20, 51, 1.0},  {0, 915, 26, 45, 1.0},  {0, 916, 17, 82, 1.0},
  {0, 916, 20, 52, 1.0},  {0, 916, 27, 45, 1.0},  {0, 917, 17, 83, 1.0},  {0, 917, 20, 53, 1.0},  {0, 917, 28, 45, 1.0},  {0, 918, 17, 84, 1.0},
  {0, 918, 21, 46, 1.0},  {0, 919, 17, 85, 1.0},  {0, 919, 21, 47, 1.0},  {0, 919, 22, 46, 1.0},  {0, 920, 17, 86, 1.0},  {0, 920, 21, 48, 1.0},
  {0, 920, 23, 46, 1.0},  {0, 921, 17, 87, 1.0},  {0, 921, 21, 49, 1.0},  {0, 921, 24, 46, 1.0},  {0, 922, 17, 88, 1.0},  {0, 922, 21, 50, 1.0},
  {0, 922, 25, 46, 1.0},  {0, 923, 17, 89, 1.0},  {0, 923, 21, 51, 1.0},  {0, 923, 26, 46, 1.0},  {0, 924, 17, 90, 1.0},  {0, 924, 21, 52, 1.0},
  {0, 924, 27, 46, 1.0},  {0, 925, 17, 91, 1.0},  {0, 925, 21, 53, 1.0},  {0, 925, 28, 46, 1.0},  {0, 926, 17, 92, 1.0},  {0, 926, 22, 47, 1.0},
  {0, 927, 17, 93, 1.0},  {0, 927, 22, 48, 1.0},  {0, 927, 23, 47, 1.0},  {0, 928, 17, 94, 1.0},  {0, 928, 22, 49, 1.0},  {0, 928, 24, 47, 1.0},
  {0, 929, 17, 95, 1.0},  {0, 929, 22, 50, 1.0},  {0, 929, 25, 47, 1.0},  {0, 930, 17, 96, 1.0},  {0, 930, 22, 51, 1.0},  {0, 930, 26, 47, 1.0},
  {0, 931, 17, 97, 1.0},  {0, 931, 22, 52, 1.0},  {0, 931, 27, 47, 1.0},  {0, 932, 17, 98, 1.0},  {0, 932, 22, 53, 1.0},  {0, 932, 28, 47, 1.0},
  {0, 933, 17, 99, 1.0},  {0, 933, 23, 48, 1.0},  {0, 934, 17, 100, 1.0},  {0, 934, 23, 49, 1.0},  {0, 934, 24, 48, 1.0},  {0, 935, 17, 101, 1.0},
  {0, 935, 23, 50, 1.0},  {0, 935, 25, 48, 1.0},  {0, 936, 17, 102, 1.0},  {0, 936, 23, 51, 1.0},  {0, 936, 26, 48, 1.0},  {0, 937, 17, 103, 1.0},
  {0, 937, 23, 52, 1.0},  {0, 937, 27, 48, 1.0},  {0, 938, 17, 104, 1.0},  {0, 938, 23, 53, 1.0},  {0, 938, 28, 48, 1.0},  {0, 939, 17, 105, 1.0},
  {0, 939, 24, 49, 1.0},  {0, 940, 17, 106, 1.0},  {0, 940, 24, 50, 1.0},  {0, 940, 25, 49, 1.0},  {0, 941, 17, 107, 1.0},  {0, 941, 24, 51, 1.0},
  {0, 941, 26, 49, 1.0},  {0, 942, 17, 108, 1.0},  {0, 942, 24, 52, 1.0},  {0, 942, 27, 49, 1.0},  {0, 943, 17, 109, 1.0},  {0, 943, 24, 53, 1.0},
  {0, 943, 28, 49, 1.0},  {0, 944, 17, 110, 1.0},  {0, 944, 25, 50, 1.0},  {0, 945, 17, 111, 1.0},  {0, 945, 25, 51, 1.0},  {0, 945, 26, 50, 1.0},
  {0, 946, 17, 112, 1.0},  {0, 946, 25, 52, 1.0},  {0, 946, 27, 50, 1.0},  {0, 947, 17, 113, 1.0},  {0, 947, 25, 53, 1.0},  {0, 947, 28, 50, 1.0},
  {0, 948, 17, 114, 1.0},  {0, 948, 26, 51, 1.0},  {0, 949, 17, 115, 1.0},  {0, 949, 26, 52, 1.0},  {0, 949, 27, 51, 1.0},  {0, 950, 17, 116, 1.0},
  {0, 950, 26, 53, 1.0},  {0, 950, 28, 51, 1.0},  {0, 951, 17, 117, 1.0},  {0, 951, 27, 52, 1.0},  {0, 952, 17, 118, 1.0},  {0, 952, 27, 53, 1.0},
  {0, 952, 28, 52, 1.0},  {0, 953, 17, 119, 1.0},  {0, 953, 28, 53, 1.0},  {0, 954, 18, 54, 1.0},  {0, 955, 18, 55, 1.0},  {0, 955, 19, 54, 1.0},
  {0, 956, 18, 56, 1.0},  {0, 956, 20, 54, 1.0},  {0, 957, 18, 57, 1.0},  {0, 957, 21, 54, 1.0},  {0, 958, 18, 58, 1.0},  {0, 958, 22, 54, 1.0},
  {0, 959, 18, 59, 1.0},  {0, 959, 23, 54, 1.0},  {0, 960, 18, 60, 1.0},  {0, 960, 24, 54, 1.0},  {0, 961, 18, 61, 1.0},  {0, 961, 25, 54, 1.0},
  {0, 962, 18, 62, 1.0},  {0, 962, 26, 54, 1.0},  {0, 963, 18, 63, 1.0},  {0, 963, 27, 54, 1.0},  {0, 964, 18, 64, 1.0},  {0, 964, 28, 54, 1.0},
  {0, 965, 18, 65, 1.0},  {0, 965, 19, 55, 1.0},  {0, 966, 18, 66, 1.0},  {0, 966, 19, 56, 1.0},  {0, 966, 20, 55, 1.0},  {0, 967, 18, 67, 1.0},
  {0, 967, 19, 57, 1.0},  {0, 967, 21, 55, 1.0},  {0, 968, 18, 68, 1.0},  {0, 968, 19, 58, 1.0},  {0, 968, 22, 55, 1.0},  {0, 969, 18, 69, 1.0},
  {0, 969, 19, 59, 1.0},  {0, 969, 23, 55, 1.0},  {0, 970, 18, 70, 1.0},  {0, 970, 19, 60, 1.0},  {0, 970, 24, 55, 1.0},  {0, 971, 18, 71, 1.0},
  {0, 971, 19, 61, 1.0},  {0, 971, 25, 55, 1.0},  {0, 972, 18, 72, 1.0},  {0, 972, 19, 62, 1.0},  {0, 972, 26, 55, 1.0},  {0, 973, 18, 73, 1.0},
  {0, 973, 19, 63, 1.0},  {0, 973, 27, 55, 1.0},  {0, 974, 18, 74, 1.0},  {0, 974, 19, 64, 1.0},  {0, 974, 28, 55, 1.0},  {0, 975, 18, 75, 1.0},
  {0, 975, 20, 56, 1.0},  {0, 976, 18, 76, 1.0},  {0, 976, 20, 57, 1.0},  {0, 976, 21, 56, 1.0},  {0, 977, 18, 77, 1.0},  {0, 977, 20, 58, 1.0},
  {0, 977, 22, 56, 1.0},  {0, 978, 18, 78, 1.0},  {0, 978, 20, 59, 1.0},  {0, 978, 23, 56, 1.0},  {0, 979, 18, 79, 1.0},  {0, 979, 20, 60, 1.0},
  {0, 979, 24, 56, 1.0},  {0, 980, 18, 80, 1.0},  {0, 980, 20, 61, 1.0},  {0, 980, 25, 56, 1.0},  {0, 981, 18, 81, 1.0},  {0, 981, 20, 62, 1.0},
  {0, 981, 26, 56, 1.0},  {0, 982, 18, 82, 1.0},  {0, 982, 20, 63, 1.0},  {0, 982, 27, 56, 1.0},  {0, 983, 18, 83, 1.0},  {0, 983, 20, 64, 1.0},
  {0, 983, 28, 56, 1.0},  {0, 984, 18, 84, 1.0},  {0, 984, 21, 57, 1.0},  {0, 985, 18, 85, 1.0},  {0, 985, 21, 58, 1.0},  {0, 985, 22, 57, 1.0},
  {0, 986, 18, 86, 1.0},  {0, 986, 21, 59, 1.0},  {0, 986, 23, 57, 1.0},  {0, 987, 18, 87, 1.0},  {0, 987, 21, 60, 1.0},  {0, 987, 24, 57, 1.0},
  {0, 988, 18, 88, 1.0},  {0, 988, 21, 61, 1.0},  {0, 988, 25, 57, 1.0},  {0, 989, 18, 89, 1.0},  {0, 989, 21, 62, 1.0},  {0, 989, 26, 57, 1.0},
  {0, 990, 18, 90, 1.0},  {0, 990, 21, 63, 1.0},  {0, 990, 27, 57, 1.0},  {0, 991, 18, 91, 1.0},  {0, 991, 21, 64, 1.0},  {0, 991, 28, 57, 1.0},
  {0, 992, 18, 92, 1.0},  {0, 992, 22, 58, 1.0},  {0, 993, 18, 93, 1.0},  {0, 993, 22, 59, 1.0},  {0, 993, 23, 58, 1.0},  {0, 994, 18, 94, 1.0},
  {0, 994, 22, 60, 1.0},  {0, 994, 24, 58, 1.0},  {0, 995, 18, 95, 1.0},  {0, 995, 22, 61, 1.0},  {0, 995, 25, 58, 1.0},  {0, 996, 18, 96, 1.0},
  {0, 996, 22, 62, 1.0},  {0, 996, 26, 58, 1.0},  {0, 997, 18, 97, 1.0},  {0, 997, 22, 63, 1.0},  {0, 997, 27, 58, 1.0},  {0, 998, 18, 98, 1.0},
  {0, 998, 22, 64, 1.0},  {0, 998, 28, 58, 1.0},  {0, 999, 18, 99, 1.0},  {0, 999, 23, 59, 1.0},  {0, 1000, 18, 100, 1.0},  {0, 1000, 23, 60, 1.0},
  {0, 1000, 24, 59, 1.0},  {0, 1001, 18, 101, 1.0},  {0, 1001, 23, 61, 1.0},  {0, 1001, 25, 59, 1.0},  {0, 1002, 18, 102, 1.0},  {0, 1002, 23, 62, 1.0},
  {0, 1002, 26, 59, 1.0},  {0, 1003, 18, 103, 1.0},  {0, 1003, 23, 63, 1.0},  {0, 1003, 27, 59, 1.0},  {0, 1004, 18, 104, 1.0},  {0, 1004, 23, 64, 1.0},
  {0, 1004, 28, 59, 1.0},  {0, 1005, 18, 105, 1.0},  {0, 1005, 24, 60, 1.0},  {0, 1006, 18, 106, 1.0},  {0, 1006, 24, 61, 1.0},  {0, 1006, 25, 60, 1.0},
  {0, 1007, 18, 107, 1.0},  {0, 1007, 24, 62, 1.0},  {0, 1007, 26, 60, 1.0},  {0, 1008, 18, 108, 1.0},  {0, 1008, 24, 63, 1.0},  {0, 1008, 27, 60, 1.0},
  {0, 1009, 18, 109, 1.0},  {0, 1009, 24, 64, 1.0},  {0, 1009, 28, 60, 1.0},  {0, 1010, 18, 110, 1.0},  {0, 1010, 25, 61, 1.0},  {0, 1011, 18, 111, 1.0},
  {0, 1011, 25, 62, 1.0},  {0, 1011, 26, 61, 1.0},  {0, 1012, 18, 112, 1.0},  {0, 1012, 25, 63, 1.0},  {0, 1012, 27, 61, 1.0},  {0, 1013, 18, 113, 1.0},
  {0, 1013, 25, 64, 1.0},  {0, 1013, 28, 61, 1.0},  {0, 1014, 18, 114, 1.0},  {0, 1014, 26, 62, 1.0},  {0, 1015, 18, 115, 1.0},  {0, 1015, 26, 63, 1.0},
  {0, 1015, 27, 62, 1.0},  {0, 1016, 18, 116, 1.0},  {0, 1016, 26, 64, 1.0},  {0, 1016, 28, 62, 1.0},  {0, 1017, 18, 117, 1.0},  {0, 1017, 27, 63, 1.0},
  {0, 1018, 18, 118, 1.0},  {0, 1018, 27, 64, 1.0},  {0, 1018, 28, 63, 1.0},  {0, 1019, 18, 119, 1.0},  {0, 1019, 28, 64, 1.0},  {0, 1020, 19, 65, 1.0},
  {0, 1021, 19, 66, 1.0},  {0, 1021, 20, 65, 1.0},  {0, 1022, 19, 67, 1.0},  {0, 1022, 21, 65, 1.0},  {0, 1023, 19, 68, 1.0},  {0, 1023, 22, 65, 1.0},
  {0, 1024, 19, 69, 1.0},  {0, 1024, 23, 65, 1.0},  {0, 1025, 19, 70, 1.0},  {0, 1025, 24, 65, 1.0},  {0, 1026, 19, 71, 1.0},  {0, 1026, 25, 65, 1.0},
  {0, 1027, 19, 72, 1.0},  {0, 1027, 26, 65, 1.0},  {0, 1028, 19, 73, 1.0},  {0, 1028, 27, 65, 1.0},  {0, 1029, 19, 74, 1.0},  {0, 1029, 28, 65, 1.0},
  {0, 1030, 19, 75, 1.0},  {0, 1030, 20, 66, 1.0},  {0, 1031, 19, 76, 1.0},  {0, 1031, 20, 67, 1.0},  {0, 1031, 21, 66, 1.0},  {0, 1032, 19, 77, 1.0},
  {0, 1032, 20, 68, 1.0},  {0, 1032, 22, 66, 1.0},  {0, 1033, 19, 78, 1.0},  {0, 1033, 20, 69, 1.0},  {0, 1033, 23, 66, 1.0},  {0, 1034, 19, 79, 1.0},
  {0, 1034, 20, 70, 1.0},  {0, 1034, 24, 66, 1.0},  {0, 1035, 19, 80, 1.0},  {0, 1035, 20, 71, 1.0},  {0, 1035, 25, 66, 1.0},  {0, 1036, 19, 81, 1.0},
  {0, 1036, 20, 72, 1.0},  {0, 1036, 26, 66, 1.0},  {0, 1037, 19, 82, 1.0},  {0, 1037, 20, 73, 1.0},  {0, 1037, 27, 66, 1.0},  {0, 1038, 19, 83, 1.0},
  {0, 1038, 20, 74, 1.0},  {0, 1038, 28, 66, 1.0},  {0, 1039, 19, 84, 1.0},  {0, 1039, 21, 67, 1.0},  {0, 1040, 19, 85, 1.0},  {0, 1040, 21, 68, 1.0},
  {0, 1040, 22, 67, 1.0},  {0, 1041, 19, 86, 1.0},  {0, 1041, 21, 69, 1.0},  {0, 1041, 23, 67, 1.0},  {0, 1042, 19, 87, 1.0},  {0, 1042, 21, 70, 1.0},
  {0, 1042, 24, 67, 1.0},  {0, 1043, 19, 88, 1.0},  {0, 1043, 21, 71, 1.0},  {0, 1043, 25, 67, 1.0},  {0, 1044, 19, 89, 1.0},  {0, 1044, 21, 72, 1.0},
  {0, 1044, 26, 67, 1.0},  {0, 1045, 19, 90, 1.0},  {0, 1045, 21, 73, 1.0},  {0, 1045, 27, 67, 1.0},  {0, 1046, 19, 91, 1.0},  {0, 1046, 21, 74, 1.0},
  {0, 1046, 28, 67, 1.0},  {0, 1047, 19, 92, 1.0},  {0, 1047, 22, 68, 1.0},  {0, 1048, 19, 93, 1.0},  {0, 1048, 22, 69, 1.0},  {0, 1048, 23, 68, 1.0},
  {0, 1049, 19, 94, 1.0},  {0, 1049, 22, 70, 1.0},  {0, 1049, 24, 68, 1.0},  {0, 1050, 19, 95, 1.0},  {0, 1050, 22, 71, 1.0},  {0, 1050, 25, 68, 1.0},
  {0, 1051, 19, 96, 1.0},  {0, 1051, 22, 72, 1.0},  {0, 1051, 26, 68, 1.0},  {0, 1052, 19, 97, 1.0},  {0, 1052, 22, 73, 1.0},  {0, 1052, 27, 68, 1.0},
  {0, 1053, 19, 98, 1.0},  {0, 1053, 22, 74, 1.0},  {0, 1053, 28, 68, 1.0},  {0, 1054, 19, 99, 1.0},  {0, 1054, 23, 69, 1.0},  {0, 1055, 19, 100, 1.0},
  {0, 1055, 23, 70, 1.0},  {0, 1055, 24, 69, 1.0},  {0, 1056, 19, 101, 1.0},  {0, 1056, 23, 71, 1.0},  {0, 1056, 25, 69, 1.0},  {0, 1057, 19, 102, 1.0},
  {0, 1057, 23, 72, 1.0},  {0, 1057, 26, 69, 1.0},  {0, 1058, 19, 103, 1.0},  {0, 1058, 23, 73, 1.0},  {0, 1058, 27, 69, 1.0},  {0, 1059, 19, 104, 1.0},
  {0, 1059, 23, 74, 1.0},  {0, 1059, 28, 69, 1.0},  {0, 1060, 19, 105, 1.0},  {0, 1060, 24, 70, 1.0},  {0, 1061, 19, 106, 1.0},  {0, 1061, 24, 71, 1.0},
  {0, 1061, 25, 70, 1.0},  {0, 1062, 19, 107, 1.0},  {0, 1062, 24, 72, 1.0},  {0, 1062, 26, 70, 1.0},  {0, 1063, 19, 108, 1.0},  {0, 1063, 24, 73, 1.0},
  {0, 1063, 27, 70, 1.0},  {0, 1064, 19, 109, 1.0},  {0, 1064, 24, 74, 1.0},  {0, 1064, 28, 70, 1.0},  {0, 1065, 19, 110, 1.0},  {0, 1065, 25, 71, 1.0},
  {0, 1066, 19, 111, 1.0},  {0, 1066, 25, 72, 1.0},  {0, 1066, 26, 71, 1.0},  {0, 1067, 19, 112, 1.0},  {0, 1067, 25, 73, 1.0},  {0, 1067, 27, 71, 1.0},
  {0, 1068, 19, 113, 1.0},  {0, 1068, 25, 74, 1.0},  {0, 1068, 28, 71, 1.0},  {0, 1069, 19, 114, 1.0},  {0, 1069, 26, 72, 1.0},  {0, 1070, 19, 115, 1.0},
  {0, 1070, 26, 73, 1.0},  {0, 1070, 27, 72, 1.0},  {0, 1071, 19, 116, 1.0},  {0, 1071, 26, 74, 1.0},  {0, 1071, 28, 72, 1.0},  {0, 1072, 19, 117, 1.0},
  {0, 1072, 27, 73, 1.0},  {0, 1073, 19, 118, 1.0},  {0, 1073, 27, 74, 1.0},  {0, 1073, 28, 73, 1.0},  {0, 1074, 19, 119, 1.0},  {0, 1074, 28, 74, 1.0},
  {0, 1075, 20, 75, 1.0},  {0, 1076, 20, 76, 1.0},  {0, 1076, 21, 75, 1.0},  {0, 1077, 20, 77, 1.0},  {0, 1077, 22, 75, 1.0},  {0, 1078, 20, 78, 1.0},
  {0, 1078, 23, 75, 1.0},  {0, 1079, 20, 79, 1.0},  {0, 1079, 24, 75, 1.0},  {0, 1080, 20, 80, 1.0},  {0, 1080, 25, 75, 1.0},  {0, 1081, 20, 81, 1.0},
  {0, 1081, 26, 75, 1.0},  {0, 1082, 20, 82, 1.0},  {0, 1082, 27, 75, 1.0},  {0, 1083, 20, 83, 1.0},  {0, 1083, 28, 75, 1.0},  {0, 1084, 20, 84, 1.0},
  {0, 1084, 21, 76, 1.0},  {0, 1085, 20, 85, 1.0},  {0, 1085, 21, 77, 1.0},  {0, 1085, 22, 76, 1.0},  {0, 1086, 20, 86, 1.0},  {0, 1086, 21, 78, 1.0},
  {0, 1086, 23, 76, 1.0},  {0, 1087, 20, 87, 1.0},  {0, 1087, 21, 79, 1.0},  {0, 1087, 24, 76, 1.0},  {0, 1088, 20, 88, 1.0},  {0, 1088, 21, 80, 1.0},
  {0, 1088, 25, 76, 1.0},  {0, 1089, 20, 89, 1.0},  {0, 1089, 21, 81, 1.0},  {0, 1089, 26, 76, 1.0},  {0, 1090, 20, 90, 1.0},  {0, 1090, 21, 82, 1.0},
  {0, 1090, 27, 76, 1.0},  {0, 1091, 20, 91, 1.0},  {0, 1091, 21, 83, 1.0},  {0, 1091, 28, 76, 1.0},  {0, 1092, 20, 92, 1.0},  {0, 1092, 22, 77, 1.0},
  {0, 1093, 20, 93, 1.0},  {0, 1093, 22, 78, 1.0},  {0, 1093, 23, 77, 1.0},  {0, 1094, 20, 94, 1.0},  {0, 1094, 22, 79, 1.0},  {0, 1094, 24, 77, 1.0},
  {0, 1095, 20, 95, 1.0},  {0, 1095, 22, 80, 1.0},  {0, 1095, 25, 77, 1.0},  {0, 1096, 20, 96, 1.0},  {0, 1096, 22, 81, 1.0},  {0, 1096, 26, 77, 1.0},
  {0, 1097, 20, 97, 1.0},  {0, 1097, 22, 82, 1.0},  {0, 1097, 27, 77, 1.0},  {0, 1098, 20, 98, 1.0},  {0, 1098, 22, 83, 1.0},  {0, 1098, 28, 77, 1.0},
  {0, 1099, 20, 99, 1.0},  {0, 1099, 23, 78, 1.0},  {0, 1100, 20, 100, 1.0},  {0, 1100, 23, 79, 1.0},  {0, 1100, 24, 78, 1.0},  {0, 1101, 20, 101, 1.0},
  {0, 1101, 23, 80, 1.0},  {0, 1101, 25, 78, 1.0},  {0, 1102, 20, 102, 1.0},  {0, 1102, 23, 81, 1.0},  {0, 1102, 26, 78, 1.0},  {0, 1103, 20, 103, 1.0},
  {0, 1103, 23, 82, 1.0},  {0, 1103, 27, 78, 1.0},  {0, 1104, 20, 104, 1.0},  {0, 1104, 23, 83, 1.0},  {0, 1104, 28, 78, 1.0},  {0, 1105, 20, 105, 1.0},
  {0, 1105, 24, 79, 1.0},  {0, 1106, 20, 106, 1.0},  {0, 1106, 24, 80, 1.0},  {0, 1106, 25, 79, 1.0},  {0, 1107, 20, 107, 1.0},  {0, 1107, 24, 81, 1.0},
  {0, 1107, 26, 79, 1.0},  {0, 1108, 20, 108, 1.0},  {0, 1108, 24, 82, 1.0},  {0, 1108, 27, 79, 1.0},  {0, 1109, 20, 109, 1.0},  {0, 1109, 24, 83, 1.0},
  {0, 1109, 28, 79, 1.0},  {0, 1110, 20, 110, 1.0},  {0, 1110, 25, 80, 1.0},  {0, 1111, 20, 111, 1.0},  {0, 1111, 25, 81, 1.0},  {0, 1111, 26, 80, 1.0},
  {0, 1112, 20, 112, 1.0},  {0, 1112, 25, 82, 1.0},  {0, 1112, 27, 80, 1.0},  {0, 1113, 20, 113, 1.0},  {0, 1113, 25, 83, 1.0},  {0, 1113, 28, 80, 1.0},
  {0, 1114, 20, 114, 1.0},  {0, 1114, 26, 81, 1.0},  {0, 1115, 20, 115, 1.0},  {0, 1115, 26, 82, 1.0},  {0, 1115, 27, 81, 1.0},  {0, 1116, 20, 116, 1.0},
  {0, 1116, 26, 83, 1.0},  {0, 1116, 28, 81, 1.0},  {0, 1117, 20, 117, 1.0},  {0, 1117, 27, 82, 1.0},  {0, 1118, 20, 118, 1.0},  {0, 1118, 27, 83, 1.0},
  {0, 1118, 28, 82, 1.0},  {0, 1119, 20, 119, 1.0},  {0, 1119, 28, 83, 1.0},  {0, 1120, 21, 84, 1.0},  {0, 1121, 21, 85, 1.0},  {0, 1121, 22, 84, 1.0},
  {0, 1122, 21, 86, 1.0},  {0, 1122, 23, 84, 1.0},  {0, 1123, 21, 87, 1.0},  {0, 1123, 24, 84, 1.0},  {0, 1124, 21, 88, 1.0},  {0, 1124, 25, 84, 1.0},
  {0, 1125, 21, 89, 1.0},  {0, 1125, 26, 84, 1.0},  {0, 1126, 21, 90, 1.0},  {0, 1126, 27, 84, 1.0},  {0, 1127, 21, 91, 1.0},  {0, 1127, 28, 84, 1.0},
  {0, 1128, 21, 92, 1.0},  {0, 1128, 22, 85, 1.0},  {0, 1129, 21, 93, 1.0},  {0, 1129, 22, 86, 1.0},  {0, 1129, 23, 85, 1.0},  {0, 1130, 21, 94, 1.0},
  {0, 1130, 22, 87, 1.0},  {0, 1130, 24, 85, 1.0},  {0, 1131, 21, 95, 1.0},  {0, 1131, 22, 88, 1.0},  {0, 1131, 25, 85, 1.0},  {0, 1132, 21, 96, 1.0},
  {0, 1132, 22, 89, 1.0},  {0, 1132, 26, 85, 1.0},  {0, 1133, 21, 97, 1.0},  {0, 1133, 22, 90, 1.0},  {0, 1133, 27, 85, 1.0},  {0, 1134, 21, 98, 1.0},
  {0, 1134, 22, 91, 1.0},  {0, 1134, 28, 85, 1.0},  {0, 1135, 21, 99, 1.0},  {0, 1135, 23, 86, 1.0},  {0, 1136, 21, 100, 1.0},  {0, 1136, 23, 87, 1.0},
  {0, 1136, 24, 86, 1.0},  {0, 1137, 21, 101, 1.0},  {0, 1137, 23, 88, 1.0},  {0, 1137, 25, 86, 1.0},  {0, 1138, 21, 102, 1.0},  {0, 1138, 23, 89, 1.0},
  {0, 1138, 26, 86, 1.0},  {0, 1139, 21, 103, 1.0},  {0, 1139, 23, 90, 1.0},  {0, 1139, 27, 86, 1.0},  {0, 1140, 21, 104, 1.0},  {0, 1140, 23, 91, 1.0},
  {0, 1140, 28, 86, 1.0},  {0, 1141, 21, 105, 1.0},  {0, 1141, 24, 87, 1.0},  {0, 1142, 21, 106, 1.0},  {0, 1142, 24, 88, 1.0},  {0, 1142, 25, 87, 1.0},
  {0, 1143, 21, 107, 1.0},  {0, 1143, 24, 89, 1.0},  {0, 1143, 26, 87, 1.0},  {0, 1144, 21, 108, 1.0},  {0, 1144, 24, 90, 1.0},  {0, 1144, 27, 87, 1.0},
  {0, 1145, 21, 109, 1.0},  {0, 1145, 24, 91, 1.0},  {0, 1145, 28, 87, 1.0},  {0, 1146, 21, 110, 1.0},  {0, 1146, 25, 88, 1.0},  {0, 1147, 21, 111, 1.0},
  {0, 1147, 25, 89, 1.0},  {0, 1147, 26, 88, 1.0},  {0, 1148, 21, 112, 1.0},  {0, 1148, 25, 90, 1.0},  {0, 1148, 27, 88, 1.0},  {0, 1149, 21, 113, 1.0},
  {0, 1149, 25, 91, 1.0},  {0, 1149, 28, 88, 1.0},  {0, 1150, 21, 114, 1.0},  {0, 1150, 26, 89, 1.0},  {0, 1151, 21, 115, 1.0},  {0, 1151, 26, 90, 1.0},
  {0, 1151, 27, 89, 1.0},  {0, 1152, 21, 116, 1.0},  {0, 1152, 26, 91, 1.0},  {0, 1152, 28, 89, 1.0},  {0, 1153, 21, 117, 1.0},  {0, 1153, 27, 90, 1.0},
  {0, 1154, 21, 118, 1.0},  {0, 1154, 27, 91, 1.0},  {0, 1154, 28, 90, 1.0},  {0, 1155, 21, 119, 1.0},  {0, 1155, 28, 91, 1.0},  {0, 1156, 22, 92, 1.0},
  {0, 1157, 22, 93, 1.0},  {0, 1157, 23, 92, 1.0},  {0, 1158, 22, 94, 1.0},  {0, 1158, 24, 92, 1.0},  {0, 1159, 22, 95, 1.0},  {0, 1159, 25, 92, 1.0},
  {0, 1160, 22, 96, 1.0},  {0, 1160, 26, 92, 1.0},  {0, 1161, 22, 97, 1.0},  {0, 1161, 27, 92, 1.0},  {0, 1162, 22, 98, 1.0},  {0, 1162, 28, 92, 1.0},
  {0, 1163, 22, 99, 1.0},  {0, 1163, 23, 93, 1.0},  {0, 1164, 22, 100, 1.0},  {0, 1164, 23, 94, 1.0},  {0, 1164, 24, 93, 1.0},  {0, 1165, 22, 101, 1.0},
  {0, 1165, 23, 95, 1.0},  {0, 1165, 25, 93, 1.0},  {0, 1166, 22, 102, 1.0},  {0, 1166, 23, 96, 1.0},  {0, 1166, 26, 93, 1.0},  {0, 1167, 22, 103, 1.0},
  {0, 1167, 23, 97, 1.0},  {0, 1167, 27, 93, 1.0},  {0, 1168, 22, 104, 1.0},  {0, 1168, 23, 98, 1.0},  {0, 1168, 28, 93, 1.0},  {0, 1169, 22, 105, 1.0},
  {0, 1169, 24, 94, 1.0},  {0, 1170, 22, 106, 1.0},  {0, 1170, 24, 95, 1.0},  {0, 1170, 25, 94, 1.0},  {0, 1171, 22, 107, 1.0},  {0, 1171, 24, 96, 1.0},
  {0, 1171, 26, 94, 1.0},  {0, 1172, 22, 108, 1.0},  {0, 1172, 24, 97, 1.0},  {0, 1172, 27, 94, 1.0},  {0, 1173, 22, 109, 1.0},  {0, 1173, 24, 98, 1.0},
  {0, 1173, 28, 94, 1.0},  {0, 1174, 22, 110, 1.0},  {0, 1174, 25, 95, 1.0},  {0, 1175, 22, 111, 1.0},  {0, 1175, 25, 96, 1.0},  {0, 1175, 26, 95, 1.0},
  {0, 1176, 22, 112, 1.0},  {0, 1176, 25, 97, 1.0},  {0, 1176, 27, 95, 1.0},  {0, 1177, 22, 113, 1.0},  {0, 1177, 25, 98, 1.0},  {0, 1177, 28, 95, 1.0},
  {0, 1178, 22, 114, 1.0},  {0, 1178, 26, 96, 1.0},  {0, 1179, 22, 115, 1.0},  {0, 1179, 26, 97, 1.0},  {0, 1179, 27, 96, 1.0},  {0, 1180, 22, 116, 1.0},
  {0, 1180, 26, 98, 1.0},  {0, 1180, 28, 96, 1.0},  {0, 1181, 22, 117, 1.0},  {0, 1181, 27, 97, 1.0},  {0, 1182, 22, 118, 1.0},  {0, 1182, 27, 98, 1.0},
  {0, 1182, 28, 97, 1.0},  {0, 1183, 22, 119, 1.0},  {0, 1183, 28, 98, 1.0},  {0, 1184, 23, 99, 1.0},  {0, 1185, 23, 100, 1.0},  {0, 1185, 24, 99, 1.0},
  {0, 1186, 23, 101, 1.0},  {0, 1186, 25, 99, 1.0},  {0, 1187, 23, 102, 1.0},  {0, 1187, 26, 99, 1.0},  {0, 1188, 23, 103, 1.0},  {0, 1188, 27, 99, 1.0},
  {0, 1189, 23, 104, 1.0},  {0, 1189, 28, 99, 1.0},  {0, 1190, 23, 105, 1.0},  {0, 1190, 24, 100, 1.0},  {0, 1191, 23, 106, 1.0},  {0, 1191, 24, 101, 1.0},
  {0, 1191, 25, 100, 1.0},  {0, 1192, 23, 107, 1.0},  {0, 1192, 24, 102, 1.0},  {0, 1192, 26, 100, 1.0},  {0, 1193, 23, 108, 1.0},  {0, 1193, 24, 103, 1.0},
  {0, 1193, 27, 100, 1.0},  {0, 1194, 23, 109, 1.0},  {0, 1194, 24, 104, 1.0},  {0, 1194, 28, 100, 1.0},  {0, 1195, 23, 110, 1.0},  {0, 1195, 25, 101, 1.0},
  {0, 1196, 23, 111, 1.0},  {0, 1196, 25, 102, 1.0},  {0, 1196, 26, 101, 1.0},  {0, 1197, 23, 112, 1.0},  {0, 1197, 25, 103, 1.0},  {0, 1197, 27, 101, 1.0},
  {0, 1198, 23, 113, 1.0},  {0, 1198, 25, 104, 1.0},  {0, 1198, 28, 101, 1.0},  {0, 1199, 23, 114, 1.0},  {0, 1199, 26, 102, 1.0},  {0, 1200, 23, 115, 1.0},
  {0, 1200, 26, 103, 1.0},  {0, 1200, 27, 102, 1.0},  {0, 1201, 23, 116, 1.0},  {0, 1201, 26, 104, 1.0},  {0, 1201, 28, 102, 1.0},  {0, 1202, 23, 117, 1.0},
  {0, 1202, 27, 103, 1.0},  {0, 1203, 23, 118, 1.0},  {0, 1203, 27, 104, 1.0},  {0, 1203, 28, 103, 1.0},  {0, 1204, 23, 119, 1.0},  {0, 1204, 28, 104, 1.0},
  {0, 1205, 24, 105, 1.0},  {0, 1206, 24, 106, 1.0},  {0, 1206, 25, 105, 1.0},  {0, 1207, 24, 107, 1.0},  {0, 1207, 26, 105, 1.0},  {0, 1208, 24, 108, 1.0},
  {0, 1208, 27, 105, 1.0},  {0, 1209, 24, 109, 1.0},  {0, 1209, 28, 105, 1.0},  {0, 1210, 24, 110, 1.0},  {0, 1210, 25, 106, 1.0},  {0, 1211, 24, 111, 1.0},
  {0, 1211, 25, 107, 1.0},  {0, 1211, 26, 106, 1.0},  {0, 1212, 24, 112, 1.0},  {0, 1212, 25, 108, 1.0},  {0, 1212, 27, 106, 1.0},  {0, 1213, 24, 113, 1.0},
  {0, 1213, 25, 109, 1.0},  {0, 1213, 28, 106, 1.0},  {0, 1214, 24, 114, 1.0},  {0, 1214, 26, 107, 1.0},  {0, 1215, 24, 115, 1.0},  {0, 1215, 26, 108, 1.0},
  {0, 1215, 27, 107, 1.0},  {0, 1216, 24, 116, 1.0},  {0, 1216, 26, 109, 1.0},  {0, 1216, 28, 107, 1.0},  {0, 1217, 24, 117, 1.0},  {0, 1217, 27, 108, 1.0},
  {0, 1218, 24, 118, 1.0},  {0, 1218, 27, 109, 1.0},  {0, 1218, 28, 108, 1.0},  {0, 1219, 24, 119, 1.0},  {0, 1219, 28, 109, 1.0},  {0, 1220, 25, 110, 1.0},
  {0, 1221, 25, 111, 1.0},  {0, 1221, 26, 110, 1.0},  {0, 1222, 25, 112, 1.0},  {0, 1222, 27, 110, 1.0},  {0, 1223, 25, 113, 1.0},  {0, 1223, 28, 110, 1.0},
  {0, 1224, 25, 114, 1.0},  {0, 1224, 26, 111, 1.0},  {0, 1225, 25, 115, 1.0},  {0, 1225, 26, 112, 1.0},  {0, 1225, 27, 111, 1.0},  {0, 1226, 25, 116, 1.0},
  {0, 1226, 26, 113, 1.0},  {0, 1226, 28, 111, 1.0},  {0, 1227, 25, 117, 1.0},  {0, 1227, 27, 112, 1.0},  {0, 1228, 25, 118, 1.0},  {0, 1228, 27, 113, 1.0},
  {0, 1228, 28, 112, 1.0},  {0, 1229, 25, 119, 1.0},  {0, 1229, 28, 113, 1.0},  {0, 1230, 26, 114, 1.0},  {0, 1231, 26, 115, 1.0},  {0, 1231, 27, 114, 1.0},
  {0, 1232, 26, 116, 1.0},  {0, 1232, 28, 114, 1.0},  {0, 1233, 26, 117, 1.0},  {0, 1233, 27, 115, 1.0},  {0, 1234, 26, 118, 1.0},  {0, 1234, 27, 116, 1.0},
  {0, 1234, 28, 115, 1.0},  {0, 1235, 26, 119, 1.0},  {0, 1235, 28, 116, 1.0},  {0, 1236, 27, 117, 1.0},  {0, 1237, 27, 118, 1.0},  {0, 1237, 28, 117, 1.0},
  {0, 1238, 27, 119, 1.0},  {0, 1238, 28, 118, 1.0},  {0, 1239, 28, 119, 1.0},  {0, 1240, 29, 29, 1.0},  {0, 1241, 29, 30, 1.0},  {0, 1242, 29, 31, 1.0},
  {0, 1243, 29, 32, 1.0},  {0, 1244, 29, 33, 1.0},  {0, 1245, 29, 34, 1.0},  {0, 1246, 29, 35, 1.0},  {0, 1247, 29, 36, 1.0},  {0, 1248, 29, 37, 1.0},
  {0, 1249, 29, 38, 1.0},  {0, 1250, 29, 39, 1.0},  {0, 1251, 29, 40, 1.0},  {0, 1252, 29, 41, 1.0},  {0, 1253, 29, 42, 1.0},  {0, 1253, 30, 30, 1.0},
  {0, 1254, 29, 43, 1.0},  {0, 1254, 30, 31, 1.0},  {0, 1255, 29, 44, 1.0},  {0, 1255, 30, 32, 1.0},  {0, 1256, 29, 45, 1.0},  {0, 1256, 30, 33, 1.0},
  {0, 1257, 29, 46, 1.0},  {0, 1257, 30, 34, 1.0},  {0, 1258, 29, 47, 1.0},  {0, 1258, 30, 35, 1.0},  {0, 1259, 29, 48, 1.0},  {0, 1259, 30, 36, 1.0},
  {0, 1260, 29, 49, 1.0},  {0, 1260, 30, 37, 1.0},  {0, 1261, 29, 50, 1.0},  {0, 1261, 30, 38, 1.0},  {0, 1262, 29, 51, 1.0},  {0, 1262, 30, 39, 1.0},
  {0, 1263, 29, 52, 1.0},  {0, 1263, 30, 40, 1.0},  {0, 1264, 29, 53, 1.0},  {0, 1264, 30, 41, 1.0},  {0, 1265, 29, 54, 1.0},  {0, 1265, 31, 31, 1.0},
  {0, 1266, 29, 55, 1.0},  {0, 1266, 31, 32, 1.0},  {0, 1267, 29, 56, 1.0},  {0, 1267, 31, 33, 1.0},  {0, 1268, 29, 57, 1.0},  {0, 1268, 31, 34, 1.0},
  {0, 1269, 29, 58, 1.0},  {0, 1269, 31, 35, 1.0},  {0, 1270, 29, 59, 1.0},  {0, 1270, 31, 36, 1.0},  {0, 1271, 29, 60, 1.0},  {0, 1271, 31, 37, 1.0},
  {0, 1272, 29, 61, 1.0},  {0, 1272, 31, 38, 1.0},  {0, 1273, 29, 62, 1.0},  {0, 1273, 31, 39, 1.0},  {0, 1274, 29, 63, 1.0},  {0, 1274, 31, 40, 1.0},
  {0, 1275, 29, 64, 1.0},  {0, 1275, 31, 41, 1.0},  {0, 1276, 29, 65, 1.0},  {0, 1276, 32, 32, 1.0},  {0, 1277, 29, 66, 1.0},  {0, 1277, 32, 33, 1.0},
  {0, 1278, 29, 67, 1.0},  {0, 1278, 32, 34, 1.0},  {0, 1279, 29, 68, 1.0},  {0, 1279, 32, 35, 1.0},  {0, 1280, 29, 69, 1.0},  {0, 1280, 32, 36, 1.0},
  {0, 1281, 29, 70, 1.0},  {0, 1281, 32, 37, 1.0},  {0, 1282, 29, 71, 1.0},  {0, 1282, 32, 38, 1.0},  {0, 1283, 29, 72, 1.0},  {0, 1283, 32, 39, 1.0},
  {0, 1284, 29, 73, 1.0},  {0, 1284, 32, 40, 1.0},  {0, 1285, 29, 74, 1.0},  {0, 1285, 32, 41, 1.0},  {0, 1286, 29, 75, 1.0},  {0, 1286, 33, 33, 1.0},
  {0, 1287, 29, 76, 1.0},  {0, 1287, 33, 34, 1.0},  {0, 1288, 29, 77, 1.0},  {0, 1288, 33, 35, 1.0},  {0, 1289, 29, 78, 1.0},  {0, 1289, 33, 36, 1.0},
  {0, 1290, 29, 79, 1.0},  {0, 1290, 33, 37, 1.0},  {0, 1291, 29, 80, 1.0},  {0, 1291, 33, 38, 1.0},  {0, 1292, 29, 81, 1.0},  {0, 1292, 33, 39, 1.0},
  {0, 1293, 29, 82, 1.0},  {0, 1293, 33, 40, 1.0},  {0, 1294, 29, 83, 1.0},  {0, 1294, 33, 41, 1.0},  {0, 1295, 29, 84, 1.0},  {0, 1295, 34, 34, 1.0},
  {0, 1296, 29, 85, 1.0},  {0, 1296, 34, 35, 1.0},  {0, 1297, 29, 86, 1.0},  {0, 1297, 34, 36, 1.0},  {0, 1298, 29, 87, 1.0},  {0, 1298, 34, 37, 1.0},
  {0, 1299, 29, 88, 1.0},  {0, 1299, 34, 38, 1.0},  {0, 1300, 29, 89, 1.0},  {0, 1300, 34, 39, 1.0},  {0, 1301, 29, 90, 1.0},  {0, 1301, 34, 40, 1.0},
  {0, 1302, 29, 91, 1.0},  {0, 1302, 34, 41, 1.0},  {0, 1303, 29, 92, 1.0},  {0, 1303, 35, 35, 1.0},  {0, 1304, 29, 93, 1.0},  {0, 1304, 35, 36, 1.0},
  {0, 1305, 29, 94, 1.0},  {0, 1305, 35, 37, 1.0},  {0, 1306, 29, 95, 1.0},  {0, 1306, 35, 38, 1.0},  {0, 1307, 29, 96, 1.0},  {0, 1307, 35, 39, 1.0},
  {0, 1308, 29, 97, 1.0},  {0, 1308, 35, 40, 1.0},  {0, 1309, 29, 98, 1.0},  {0, 1309, 35, 41, 1.0},  {0, 1310, 29, 99, 1.0},  {0, 1310, 36, 36, 1.0},
  {0, 1311, 29, 100, 1.0},  {0, 1311, 36, 37, 1.0},  {0, 1312, 29, 101, 1.0},  {0, 1312, 36, 38, 1.0},  {0, 1313, 29, 102, 1.0},  {0, 1313, 36, 39, 1.0},
  {0, 1314, 29, 103, 1.0},  {0, 1314, 36, 40, 1.0},  {0, 1315, 29, 104, 1.0},  {0, 1315, 36, 41, 1.0},  {0, 1316, 29, 105, 1.0},  {0, 1316, 37, 37, 1.0},
  {0, 1317, 29, 106, 1.0},  {0, 1317, 37, 38, 1.0},  {0, 1318, 29, 107, 1.0},  {0, 1318, 37, 39, 1.0},  {0, 1319, 29, 108, 1.0},  {0, 1319, 37, 40, 1.0},
  {0, 1320, 29, 109, 1.0},  {0, 1320, 37, 41, 1.0},  {0, 1321, 29, 110, 1.0},  {0, 1321, 38, 38, 1.0},  {0, 1322, 29, 111, 1.0},  {0, 1322, 38, 39, 1.0},
  {0, 1323, 29, 112, 1.0},  {0, 1323, 38, 40, 1.0},  {0, 1324, 29, 113, 1.0},  {0, 1324, 38, 41, 1.0},  {0, 1325, 29, 114, 1.0},  {0, 1325, 39, 39, 1.0},
  {0, 1326, 29, 115, 1.0},  {0, 1326, 39, 40, 1.0},  {0, 1327, 29, 116, 1.0},  {0, 1327, 39, 41, 1.0},  {0, 1328, 29, 117, 1.0},  {0, 1328, 40, 40, 1.0},
  {0, 1329, 29, 118, 1.0},  {0, 1329, 40, 41, 1.0},  {0, 1330, 29, 119, 1.0},  {0, 1330, 41, 41, 1.0},  {0, 1331, 30, 42, 1.0},  {0, 1332, 30, 43, 1.0},
  {0, 1332, 31, 42, 1.0},  {0, 1333, 30, 44, 1.0},  {0, 1333, 32, 42, 1.0},  {0, 1334, 30, 45, 1.0},  {0, 1334, 33, 42, 1.0},  {0, 1335, 30, 46, 1.0},
  {0, 1335, 34, 42, 1.0},  {0, 1336, 30, 47, 1.0},  {0, 1336, 35, 42, 1.0},  {0, 1337, 30, 48, 1.0},  {0, 1337, 36, 42, 1.0},  {0, 1338, 30, 49, 1.0},
  {0, 1338, 37, 42, 1.0},  {0, 1339, 30, 50, 1.0},  {0, 1339, 38, 42, 1.0},  {0, 1340, 30, 51, 1.0},  {0, 1340, 39, 42, 1.0},  {0, 1341, 30, 52, 1.0},
  {0, 1341, 40, 42, 1.0},  {0, 1342, 30, 53, 1.0},  {0, 1342, 41, 42, 1.0},  {0, 1343, 30, 54, 1.0},  {0, 1343, 31, 43, 1.0},  {0, 1344, 30, 55, 1.0},
  {0, 1344, 31, 44, 1.0},  {0, 1344, 32, 43, 1.0},  {0, 1345, 30, 56, 1.0},  {0, 1345, 31, 45, 1.0},  {0, 1345, 33, 43, 1.0},  {0, 1346, 30, 57, 1.0},
  {0, 1346, 31, 46, 1.0},  {0, 1346, 34, 43, 1.0},  {0, 1347, 30, 58, 1.0},  {0, 1347, 31, 47, 1.0},  {0, 1347, 35, 43, 1.0},  {0, 1348, 30, 59, 1.0},
  {0, 1348, 31, 48, 1.0},  {0, 1348, 36, 43, 1.0},  {0, 1349, 30, 60, 1.0},  {0, 1349, 31, 49, 1.0},  {0, 1349, 37, 43, 1.0},  {0, 1350, 30, 61, 1.0},
  {0, 1350, 31, 50, 1.0},  {0, 1350, 38, 43, 1.0},  {0, 1351, 30, 62, 1.0},  {0, 1351, 31, 51, 1.0},  {0, 1351, 39, 43, 1.0},  {0, 1352, 30, 63, 1.0},
  {0, 1352, 31, 52, 1.0},  {0, 1352, 40, 43, 1.0},  {0, 1353, 30, 64, 1.0},  {0, 1353, 31, 53, 1.0},  {0, 1353, 41, 43, 1.0},  {0, 1354, 30, 65, 1.0},
  {0, 1354, 32, 44, 1.0},  {0, 1355, 30, 66, 1.0},  {0, 1355, 32, 45, 1.0},  {0, 1355, 33, 44, 1.0},  {0, 1356, 30, 67, 1.0},  {0, 1356, 32, 46, 1.0},
  {0, 1356, 34, 44, 1.0},  {0, 1357, 30, 68, 1.0},  {0, 1357, 32, 47, 1.0},  {0, 1357, 35, 44, 1.0},  {0, 1358, 30, 69, 1.0},  {0, 1358, 32, 48, 1.0},
  {0, 1358, 36, 44, 1.0},  {0, 1359, 30, 70, 1.0},  {0, 1359, 32, 49, 1.0},  {0, 1359, 37, 44, 1.0},  {0, 1360, 30, 71, 1.0},  {0, 1360, 32, 50, 1.0},
  {0, 1360, 38, 44, 1.0},  {0, 1361, 30, 72, 1.0},  {0, 1361, 32, 51, 1.0},  {0, 1361, 39, 44, 1.0},  {0, 1362, 30, 73, 1.0},  {0, 1362, 32, 52, 1.0},
  {0, 1362, 40, 44, 1.0},  {0, 1363, 30, 74, 1.0},  {0, 1363, 32, 53, 1.0},  {0, 1363, 41, 44, 1.0},  {0, 1364, 30, 75, 1.0},  {0, 1364, 33, 45, 1.0},
  {0, 1365, 30, 76, 1.0},  {0, 1365, 33, 46, 1.0},  {0, 1365, 34, 45, 1.0},  {0, 1366, 30, 77, 1.0},  {0, 1366, 33, 47, 1.0},  {0, 1366, 35, 45, 1.0},
  {0, 1367, 30, 78, 1.0},  {0, 1367, 33, 48, 1.0},  {0, 1367, 36, 45, 1.0},  {0, 1368, 30, 79, 1.0},  {0, 1368, 33, 49, 1.0},  {0, 1368, 37, 45, 1.0},
  {0, 1369, 30, 80, 1.0},  {0, 1369, 33, 50, 1.0},  {0, 1369, 38, 45, 1.0},  {0, 1370, 30, 81, 1.0},  {0, 1370, 33, 51, 1.0},  {0, 1370, 39, 45, 1.0},
  {0, 1371, 30, 82, 1.0},  {0, 1371, 33, 52, 1.0},  {0, 1371, 40, 45, 1.0},  {0, 1372, 30, 83, 1.0},  {0, 1372, 33, 53, 1.0},  {0, 1372, 41, 45, 1.0},
  {0, 1373, 30, 84, 1.0},  {0, 1373, 34, 46, 1.0},  {0, 1374, 30, 85, 1.0},  {0, 1374, 34, 47, 1.0},  {0, 1374, 35, 46, 1.0},  {0, 1375, 30, 86, 1.0},
  {0, 1375, 34, 48, 1.0},  {0, 1375, 36, 46, 1.0},  {0, 1376, 30, 87, 1.0},  {0, 1376, 34, 49, 1.0},  {0, 1376, 37, 46, 1.0},  {0, 1377, 30, 88, 1.0},
  {0, 1377, 34, 50, 1.0},  {0, 1377, 38, 46, 1.0},  {0, 1378, 30, 89, 1.0},  {0, 1378, 34, 51, 1.0},  {0, 1378, 39, 46, 1.0},  {0, 1379, 30, 90, 1.0},
  {0, 1379, 34, 52, 1.0},  {0, 1379, 40, 46, 1.0},  {0, 1380, 30, 91, 1.0},  {0, 1380, 34, 53, 1.0},  {0, 1380, 41, 46, 1.0},  {0, 1381, 30, 92, 1.0},
  {0, 1381, 35, 47, 1.0},  {0, 1382, 30, 93, 1.0},  {0, 1382, 35, 48, 1.0},  {0, 1382, 36, 47, 1.0},  {0, 1383, 30, 94, 1.0},  {0, 1383, 35, 49, 1.0},
  {0, 1383, 37, 47, 1.0},  {0, 1384, 30, 95, 1.0},  {0, 1384, 35, 50, 1.0},  {0, 1384, 38, 47, 1.0},  {0, 1385, 30, 96, 1.0},  {0, 1385, 35, 51, 1.0},
  {0, 1385, 39, 47, 1.0},  {0, 1386, 30, 97, 1.0},  {0, 1386, 35, 52, 1.0},  {0, 1386, 40, 47, 1.0},  {0, 1387, 30, 98, 1.0},  {0, 1387, 35, 53, 1.0},
  {0, 1387, 41, 47, 1.0},  {0, 1388, 30, 99, 1.0},  {0, 1388, 36, 48, 1.0},  {0, 1389, 30, 100, 1.0},  {0, 1389, 36, 49, 1.0},  {0, 1389, 37, 48, 1.0},
  {0, 1390, 30, 101, 1.0},  {0, 1390, 36, 50, 1.0},  {0, 1390, 38, 48, 1.0},  {0, 1391, 30, 102, 1.0},  {0, 1391, 36, 51, 1.0},  {0, 1391, 39, 48, 1.0},
  {0, 1392, 30, 103, 1.0},  {0, 1392, 36, 52, 1.0},  {0, 1392, 40, 48, 1.0},  {0, 1393, 30, 104, 1.0},  {0, 1393, 36, 53, 1.0},  {0, 1393, 41, 48, 1.0},
  {0, 1394, 30, 105, 1.0},  {0, 1394, 37, 49, 1.0},  {0, 1395, 30, 106, 1.0},  {0, 1395, 37, 50, 1.0},  {0, 1395, 38, 49, 1.0},  {0, 1396, 30, 107, 1.0},
  {0, 1396, 37, 51, 1.0},  {0, 1396, 39, 49, 1.0},  {0, 1397, 30, 108, 1.0},  {0, 1397, 37, 52, 1.0},  {0, 1397, 40, 49, 1.0},  {0, 1398, 30, 109, 1.0},
  {0, 1398, 37, 53, 1.0},  {0, 1398, 41, 49, 1.0},  {0, 1399, 30, 110, 1.0},  {0, 1399, 38, 50, 1.0},  {0, 1400, 30, 111, 1.0},  {0, 1400, 38, 51, 1.0},
  {0, 1400, 39, 50, 1.0},  {0, 1401, 30, 112, 1.0},  {0, 1401, 38, 52, 1.0},  {0, 1401, 40, 50, 1.0},  {0, 1402, 30, 113, 1.0},  {0, 1402, 38, 53, 1.0},
  {0, 1402, 41, 50, 1.0},  {0, 1403, 30, 114, 1.0},  {0, 1403, 39, 51, 1.0},  {0, 1404, 30, 115, 1.0},  {0, 1404, 39, 52, 1.0},  {0, 1404, 40, 51, 1.0},
  {0, 1405, 30, 116, 1.0},  {0, 1405, 39, 53, 1.0},  {0, 1405, 41, 51, 1.0},  {0, 1406, 30, 117, 1.0},  {0, 1406, 40, 52, 1.0},  {0, 1407, 30, 118, 1.0},
  {0, 1407, 40, 53, 1.0},  {0, 1407, 41, 52, 1.0},  {0, 1408, 30, 119, 1.0},  {0, 1408, 41, 53, 1.0},  {0, 1409, 31, 54, 1.0},  {0, 1410, 31, 55, 1.0},
  {0, 1410, 32, 54, 1.0},  {0, 1411, 31, 56, 1.0},  {0, 1411, 33, 54, 1.0},  {0, 1412, 31, 57, 1.0},  {0, 1412, 34, 54, 1.0},  {0, 1413, 31, 58, 1.0},
  {0, 1413, 35, 54, 1.0},  {0, 1414, 31, 59, 1.0},  {0, 1414, 36, 54, 1.0},  {0, 1415, 31, 60, 1.0},  {0, 1415, 37, 54, 1.0},  {0, 1416, 31, 61, 1.0},
  {0, 1416, 38, 54, 1.0},  {0, 1417, 31, 62, 1.0},  {0, 1417, 39, 54, 1.0},  {0, 1418, 31, 63, 1.0},  {0, 1418, 40, 54, 1.0},  {0, 1419, 31, 64, 1.0},
  {0, 1419, 41, 54, 1.0},  {0, 1420, 31, 65, 1.0},  {0, 1420, 32, 55, 1.0},  {0, 1421, 31, 66, 1.0},  {0, 1421, 32, 56, 1.0},  {0, 1421, 33, 55, 1.0},
  {0, 1422, 31, 67, 1.0},  {0, 1422, 32, 57, 1.0},  {0, 1422, 34, 55, 1.0},  {0, 1423, 31, 68, 1.0},  {0, 1423, 32, 58, 1.0},  {0, 1423, 35, 55, 1.0},
  {0, 1424, 31, 69, 1.0},  {0, 1424, 32, 59, 1.0},  {0, 1424, 36, 55, 1.0},  {0, 1425, 31, 70, 1.0},  {0, 1425, 32, 60, 1.0},  {0, 1425, 37, 55, 1.0},
  {0, 1426, 31, 71, 1.0},  {0, 1426, 32, 61, 1.0},  {0, 1426, 38, 55, 1.0},  {0, 1427, 31, 72, 1.0},  {0, 1427, 32, 62, 1.0},  {0, 1427, 39, 55, 1.0},
  {0, 1428, 31, 73, 1.0},  {0, 1428, 32, 63, 1.0},  {0, 1428, 40, 55, 1.0},  {0, 1429, 31, 74, 1.0},  {0, 1429, 32, 64, 1.0},  {0, 1429, 41, 55, 1.0},
  {0, 1430, 31, 75, 1.0},  {0, 1430, 33, 56, 1.0},  {0, 1431, 31, 76, 1.0},  {0, 1431, 33, 57, 1.0},  {0, 1431, 34, 56, 1.0},  {0, 1432, 31, 77, 1.0},
  {0, 1432, 33, 58, 1.0},  {0, 1432, 35, 56, 1.0},  {0, 1433, 31, 78, 1.0},  {0, 1433, 33, 59, 1.0},  {0, 1433, 36, 56, 1.0},  {0, 1434, 31, 79, 1.0},
  {0, 1434, 33, 60, 1.0},  {0, 1434, 37, 56, 1.0},  {0, 1435, 31, 80, 1.0},  {0, 1435, 33, 61, 1.0},  {0, 1435, 38, 56, 1.0},  {0, 1436, 31, 81, 1.0},
  {0, 1436, 33, 62, 1.0},  {0, 1436, 39, 56, 1.0},  {0, 1437, 31, 82, 1.0},  {0, 1437, 33, 63, 1.0},  {0, 1437, 40, 56, 1.0},  {0, 1438, 31, 83, 1.0},
  {0, 1438, 33, 64, 1.0},  {0, 1438, 41, 56, 1.0},  {0, 1439, 31, 84, 1.0},  {0, 1439, 34, 57, 1.0},  {0, 1440, 31, 85, 1.0},  {0, 1440, 34, 58, 1.0},
  {0, 1440, 35, 57, 1.0},  {0, 1441, 31, 86, 1.0},  {0, 1441, 34, 59, 1.0},  {0, 1441, 36, 57, 1.0},  {0, 1442, 31, 87, 1.0},  {0, 1442, 34, 60, 1.0},
  {0, 1442, 37, 57, 1.0},  {0, 1443, 31, 88, 1.0},  {0, 1443, 34, 61, 1.0},  {0, 1443, 38, 57, 1.0},  {0, 1444, 31, 89, 1.0},  {0, 1444, 34, 62, 1.0},
  {0, 1444, 39, 57, 1.0},  {0, 1445, 31, 90, 1.0},  {0, 1445, 34, 63, 1.0},  {0, 1445, 40, 57, 1.0},  {0, 1446, 31, 91, 1.0},  {0, 1446, 34, 64, 1.0},
  {0, 1446, 41, 57, 1.0},  {0, 1447, 31, 92, 1.0},  {0, 1447, 35, 58, 1.0},  {0, 1448, 31, 93, 1.0},  {0, 1448, 35, 59, 1.0},  {0, 1448, 36, 58, 1.0},
  {0, 1449, 31, 94, 1.0},  {0, 1449, 35, 60, 1.0},  {0, 1449, 37, 58, 1.0},  {0, 1450, 31, 95, 1.0},  {0, 1450, 35, 61, 1.0},  {0, 1450, 38, 58, 1.0},
  {0, 1451, 31, 96, 1.0},  {0, 1451, 35, 62, 1.0},  {0, 1451, 39, 58, 1.0},  {0, 1452, 31, 97, 1.0},  {0, 1452, 35, 63, 1.0},  {0, 1452, 40, 58, 1.0},
  {0, 1453, 31, 98, 1.0},  {0, 1453, 35, 64, 1.0},  {0, 1453, 41, 58, 1.0},  {0, 1454, 31, 99, 1.0},  {0, 1454, 36, 59, 1.0},  {0, 1455, 31, 100, 1.0},
  {0, 1455, 36, 60, 1.0},  {0, 1455, 37, 59, 1.0},  {0, 1456, 31, 101, 1.0},  {0, 1456, 36, 61, 1.0},  {0, 1456, 38, 59, 1.0},  {0, 1457, 31, 102, 1.0},
  {0, 1457, 36, 62, 1.0},  {0, 1457, 39, 59, 1.0},  {0, 1458, 31, 103, 1.0},  {0, 1458, 36, 63, 1.0},  {0, 1458, 40, 59, 1.0},  {0, 1459, 31, 104, 1.0},
  {0, 1459, 36, 64, 1.0},  {0, 1459, 41, 59, 1.0},  {0, 1460, 31, 105, 1.0},  {0, 1460, 37, 60, 1.0},  {0, 1461, 31, 106, 1.0},  {0, 1461, 37, 61, 1.0},
  {0, 1461, 38, 60, 1.0},  {0, 1462, 31, 107, 1.0},  {0, 1462, 37, 62, 1.0},  {0, 1462, 39, 60, 1.0},  {0, 1463, 31, 108, 1.0},  {0, 1463, 37, 63, 1.0},
  {0, 1463, 40, 60, 1.0},  {0, 1464, 31, 109, 1.0},  {0, 1464, 37, 64, 1.0},  {0, 1464, 41, 60, 1.0},  {0, 1465, 31, 110, 1.0},  {0, 1465, 38, 61, 1.0},
  {0, 1466, 31, 111, 1.0},  {0, 1466, 38, 62, 1.0},  {0, 1466, 39, 61, 1.0},  {0, 1467, 31, 112, 1.0},  {0, 1467, 38, 63, 1.0},  {0, 1467, 40, 61, 1.0},
  {0, 1468, 31, 113, 1.0},  {0, 1468, 38, 64, 1.0},  {0, 1468, 41, 61, 1.0},  {0, 1469, 31, 114, 1.0},  {0, 1469, 39, 62, 1.0},  {0, 1470, 31, 115, 1.0},
  {0, 1470, 39, 63, 1.0},  {0, 1470, 40, 62, 1.0},  {0, 1471, 31, 116, 1.0},  {0, 1471, 39, 64, 1.0},  {0, 1471, 41, 62, 1.0},  {0, 1472, 31, 117, 1.0},
  {0, 1472, 40, 63, 1.0},  {0, 1473, 31, 118, 1.0},  {0, 1473, 40, 64, 1.0},  {0, 1473, 41, 63, 1.0},  {0, 1474, 31, 119, 1.0},  {0, 1474, 41, 64, 1.0},
  {0, 1475, 32, 65, 1.0},  {0, 1476, 32, 66, 1.0},  {0, 1476, 33, 65, 1.0},  {0, 1477, 32, 67, 1.0},  {0, 1477, 34, 65, 1.0},  {0, 1478, 32, 68, 1.0},
  {0, 1478, 35, 65, 1.0},  {0, 1479, 32, 69, 1.0},  {0, 1479, 36, 65, 1.0},  {0, 1480, 32, 70, 1.0},  {0, 1480, 37, 65, 1.0},  {0, 1481, 32, 71, 1.0},
  {0, 1481, 38, 65, 1.0},  {0, 1482, 32, 72, 1.0},  {0, 1482, 39, 65, 1.0},  {0, 1483, 32, 73, 1.0},  {0, 1483, 40, 65, 1.0},  {0, 1484, 32, 74, 1.0},
  {0, 1484, 41, 65, 1.0},  {0, 1485, 32, 75, 1.0},  {0, 1485, 33, 66, 1.0},  {0, 1486, 32, 76, 1.0},  {0, 1486, 33, 67, 1.0},  {0, 1486, 34, 66, 1.0},
  {0, 1487, 32, 77, 1.0},  {0, 1487, 33, 68, 1.0},  {0, 1487, 35, 66, 1.0},  {0, 1488, 32, 78, 1.0},  {0, 1488, 33, 69, 1.0},  {0, 1488, 36, 66, 1.0},
  {0, 1489, 32, 79, 1.0},  {0, 1489, 33, 70, 1.0},  {0, 1489, 37, 66, 1.0},  {0, 1490, 32, 80, 1.0},  {0, 1490, 33, 71, 1.0},  {0, 1490, 38, 66, 1.0},
  {0, 1491, 32, 81, 1.0},  {0, 1491, 33, 72, 1.0},  {0, 1491, 39, 66, 1.0},  {0, 1492, 32, 82, 1.0},  {0, 1492, 33, 73, 1.0},  {0, 1492, 40, 66, 1.0},
  {0, 1493, 32, 83, 1.0},  {0, 1493, 33, 74, 1.0},  {0, 1493, 41, 66, 1.0},  {0, 1494, 32, 84, 1.0},  {0, 1494, 34, 67, 1.0},  {0, 1495, 32, 85, 1.0},
  {0, 1495, 34, 68, 1.0},  {0, 1495, 35, 67, 1.0},  {0, 1496, 32, 86, 1.0},  {0, 1496, 34, 69, 1.0},  {0, 1496, 36, 67, 1.0},  {0, 1497, 32, 87, 1.0},
  {0, 1497, 34, 70, 1.0},  {0, 1497, 37, 67, 1.0},  {0, 1498, 32, 88, 1.0},  {0, 1498, 34, 71, 1.0},  {0, 1498, 38, 67, 1.0},  {0, 1499, 32, 89, 1.0},
  {0, 1499, 34, 72, 1.0},  {0, 1499, 39, 67, 1.0},  {0, 1500, 32, 90, 1.0},  {0, 1500, 34, 73, 1.0},  {0, 1500, 40, 67, 1.0},  {0, 1501, 32, 91, 1.0},
  {0, 1501, 34, 74, 1.0},  {0, 1501, 41, 67, 1.0},  {0, 1502, 32, 92, 1.0},  {0, 1502, 35, 68, 1.0},  {0, 1503, 32, 93, 1.0},  {0, 1503, 35, 69, 1.0},
  {0, 1503, 36, 68, 1.0},  {0, 1504, 32, 94, 1.0},  {0, 1504, 35, 70, 1.0},  {0, 1504, 37, 68, 1.0},  {0, 1505, 32, 95, 1.0},  {0, 1505, 35, 71, 1.0},
  {0, 1505, 38, 68, 1.0},  {0, 1506, 32, 96, 1.0},  {0, 1506, 35, 72, 1.0},  {0, 1506, 39, 68, 1.0},  {0, 1507, 32, 97, 1.0},  {0, 1507, 35, 73, 1.0},
  {0, 1507, 40, 68, 1.0},  {0, 1508, 32, 98, 1.0},  {0, 1508, 35, 74, 1.0},  {0, 1508, 41, 68, 1.0},  {0, 1509, 32, 99, 1.0},  {0, 1509, 36, 69, 1.0},
  {0, 1510, 32, 100, 1.0},  {0, 1510, 36, 70, 1.0},  {0, 1510, 37, 69, 1.0},  {0, 1511, 32, 101, 1.0},  {0, 1511, 36, 71, 1.0},  {0, 1511, 38, 69, 1.0},
  {0, 1512, 32, 102, 1.0},  {0, 1512, 36, 72, 1.0},  {0, 1512, 39, 69, 1.0},  {0, 1513, 32, 103, 1.0},  {0, 1513, 36, 73, 1.0},  {0, 1513, 40, 69, 1.0},
  {0, 1514, 32, 104, 1.0},  {0, 1514, 36, 74, 1.0},  {0, 1514, 41, 69, 1.0},  {0, 1515, 32, 105, 1.0},  {0, 1515, 37, 70, 1.0},  {0, 1516, 32, 106, 1.0},
  {0, 1516, 37, 71, 1.0},  {0, 1516, 38, 70, 1.0},  {0, 1517, 32, 107, 1.0},  {0, 1517, 37, 72, 1.0},  {0, 1517, 39, 70, 1.0},  {0, 1518, 32, 108, 1.0},
  {0, 1518, 37, 73, 1.0},  {0, 1518, 40, 70, 1.0},  {0, 1519, 32, 109, 1.0},  {0, 1519, 37, 74, 1.0},  {0, 1519, 41, 70, 1.0},  {0, 1520, 32, 110, 1.0},
  {0, 1520, 38, 71, 1.0},  {0, 1521, 32, 111, 1.0},  {0, 1521, 38, 72, 1.0},  {0, 1521, 39, 71, 1.0},  {0, 1522, 32, 112, 1.0},  {0, 1522, 38, 73, 1.0},
  {0, 1522, 40, 71, 1.0},  {0, 1523, 32, 113, 1.0},  {0, 1523, 38, 74, 1.0},  {0, 1523, 41, 71, 1.0},  {0, 1524, 32, 114, 1.0},  {0, 1524, 39, 72, 1.0},
  {0, 1525, 32, 115, 1.0},  {0, 1525, 39, 73, 1.0},  {0, 1525, 40, 72, 1.0},  {0, 1526, 32, 116, 1.0},  {0, 1526, 39, 74, 1.0},  {0, 1526, 41, 72, 1.0},
  {0, 1527, 32, 117, 1.0},  {0, 1527, 40, 73, 1.0},  {0, 1528, 32, 118, 1.0},  {0, 1528, 40, 74, 1.0},  {0, 1528, 41, 73, 1.0},  {0, 1529, 32, 119, 1.0},
  {0, 1529, 41, 74, 1.0},  {0, 1530, 33, 75, 1.0},  {0, 1531, 33, 76, 1.0},  {0, 1531, 34, 75, 1.0},  {0, 1532, 33, 77, 1.0},  {0, 1532, 35, 75, 1.0},
  {0, 1533, 33, 78, 1.0},  {0, 1533, 36, 75, 1.0},  {0, 1534, 33, 79, 1.0},  {0, 1534, 37, 75, 1.0},  {0, 1535, 33, 80, 1.0},  {0, 1535, 38, 75, 1.0},
  {0, 1536, 33, 81, 1.0},  {0, 1536, 39, 75, 1.0},  {0, 1537, 33, 82, 1.0},  {0, 1537, 40, 75, 1.0},  {0, 1538, 33, 83, 1.0},  {0, 1538, 41, 75, 1.0},
  {0, 1539, 33, 84, 1.0},  {0, 1539, 34, 76, 1.0},  {0, 1540, 33, 85, 1.0},  {0, 1540, 34, 77, 1.0},  {0, 1540, 35, 76, 1.0},  {0, 1541, 33, 86, 1.0},
  {0, 1541, 34, 78, 1.0},  {0, 1541, 36, 76, 1.0},  {0, 1542, 33, 87, 1.0},  {0, 1542, 34, 79, 1.0},  {0, 1542, 37, 76, 1.0},  {0, 1543, 33, 88, 1.0},
  {0, 1543, 34, 80, 1.0},  {0, 1543, 38, 76, 1.0},  {0, 1544, 33, 89, 1.0},  {0, 1544, 34, 81, 1.0},  {0, 1544, 39, 76, 1.0},  {0, 1545, 33, 90, 1.0},
  {0, 1545, 34, 82, 1.0},  {0, 1545, 40, 76, 1.0},  {0, 1546, 33, 91, 1.0},  {0, 1546, 34, 83, 1.0},  {0, 1546, 41, 76, 1.0},  {0, 1547, 33, 92, 1.0},
  {0, 1547, 35, 77, 1.0},  {0, 1548, 33, 93, 1.0},  {0, 1548, 35, 78, 1.0},  {0, 1548, 36, 77, 1.0},  {0, 1549, 33, 94, 1.0},  {0, 1549, 35, 79, 1.0},
  {0, 1549, 37, 77, 1.0},  {0, 1550, 33, 95, 1.0},  {0, 1550, 35, 80, 1.0},  {0, 1550, 38, 77, 1.0},  {0, 1551, 33, 96, 1.0},  {0, 1551, 35, 81, 1.0},
  {0, 1551, 39, 77, 1.0},  {0, 1552, 33, 97, 1.0},  {0, 1552, 35, 82, 1.0},  {0, 1552, 40, 77, 1.0},  {0, 1553, 33, 98, 1.0},  {0, 1553, 35, 83, 1.0},
  {0, 1553, 41, 77, 1.0},  {0, 1554, 33, 99, 1.0},  {0, 1554, 36, 78, 1.0},  {0, 1555, 33, 100, 1.0},  {0, 1555, 36, 79, 1.0},  {0, 1555, 37, 78, 1.0},
  {0, 1556, 33, 101, 1.0},  {0, 1556, 36, 80, 1.0},  {0, 1556, 38, 78, 1.0},  {0, 1557, 33, 102, 1.0},  {0, 1557, 36, 81, 1.0},  {0, 1557, 39, 78, 1.0},
  {0, 1558, 33, 103, 1.0},  {0, 1558, 36, 82, 1.0},  {0, 1558, 40, 78, 1.0},  {0, 1559, 33, 104, 1.0},  {0, 1559, 36, 83, 1.0},  {0, 1559, 41, 78, 1.0},
  {0, 1560, 33, 105, 1.0},  {0, 1560, 37, 79, 1.0},  {0, 1561, 33, 106, 1.0},  {0, 1561, 37, 80, 1.0},  {0, 1561, 38, 79, 1.0},  {0, 1562, 33, 107, 1.0},
  {0, 1562, 37, 81, 1.0},  {0, 1562, 39, 79, 1.0},  {0, 1563, 33, 108, 1.0},  {0, 1563, 37, 82, 1.0},  {0, 1563, 40, 79, 1.0},  {0, 1564, 33, 109, 1.0},
  {0, 1564, 37, 83, 1.0},  {0, 1564, 41, 79, 1.0},  {0, 1565, 33, 110, 1.0},  {0, 1565, 38, 80, 1.0},  {0, 1566, 33, 111, 1.0},  {0, 1566, 38, 81, 1.0},
  {0, 1566, 39, 80, 1.0},  {0, 1567, 33, 112, 1.0},  {0, 1567, 38, 82, 1.0},  {0, 1567, 40, 80, 1.0},  {0, 1568, 33, 113, 1.0},  {0, 1568, 38, 83, 1.0},
  {0, 1568, 41, 80, 1.0},  {0, 1569, 33, 114, 1.0},  {0, 1569, 39, 81, 1.0},  {0, 1570, 33, 115, 1.0},  {0, 1570, 39, 82, 1.0},  {0, 1570, 40, 81, 1.0},
  {0, 1571, 33, 116, 1.0},  {0, 1571, 39, 83, 1.0},  {0, 1571, 41, 81, 1.0},  {0, 1572, 33, 117, 1.0},  {0, 1572, 40, 82, 1.0},  {0, 1573, 33, 118, 1.0},
  {0, 1573, 40, 83, 1.0},  {0, 1573, 41, 82, 1.0},  {0, 1574, 33, 119, 1.0},  {0, 1574, 41, 83, 1.0},  {0, 1575, 34, 84, 1.0},  {0, 1576, 34, 85, 1.0},
  {0, 1576, 35, 84, 1.0},  {0, 1577, 34, 86, 1.0},  {0, 1577, 36, 84, 1.0},  {0, 1578, 34, 87, 1.0},  {0, 1578, 37, 84, 1.0},  {0, 1579, 34, 88, 1.0},
  {0, 1579, 38, 84, 1.0},  {0, 1580, 34, 89, 1.0},  {0, 1580, 39, 84, 1.0},  {0, 1581, 34, 90, 1.0},  {0, 1581, 40, 84, 1.0},  {0, 1582, 34, 91, 1.0},
  {0, 1582, 41, 84, 1.0},  {0, 1583, 34, 92, 1.0},  {0, 1583, 35, 85, 1.0},  {0, 1584, 34, 93, 1.0},  {0, 1584, 35, 86, 1.0},  {0, 1584, 36, 85, 1.0},
  {0, 1585, 34, 94, 1.0},  {0, 1585, 35, 87, 1.0},  {0, 1585, 37, 85, 1.0},  {0, 1586, 34, 95, 1.0},  {0, 1586, 35, 88, 1.0},  {0, 1586, 38, 85, 1.0},
  {0, 1587, 34, 96, 1.0},  {0, 1587, 35, 89, 1.0},  {0, 1587, 39, 85, 1.0},  {0, 1588, 34, 97, 1.0},  {0, 1588, 35, 90, 1.0},  {0, 1588, 40, 85, 1.0},
  {0, 1589, 34, 98, 1.0},  {0, 1589, 35, 91, 1.0},  {0, 1589, 41, 85, 1.0},  {0, 1590, 34, 99, 1.0},  {0, 1590, 36, 86, 1.0},  {0, 1591, 34, 100, 1.0},
  {0, 1591, 36, 87, 1.0},  {0, 1591, 37, 86, 1.0},  {0, 1592, 34, 101, 1.0},  {0, 1592, 36, 88, 1.0},  {0, 1592, 38, 86, 1.0},  {0, 1593, 34, 102, 1.0},
  {0, 1593, 36, 89, 1.0},  {0, 1593, 39, 86, 1.0},  {0, 1594, 34, 103, 1.0},  {0, 1594, 36, 90, 1.0},  {0, 1594, 40, 86, 1.0},  {0, 1595, 34, 104, 1.0},
  {0, 1595, 36, 91, 1.0},  {0, 1595, 41, 86, 1.0},  {0, 1596, 34, 105, 1.0},  {0, 1596, 37, 87, 1.0},  {0, 1597, 34, 106, 1.0},  {0, 1597, 37, 88, 1.0},
  {0, 1597, 38, 87, 1.0},  {0, 1598, 34, 107, 1.0},  {0, 1598, 37, 89, 1.0},  {0, 1598, 39, 87, 1.0},  {0, 1599, 34, 108, 1.0},  {0, 1599, 37, 90, 1.0},
  {0, 1599, 40, 87, 1.0},  {0, 1600, 34, 109, 1.0},  {0, 1600, 37, 91, 1.0},  {0, 1600, 41, 87, 1.0},  {0, 1601, 34, 110, 1.0},  {0, 1601, 38, 88, 1.0},
  {0, 1602, 34, 111, 1.0},  {0, 1602, 38, 89, 1.0},  {0, 1602, 39, 88, 1.0},  {0, 1603, 34, 112, 1.0},  {0, 1603, 38, 90, 1.0},  {0, 1603, 40, 88, 1.0},
  {0, 1604, 34, 113, 1.0},  {0, 1604, 38, 91, 1.0},  {0, 1604, 41, 88, 1.0},  {0, 1605, 34, 114, 1.0},  {0, 1605, 39, 89, 1.0},  {0, 1606, 34, 115, 1.0},
  {0, 1606, 39, 90, 1.0},  {0, 1606, 40, 89, 1.0},  {0, 1607, 34, 116, 1.0},  {0, 1607, 39, 91, 1.0},  {0, 1607, 41, 89, 1.0},  {0, 1608, 34, 117, 1.0},
  {0, 1608, 40, 90, 1.0},  {0, 1609, 34, 118, 1.0},  {0, 1609, 40, 91, 1.0},  {0, 1609, 41, 90, 1.0},  {0, 1610, 34, 119, 1.0},  {0, 1610, 41, 91, 1.0},
  {0, 1611, 35, 92, 1.0},  {0, 1612, 35, 93, 1.0},  {0, 1612, 36, 92, 1.0},  {0, 1613, 35, 94, 1.0},  {0, 1613, 37, 92, 1.0},  {0, 1614, 35, 95, 1.0},
  {0, 1614, 38, 92, 1.0},  {0, 1615, 35, 96, 1.0},  {0, 1615, 39, 92, 1.0},  {0, 1616, 35, 97, 1.0},  {0, 1616, 40, 92, 1.0},  {0, 1617, 35, 98, 1.0},
  {0, 1617, 41, 92, 1.0},  {0, 1618, 35, 99, 1.0},  {0, 1618, 36, 93, 1.0},  {0, 1619, 35, 100, 1.0},  {0, 1619, 36, 94, 1.0},  {0, 1619, 37, 93, 1.0},
  {0, 1620, 35, 101, 1.0},  {0, 1620, 36, 95, 1.0},  {0, 1620, 38, 93, 1.0},  {0, 1621, 35, 102, 1.0},  {0, 1621, 36, 96, 1.0},  {0, 1621, 39, 93, 1.0},
  {0, 1622, 35, 103, 1.0},  {0, 1622, 36, 97, 1.0},  {0, 1622, 40, 93, 1.0},  {0, 1623, 35, 104, 1.0},  {0, 1623, 36, 98, 1.0},  {0, 1623, 41, 93, 1.0},
  {0, 1624, 35, 105, 1.0},  {0, 1624, 37, 94, 1.0},  {0, 1625, 35, 106, 1.0},  {0, 1625, 37, 95, 1.0},  {0, 1625, 38, 94, 1.0},  {0, 1626, 35, 107, 1.0},
  {0, 1626, 37, 96, 1.0},  {0, 1626, 39, 94, 1.0},  {0, 1627, 35, 108, 1.0},  {0, 1627, 37, 97, 1.0},  {0, 1627, 40, 94, 1.0},  {0, 1628, 35, 109, 1.0},
  {0, 1628, 37, 98, 1.0},  {0, 1628, 41, 94, 1.0},  {0, 1629, 35, 110, 1.0},  {0, 1629, 38, 95, 1.0},  {0, 1630, 35, 111, 1.0},  {0, 1630, 38, 96, 1.0},
  {0, 1630, 39, 95, 1.0},  {0, 1631, 35, 112, 1.0},  {0, 1631, 38, 97, 1.0},  {0, 1631, 40, 95, 1.0},  {0, 1632, 35, 113, 1.0},  {0, 1632, 38, 98, 1.0},
  {0, 1632, 41, 95, 1.0},  {0, 1633, 35, 114, 1.0},  {0, 1633, 39, 96, 1.0},  {0, 1634, 35, 115, 1.0},  {0, 1634, 39, 97, 1.0},  {0, 1634, 40, 96, 1.0},
  {0, 1635, 35, 116, 1.0},  {0, 1635, 39, 98, 1.0},  {0, 1635, 41, 96, 1.0},  {0, 1636, 35, 117, 1.0},  {0, 1636, 40, 97, 1.0},  {0, 1637, 35, 118, 1.0},
  {0, 1637, 40, 98, 1.0},  {0, 1637, 41, 97, 1.0},  {0, 1638, 35, 119, 1.0},  {0, 1638, 41, 98, 1.0},  {0, 1639, 36, 99, 1.0},  {0, 1640, 36, 100, 1.0},
  {0, 1640, 37, 99, 1.0},  {0, 1641, 36, 101, 1.0},  {0, 1641, 38, 99, 1.0},  {0, 1642, 36, 102, 1.0},  {0, 1642, 39, 99, 1.0},  {0, 1643, 36, 103, 1.0},
  {0, 1643, 40, 99, 1.0},  {0, 1644, 36, 104, 1.0},  {0, 1644, 41, 99, 1.0},  {0, 1645, 36, 105, 1.0},  {0, 1645, 37, 100, 1.0},  {0, 1646, 36, 106, 1.0},
  {0, 1646, 37, 101, 1.0},  {0, 1646, 38, 100, 1.0},  {0, 1647, 36, 107, 1.0},  {0, 1647, 37, 102, 1.0},  {0, 1647, 39, 100, 1.0},  {0, 1648, 36, 108, 1.0},
  {0, 1648, 37, 103, 1.0},  {0, 1648, 40, 100, 1.0},  {0, 1649, 36, 109, 1.0},  {0, 1649, 37, 104, 1.0},  {0, 1649, 41, 100, 1.0},  {0, 1650, 36, 110, 1.0},
  {0, 1650, 38, 101, 1.0},  {0, 1651, 36, 111, 1.0},  {0, 1651, 38, 102, 1.0},  {0, 1651, 39, 101, 1.0},  {0, 1652, 36, 112, 1.0},  {0, 1652, 38, 103, 1.0},
  {0, 1652, 40, 101, 1.0},  {0, 1653, 36, 113, 1.0},  {0, 1653, 38, 104, 1.0},  {0, 1653, 41, 101, 1.0},  {0, 1654, 36, 114, 1.0},  {0, 1654, 39, 102, 1.0},
  {0, 1655, 36, 115, 1.0},  {0, 1655, 39, 103, 1.0},  {0, 1655, 40, 102, 1.0},  {0, 1656, 36, 116, 1.0},  {0, 1656, 39, 104, 1.0},  {0, 1656, 41, 102, 1.0},
  {0, 1657, 36, 117, 1.0},  {0, 1657, 40, 103, 1.0},  {0, 1658, 36, 118, 1.0},  {0, 1658, 40, 104, 1.0},  {0, 1658, 41, 103, 1.0},  {0, 1659, 36, 119, 1.0},
  {0, 1659, 41, 104, 1.0},  {0, 1660, 37, 105, 1.0},  {0, 1661, 37, 106, 1.0},  {0, 1661, 38, 105, 1.0},  {0, 1662, 37, 107, 1.0},  {0, 1662, 39, 105, 1.0},
  {0, 1663, 37, 108, 1.0},  {0, 1663, 40, 105, 1.0},  {0, 1664, 37, 109, 1.0},  {0, 1664, 41, 105, 1.0},  {0, 1665, 37, 110, 1.0},  {0, 1665, 38, 106, 1.0},
  {0, 1666, 37, 111, 1.0},  {0, 1666, 38, 107, 1.0},  {0, 1666, 39, 106, 1.0},  {0, 1667, 37, 112, 1.0},  {0, 1667, 38, 108, 1.0},  {0, 1667, 40, 106, 1.0},
  {0, 1668, 37, 113, 1.0},  {0, 1668, 38, 109, 1.0},  {0, 1668, 41, 106, 1.0},  {0, 1669, 37, 114, 1.0},  {0, 1669, 39, 107, 1.0},  {0, 1670, 37, 115, 1.0},
  {0, 1670, 39, 108, 1.0},  {0, 1670, 40, 107, 1.0},  {0, 1671, 37, 116, 1.0},  {0, 1671, 39, 109, 1.0},  {0, 1671, 41, 107, 1.0},  {0, 1672, 37, 117, 1.0},
  {0, 1672, 40, 108, 1.0},  {0, 1673, 37, 118, 1.0},  {0, 1673, 40, 109, 1.0},  {0, 1673, 41, 108, 1.0},  {0, 1674, 37, 119, 1.0},  {0, 1674, 41, 109, 1.0},
  {0, 1675, 38, 110, 1.0},  {0, 1676, 38, 111, 1.0},  {0, 1676, 39, 110, 1.0},  {0, 1677, 38, 112, 1.0},  {0, 1677, 40, 110, 1.0},  {0, 1678, 38, 113, 1.0},
  {0, 1678, 41, 110, 1.0},  {0, 1679, 38, 114, 1.0},  {0, 1679, 39, 111, 1.0},  {0, 1680, 38, 115, 1.0},  {0, 1680, 39, 112, 1.0},  {0, 1680, 40, 111, 1.0},
  {0, 1681, 38, 116, 1.0},  {0, 1681, 39, 113, 1.0},  {0, 1681, 41, 111, 1.0},  {0, 1682, 38, 117, 1.0},  {0, 1682, 40, 112, 1.0},  {0, 1683, 38, 118, 1.0},
  {0, 1683, 40, 113, 1.0},  {0, 1683, 41, 112, 1.0},  {0, 1684, 38, 119, 1.0},  {0, 1684, 41, 113, 1.0},  {0, 1685, 39, 114, 1.0},  {0, 1686, 39, 115, 1.0},
  {0, 1686, 40, 114, 1.0},  {0, 1687, 39, 116, 1.0},  {0, 1687, 41, 114, 1.0},  {0, 1688, 39, 117, 1.0},  {0, 1688, 40, 115, 1.0},  {0, 1689, 39, 118, 1.0},
  {0, 1689, 40, 116, 1.0},  {0, 1689, 41, 115, 1.0},  {0, 1690, 39, 119, 1.0},  {0, 1690, 41, 116, 1.0},  {0, 1691, 40, 117, 1.0},  {0, 1692, 40, 118, 1.0},
  {0, 1692, 41, 117, 1.0},  {0, 1693, 40, 119, 1.0},  {0, 1693, 41, 118, 1.0},  {0, 1694, 41, 119, 1.0},  {0, 1695, 42, 42, 1.0},  {0, 1696, 42, 43, 1.0},
  {0, 1697, 42, 44, 1.0},  {0, 1698, 42, 45, 1.0},  {0, 1699, 42, 46, 1.0},  {0, 1700, 42, 47, 1.0},  {0, 1701, 42, 48, 1.0},  {0, 1702, 42, 49, 1.0},
  {0, 1703, 42, 50, 1.0},  {0, 1704, 42, 51, 1.0},  {0, 1705, 42, 52, 1.0},  {0, 1706, 42, 53, 1.0},  {0, 1707, 42, 54, 1.0},  {0, 1707, 43, 43, 1.0},
  {0, 1708, 42, 55, 1.0},  {0, 1708, 43, 44, 1.0},  {0, 1709, 42, 56, 1.0},  {0, 1709, 43, 45, 1.0},  {0, 1710, 42, 57, 1.0},  {0, 1710, 43, 46, 1.0},
  {0, 1711, 42, 58, 1.0},  {0, 1711, 43, 47, 1.0},  {0, 1712, 42, 59, 1.0},  {0, 1712, 43, 48, 1.0},  {0, 1713, 42, 60, 1.0},  {0, 1713, 43, 49, 1.0},
  {0, 1714, 42, 61, 1.0},  {0, 1714, 43, 50, 1.0},  {0, 1715, 42, 62, 1.0},  {0, 1715, 43, 51, 1.0},  {0, 1716, 42, 63, 1.0},  {0, 1716, 43, 52, 1.0},
  {0, 1717, 42, 64, 1.0},  {0, 1717, 43, 53, 1.0},  {0, 1718, 42, 65, 1.0},  {0, 1718, 44, 44, 1.0},  {0, 1719, 42, 66, 1.0},  {0, 1719, 44, 45, 1.0},
  {0, 1720, 42, 67, 1.0},  {0, 1720, 44, 46, 1.0},  {0, 1721, 42, 68, 1.0},  {0, 1721, 44, 47, 1.0},  {0, 1722, 42, 69, 1.0},  {0, 1722, 44, 48, 1.0},
  {0, 1723, 42, 70, 1.0},  {0, 1723, 44, 49, 1.0},  {0, 1724, 42, 71, 1.0},  {0, 1724, 44, 50, 1.0},  {0, 1725, 42, 72, 1.0},  {0, 1725, 44, 51, 1.0},
  {0, 1726, 42, 73, 1.0},  {0, 1726, 44, 52, 1.0},  {0, 1727, 42, 74, 1.0},  {0, 1727, 44, 53, 1.0},  {0, 1728, 42, 75, 1.0},  {0, 1728, 45, 45, 1.0},
  {0, 1729, 42, 76, 1.0},  {0, 1729, 45, 46, 1.0},  {0, 1730, 42, 77, 1.0},  {0, 1730, 45, 47, 1.0},  {0, 1731, 42, 78, 1.0},  {0, 1731, 45, 48, 1.0},
  {0, 1732, 42, 79, 1.0},  {0, 1732, 45, 49, 1.0},  {0, 1733, 42, 80, 1.0},  {0, 1733, 45, 50, 1.0},  {0, 1734, 42, 81, 1.0},  {0, 1734, 45, 51, 1.0},
  {0, 1735, 42, 82, 1.0},  {0, 1735, 45, 52, 1.0},  {0, 1736, 42, 83, 1.0},  {0, 1736, 45, 53, 1.0},  {0, 1737, 42, 84, 1.0},  {0, 1737, 46, 46, 1.0},
  {0, 1738, 42, 85, 1.0},  {0, 1738, 46, 47, 1.0},  {0, 1739, 42, 86, 1.0},  {0, 1739, 46, 48, 1.0},  {0, 1740, 42, 87, 1.0},  {0, 1740, 46, 49, 1.0},
  {0, 1741, 42, 88, 1.0},  {0, 1741, 46, 50, 1.0},  {0, 1742, 42, 89, 1.0},  {0, 1742, 46, 51, 1.0},  {0, 1743, 42, 90, 1.0},  {0, 1743, 46, 52, 1.0},
  {0, 1744, 42, 91, 1.0},  {0, 1744, 46, 53, 1.0},  {0, 1745, 42, 92, 1.0},  {0, 1745, 47, 47, 1.0},  {0, 1746, 42, 93, 1.0},  {0, 1746, 47, 48, 1.0},
  {0, 1747, 42, 94, 1.0},  {0, 1747, 47, 49, 1.0},  {0, 1748, 42, 95, 1.0},  {0, 1748, 47, 50, 1.0},  {0, 1749, 42, 96, 1.0},  {0, 1749, 47, 51, 1.0},
  {0, 1750, 42, 97, 1.0},  {0, 1750, 47, 52, 1.0},  {0, 1751, 42, 98, 1.0},  {0, 1751, 47, 53, 1.0},  {0, 1752, 42, 99, 1.0},  {0, 1752, 48, 48, 1.0},
  {0, 1753, 42, 100, 1.0},  {0, 1753, 48, 49, 1.0},  {0, 1754, 42, 101, 1.0},  {0, 1754, 48, 50, 1.0},  {0, 1755, 42, 102, 1.0},  {0, 1755, 48, 51, 1.0},
  {0, 1756, 42, 103, 1.0},  {0, 1756, 48, 52, 1.0},  {0, 1757, 42, 104, 1.0},  {0, 1757, 48, 53, 1.0},  {0, 1758, 42, 105, 1.0},  {0, 1758, 49, 49, 1.0},
  {0, 1759, 42, 106, 1.0},  {0, 1759, 49, 50, 1.0},  {0, 1760, 42, 107, 1.0},  {0, 1760, 49, 51, 1.0},  {0, 1761, 42, 108, 1.0},  {0, 1761, 49, 52, 1.0},
  {0, 1762, 42, 109, 1.0},  {0, 1762, 49, 53, 1.0},  {0, 1763, 42, 110, 1.0},  {0, 1763, 50, 50, 1.0},  {0, 1764, 42, 111, 1.0},  {0, 1764, 50, 51, 1.0},
  {0, 1765, 42, 112, 1.0},  {0, 1765, 50, 52, 1.0},  {0, 1766, 42, 113, 1.0},  {0, 1766, 50, 53, 1.0},  {0, 1767, 42, 114, 1.0},  {0, 1767, 51, 51, 1.0},
  {0, 1768, 42, 115, 1.0},  {0, 1768, 51, 52, 1.0},  {0, 1769, 42, 116, 1.0},  {0, 1769, 51, 53, 1.0},  {0, 1770, 42, 117, 1.0},  {0, 1770, 52, 52, 1.0},
  {0, 1771, 42, 118, 1.0},  {0, 1771, 52, 53, 1.0},  {0, 1772, 42, 119, 1.0},  {0, 1772, 53, 53, 1.0},  {0, 1773, 43, 54, 1.0},  {0, 1774, 43, 55, 1.0},
  {0, 1774, 44, 54, 1.0},  {0, 1775, 43, 56, 1.0},  {0, 1775, 45, 54, 1.0},  {0, 1776, 43, 57, 1.0},  {0, 1776, 46, 54, 1.0},  {0, 1777, 43, 58, 1.0},
  {0, 1777, 47, 54, 1.0},  {0, 1778, 43, 59, 1.0},  {0, 1778, 48, 54, 1.0},  {0, 1779, 43, 60, 1.0},  {0, 1779, 49, 54, 1.0},  {0, 1780, 43, 61, 1.0},
  {0, 1780, 50, 54, 1.0},  {0, 1781, 43, 62, 1.0},  {0, 1781, 51, 54, 1.0},  {0, 1782, 43, 63, 1.0},  {0, 1782, 52, 54, 1.0},  {0, 1783, 43, 64, 1.0},
  {0, 1783, 53, 54, 1.0},  {0, 1784, 43, 65, 1.0},  {0, 1784, 44, 55, 1.0},  {0, 1785, 43, 66, 1.0},  {0, 1785, 44, 56, 1.0},  {0, 1785, 45, 55, 1.0},
  {0, 1786, 43, 67, 1.0},  {0, 1786, 44, 57, 1.0},  {0, 1786, 46, 55, 1.0},  {0, 1787, 43, 68, 1.0},  {0, 1787, 44, 58, 1.0},  {0, 1787, 47, 55, 1.0},
  {0, 1788, 43, 69, 1.0},  {0, 1788, 44, 59, 1.0},  {0, 1788, 48, 55, 1.0},  {0, 1789, 43, 70, 1.0},  {0, 1789, 44, 60, 1.0},  {0, 1789, 49, 55, 1.0},
  {0, 1790, 43, 71, 1.0},  {0, 1790, 44, 61, 1.0},  {0, 1790, 50, 55, 1.0},  {0, 1791, 43, 72, 1.0},  {0, 1791, 44, 62, 1.0},  {0, 1791, 51, 55, 1.0},
  {0, 1792, 43, 73, 1.0},  {0, 1792, 44, 63, 1.0},  {0, 1792, 52, 55, 1.0},  {0, 1793, 43, 74, 1.0},  {0, 1793, 44, 64, 1.0},  {0, 1793, 53, 55, 1.0},
  {0, 1794, 43, 75, 1.0},  {0, 1794, 45, 56, 1.0},  {0, 1795, 43, 76, 1.0},  {0, 1795, 45, 57, 1.0},  {0, 1795, 46, 56, 1.0},  {0, 1796, 43, 77, 1.0},
  {0, 1796, 45, 58, 1.0},  {0, 1796, 47, 56, 1.0},  {0, 1797, 43, 78, 1.0},  {0, 1797, 45, 59, 1.0},  {0, 1797, 48, 56, 1.0},  {0, 1798, 43, 79, 1.0},
  {0, 1798, 45, 60, 1.0},  {0, 1798, 49, 56, 1.0},  {0, 1799, 43, 80, 1.0},  {0, 1799, 45, 61, 1.0},  {0, 1799, 50, 56, 1.0},  {0, 1800, 43, 81, 1.0},
  {0, 1800, 45, 62, 1.0},  {0, 1800, 51, 56, 1.0},  {0, 1801, 43, 82, 1.0},  {0, 1801, 45, 63, 1.0},  {0, 1801, 52, 56, 1.0},  {0, 1802, 43, 83, 1.0},
  {0, 1802, 45, 64, 1.0},  {0, 1802, 53, 56, 1.0},  {0, 1803, 43, 84, 1.0},  {0, 1803, 46, 57, 1.0},  {0, 1804, 43, 85, 1.0},  {0, 1804, 46, 58, 1.0},
  {0, 1804, 47, 57, 1.0},  {0, 1805, 43, 86, 1.0},  {0, 1805, 46, 59, 1.0},  {0, 1805, 48, 57, 1.0},  {0, 1806, 43, 87, 1.0},  {0, 1806, 46, 60, 1.0},
  {0, 1806, 49, 57, 1.0},  {0, 1807, 43, 88, 1.0},  {0, 1807, 46, 61, 1.0},  {0, 1807, 50, 57, 1.0},  {0, 1808, 43, 89, 1.0},  {0, 1808, 46, 62, 1.0},
  {0, 1808, 51, 57, 1.0},  {0, 1809, 43, 90, 1.0},  {0, 1809, 46, 63, 1.0},  {0, 1809, 52, 57, 1.0},  {0, 1810, 43, 91, 1.0},  {0, 1810, 46, 64, 1.0},
  {0, 1810, 53, 57, 1.0},  {0, 1811, 43, 92, 1.0},  {0, 1811, 47, 58, 1.0},  {0, 1812, 43, 93, 1.0},  {0, 1812, 47, 59, 1.0},  {0, 1812, 48, 58, 1.0},
  {0, 1813, 43, 94, 1.0},  {0, 1813, 47, 60, 1.0},  {0, 1813, 49, 58, 1.0},  {0, 1814, 43, 95, 1.0},  {0, 1814, 47, 61, 1.0},  {0, 1814, 50, 58, 1.0},
  {0, 1815, 43, 96, 1.0},  {0, 1815, 47, 62, 1.0},  {0, 1815, 51, 58, 1.0},  {0, 1816, 43, 97, 1.0},  {0, 1816, 47, 63, 1.0},  {0, 1816, 52, 58, 1.0},
  {0, 1817, 43, 98, 1.0},  {0, 1817, 47, 64, 1.0},  {0, 1817, 53, 58, 1.0},  {0, 1818, 43, 99, 1.0},  {0, 1818, 48, 59, 1.0},  {0, 1819, 43, 100, 1.0},
  {0, 1819, 48, 60, 1.0},  {0, 1819, 49, 59, 1.0},  {0, 1820, 43, 101, 1.0},  {0, 1820, 48, 61, 1.0},  {0, 1820, 50, 59, 1.0},  {0, 1821, 43, 102, 1.0},
  {0, 1821, 48, 62, 1.0},  {0, 1821, 51, 59, 1.0},  {0, 1822, 43, 103, 1.0},  {0, 1822, 48, 63, 1.0},  {0, 1822, 52, 59, 1.0},  {0, 1823, 43, 104, 1.0},
  {0, 1823, 48, 64, 1.0},  {0, 1823, 53, 59, 1.0},  {0, 1824, 43, 105, 1.0},  {0, 1824, 49, 60, 1.0},  {0, 1825, 43, 106, 1.0},  {0, 1825, 49, 61, 1.0},
  {0, 1825, 50, 60, 1.0},  {0, 1826, 43, 107, 1.0},  {0, 1826, 49, 62, 1.0},  {0, 1826, 51, 60, 1.0},  {0, 1827, 43, 108, 1.0},  {0, 1827, 49, 63, 1.0},
  {0, 1827, 52, 60, 1.0},  {0, 1828, 43, 109, 1.0},  {0, 1828, 49, 64, 1.0},  {0, 1828, 53, 60, 1.0},  {0, 1829, 43, 110, 1.0},  {0, 1829, 50, 61, 1.0},
  {0, 1830, 43, 111, 1.0},  {0, 1830, 50, 62, 1.0},  {0, 1830, 51, 61, 1.0},  {0, 1831, 43, 112, 1.0},  {0, 1831, 50, 63, 1.0},  {0, 1831, 52, 61, 1.0},
  {0, 1832, 43, 113, 1.0},  {0, 1832, 50, 64, 1.0},  {0, 1832, 53, 61, 1.0},  {0, 1833, 43, 114, 1.0},  {0, 1833, 51, 62, 1.0},  {0, 1834, 43, 115, 1.0},
  {0, 1834, 51, 63, 1.0},  {0, 1834, 52, 62, 1.0},  {0, 1835, 43, 116, 1.0},  {0, 1835, 51, 64, 1.0},  {0, 1835, 53, 62, 1.0},  {0, 1836, 43, 117, 1.0},
  {0, 1836, 52, 63, 1.0},  {0, 1837, 43, 118, 1.0},  {0, 1837, 52, 64, 1.0},  {0, 1837, 53, 63, 1.0},  {0, 1838, 43, 119, 1.0},  {0, 1838, 53, 64, 1.0},
  {0, 1839, 44, 65, 1.0},  {0, 1840, 44, 66, 1.0},  {0, 1840, 45, 65, 1.0},  {0, 1841, 44, 67, 1.0},  {0, 1841, 46, 65, 1.0},  {0, 1842, 44, 68, 1.0},
  {0, 1842, 47, 65, 1.0},  {0, 1843, 44, 69, 1.0},  {0, 1843, 48, 65, 1.0},  {0, 1844, 44, 70, 1.0},  {0, 1844, 49, 65, 1.0},  {0, 1845, 44, 71, 1.0},
  {0, 1845, 50, 65, 1.0},  {0, 1846, 44, 72, 1.0},  {0, 1846, 51, 65, 1.0},  {0, 1847, 44, 73, 1.0},  {0, 1847, 52, 65, 1.0},  {0, 1848, 44, 74, 1.0},
  {0, 1848, 53, 65, 1.0},  {0, 1849, 44, 75, 1.0},  {0, 1849, 45, 66, 1.0},  {0, 1850, 44, 76, 1.0},  {0, 1850, 45, 67, 1.0},  {0, 1850, 46, 66, 1.0},
  {0, 1851, 44, 77, 1.0},  {0, 1851, 45, 68, 1.0},  {0, 1851, 47, 66, 1.0},  {0, 1852, 44, 78, 1.0},  {0, 1852, 45, 69, 1.0},  {0, 1852, 48, 66, 1.0},
  {0, 1853, 44, 79, 1.0},  {0, 1853, 45, 70, 1.0},  {0, 1853, 49, 66, 1.0},  {0, 1854, 44, 80, 1.0},  {0, 1854, 45, 71, 1.0},  {0, 1854, 50, 66, 1.0},
  {0, 1855, 44, 81, 1.0},  {0, 1855, 45, 72, 1.0},  {0, 1855, 51, 66, 1.0},  {0, 1856, 44, 82, 1.0},  {0, 1856, 45, 73, 1.0},  {0, 1856, 52, 66, 1.0},
  {0, 1857, 44, 83, 1.0},  {0, 1857, 45, 74, 1.0},  {0, 1857, 53, 66, 1.0},  {0, 1858, 44, 84, 1.0},  {0, 1858, 46, 67, 1.0},  {0, 1859, 44, 85, 1.0},
  {0, 1859, 46, 68, 1.0},  {0, 1859, 47, 67, 1.0},  {0, 1860, 44, 86, 1.0},  {0, 1860, 46, 69, 1.0},  {0, 1860, 48, 67, 1.0},  {0, 1861, 44, 87, 1.0},
  {0, 1861, 46, 70, 1.0},  {0, 1861, 49, 67, 1.0},  {0, 1862, 44, 88, 1.0},  {0, 1862, 46, 71, 1.0},  {0, 1862, 50, 67, 1.0},  {0, 1863, 44, 89, 1.0},
  {0, 1863, 46, 72, 1.0},  {0, 1863, 51, 67, 1.0},  {0, 1864, 44, 90, 1.0},  {0, 1864, 46, 73, 1.0},  {0, 1864, 52, 67, 1.0},  {0, 1865, 44, 91, 1.0},
  {0, 1865, 46, 74, 1.0},  {0, 1865, 53, 67, 1.0},  {0, 1866, 44, 92, 1.0},  {0, 1866, 47, 68, 1.0},  {0, 1867, 44, 93, 1.0},  {0, 1867, 47, 69, 1.0},
  {0, 1867, 48, 68, 1.0},  {0, 1868, 44, 94, 1.0},  {0, 1868, 47, 70, 1.0},  {0, 1868, 49, 68, 1.0},  {0, 1869, 44, 95, 1.0},  {0, 1869, 47, 71, 1.0},
  {0, 1869, 50, 68, 1.0},  {0, 1870, 44, 96, 1.0},  {0, 1870, 47, 72, 1.0},  {0, 1870, 51, 68, 1.0},  {0, 1871, 44, 97, 1.0},  {0, 1871, 47, 73, 1.0},
  {0, 1871, 52, 68, 1.0},  {0, 1872, 44, 98, 1.0},  {0, 1872, 47, 74, 1.0},  {0, 1872, 53, 68, 1.0},  {0, 1873, 44, 99, 1.0},  {0, 1873, 48, 69, 1.0},
  {0, 1874, 44, 100, 1.0},  {0, 1874, 48, 70, 1.0},  {0, 1874, 49, 69, 1.0},  {0, 1875, 44, 101, 1.0},  {0, 1875, 48, 71, 1.0},  {0, 1875, 50, 69, 1.0},
  {0, 1876, 44, 102, 1.0},  {0, 1876, 48, 72, 1.0},  {0, 1876, 51, 69, 1.0},  {0, 1877, 44, 103, 1.0},  {0, 1877, 48, 73, 1.0},  {0, 1877, 52, 69, 1.0},
  {0, 1878, 44, 104, 1.0},  {0, 1878, 48, 74, 1.0},  {0, 1878, 53, 69, 1.0},  {0, 1879, 44, 105, 1.0},  {0, 1879, 49, 70, 1.0},  {0, 1880, 44, 106, 1.0},
  {0, 1880, 49, 71, 1.0},  {0, 1880, 50, 70, 1.0},  {0, 1881, 44, 107, 1.0},  {0, 1881, 49, 72, 1.0},  {0, 1881, 51, 70, 1.0},  {0, 1882, 44, 108, 1.0},
  {0, 1882, 49, 73, 1.0},  {0, 1882, 52, 70, 1.0},  {0, 1883, 44, 109, 1.0},  {0, 1883, 49, 74, 1.0},  {0, 1883, 53, 70, 1.0},  {0, 1884, 44, 110, 1.0},
  {0, 1884, 50, 71, 1.0},  {0, 1885, 44, 111, 1.0},  {0, 1885, 50, 72, 1.0},  {0, 1885, 51, 71, 1.0},  {0, 1886, 44, 112, 1.0},  {0, 1886, 50, 73, 1.0},
  {0, 1886, 52, 71, 1.0},  {0, 1887, 44, 113, 1.0},  {0, 1887, 50, 74, 1.0},  {0, 1887, 53, 71, 1.0},  {0, 1888, 44, 114, 1.0},  {0, 1888, 51, 72, 1.0},
  {0, 1889, 44, 115, 1.0},  {0, 1889, 51, 73, 1.0},  {0, 1889, 52, 72, 1.0},  {0, 1890, 44, 116, 1.0},  {0, 1890, 51, 74, 1.0},  {0, 1890, 53, 72, 1.0},
  {0, 1891, 44, 117, 1.0},  {0, 1891, 52, 73, 1.0},  {0, 1892, 44, 118, 1.0},  {0, 1892, 52, 74, 1.0},  {0, 1892, 53, 73, 1.0},  {0, 1893, 44, 119, 1.0},
  {0, 1893, 53, 74, 1.0},  {0, 1894, 45, 75, 1.0},  {0, 1895, 45, 76, 1.0},  {0, 1895, 46, 75, 1.0},  {0, 1896, 45, 77, 1.0},  {0, 1896, 47, 75, 1.0},
  {0, 1897, 45, 78, 1.0},  {0, 1897, 48, 75, 1.0},  {0, 1898, 45, 79, 1.0},  {0, 1898, 49, 75, 1.0},  {0, 1899, 45, 80, 1.0},  {0, 1899, 50, 75, 1.0},
  {0, 1900, 45, 81, 1.0},  {0, 1900, 51, 75, 1.0},  {0, 1901, 45, 82, 1.0},  {0, 1901, 52, 75, 1.0},  {0, 1902, 45, 83, 1.0},  {0, 1902, 53, 75, 1.0},
  {0, 1903, 45, 84, 1.0},  {0, 1903, 46, 76, 1.0},  {0, 1904, 45, 85, 1.0},  {0, 1904, 46, 77, 1.0},  {0, 1904, 47, 76, 1.0},  {0, 1905, 45, 86, 1.0},
  {0, 1905, 46, 78, 1.0},  {0, 1905, 48, 76, 1.0},  {0, 1906, 45, 87, 1.0},  {0, 1906, 46, 79, 1.0},  {0, 1906, 49, 76, 1.0},  {0, 1907, 45, 88, 1.0},
  {0, 1907, 46, 80, 1.0},  {0, 1907, 50, 76, 1.0},  {0, 1908, 45, 89, 1.0},  {0, 1908, 46, 81, 1.0},  {0, 1908, 51, 76, 1.0},  {0, 1909, 45, 90, 1.0},
  {0, 1909, 46, 82, 1.0},  {0, 1909, 52, 76, 1.0},  {0, 1910, 45, 91, 1.0},  {0, 1910, 46, 83, 1.0},  {0, 1910, 53, 76, 1.0},  {0, 1911, 45, 92, 1.0},
  {0, 1911, 47, 77, 1.0},  {0, 1912, 45, 93, 1.0},  {0, 1912, 47, 78, 1.0},  {0, 1912, 48, 77, 1.0},  {0, 1913, 45, 94, 1.0},  {0, 1913, 47, 79, 1.0},
  {0, 1913, 49, 77, 1.0},  {0, 1914, 45, 95, 1.0},  {0, 1914, 47, 80, 1.0},  {0, 1914, 50, 77, 1.0},  {0, 1915, 45, 96, 1.0},  {0, 1915, 47, 81, 1.0},
  {0, 1915, 51, 77, 1.0},  {0, 1916, 45, 97, 1.0},  {0, 1916, 47, 82, 1.0},  {0, 1916, 52, 77, 1.0},  {0, 1917, 45, 98, 1.0},  {0, 1917, 47, 83, 1.0},
  {0, 1917, 53, 77, 1.0},  {0, 1918, 45, 99, 1.0},  {0, 1918, 48, 78, 1.0},  {0, 1919, 45, 100, 1.0},  {0, 1919, 48, 79, 1.0},  {0, 1919, 49, 78, 1.0},
  {0, 1920, 45, 101, 1.0},  {0, 1920, 48, 80, 1.0},  {0, 1920, 50, 78, 1.0},  {0, 1921, 45, 102, 1.0},  {0, 1921, 48, 81, 1.0},  {0, 1921, 51, 78, 1.0},
  {0, 1922, 45, 103, 1.0},  {0, 1922, 48, 82, 1.0},  {0, 1922, 52, 78, 1.0},  {0, 1923, 45, 104, 1.0},  {0, 1923, 48, 83, 1.0},  {0, 1923, 53, 78, 1.0},
  {0, 1924, 45, 105, 1.0},  {0, 1924, 49, 79, 1.0},  {0, 1925, 45, 106, 1.0},  {0, 1925, 49, 80, 1.0},  {0, 1925, 50, 79, 1.0},  {0, 1926, 45, 107, 1.0},
  {0, 1926, 49, 81, 1.0},  {0, 1926, 51, 79, 1.0},  {0, 1927, 45, 108, 1.0},  {0, 1927, 49, 82, 1.0},  {0, 1927, 52, 79, 1.0},  {0, 1928, 45, 109, 1.0},
  {0, 1928, 49, 83, 1.0},  {0, 1928, 53, 79, 1.0},  {0, 1929, 45, 110, 1.0},  {0, 1929, 50, 80, 1.0},  {0, 1930, 45, 111, 1.0},  {0, 1930, 50, 81, 1.0},
  {0, 1930, 51, 80, 1.0},  {0, 1931, 45, 112, 1.0},  {0, 1931, 50, 82, 1.0},  {0, 1931, 52, 80, 1.0},  {0, 1932, 45, 113, 1.0},  {0, 1932, 50, 83, 1.0},
  {0, 1932, 53, 80, 1.0},  {0, 1933, 45, 114, 1.0},  {0, 1933, 51, 81, 1.0},  {0, 1934, 45, 115, 1.0},  {0, 1934, 51, 82, 1.0},  {0, 1934, 52, 81, 1.0},
  {0, 1935, 45, 116, 1.0},  {0, 1935, 51, 83, 1.0},  {0, 1935, 53, 81, 1.0},  {0, 1936, 45, 117, 1.0},  {0, 1936, 52, 82, 1.0},  {0, 1937, 45, 118, 1.0},
  {0, 1937, 52, 83, 1.0},  {0, 1937, 53, 82, 1.0},  {0, 1938, 45, 119, 1.0},  {0, 1938, 53, 83, 1.0},  {0, 1939, 46, 84, 1.0},  {0, 1940, 46, 85, 1.0},
  {0, 1940, 47, 84, 1.0},  {0, 1941, 46, 86, 1.0},  {0, 1941, 48, 84, 1.0},  {0, 1942, 46, 87, 1.0},  {0, 1942, 49, 84, 1.0},  {0, 1943, 46, 88, 1.0},
  {0, 1943, 50, 84, 1.0},  {0, 1944, 46, 89, 1.0},  {0, 1944, 51, 84, 1.0},  {0, 1945, 46, 90, 1.0},  {0, 1945, 52, 84, 1.0},  {0, 1946, 46, 91, 1.0},
  {0, 1946, 53, 84, 1.0},  {0, 1947, 46, 92, 1.0},  {0, 1947, 47, 85, 1.0},  {0, 1948, 46, 93, 1.0},  {0, 1948, 47, 86, 1.0},  {0, 1948, 48, 85, 1.0},
  {0, 1949, 46, 94, 1.0},  {0, 1949, 47, 87, 1.0},  {0, 1949, 49, 85, 1.0},  {0, 1950, 46, 95, 1.0},  {0, 1950, 47, 88, 1.0},  {0, 1950, 50, 85, 1.0},
  {0, 1951, 46, 96, 1.0},  {0, 1951, 47, 89, 1.0},  {0, 1951, 51, 85, 1.0},  {0, 1952, 46, 97, 1.0},  {0, 1952, 47, 90, 1.0},  {0, 1952, 52, 85, 1.0},
  {0, 1953, 46, 98, 1.0},  {0, 1953, 47, 91, 1.0},  {0, 1953, 53, 85, 1.0},  {0, 1954, 46, 99, 1.0},  {0, 1954, 48, 86, 1.0},  {0, 1955, 46, 100, 1.0},
  {0, 1955, 48, 87, 1.0},  {0, 1955, 49, 86, 1.0},  {0, 1956, 46, 101, 1.0},  {0, 1956, 48, 88, 1.0},  {0, 1956, 50, 86, 1.0},  {0, 1957, 46, 102, 1.0},
  {0, 1957, 48, 89, 1.0},  {0, 1957, 51, 86, 1.0},  {0, 1958, 46, 103, 1.0},  {0, 1958, 48, 90, 1.0},  {0, 1958, 52, 86, 1.0},  {0, 1959, 46, 104, 1.0},
  {0, 1959, 48, 91, 1.0},  {0, 1959, 53, 86, 1.0},  {0, 1960, 46, 105, 1.0},  {0, 1960, 49, 87, 1.0},  {0, 1961, 46, 106, 1.0},  {0, 1961, 49, 88, 1.0},
  {0, 1961, 50, 87, 1.0},  {0, 1962, 46, 107, 1.0},  {0, 1962, 49, 89, 1.0},  {0, 1962, 51, 87, 1.0},  {0, 1963, 46, 108, 1.0},  {0, 1963, 49, 90, 1.0},
  {0, 1963, 52, 87, 1.0},  {0, 1964, 46, 109, 1.0},  {0, 1964, 49, 91, 1.0},  {0, 1964, 53, 87, 1.0},  {0, 1965, 46, 110, 1.0},  {0, 1965, 50, 88, 1.0},
  {0, 1966, 46, 111, 1.0},  {0, 1966, 50, 89, 1.0},  {0, 1966, 51, 88, 1.0},  {0, 1967, 46, 112, 1.0},  {0, 1967, 50, 90, 1.0},  {0, 1967, 52, 88, 1.0},
  {0, 1968, 46, 113, 1.0},  {0, 1968, 50, 91, 1.0},  {0, 1968, 53, 88, 1.0},  {0, 1969, 46, 114, 1.0},  {0, 1969, 51, 89, 1.0},  {0, 1970, 46, 115, 1.0},
  {0, 1970, 51, 90, 1.0},  {0, 1970, 52, 89, 1.0},  {0, 1971, 46, 116, 1.0},  {0, 1971, 51, 91, 1.0},  {0, 1971, 53, 89, 1.0},  {0, 1972, 46, 117, 1.0},
  {0, 1972, 52, 90, 1.0},  {0, 1973, 46, 118, 1.0},  {0, 1973, 52, 91, 1.0},  {0, 1973, 53, 90, 1.0},  {0, 1974, 46, 119, 1.0},  {0, 1974, 53, 91, 1.0},
  {0, 1975, 47, 92, 1.0},  {0, 1976, 47, 93, 1.0},  {0, 1976, 48, 92, 1.0},  {0, 1977, 47, 94, 1.0},  {0, 1977, 49, 92, 1.0},  {0, 1978, 47, 95, 1.0},
  {0, 1978, 50, 92, 1.0},  {0, 1979, 47, 96, 1.0},  {0, 1979, 51, 92, 1.0},  {0, 1980, 47, 97, 1.0},  {0, 1980, 52, 92, 1.0},  {0, 1981, 47, 98, 1.0},
  {0, 1981, 53, 92, 1.0},  {0, 1982, 47, 99, 1.0},  {0, 1982, 48, 93, 1.0},  {0, 1983, 47, 100, 1.0},  {0, 1983, 48, 94, 1.0},  {0, 1983, 49, 93, 1.0},
  {0, 1984, 47, 101, 1.0},  {0, 1984, 48, 95, 1.0},  {0, 1984, 50, 93, 1.0},  {0, 1985, 47, 102, 1.0},  {0, 1985, 48, 96, 1.0},  {0, 1985, 51, 93, 1.0},
  {0, 1986, 47, 103, 1.0},  {0, 1986, 48, 97, 1.0},  {0, 1986, 52, 93, 1.0},  {0, 1987, 47, 104, 1.0},  {0, 1987, 48, 98, 1.0},  {0, 1987, 53, 93, 1.0},
  {0, 1988, 47, 105, 1.0},  {0, 1988, 49, 94, 1.0},  {0, 1989, 47, 106, 1.0},  {0, 1989, 49, 95, 1.0},  {0, 1989, 50, 94, 1.0},  {0, 1990, 47, 107, 1.0},
  {0, 1990, 49, 96, 1.0},  {0, 1990, 51, 94, 1.0},  {0, 1991, 47, 108, 1.0},  {0, 1991, 49, 97, 1.0},  {0, 1991, 52, 94, 1.0},  {0, 1992, 47, 109, 1.0},
  {0, 1992, 49, 98, 1.0},  {0, 1992, 53, 94, 1.0},  {0, 1993, 47, 110, 1.0},  {0, 1993, 50, 95, 1.0},  {0, 1994, 47, 111, 1.0},  {0, 1994, 50, 96, 1.0},
  {0, 1994, 51, 95, 1.0},  {0, 1995, 47, 112, 1.0},  {0, 1995, 50, 97, 1.0},  {0, 1995, 52, 95, 1.0},  {0, 1996, 47, 113, 1.0},  {0, 1996, 50, 98, 1.0},
  {0, 1996, 53, 95, 1.0},  {0, 1997, 47, 114, 1.0},  {0, 1997, 51, 96, 1.0},  {0, 1998, 47, 115, 1.0},  {0, 1998, 51, 97, 1.0},  {0, 1998, 52, 96, 1.0},
  {0, 1999, 47, 116, 1.0},  {0, 1999, 51, 98, 1.0},  {0, 1999, 53, 96, 1.0},  {0, 2000, 47, 117, 1.0},  {0, 2000, 52, 97, 1.0},  {0, 2001, 47, 118, 1.0},
  {0, 2001, 52, 98, 1.0},  {0, 2001, 53, 97, 1.0},  {0, 2002, 47, 119, 1.0},  {0, 2002, 53, 98, 1.0},  {0, 2003, 48, 99, 1.0},  {0, 2004, 48, 100, 1.0},
  {0, 2004, 49, 99, 1.0},  {0, 2005, 48, 101, 1.0},  {0, 2005, 50, 99, 1.0},  {0, 2006, 48, 102, 1.0},  {0, 2006, 51, 99, 1.0},  {0, 2007, 48, 103, 1.0},
  {0, 2007, 52, 99, 1.0},  {0, 2008, 48, 104, 1.0},  {0, 2008, 53, 99, 1.0},  {0, 2009, 48, 105, 1.0},  {0, 2009, 49, 100, 1.0},  {0, 2010, 48, 106, 1.0},
  {0, 2010, 49, 101, 1.0},  {0, 2010, 50, 100, 1.0},  {0, 2011, 48, 107, 1.0},  {0, 2011, 49, 102, 1.0},  {0, 2011, 51, 100, 1.0},  {0, 2012, 48, 108, 1.0},
  {0, 2012, 49, 103, 1.0},  {0, 2012, 52, 100, 1.0},  {0, 2013, 48, 109, 1.0},  {0, 2013, 49, 104, 1.0},  {0, 2013, 53, 100, 1.0},  {0, 2014, 48, 110, 1.0},
  {0, 2014, 50, 101, 1.0},  {0, 2015, 48, 111, 1.0},  {0, 2015, 50, 102, 1.0},  {0, 2015, 51, 101, 1.0},  {0, 2016, 48, 112, 1.0},  {0, 2016, 50, 103, 1.0},
  {0, 2016, 52, 101, 1.0},  {0, 2017, 48, 113, 1.0},  {0, 2017, 50, 104, 1.0},  {0, 2017, 53, 101, 1.0},  {0, 2018, 48, 114, 1.0},  {0, 2018, 51, 102, 1.0},
  {0, 2019, 48, 115, 1.0},  {0, 2019, 51, 103, 1.0},  {0, 2019, 52, 102, 1.0},  {0, 2020, 48, 116, 1.0},  {0, 2020, 51, 104, 1.0},  {0, 2020, 53, 102, 1.0},
  {0, 2021, 48, 117, 1.0},  {0, 2021, 52, 103, 1.0},  {0, 2022, 48, 118, 1.0},  {0, 2022, 52, 104, 1.0},  {0, 2022, 53, 103, 1.0},  {0, 2023, 48, 119, 1.0},
  {0, 2023, 53, 104, 1.0},  {0, 2024, 49, 105, 1.0},  {0, 2025, 49, 106, 1.0},  {0, 2025, 50, 105, 1.0},  {0, 2026, 49, 107, 1.0},  {0, 2026, 51, 105, 1.0},
  {0, 2027, 49, 108, 1.0},  {0, 2027, 52, 105, 1.0},  {0, 2028, 49, 109, 1.0},  {0, 2028, 53, 105, 1.0},  {0, 2029, 49, 110, 1.0},  {0, 2029, 50, 106, 1.0},
  {0, 2030, 49, 111, 1.0},  {0, 2030, 50, 107, 1.0},  {0, 2030, 51, 106, 1.0},  {0, 2031, 49, 112, 1.0},  {0, 2031, 50, 108, 1.0},  {0, 2031, 52, 106, 1.0},
  {0, 2032, 49, 113, 1.0},  {0, 2032, 50, 109, 1.0},  {0, 2032, 53, 106, 1.0},  {0, 2033, 49, 114, 1.0},  {0, 2033, 51, 107, 1.0},  {0, 2034, 49, 115, 1.0},
  {0, 2034, 51, 108, 1.0},  {0, 2034, 52, 107, 1.0},  {0, 2035, 49, 116, 1.0},  {0, 2035, 51, 109, 1.0},  {0, 2035, 53, 107, 1.0},  {0, 2036, 49, 117, 1.0},
  {0, 2036, 52, 108, 1.0},  {0, 2037, 49, 118, 1.0},  {0, 2037, 52, 109, 1.0},  {0, 2037, 53, 108, 1.0},  {0, 2038, 49, 119, 1.0},  {0, 2038, 53, 109, 1.0},
  {0, 2039, 50, 110, 1.0},  {0, 2040, 50, 111, 1.0},  {0, 2040, 51, 110, 1.0},  {0, 2041, 50, 112, 1.0},  {0, 2041, 52, 110, 1.0},  {0, 2042, 50, 113, 1.0},
  {0, 2042, 53, 110, 1.0},  {0, 2043, 50, 114, 1.0},  {0, 2043, 51, 111, 1.0},  {0, 2044, 50, 115, 1.0},  {0, 2044, 51, 112, 1.0},  {0, 2044, 52, 111, 1.0},
  {0, 2045, 50, 116, 1.0},  {0, 2045, 51, 113, 1.0},  {0, 2045, 53, 111, 1.0},  {0, 2046, 50, 117, 1.0},  {0, 2046, 52, 112, 1.0},  {0, 2047, 50, 118, 1.0},
  {0, 2047, 52, 113, 1.0},  {0, 2047, 53, 112, 1.0},  {0, 2048, 50, 119, 1.0},  {0, 2048, 53, 113, 1.0},  {0, 2049, 51, 114, 1.0},  {0, 2050, 51, 115, 1.0},
  {0, 2050, 52, 114, 1.0},  {0, 2051, 51, 116, 1.0},  {0, 2051, 53, 114, 1.0},  {0, 2052, 51, 117, 1.0},  {0, 2052, 52, 115, 1.0},  {0, 2053, 51, 118, 1.0},
  {0, 2053, 52, 116, 1.0},  {0, 2053, 53, 115, 1.0},  {0, 2054, 51, 119, 1.0},  {0, 2054, 53, 116, 1.0},  {0, 2055, 52, 117, 1.0},  {0, 2056, 52, 118, 1.0},
  {0, 2056, 53, 117, 1.0},  {0, 2057, 52, 119, 1.0},  {0, 2057, 53, 118, 1.0},  {0, 2058, 53, 119, 1.0},  {0, 2059, 54, 54, 1.0},  {0, 2060, 54, 55, 1.0},
  {0, 2061, 54, 56, 1.0},  {0, 2062, 54, 57, 1.0},  {0, 2063, 54, 58, 1.0},  {0, 2064, 54, 59, 1.0},  {0, 2065, 54, 60, 1.0},  {0, 2066, 54, 61, 1.0},
  {0, 2067, 54, 62, 1.0},  {0, 2068, 54, 63, 1.0},  {0, 2069, 54, 64, 1.0},  {0, 2070, 54, 65, 1.0},  {0, 2070, 55, 55, 1.0},  {0, 2071, 54, 66, 1.0},
  {0, 2071, 55, 56, 1.0},  {0, 2072, 54, 67, 1.0},  {0, 2072, 55, 57, 1.0},  {0, 2073, 54, 68, 1.0},  {0, 2073, 55, 58, 1.0},  {0, 2074, 54, 69, 1.0},
  {0, 2074, 55, 59, 1.0},  {0, 2075, 54, 70, 1.0},  {0, 2075, 55, 60, 1.0},  {0, 2076, 54, 71, 1.0},  {0, 2076, 55, 61, 1.0},  {0, 2077, 54, 72, 1.0},
  {0, 2077, 55, 62, 1.0},  {0, 2078, 54, 73, 1.0},  {0, 2078, 55, 63, 1.0},  {0, 2079, 54, 74, 1.0},  {0, 2079, 55, 64, 1.0},  {0, 2080, 54, 75, 1.0},
  {0, 2080, 56, 56, 1.0},  {0, 2081, 54, 76, 1.0},  {0, 2081, 56, 57, 1.0},  {0, 2082, 54, 77, 1.0},  {0, 2082, 56, 58, 1.0},  {0, 2083, 54, 78, 1.0},
  {0, 2083, 56, 59, 1.0},  {0, 2084, 54, 79, 1.0},  {0, 2084, 56, 60, 1.0},  {0, 2085, 54, 80, 1.0},  {0, 2085, 56, 61, 1.0},  {0, 2086, 54, 81, 1.0},
  {0, 2086, 56, 62, 1.0},  {0, 2087, 54, 82, 1.0},  {0, 2087, 56, 63, 1.0},  {0, 2088, 54, 83, 1.0},  {0, 2088, 56, 64, 1.0},  {0, 2089, 54, 84, 1.0},
  {0, 2089, 57, 57, 1.0},  {0, 2090, 54, 85, 1.0},  {0, 2090, 57, 58, 1.0},  {0, 2091, 54, 86, 1.0},  {0, 2091, 57, 59, 1.0},  {0, 2092, 54, 87, 1.0},
  {0, 2092, 57, 60, 1.0},  {0, 2093, 54, 88, 1.0},  {0, 2093, 57, 61, 1.0},  {0, 2094, 54, 89, 1.0},  {0, 2094, 57, 62, 1.0},  {0, 2095, 54, 90, 1.0},
  {0, 2095, 57, 63, 1.0},  {0, 2096, 54, 91, 1.0},  {0, 2096, 57, 64, 1.0},  {0, 2097, 54, 92, 1.0},  {0, 2097, 58, 58, 1.0},  {0, 2098, 54, 93, 1.0},
  {0, 2098, 58, 59, 1.0},  {0, 2099, 54, 94, 1.0},  {0, 2099, 58, 60, 1.0},  {0, 2100, 54, 95, 1.0},  {0, 2100, 58, 61, 1.0},  {0, 2101, 54, 96, 1.0},
  {0, 2101, 58, 62, 1.0},  {0, 2102, 54, 97, 1.0},  {0, 2102, 58, 63, 1.0},  {0, 2103, 54, 98, 1.0},  {0, 2103, 58, 64, 1.0},  {0, 2104, 54, 99, 1.0},
  {0, 2104, 59, 59, 1.0},  {0, 2105, 54, 100, 1.0},  {0, 2105, 59, 60, 1.0},  {0, 2106, 54, 101, 1.0},  {0, 2106, 59, 61, 1.0},  {0, 2107, 54, 102, 1.0},
  {0, 2107, 59, 62, 1.0},  {0, 2108, 54, 103, 1.0},  {0, 2108, 59, 63, 1.0},  {0, 2109, 54, 104, 1.0},  {0, 2109, 59, 64, 1.0},  {0, 2110, 54, 105, 1.0},
  {0, 2110, 60, 60, 1.0},  {0, 2111, 54, 106, 1.0},  {0, 2111, 60, 61, 1.0},  {0, 2112, 54, 107, 1.0},  {0, 2112, 60, 62, 1.0},  {0, 2113, 54, 108, 1.0},
  {0, 2113, 60, 63, 1.0},  {0, 2114, 54, 109, 1.0},  {0, 2114, 60, 64, 1.0},  {0, 2115, 54, 110, 1.0},  {0, 2115, 61, 61, 1.0},  {0, 2116, 54, 111, 1.0},
  {0, 2116, 61, 62, 1.0},  {0, 2117, 54, 112, 1.0},  {0, 2117, 61, 63, 1.0},  {0, 2118, 54, 113, 1.0},  {0, 2118, 61, 64, 1.0},  {0, 2119, 54, 114, 1.0},
  {0, 2119, 62, 62, 1.0},  {0, 2120, 54, 115, 1.0},  {0, 2120, 62, 63, 1.0},  {0, 2121, 54, 116, 1.0},  {0, 2121, 62, 64, 1.0},  {0, 2122, 54, 117, 1.0},
  {0, 2122, 63, 63, 1.0},  {0, 2123, 54, 118, 1.0},  {0, 2123, 63, 64, 1.0},  {0, 2124, 54, 119, 1.0},  {0, 2124, 64, 64, 1.0},  {0, 2125, 55, 65, 1.0},
  {0, 2126, 55, 66, 1.0},  {0, 2126, 56, 65, 1.0},  {0, 2127, 55, 67, 1.0},  {0, 2127, 57, 65, 1.0},  {0, 2128, 55, 68, 1.0},  {0, 2128, 58, 65, 1.0},
  {0, 2129, 55, 69, 1.0},  {0, 2129, 59, 65, 1.0},  {0, 2130, 55, 70, 1.0},  {0, 2130, 60, 65, 1.0},  {0, 2131, 55, 71, 1.0},  {0, 2131, 61, 65, 1.0},
  {0, 2132, 55, 72, 1.0},  {0, 2132, 62, 65, 1.0},  {0, 2133, 55, 73, 1.0},  {0, 2133, 63, 65, 1.0},  {0, 2134, 55, 74, 1.0},  {0, 2134, 64, 65, 1.0},
  {0, 2135, 55, 75, 1.0},  {0, 2135, 56, 66, 1.0},  {0, 2136, 55, 76, 1.0},  {0, 2136, 56, 67, 1.0},  {0, 2136, 57, 66, 1.0},  {0, 2137, 55, 77, 1.0},
  {0, 2137, 56, 68, 1.0},  {0, 2137, 58, 66, 1.0},  {0, 2138, 55, 78, 1.0},  {0, 2138, 56, 69, 1.0},  {0, 2138, 59, 66, 1.0},  {0, 2139, 55, 79, 1.0},
  {0, 2139, 56, 70, 1.0},  {0, 2139, 60, 66, 1.0},  {0, 2140, 55, 80, 1.0},  {0, 2140, 56, 71, 1.0},  {0, 2140, 61, 66, 1.0},  {0, 2141, 55, 81, 1.0},
  {0, 2141, 56, 72, 1.0},  {0, 2141, 62, 66, 1.0},  {0, 2142, 55, 82, 1.0},  {0, 2142, 56, 73, 1.0},  {0, 2142, 63, 66, 1.0},  {0, 2143, 55, 83, 1.0},
  {0, 2143, 56, 74, 1.0},  {0, 2143, 64, 66, 1.0},  {0, 2144, 55, 84, 1.0},  {0, 2144, 57, 67, 1.0},  {0, 2145, 55, 85, 1.0},  {0, 2145, 57, 68, 1.0},
  {0, 2145, 58, 67, 1.0},  {0, 2146, 55, 86, 1.0},  {0, 2146, 57, 69, 1.0},  {0, 2146, 59, 67, 1.0},  {0, 2147, 55, 87, 1.0},  {0, 2147, 57, 70, 1.0},
  {0, 2147, 60, 67, 1.0},  {0, 2148, 55, 88, 1.0},  {0, 2148, 57, 71, 1.0},  {0, 2148, 61, 67, 1.0},  {0, 2149, 55, 89, 1.0},  {0, 2149, 57, 72, 1.0},
  {0, 2149, 62, 67, 1.0},  {0, 2150, 55, 90, 1.0},  {0, 2150, 57, 73, 1.0},  {0, 2150, 63, 67, 1.0},  {0, 2151, 55, 91, 1.0},  {0, 2151, 57, 74, 1.0},
  {0, 2151, 64, 67, 1.0},  {0, 2152, 55, 92, 1.0},  {0, 2152, 58, 68, 1.0},  {0, 2153, 55, 93, 1.0},  {0, 2153, 58, 69, 1.0},  {0, 2153, 59, 68, 1.0},
  {0, 2154, 55, 94, 1.0},  {0, 2154, 58, 70, 1.0},  {0, 2154, 60, 68, 1.0},  {0, 2155, 55, 95, 1.0},  {0, 2155, 58, 71, 1.0},  {0, 2155, 61, 68, 1.0},
  {0, 2156, 55, 96, 1.0},  {0, 2156, 58, 72, 1.0},  {0, 2156, 62, 68, 1.0},  {0, 2157, 55, 97, 1.0},  {0, 2157, 58, 73, 1.0},  {0, 2157, 63, 68, 1.0},
  {0, 2158, 55, 98, 1.0},  {0, 2158, 58, 74, 1.0},  {0, 2158, 64, 68, 1.0},  {0, 2159, 55, 99, 1.0},  {0, 2159, 59, 69, 1.0},  {0, 2160, 55, 100, 1.0},
  {0, 2160, 59, 70, 1.0},  {0, 2160, 60, 69, 1.0},  {0, 2161, 55, 101, 1.0},  {0, 2161, 59, 71, 1.0},  {0, 2161, 61, 69, 1.0},  {0, 2162, 55, 102, 1.0},
  {0, 2162, 59, 72, 1.0},  {0, 2162, 62, 69, 1.0},  {0, 2163, 55, 103, 1.0},  {0, 2163, 59, 73, 1.0},  {0, 2163, 63, 69, 1.0},  {0, 2164, 55, 104, 1.0},
  {0, 2164, 59, 74, 1.0},  {0, 2164, 64, 69, 1.0},  {0, 2165, 55, 105, 1.0},  {0, 2165, 60, 70, 1.0},  {0, 2166, 55, 106, 1.0},  {0, 2166, 60, 71, 1.0},
  {0, 2166, 61, 70, 1.0},  {0, 2167, 55, 107, 1.0},  {0, 2167, 60, 72, 1.0},  {0, 2167, 62, 70, 1.0},  {0, 2168, 55, 108, 1.0},  {0, 2168, 60, 73, 1.0},
  {0, 2168, 63, 70, 1.0},  {0, 2169, 55, 109, 1.0},  {0, 2169, 60, 74, 1.0},  {0, 2169, 64, 70, 1.0},  {0, 2170, 55, 110, 1.0},  {0, 2170, 61, 71, 1.0},
  {0, 2171, 55, 111, 1.0},  {0, 2171, 61, 72, 1.0},  {0, 2171, 62, 71, 1.0},  {0, 2172, 55, 112, 1.0},  {0, 2172, 61, 73, 1.0},  {0, 2172, 63, 71, 1.0},
  {0, 2173, 55, 113, 1.0},  {0, 2173, 61, 74, 1.0},  {0, 2173, 64, 71, 1.0},  {0, 2174, 55, 114, 1.0},  {0, 2174, 62, 72, 1.0},  {0, 2175, 55, 115, 1.0},
  {0, 2175, 62, 73, 1.0},  {0, 2175, 63, 72, 1.0},  {0, 2176, 55, 116, 1.0},  {0, 2176, 62, 74, 1.0},  {0, 2176, 64, 72, 1.0},  {0, 2177, 55, 117, 1.0},
  {0, 2177, 63, 73, 1.0},  {0, 2178, 55, 118, 1.0},  {0, 2178, 63, 74, 1.0},  {0, 2178, 64, 73, 1.0},  {0, 2179, 55, 119, 1.0},  {0, 2179, 64, 74, 1.0},
  {0, 2180, 56, 75, 1.0},  {0, 2181, 56, 76, 1.0},  {0, 2181, 57, 75, 1.0},  {0, 2182, 56, 77, 1.0},  {0, 2182, 58, 75, 1.0},  {0, 2183, 56, 78, 1.0},
  {0, 2183, 59, 75, 1.0},  {0, 2184, 56, 79, 1.0},  {0, 2184, 60, 75, 1.0},  {0, 2185, 56, 80, 1.0},  {0, 2185, 61, 75, 1.0},  {0, 2186, 56, 81, 1.0},
  {0, 2186, 62, 75, 1.0},  {0, 2187, 56, 82, 1.0},  {0, 2187, 63, 75, 1.0},  {0, 2188, 56, 83, 1.0},  {0, 2188, 64, 75, 1.0},  {0, 2189, 56, 84, 1.0},
  {0, 2189, 57, 76, 1.0},  {0, 2190, 56, 85, 1.0},  {0, 2190, 57, 77, 1.0},  {0, 2190, 58, 76, 1.0},  {0, 2191, 56, 86, 1.0},  {0, 2191, 57, 78, 1.0},
  {0, 2191, 59, 76, 1.0},  {0, 2192, 56, 87, 1.0},  {0, 2192, 57, 79, 1.0},  {0, 2192, 60, 76, 1.0},  {0, 2193, 56, 88, 1.0},  {0, 2193, 57, 80, 1.0},
  {0, 2193, 61, 76, 1.0},  {0, 2194, 56, 89, 1.0},  {0, 2194, 57, 81, 1.0},  {0, 2194, 62, 76, 1.0},  {0, 2195, 56, 90, 1.0},  {0, 2195, 57, 82, 1.0},
  {0, 2195, 63, 76, 1.0},  {0, 2196, 56, 91, 1.0},  {0, 2196, 57, 83, 1.0},  {0, 2196, 64, 76, 1.0},  {0, 2197, 56, 92, 1.0},  {0, 2197, 58, 77, 1.0},
  {0, 2198, 56, 93, 1.0},  {0, 2198, 58, 78, 1.0},  {0, 2198, 59, 77, 1.0},  {0, 2199, 56, 94, 1.0},  {0, 2199, 58, 79, 1.0},  {0, 2199, 60, 77, 1.0},
  {0, 2200, 56, 95, 1.0},  {0, 2200, 58, 80, 1.0},  {0, 2200, 61, 77, 1.0},  {0, 2201, 56, 96, 1.0},  {0, 2201, 58, 81, 1.0},  {0, 2201, 62, 77, 1.0},
  {0, 2202, 56, 97, 1.0},  {0, 2202, 58, 82, 1.0},  {0, 2202, 63, 77, 1.0},  {0, 2203, 56, 98, 1.0},  {0, 2203, 58, 83, 1.0},  {0, 2203, 64, 77, 1.0},
  {0, 2204, 56, 99, 1.0},  {0, 2204, 59, 78, 1.0},  {0, 2205, 56, 100, 1.0},  {0, 2205, 59, 79, 1.0},  {0, 2205, 60, 78, 1.0},  {0, 2206, 56, 101, 1.0},
  {0, 2206, 59, 80, 1.0},  {0, 2206, 61, 78, 1.0},  {0, 2207, 56, 102, 1.0},  {0, 2207, 59, 81, 1.0},  {0, 2207, 62, 78, 1.0},  {0, 2208, 56, 103, 1.0},
  {0, 2208, 59, 82, 1.0},  {0, 2208, 63, 78, 1.0},  {0, 2209, 56, 104, 1.0},  {0, 2209, 59, 83, 1.0},  {0, 2209, 64, 78, 1.0},  {0, 2210, 56, 105, 1.0},
  {0, 2210, 60, 79, 1.0},  {0, 2211, 56, 106, 1.0},  {0, 2211, 60, 80, 1.0},  {0, 2211, 61, 79, 1.0},  {0, 2212, 56, 107, 1.0},  {0, 2212, 60, 81, 1.0},
  {0, 2212, 62, 79, 1.0},  {0, 2213, 56, 108, 1.0},  {0, 2213, 60, 82, 1.0},  {0, 2213, 63, 79, 1.0},  {0, 2214, 56, 109, 1.0},  {0, 2214, 60, 83, 1.0},
  {0, 2214, 64, 79, 1.0},  {0, 2215, 56, 110, 1.0},  {0, 2215, 61, 80, 1.0},  {0, 2216, 56, 111, 1.0},  {0, 2216, 61, 81, 1.0},  {0, 2216, 62, 80, 1.0},
  {0, 2217, 56, 112, 1.0},  {0, 2217, 61, 82, 1.0},  {0, 2217, 63, 80, 1.0},  {0, 2218, 56, 113, 1.0},  {0, 2218, 61, 83, 1.0},  {0, 2218, 64, 80, 1.0},
  {0, 2219, 56, 114, 1.0},  {0, 2219, 62, 81, 1.0},  {0, 2220, 56, 115, 1.0},  {0, 2220, 62, 82, 1.0},  {0, 2220, 63, 81, 1.0},  {0, 2221, 56, 116, 1.0},
  {0, 2221, 62, 83, 1.0},  {0, 2221, 64, 81, 1.0},  {0, 2222, 56, 117, 1.0},  {0, 2222, 63, 82, 1.0},  {0, 2223, 56, 118, 1.0},  {0, 2223, 63, 83, 1.0},
  {0, 2223, 64, 82, 1.0},  {0, 2224, 56, 119, 1.0},  {0, 2224, 64, 83, 1.0},  {0, 2225, 57, 84, 1.0},  {0, 2226, 57, 85, 1.0},  {0, 2226, 58, 84, 1.0},
  {0, 2227, 57, 86, 1.0},  {0, 2227, 59, 84, 1.0},  {0, 2228, 57, 87, 1.0},  {0, 2228, 60, 84, 1.0},  {0, 2229, 57, 88, 1.0},  {0, 2229, 61, 84, 1.0},
  {0, 2230, 57, 89, 1.0},  {0, 2230, 62, 84, 1.0},  {0, 2231, 57, 90, 1.0},  {0, 2231, 63, 84, 1.0},  {0, 2232, 57, 91, 1.0},  {0, 2232, 64, 84, 1.0},
  {0, 2233, 57, 92, 1.0},  {0, 2233, 58, 85, 1.0},  {0, 2234, 57, 93, 1.0},  {0, 2234, 58, 86, 1.0},  {0, 2234, 59, 85, 1.0},  {0, 2235, 57, 94, 1.0},
  {0, 2235, 58, 87, 1.0},  {0, 2235, 60, 85, 1.0},  {0, 2236, 57, 95, 1.0},  {0, 2236, 58, 88, 1.0},  {0, 2236, 61, 85, 1.0},  {0, 2237, 57, 96, 1.0},
  {0, 2237, 58, 89, 1.0},  {0, 2237, 62, 85, 1.0},  {0, 2238, 57, 97, 1.0},  {0, 2238, 58, 90, 1.0},  {0, 2238, 63, 85, 1.0},  {0, 2239, 57, 98, 1.0},
  {0, 2239, 58, 91, 1.0},  {0, 2239, 64, 85, 1.0},  {0, 2240, 57, 99, 1.0},  {0, 2240, 59, 86, 1.0},  {0, 2241, 57, 100, 1.0},  {0, 2241, 59, 87, 1.0},
  {0, 2241, 60, 86, 1.0},  {0, 2242, 57, 101, 1.0},  {0, 2242, 59, 88, 1.0},  {0, 2242, 61, 86, 1.0},  {0, 2243, 57, 102, 1.0},  {0, 2243, 59, 89, 1.0},
  {0, 2243, 62, 86, 1.0},  {0, 2244, 57, 103, 1.0},  {0, 2244, 59, 90, 1.0},  {0, 2244, 63, 86, 1.0},  {0, 2245, 57, 104, 1.0},  {0, 2245, 59, 91, 1.0},
  {0, 2245, 64, 86, 1.0},  {0, 2246, 57, 105, 1.0},  {0, 2246, 60, 87, 1.0},  {0, 2247, 57, 106, 1.0},  {0, 2247, 60, 88, 1.0},  {0, 2247, 61, 87, 1.0},
  {0, 2248, 57, 107, 1.0},  {0, 2248, 60, 89, 1.0},  {0, 2248, 62, 87, 1.0},  {0, 2249, 57, 108, 1.0},  {0, 2249, 60, 90, 1.0},  {0, 2249, 63, 87, 1.0},
  {0, 2250, 57, 109, 1.0},  {0, 2250, 60, 91, 1.0},  {0, 2250, 64, 87, 1.0},  {0, 2251, 57, 110, 1.0},  {0, 2251, 61, 88, 1.0},  {0, 2252, 57, 111, 1.0},
  {0, 2252, 61, 89, 1.0},  {0, 2252, 62, 88, 1.0},  {0, 2253, 57, 112, 1.0},  {0, 2253, 61, 90, 1.0},  {0, 2253, 63, 88, 1.0},  {0, 2254, 57, 113, 1.0},
  {0, 2254, 61, 91, 1.0},  {0, 2254, 64, 88, 1.0},  {0, 2255, 57, 114, 1.0},  {0, 2255, 62, 89, 1.0},  {0, 2256, 57, 115, 1.0},  {0, 2256, 62, 90, 1.0},
  {0, 2256, 63, 89, 1.0},  {0, 2257, 57, 116, 1.0},  {0, 2257, 62, 91, 1.0},  {0, 2257, 64, 89, 1.0},  {0, 2258, 57, 117, 1.0},  {0, 2258, 63, 90, 1.0},
  {0, 2259, 57, 118, 1.0},  {0, 2259, 63, 91, 1.0},  {0, 2259, 64, 90, 1.0},  {0, 2260, 57, 119, 1.0},  {0, 2260, 64, 91, 1.0},  {0, 2261, 58, 92, 1.0},
  {0, 2262, 58, 93, 1.0},  {0, 2262, 59, 92, 1.0},  {0, 2263, 58, 94, 1.0},  {0, 2263, 60, 92, 1.0},  {0, 2264, 58, 95, 1.0},  {0, 2264, 61, 92, 1.0},
  {0, 2265, 58, 96, 1.0},  {0, 2265, 62, 92, 1.0},  {0, 2266, 58, 97, 1.0},  {0, 2266, 63, 92, 1.0},  {0, 2267, 58, 98, 1.0},  {0, 2267, 64, 92, 1.0},
  {0, 2268, 58, 99, 1.0},  {0, 2268, 59, 93, 1.0},  {0, 2269, 58, 100, 1.0},  {0, 2269, 59, 94, 1.0},  {0, 2269, 60, 93, 1.0},  {0, 2270, 58, 101, 1.0},
  {0, 2270, 59, 95, 1.0},  {0, 2270, 61, 93, 1.0},  {0, 2271, 58, 102, 1.0},  {0, 2271, 59, 96, 1.0},  {0, 2271, 62, 93, 1.0},  {0, 2272, 58, 103, 1.0},
  {0, 2272, 59, 97, 1.0},  {0, 2272, 63, 93, 1.0},  {0, 2273, 58, 104, 1.0},  {0, 2273, 59, 98, 1.0},  {0, 2273, 64, 93, 1.0},  {0, 2274, 58, 105, 1.0},
  {0, 2274, 60, 94, 1.0},  {0, 2275, 58, 106, 1.0},  {0, 2275, 60, 95, 1.0},  {0, 2275, 61, 94, 1.0},  {0, 2276, 58, 107, 1.0},  {0, 2276, 60, 96, 1.0},
  {0, 2276, 62, 94, 1.0},  {0, 2277, 58, 108, 1.0},  {0, 2277, 60, 97, 1.0},  {0, 2277, 63, 94, 1.0},  {0, 2278, 58, 109, 1.0},  {0, 2278, 60, 98, 1.0},
  {0, 2278, 64, 94, 1.0},  {0, 2279, 58, 110, 1.0},  {0, 2279, 61, 95, 1.0},  {0, 2280, 58, 111, 1.0},  {0, 2280, 61, 96, 1.0},  {0, 2280, 62, 95, 1.0},
  {0, 2281, 58, 112, 1.0},  {0, 2281, 61, 97, 1.0},  {0, 2281, 63, 95, 1.0},  {0, 2282, 58, 113, 1.0},  {0, 2282, 61, 98, 1.0},  {0, 2282, 64, 95, 1.0},
  {0, 2283, 58, 114, 1.0},  {0, 2283, 62, 96, 1.0},  {0, 2284, 58, 115, 1.0},  {0, 2284, 62, 97, 1.0},  {0, 2284, 63, 96, 1.0},  {0, 2285, 58, 116, 1.0},
  {0, 2285, 62, 98, 1.0},  {0, 2285, 64, 96, 1.0},  {0, 2286, 58, 117, 1.0},  {0, 2286, 63, 97, 1.0},  {0, 2287, 58, 118, 1.0},  {0, 2287, 63, 98, 1.0},
  {0, 2287, 64, 97, 1.0},  {0, 2288, 58, 119, 1.0},  {0, 2288, 64, 98, 1.0},  {0, 2289, 59, 99, 1.0},  {0, 2290, 59, 100, 1.0},  {0, 2290, 60, 99, 1.0},
  {0, 2291, 59, 101, 1.0},  {0, 2291, 61, 99, 1.0},  {0, 2292, 59, 102, 1.0},  {0, 2292, 62, 99, 1.0},  {0, 2293, 59, 103, 1.0},  {0, 2293, 63, 99, 1.0},
  {0, 2294, 59, 104, 1.0},  {0, 2294, 64, 99, 1.0},  {0, 2295, 59, 105, 1.0},  {0, 2295, 60, 100, 1.0},  {0, 2296, 59, 106, 1.0},  {0, 2296, 60, 101, 1.0},
  {0, 2296, 61, 100, 1.0},  {0, 2297, 59, 107, 1.0},  {0, 2297, 60, 102, 1.0},  {0, 2297, 62, 100, 1.0},  {0, 2298, 59, 108, 1.0},  {0, 2298, 60, 103, 1.0},
  {0, 2298, 63, 100, 1.0},  {0, 2299, 59, 109, 1.0},  {0, 2299, 60, 104, 1.0},  {0, 2299, 64, 100, 1.0},  {0, 2300, 59, 110, 1.0},  {0, 2300, 61, 101, 1.0},
  {0, 2301, 59, 111, 1.0},  {0, 2301, 61, 102, 1.0},  {0, 2301, 62, 101, 1.0},  {0, 2302, 59, 112, 1.0},  {0, 2302, 61, 103, 1.0},  {0, 2302, 63, 101, 1.0},
  {0, 2303, 59, 113, 1.0},  {0, 2303, 61, 104, 1.0},  {0, 2303, 64, 101, 1.0},  {0, 2304, 59, 114, 1.0},  {0, 2304, 62, 102, 1.0},  {0, 2305, 59, 115, 1.0},
  {0, 2305, 62, 103, 1.0},  {0, 2305, 63, 102, 1.0},  {0, 2306, 59, 116, 1.0},  {0, 2306, 62, 104, 1.0},  {0, 2306, 64, 102, 1.0},  {0, 2307, 59, 117, 1.0},
  {0, 2307, 63, 103, 1.0},  {0, 2308, 59, 118, 1.0},  {0, 2308, 63, 104, 1.0},  {0, 2308, 64, 103, 1.0},  {0, 2309, 59, 119, 1.0},  {0, 2309, 64, 104, 1.0},
  {0, 2310, 60, 105, 1.0},  {0, 2311, 60, 106, 1.0},  {0, 2311, 61, 105, 1.0},  {0, 2312, 60, 107, 1.0},  {0, 2312, 62, 105, 1.0},  {0, 2313, 60, 108, 1.0},
  {0, 2313, 63, 105, 1.0},  {0, 2314, 60, 109, 1.0},  {0, 2314, 64, 105, 1.0},  {0, 2315, 60, 110, 1.0},  {0, 2315, 61, 106, 1.0},  {0, 2316, 60, 111, 1.0},
  {0, 2316, 61, 107, 1.0},  {0, 2316, 62, 106, 1.0},  {0, 2317, 60, 112, 1.0},  {0, 2317, 61, 108, 1.0},  {0, 2317, 63, 106, 1.0},  {0, 2318, 60, 113, 1.0},
  {0, 2318, 61, 109, 1.0},  {0, 2318, 64, 106, 1.0},  {0, 2319, 60, 114, 1.0},  {0, 2319, 62, 107, 1.0},  {0, 2320, 60, 115, 1.0},  {0, 2320, 62, 108, 1.0},
  {0, 2320, 63, 107, 1.0},  {0, 2321, 60, 116, 1.0},  {0, 2321, 62, 109, 1.0},  {0, 2321, 64, 107, 1.0},  {0, 2322, 60, 117, 1.0},  {0, 2322, 63, 108, 1.0},
  {0, 2323, 60, 118, 1.0},  {0, 2323, 63, 109, 1.0},  {0, 2323, 64, 108, 1.0},  {0, 2324, 60, 119, 1.0},  {0, 2324, 64, 109, 1.0},  {0, 2325, 61, 110, 1.0},
  {0, 2326, 61, 111, 1.0},  {0, 2326, 62, 110, 1.0},  {0, 2327, 61, 112, 1.0},  {0, 2327, 63, 110, 1.0},  {0, 2328, 61, 113, 1.0},  {0, 2328, 64, 110, 1.0},
  {0, 2329, 61, 114, 1.0},  {0, 2329, 62, 111, 1.0},  {0, 2330, 61, 115, 1.0},  {0, 2330, 62, 112, 1.0},  {0, 2330, 63, 111, 1.0},  {0, 2331, 61, 116, 1.0},
  {0, 2331, 62, 113, 1.0},  {0, 2331, 64, 111, 1.0},  {0, 2332, 61, 117, 1.0},  {0, 2332, 63, 112, 1.0},  {0, 2333, 61, 118, 1.0},  {0, 2333, 63, 113, 1.0},
  {0, 2333, 64, 112, 1.0},  {0, 2334, 61, 119, 1.0},  {0, 2334, 64, 113, 1.0},  {0, 2335, 62, 114, 1.0},  {0, 2336, 62, 115, 1.0},  {0, 2336, 63, 114, 1.0},
  {0, 2337, 62, 116, 1.0},  {0, 2337, 64, 114, 1.0},  {0, 2338, 62, 117, 1.0},  {0, 2338, 63, 115, 1.0},  {0, 2339, 62, 118, 1.0},  {0, 2339, 63, 116, 1.0},
  {0, 2339, 64, 115, 1.0},  {0, 2340, 62, 119, 1.0},  {0, 2340, 64, 116, 1.0},  {0, 2341, 63, 117, 1.0},  {0, 2342, 63, 118, 1.0},  {0, 2342, 64, 117, 1.0},
  {0, 2343, 63, 119, 1.0},  {0, 2343, 64, 118, 1.0},  {0, 2344, 64, 119, 1.0},  {0, 2345, 65, 65, 1.0},  {0, 2346, 65, 66, 1.0},  {0, 2347, 65, 67, 1.0},
  {0, 2348, 65, 68, 1.0},  {0, 2349, 65, 69, 1.0},  {0, 2350, 65, 70, 1.0},  {0, 2351, 65, 71, 1.0},  {0, 2352, 65, 72, 1.0},  {0, 2353, 65, 73, 1.0},
  {0, 2354, 65, 74, 1.0},  {0, 2355, 65, 75, 1.0},  {0, 2355, 66, 66, 1.0},  {0, 2356, 65, 76, 1.0},  {0, 2356, 66, 67, 1.0},  {0, 2357, 65, 77, 1.0},
  {0, 2357, 66, 68, 1.0},  {0, 2358, 65, 78, 1.0},  {0, 2358, 66, 69, 1.0},  {0, 2359, 65, 79, 1.0},  {0, 2359, 66, 70, 1.0},  {0, 2360, 65, 80, 1.0},
  {0, 2360, 66, 71, 1.0},  {0, 2361, 65, 81, 1.0},  {0, 2361, 66, 72, 1.0},  {0, 2362, 65, 82, 1.0},  {0, 2362, 66, 73, 1.0},  {0, 2363, 65, 83, 1.0},
  {0, 2363, 66, 74, 1.0},  {0, 2364, 65, 84, 1.0},  {0, 2364, 67, 67, 1.0},  {0, 2365, 65, 85, 1.0},  {0, 2365, 67, 68, 1.0},  {0, 2366, 65, 86, 1.0},
  {0, 2366, 67, 69, 1.0},  {0, 2367, 65, 87, 1.0},  {0, 2367, 67, 70, 1.0},  {0, 2368, 65, 88, 1.0},  {0, 2368, 67, 71, 1.0},  {0, 2369, 65, 89, 1.0},
  {0, 2369, 67, 72, 1.0},  {0, 2370, 65, 90, 1.0},  {0, 2370, 67, 73, 1.0},  {0, 2371, 65, 91, 1.0},  {0, 2371, 67, 74, 1.0},  {0, 2372, 65, 92, 1.0},
  {0, 2372, 68, 68, 1.0},  {0, 2373, 65, 93, 1.0},  {0, 2373, 68, 69, 1.0},  {0, 2374, 65, 94, 1.0},  {0, 2374, 68, 70, 1.0},  {0, 2375, 65, 95, 1.0},
  {0, 2375, 68, 71, 1.0},  {0, 2376, 65, 96, 1.0},  {0, 2376, 68, 72, 1.0},  {0, 2377, 65, 97, 1.0},  {0, 2377, 68, 73, 1.0},  {0, 2378, 65, 98, 1.0},
  {0, 2378, 68, 74, 1.0},  {0, 2379, 65, 99, 1.0},  {0, 2379, 69, 69, 1.0},  {0, 2380, 65, 100, 1.0},  {0, 2380, 69, 70, 1.0},  {0, 2381, 65, 101, 1.0},
  {0, 2381, 69, 71, 1.0},  {0, 2382, 65, 102, 1.0},  {0, 2382, 69, 72, 1.0},  {0, 2383, 65, 103, 1.0},  {0, 2383, 69, 73, 1.0},  {0, 2384, 65, 104, 1.0},
  {0, 2384, 69, 74, 1.0},  {0, 2385, 65, 105, 1.0},  {0, 2385, 70, 70, 1.0},  {0, 2386, 65, 106, 1.0},  {0, 2386, 70, 71, 1.0},  {0, 2387, 65, 107, 1.0},
  {0, 2387, 70, 72, 1.0},  {0, 2388, 65, 108, 1.0},  {0, 2388, 70, 73, 1.0},  {0, 2389, 65, 109, 1.0},  {0, 2389, 70, 74, 1.0},  {0, 2390, 65, 110, 1.0},
  {0, 2390, 71, 71, 1.0},  {0, 2391, 65, 111, 1.0},  {0, 2391, 71, 72, 1.0},  {0, 2392, 65, 112, 1.0},  {0, 2392, 71, 73, 1.0},  {0, 2393, 65, 113, 1.0},
  {0, 2393, 71, 74, 1.0},  {0, 2394, 65, 114, 1.0},  {0, 2394, 72, 72, 1.0},  {0, 2395, 65, 115, 1.0},  {0, 2395, 72, 73, 1.0},  {0, 2396, 65, 116, 1.0},
  {0, 2396, 72, 74, 1.0},  {0, 2397, 65, 117, 1.0},  {0, 2397, 73, 73, 1.0},  {0, 2398, 65, 118, 1.0},  {0, 2398, 73, 74, 1.0},  {0, 2399, 65, 119, 1.0},
  {0, 2399, 74, 74, 1.0},  {0, 2400, 66, 75, 1.0},  {0, 2401, 66, 76, 1.0},  {0, 2401, 67, 75, 1.0},  {0, 2402, 66, 77, 1.0},  {0, 2402, 68, 75, 1.0},
  {0, 2403, 66, 78, 1.0},  {0, 2403, 69, 75, 1.0},  {0, 2404, 66, 79, 1.0},  {0, 2404, 70, 75, 1.0},  {0, 2405, 66, 80, 1.0},  {0, 2405, 71, 75, 1.0},
  {0, 2406, 66, 81, 1.0},  {0, 2406, 72, 75, 1.0},  {0, 2407, 66, 82, 1.0},  {0, 2407, 73, 75, 1.0},  {0, 2408, 66, 83, 1.0},  {0, 2408, 74, 75, 1.0},
  {0, 2409, 66, 84, 1.0},  {0, 2409, 67, 76, 1.0},  {0, 2410, 66, 85, 1.0},  {0, 2410, 67, 77, 1.0},  {0, 2410, 68, 76, 1.0},  {0, 2411, 66, 86, 1.0},
  {0, 2411, 67, 78, 1.0},  {0, 2411, 69, 76, 1.0},  {0, 2412, 66, 87, 1.0},  {0, 2412, 67, 79, 1.0},  {0, 2412, 70, 76, 1.0},  {0, 2413, 66, 88, 1.0},
  {0, 2413, 67, 80, 1.0},  {0, 2413, 71, 76, 1.0},  {0, 2414, 66, 89, 1.0},  {0, 2414, 67, 81, 1.0},  {0, 2414, 72, 76, 1.0},  {0, 2415, 66, 90, 1.0},
  {0, 2415, 67, 82, 1.0},  {0, 2415, 73, 76, 1.0},  {0, 2416, 66, 91, 1.0},  {0, 2416, 67, 83, 1.0},  {0, 2416, 74, 76, 1.0},  {0, 2417, 66, 92, 1.0},
  {0, 2417, 68, 77, 1.0},  {0, 2418, 66, 93, 1.0},  {0, 2418, 68, 78, 1.0},  {0, 2418, 69, 77, 1.0},  {0, 2419, 66, 94, 1.0},  {0, 2419, 68, 79, 1.0},
  {0, 2419, 70, 77, 1.0},  {0, 2420, 66, 95, 1.0},  {0, 2420, 68, 80, 1.0},  {0, 2420, 71, 77, 1.0},  {0, 2421, 66, 96, 1.0},  {0, 2421, 68, 81, 1.0},
  {0, 2421, 72, 77, 1.0},  {0, 2422, 66, 97, 1.0},  {0, 2422, 68, 82, 1.0},  {0, 2422, 73, 77, 1.0},  {0, 2423, 66, 98, 1.0},  {0, 2423, 68, 83, 1.0},
  {0, 2423, 74, 77, 1.0},  {0, 2424, 66, 99, 1.0},  {0, 2424, 69, 78, 1.0},  {0, 2425, 66, 100, 1.0},  {0, 2425, 69, 79, 1.0},  {0, 2425, 70, 78, 1.0},
  {0, 2426, 66, 101, 1.0},  {0, 2426, 69, 80, 1.0},  {0, 2426, 71, 78, 1.0},  {0, 2427, 66, 102, 1.0},  {0, 2427, 69, 81, 1.0},  {0, 2427, 72, 78, 1.0},
  {0, 2428, 66, 103, 1.0},  {0, 2428, 69, 82, 1.0},  {0, 2428, 73, 78, 1.0},  {0, 2429, 66, 104, 1.0},  {0, 2429, 69, 83, 1.0},  {0, 2429, 74, 78, 1.0},
  {0, 2430, 66, 105, 1.0},  {0, 2430, 70, 79, 1.0},  {0, 2431, 66, 106, 1.0},  {0, 2431, 70, 80, 1.0},  {0, 2431, 71, 79, 1.0},  {0, 2432, 66, 107, 1.0},
  {0, 2432, 70, 81, 1.0},  {0, 2432, 72, 79, 1.0},  {0, 2433, 66, 108, 1.0},  {0, 2433, 70, 82, 1.0},  {0, 2433, 73, 79, 1.0},  {0, 2434, 66, 109, 1.0},
  {0, 2434, 70, 83, 1.0},  {0, 2434, 74, 79, 1.0},  {0, 2435, 66, 110, 1.0},  {0, 2435, 71, 80, 1.0},  {0, 2436, 66, 111, 1.0},  {0, 2436, 71, 81, 1.0},
  {0, 2436, 72, 80, 1.0},  {0, 2437, 66, 112, 1.0},  {0, 2437, 71, 82, 1.0},  {0, 2437, 73, 80, 1.0},  {0, 2438, 66, 113, 1.0},  {0, 2438, 71, 83, 1.0},
  {0, 2438, 74, 80, 1.0},  {0, 2439, 66, 114, 1.0},  {0, 2439, 72, 81, 1.0},  {0, 2440, 66, 115, 1.0},  {0, 2440, 72, 82, 1.0},  {0, 2440, 73, 81, 1.0},
  {0, 2441, 66, 116, 1.0},  {0, 2441, 72, 83, 1.0},  {0, 2441, 74, 81, 1.0},  {0, 2442, 66, 117, 1.0},  {0, 2442, 73, 82, 1.0},  {0, 2443, 66, 118, 1.0},
  {0, 2443, 73, 83, 1.0},  {0, 2443, 74, 82, 1.0},  {0, 2444, 66, 119, 1.0},  {0, 2444, 74, 83, 1.0},  {0, 2445, 67, 84, 1.0},  {0, 2446, 67, 85, 1.0},
  {0, 2446, 68, 84, 1.0},  {0, 2447, 67, 86, 1.0},  {0, 2447, 69, 84, 1.0},  {0, 2448, 67, 87, 1.0},  {0, 2448, 70, 84, 1.0},  {0, 2449, 67, 88, 1.0},
  {0, 2449, 71, 84, 1.0},  {0, 2450, 67, 89, 1.0},  {0, 2450, 72, 84, 1.0},  {0, 2451, 67, 90, 1.0},  {0, 2451, 73, 84, 1.0},  {0, 2452, 67, 91, 1.0},
  {0, 2452, 74, 84, 1.0},  {0, 2453, 67, 92, 1.0},  {0, 2453, 68, 85, 1.0},  {0, 2454, 67, 93, 1.0},  {0, 2454, 68, 86, 1.0},  {0, 2454, 69, 85, 1.0},
  {0, 2455, 67, 94, 1.0},  {0, 2455, 68, 87, 1.0},  {0, 2455, 70, 85, 1.0},  {0, 2456, 67, 95, 1.0},  {0, 2456, 68, 88, 1.0},  {0, 2456, 71, 85, 1.0},
  {0, 2457, 67, 96, 1.0},  {0, 2457, 68, 89, 1.0},  {0, 2457, 72, 85, 1.0},  {0, 2458, 67, 97, 1.0},  {0, 2458, 68, 90, 1.0},  {0, 2458, 73, 85, 1.0},
  {0, 2459, 67, 98, 1.0},  {0, 2459, 68, 91, 1.0},  {0, 2459, 74, 85, 1.0},  {0, 2460, 67, 99, 1.0},  {0, 2460, 69, 86, 1.0},  {0, 2461, 67, 100, 1.0},
  {0, 2461, 69, 87, 1.0},  {0, 2461, 70, 86, 1.0},  {0, 2462, 67, 101, 1.0},  {0, 2462, 69, 88, 1.0},  {0, 2462, 71, 86, 1.0},  {0, 2463, 67, 102, 1.0},
  {0, 2463, 69, 89, 1.0},  {0, 2463, 72, 86, 1.0},  {0, 2464, 67, 103, 1.0},  {0, 2464, 69, 90, 1.0},  {0, 2464, 73, 86, 1.0},  {0, 2465, 67, 104, 1.0},
  {0, 2465, 69, 91, 1.0},  {0, 2465, 74, 86, 1.0},  {0, 2466, 67, 105, 1.0},  {0, 2466, 70, 87, 1.0},  {0, 2467, 67, 106, 1.0},  {0, 2467, 70, 88, 1.0},
  {0, 2467, 71, 87, 1.0},  {0, 2468, 67, 107, 1.0},  {0, 2468, 70, 89, 1.0},  {0, 2468, 72, 87, 1.0},  {0, 2469, 67, 108, 1.0},  {0, 2469, 70, 90, 1.0},
  {0, 2469, 73, 87, 1.0},  {0, 2470, 67, 109, 1.0},  {0, 2470, 70, 91, 1.0},  {0, 2470, 74, 87, 1.0},  {0, 2471, 67, 110, 1.0},  {0, 2471, 71, 88, 1.0},
  {0, 2472, 67, 111, 1.0},  {0, 2472, 71, 89, 1.0},  {0, 2472, 72, 88, 1.0},  {0, 2473, 67, 112, 1.0},  {0, 2473, 71, 90, 1.0},  {0, 2473, 73, 88, 1.0},
  {0, 2474, 67, 113, 1.0},  {0, 2474, 71, 91, 1.0},  {0, 2474, 74, 88, 1.0},  {0, 2475, 67, 114, 1.0},  {0, 2475, 72, 89, 1.0},  {0, 2476, 67, 115, 1.0},
  {0, 2476, 72, 90, 1.0},  {0, 2476, 73, 89, 1.0},  {0, 2477, 67, 116, 1.0},  {0, 2477, 72, 91, 1.0},  {0, 2477, 74, 89, 1.0},  {0, 2478, 67, 117, 1.0},
  {0, 2478, 73, 90, 1.0},  {0, 2479, 67, 118, 1.0},  {0, 2479, 73, 91, 1.0},  {0, 2479, 74, 90, 1.0},  {0, 2480, 67, 119, 1.0},  {0, 2480, 74, 91, 1.0},
  {0, 2481, 68, 92, 1.0},  {0, 2482, 68, 93, 1.0},  {0, 2482, 69, 92, 1.0},  {0, 2483, 68, 94, 1.0},  {0, 2483, 70, 92, 1.0},  {0, 2484, 68, 95, 1.0},
  {0, 2484, 71, 92, 1.0},  {0, 2485, 68, 96, 1.0},  {0, 2485, 72, 92, 1.0},  {0, 2486, 68, 97, 1.0},  {0, 2486, 73, 92, 1.0},  {0, 2487, 68, 98, 1.0},
  {0, 2487, 74, 92, 1.0},  {0, 2488, 68, 99, 1.0},  {0, 2488, 69, 93, 1.0},  {0, 2489, 68, 100, 1.0},  {0, 2489, 69, 94, 1.0},  {0, 2489, 70, 93, 1.0},
  {0, 2490, 68, 101, 1.0},  {0, 2490, 69, 95, 1.0},  {0, 2490, 71, 93, 1.0},  {0, 2491, 68, 102, 1.0},  {0, 2491, 69, 96, 1.0},  {0, 2491, 72, 93, 1.0},
  {0, 2492, 68, 103, 1.0},  {0, 2492, 69, 97, 1.0},  {0, 2492, 73, 93, 1.0},  {0, 2493, 68, 104, 1.0},  {0, 2493, 69, 98, 1.0},  {0, 2493, 74, 93, 1.0},
  {0, 2494, 68, 105, 1.0},  {0, 2494, 70, 94, 1.0},  {0, 2495, 68, 106, 1.0},  {0, 2495, 70, 95, 1.0},  {0, 2495, 71, 94, 1.0},  {0, 2496, 68, 107, 1.0},
  {0, 2496, 70, 96, 1.0},  {0, 2496, 72, 94, 1.0},  {0, 2497, 68, 108, 1.0},  {0, 2497, 70, 97, 1.0},  {0, 2497, 73, 94, 1.0},  {0, 2498, 68, 109, 1.0},
  {0, 2498, 70, 98, 1.0},  {0, 2498, 74, 94, 1.0},  {0, 2499, 68, 110, 1.0},  {0, 2499, 71, 95, 1.0},  {0, 2500, 68, 111, 1.0},  {0, 2500, 71, 96, 1.0},
  {0, 2500, 72, 95, 1.0},  {0, 2501, 68, 112, 1.0},  {0, 2501, 71, 97, 1.0},  {0, 2501, 73, 95, 1.0},  {0, 2502, 68, 113, 1.0},  {0, 2502, 71, 98, 1.0},
  {0, 2502, 74, 95, 1.0},  {0, 2503, 68, 114, 1.0},  {0, 2503, 72, 96, 1.0},  {0, 2504, 68, 115, 1.0},  {0, 2504, 72, 97, 1.0},  {0, 2504, 73, 96, 1.0},
  {0, 2505, 68, 116, 1.0},  {0, 2505, 72, 98, 1.0},  {0, 2505, 74, 96, 1.0},  {0, 2506, 68, 117, 1.0},  {0, 2506, 73, 97, 1.0},  {0, 2507, 68, 118, 1.0},
  {0, 2507, 73, 98, 1.0},  {0, 2507, 74, 97, 1.0},  {0, 2508, 68, 119, 1.0},  {0, 2508, 74, 98, 1.0},  {0, 2509, 69, 99, 1.0},  {0, 2510, 69, 100, 1.0},
  {0, 2510, 70, 99, 1.0},  {0, 2511, 69, 101, 1.0},  {0, 2511, 71, 99, 1.0},  {0, 2512, 69, 102, 1.0},  {0, 2512, 72, 99, 1.0},  {0, 2513, 69, 103, 1.0},
  {0, 2513, 73, 99, 1.0},  {0, 2514, 69, 104, 1.0},  {0, 2514, 74, 99, 1.0},  {0, 2515, 69, 105, 1.0},  {0, 2515, 70, 100, 1.0},  {0, 2516, 69, 106, 1.0},
  {0, 2516, 70, 101, 1.0},  {0, 2516, 71, 100, 1.0},  {0, 2517, 69, 107, 1.0},  {0, 2517, 70, 102, 1.0},  {0, 2517, 72, 100, 1.0},  {0, 2518, 69, 108, 1.0},
  {0, 2518, 70, 103, 1.0},  {0, 2518, 73, 100, 1.0},  {0, 2519, 69, 109, 1.0},  {0, 2519, 70, 104, 1.0},  {0, 2519, 74, 100, 1.0},  {0, 2520, 69, 110, 1.0},
  {0, 2520, 71, 101, 1.0},  {0, 2521, 69, 111, 1.0},  {0, 2521, 71, 102, 1.0},  {0, 2521, 72, 101, 1.0},  {0, 2522, 69, 112, 1.0},  {0, 2522, 71, 103, 1.0},
  {0, 2522, 73, 101, 1.0},  {0, 2523, 69, 113, 1.0},  {0, 2523, 71, 104, 1.0},  {0, 2523, 74, 101, 1.0},  {0, 2524, 69, 114, 1.0},  {0, 2524, 72, 102, 1.0},
  {0, 2525, 69, 115, 1.0},  {0, 2525, 72, 103, 1.0},  {0, 2525, 73, 102, 1.0},  {0, 2526, 69, 116, 1.0},  {0, 2526, 72, 104, 1.0},  {0, 2526, 74, 102, 1.0},
  {0, 2527, 69, 117, 1.0},  {0, 2527, 73, 103, 1.0},  {0, 2528, 69, 118, 1.0},  {0, 2528, 73, 104, 1.0},  {0, 2528, 74, 103, 1.0},  {0, 2529, 69, 119, 1.0},
  {0, 2529, 74, 104, 1.0},  {0, 2530, 70, 105, 1.0},  {0, 2531, 70, 106, 1.0},  {0, 2531, 71, 105, 1.0},  {0, 2532, 70, 107, 1.0},  {0, 2532, 72, 105, 1.0},
  {0, 2533, 70, 108, 1.0},  {0, 2533, 73, 105, 1.0},  {0, 2534, 70, 109, 1.0},  {0, 2534, 74, 105, 1.0},  {0, 2535, 70, 110, 1.0},  {0, 2535, 71, 106, 1.0},
  {0, 2536, 70, 111, 1.0},  {0, 2536, 71, 107, 1.0},  {0, 2536, 72, 106, 1.0},  {0, 2537, 70, 112, 1.0},  {0, 2537, 71, 108, 1.0},  {0, 2537, 73, 106, 1.0},
  {0, 2538, 70, 113, 1.0},  {0, 2538, 71, 109, 1.0},  {0, 2538, 74, 106, 1.0},  {0, 2539, 70, 114, 1.0},  {0, 2539, 72, 107, 1.0},  {0, 2540, 70, 115, 1.0},
  {0, 2540, 72, 108, 1.0},  {0, 2540, 73, 107, 1.0},  {0, 2541, 70, 116, 1.0},  {0, 2541, 72, 109, 1.0},  {0, 2541, 74, 107, 1.0},  {0, 2542, 70, 117, 1.0},
  {0, 2542, 73, 108, 1.0},  {0, 2543, 70, 118, 1.0},  {0, 2543, 73, 109, 1.0},  {0, 2543, 74, 108, 1.0},  {0, 2544, 70, 119, 1.0},  {0, 2544, 74, 109, 1.0},
  {0, 2545, 71, 110, 1.0},  {0, 2546, 71, 111, 1.0},  {0, 2546, 72, 110, 1.0},  {0, 2547, 71, 112, 1.0},  {0, 2547, 73, 110, 1.0},  {0, 2548, 71, 113, 1.0},
  {0, 2548, 74, 110, 1.0},  {0, 2549, 71, 114, 1.0},  {0, 2549, 72, 111, 1.0},  {0, 2550, 71, 115, 1.0},  {0, 2550, 72, 112, 1.0},  {0, 2550, 73, 111, 1.0},
  {0, 2551, 71, 116, 1.0},  {0, 2551, 72, 113, 1.0},  {0, 2551, 74, 111, 1.0},  {0, 2552, 71, 117, 1.0},  {0, 2552, 73, 112, 1.0},  {0, 2553, 71, 118, 1.0},
  {0, 2553, 73, 113, 1.0},  {0, 2553, 74, 112, 1.0},  {0, 2554, 71, 119, 1.0},  {0, 2554, 74, 113, 1.0},  {0, 2555, 72, 114, 1.0},  {0, 2556, 72, 115, 1.0},
  {0, 2556, 73, 114, 1.0},  {0, 2557, 72, 116, 1.0},  {0, 2557, 74, 114, 1.0},  {0, 2558, 72, 117, 1.0},  {0, 2558, 73, 115, 1.0},  {0, 2559, 72, 118, 1.0},
  {0, 2559, 73, 116, 1.0},  {0, 2559, 74, 115, 1.0},  {0, 2560, 72, 119, 1.0},  {0, 2560, 74, 116, 1.0},  {0, 2561, 73, 117, 1.0},  {0, 2562, 73, 118, 1.0},
  {0, 2562, 74, 117, 1.0},  {0, 2563, 73, 119, 1.0},  {0, 2563, 74, 118, 1.0},  {0, 2564, 74, 119, 1.0},  {0, 2565, 75, 75, 1.0},  {0, 2566, 75, 76, 1.0},
  {0, 2567, 75, 77, 1.0},  {0, 2568, 75, 78, 1.0},  {0, 2569, 75, 79, 1.0},  {0, 2570, 75, 80, 1.0},  {0, 2571, 75, 81, 1.0},  {0, 2572, 75, 82, 1.0},
  {0, 2573, 75, 83, 1.0},  {0, 2574, 75, 84, 1.0},  {0, 2574, 76, 76, 1.0},  {0, 2575, 75, 85, 1.0},  {0, 2575, 76, 77, 1.0},  {0, 2576, 75, 86, 1.0},
  {0, 2576, 76, 78, 1.0},  {0, 2577, 75, 87, 1.0},  {0, 2577, 76, 79, 1.0},  {0, 2578, 75, 88, 1.0},  {0, 2578, 76, 80, 1.0},  {0, 2579, 75, 89, 1.0},
  {0, 2579, 76, 81, 1.0},  {0, 2580, 75, 90, 1.0},  {0, 2580, 76, 82, 1.0},  {0, 2581, 75, 91, 1.0},  {0, 2581, 76, 83, 1.0},  {0, 2582, 75, 92, 1.0},
  {0, 2582, 77, 77, 1.0},  {0, 2583, 75, 93, 1.0},  {0, 2583, 77, 78, 1.0},  {0, 2584, 75, 94, 1.0},  {0, 2584, 77, 79, 1.0},  {0, 2585, 75, 95, 1.0},
  {0, 2585, 77, 80, 1.0},  {0, 2586, 75, 96, 1.0},  {0, 2586, 77, 81, 1.0},  {0, 2587, 75, 97, 1.0},  {0, 2587, 77, 82, 1.0},  {0, 2588, 75, 98, 1.0},
  {0, 2588, 77, 83, 1.0},  {0, 2589, 75, 99, 1.0},  {0, 2589, 78, 78, 1.0},  {0, 2590, 75, 100, 1.0},  {0, 2590, 78, 79, 1.0},  {0, 2591, 75, 101, 1.0},
  {0, 2591, 78, 80, 1.0},  {0, 2592, 75, 102, 1.0},  {0, 2592, 78, 81, 1.0},  {0, 2593, 75, 103, 1.0},  {0, 2593, 78, 82, 1.0},  {0, 2594, 75, 104, 1.0},
  {0, 2594, 78, 83, 1.0},  {0, 2595, 75, 105, 1.0},  {0, 2595, 79, 79, 1.0},  {0, 2596, 75, 106, 1.0},  {0, 2596, 79, 80, 1.0},  {0, 2597, 75, 107, 1.0},
  {0, 2597, 79, 81, 1.0},  {0, 2598, 75, 108, 1.0},  {0, 2598, 79, 82, 1.0},  {0, 2599, 75, 109, 1.0},  {0, 2599, 79, 83, 1.0},  {0, 2600, 75, 110, 1.0},
  {0, 2600, 80, 80, 1.0},  {0, 2601, 75, 111, 1.0},  {0, 2601, 80, 81, 1.0},  {0, 2602, 75, 112, 1.0},  {0, 2602, 80, 82, 1.0},  {0, 2603, 75, 113, 1.0},
  {0, 2603, 80, 83, 1.0},  {0, 2604, 75, 114, 1.0},  {0, 2604, 81, 81, 1.0},  {0, 2605, 75, 115, 1.0},  {0, 2605, 81, 82, 1.0},  {0, 2606, 75, 116, 1.0},
  {0, 2606, 81, 83, 1.0},  {0, 2607, 75, 117, 1.0},  {0, 2607, 82, 82, 1.0},  {0, 2608, 75, 118, 1.0},  {0, 2608, 82, 83, 1.0},  {0, 2609, 75, 119, 1.0},
  {0, 2609, 83, 83, 1.0},  {0, 2610, 76, 84, 1.0},  {0, 2611, 76, 85, 1.0},  {0, 2611, 77, 84, 1.0},  {0, 2612, 76, 86, 1.0},  {0, 2612, 78, 84, 1.0},
  {0, 2613, 76, 87, 1.0},  {0, 2613, 79, 84, 1.0},  {0, 2614, 76, 88, 1.0},  {0, 2614, 80, 84, 1.0},  {0, 2615, 76, 89, 1.0},  {0, 2615, 81, 84, 1.0},
  {0, 2616, 76, 90, 1.0},  {0, 2616, 82, 84, 1.0},  {0, 2617, 76, 91, 1.0},  {0, 2617, 83, 84, 1.0},  {0, 2618, 76, 92, 1.0},  {0, 2618, 77, 85, 1.0},
  {0, 2619, 76, 93, 1.0},  {0, 2619, 77, 86, 1.0},  {0, 2619, 78, 85, 1.0},  {0, 2620, 76, 94, 1.0},  {0, 2620, 77, 87, 1.0},  {0, 2620, 79, 85, 1.0},
  {0, 2621, 76, 95, 1.0},  {0, 2621, 77, 88, 1.0},  {0, 2621, 80, 85, 1.0},  {0, 2622, 76, 96, 1.0},  {0, 2622, 77, 89, 1.0},  {0, 2622, 81, 85, 1.0},
  {0, 2623, 76, 97, 1.0},  {0, 2623, 77, 90, 1.0},  {0, 2623, 82, 85, 1.0},  {0, 2624, 76, 98, 1.0},  {0, 2624, 77, 91, 1.0},  {0, 2624, 83, 85, 1.0},
  {0, 2625, 76, 99, 1.0},  {0, 2625, 78, 86, 1.0},  {0, 2626, 76, 100, 1.0},  {0, 2626, 78, 87, 1.0},  {0, 2626, 79, 86, 1.0},  {0, 2627, 76, 101, 1.0},
  {0, 2627, 78, 88, 1.0},  {0, 2627, 80, 86, 1.0},  {0, 2628, 76, 102, 1.0},  {0, 2628, 78, 89, 1.0},  {0, 2628, 81, 86, 1.0},  {0, 2629, 76, 103, 1.0},
  {0, 2629, 78, 90, 1.0},  {0, 2629, 82, 86, 1.0},  {0, 2630, 76, 104, 1.0},  {0, 2630, 78, 91, 1.0},  {0, 2630, 83, 86, 1.0},  {0, 2631, 76, 105, 1.0},
  {0, 2631, 79, 87, 1.0},  {0, 2632, 76, 106, 1.0},  {0, 2632, 79, 88, 1.0},  {0, 2632, 80, 87, 1.0},  {0, 2633, 76, 107, 1.0},  {0, 2633, 79, 89, 1.0},
  {0, 2633, 81, 87, 1.0},  {0, 2634, 76, 108, 1.0},  {0, 2634, 79, 90, 1.0},  {0, 2634, 82, 87, 1.0},  {0, 2635, 76, 109, 1.0},  {0, 2635, 79, 91, 1.0},
  {0, 2635, 83, 87, 1.0},  {0, 2636, 76, 110, 1.0},  {0, 2636, 80, 88, 1.0},  {0, 2637, 76, 111, 1.0},  {0, 2637, 80, 89, 1.0},  {0, 2637, 81, 88, 1.0},
  {0, 2638, 76, 112, 1.0},  {0, 2638, 80, 90, 1.0},  {0, 2638, 82, 88, 1.0},  {0, 2639, 76, 113, 1.0},  {0, 2639, 80, 91, 1.0},  {0, 2639, 83, 88, 1.0},
  {0, 2640, 76, 114, 1.0},  {0, 2640, 81, 89, 1.0},  {0, 2641, 76, 115, 1.0},  {0, 2641, 81, 90, 1.0},  {0, 2641, 82, 89, 1.0},  {0, 2642, 76, 116, 1.0},
  {0, 2642, 81, 91, 1.0},  {0, 2642, 83, 89, 1.0},  {0, 2643, 76, 117, 1.0},  {0, 2643, 82, 90, 1.0},  {0, 2644, 76, 118, 1.0},  {0, 2644, 82, 91, 1.0},
  {0, 2644, 83, 90, 1.0},  {0, 2645, 76, 119, 1.0},  {0, 2645, 83, 91, 1.0},  {0, 2646, 77, 92, 1.0},  {0, 2647, 77, 93, 1.0},  {0, 2647, 78, 92, 1.0},
  {0, 2648, 77, 94, 1.0},  {0, 2648, 79, 92, 1.0},  {0, 2649, 77, 95, 1.0},  {0, 2649, 80, 92, 1.0},  {0, 2650, 77, 96, 1.0},  {0, 2650, 81, 92, 1.0},
  {0, 2651, 77, 97, 1.0},  {0, 2651, 82, 92, 1.0},  {0, 2652, 77, 98, 1.0},  {0, 2652, 83, 92, 1.0},  {0, 2653, 77, 99, 1.0},  {0, 2653, 78, 93, 1.0},
  {0, 2654, 77, 100, 1.0},  {0, 2654, 78, 94, 1.0},  {0, 2654, 79, 93, 1.0},  {0, 2655, 77, 101, 1.0},  {0, 2655, 78, 95, 1.0},  {0, 2655, 80, 93, 1.0},
  {0, 2656, 77, 102, 1.0},  {0, 2656, 78, 96, 1.0},  {0, 2656, 81, 93, 1.0},  {0, 2657, 77, 103, 1.0},  {0, 2657, 78, 97, 1.0},  {0, 2657, 82, 93, 1.0},
  {0, 2658, 77, 104, 1.0},  {0, 2658, 78, 98, 1.0},  {0, 2658, 83, 93, 1.0},  {0, 2659, 77, 105, 1.0},  {0, 2659, 79, 94, 1.0},  {0, 2660, 77, 106, 1.0},
  {0, 2660, 79, 95, 1.0},  {0, 2660, 80, 94, 1.0},  {0, 2661, 77, 107, 1.0},  {0, 2661, 79, 96, 1.0},  {0, 2661, 81, 94, 1.0},  {0, 2662, 77, 108, 1.0},
  {0, 2662, 79, 97, 1.0},  {0, 2662, 82, 94, 1.0},  {0, 2663, 77, 109, 1.0},  {0, 2663, 79, 98, 1.0},  {0, 2663, 83, 94, 1.0},  {0, 2664, 77, 110, 1.0},
  {0, 2664, 80, 95, 1.0},  {0, 2665, 77, 111, 1.0},  {0, 2665, 80, 96, 1.0},  {0, 2665, 81, 95, 1.0},  {0, 2666, 77, 112, 1.0},  {0, 2666, 80, 97, 1.0},
  {0, 2666, 82, 95, 1.0},  {0, 2667, 77, 113, 1.0},  {0, 2667, 80, 98, 1.0},  {0, 2667, 83, 95, 1.0},  {0, 2668, 77, 114, 1.0},  {0, 2668, 81, 96, 1.0},
  {0, 2669, 77, 115, 1.0},  {0, 2669, 81, 97, 1.0},  {0, 2669, 82, 96, 1.0},  {0, 2670, 77, 116, 1.0},  {0, 2670, 81, 98, 1.0},  {0, 2670, 83, 96, 1.0},
  {0, 2671, 77, 117, 1.0},  {0, 2671, 82, 97, 1.0},  {0, 2672, 77, 118, 1.0},  {0, 2672, 82, 98, 1.0},  {0, 2672, 83, 97, 1.0},  {0, 2673, 77, 119, 1.0},
  {0, 2673, 83, 98, 1.0},  {0, 2674, 78, 99, 1.0},  {0, 2675, 78, 100, 1.0},  {0, 2675, 79, 99, 1.0},  {0, 2676, 78, 101, 1.0},  {0, 2676, 80, 99, 1.0},
  {0, 2677, 78, 102, 1.0},  {0, 2677, 81, 99, 1.0},  {0, 2678, 78, 103, 1.0},  {0, 2678, 82, 99, 1.0},  {0, 2679, 78, 104, 1.0},  {0, 2679, 83, 99, 1.0},
  {0, 2680, 78, 105, 1.0},  {0, 2680, 79, 100, 1.0},  {0, 2681, 78, 106, 1.0},  {0, 2681, 79, 101, 1.0},  {0, 2681, 80, 100, 1.0},  {0, 2682, 78, 107, 1.0},
  {0, 2682, 79, 102, 1.0},  {0, 2682, 81, 100, 1.0},  {0, 2683, 78, 108, 1.0},  {0, 2683, 79, 103, 1.0},  {0, 2683, 82, 100, 1.0},  {0, 2684, 78, 109, 1.0},
  {0, 2684, 79, 104, 1.0},  {0, 2684, 83, 100, 1.0},  {0, 2685, 78, 110, 1.0},  {0, 2685, 80, 101, 1.0},  {0, 2686, 78, 111, 1.0},  {0, 2686, 80, 102, 1.0},
  {0, 2686, 81, 101, 1.0},  {0, 2687, 78, 112, 1.0},  {0, 2687, 80, 103, 1.0},  {0, 2687, 82, 101, 1.0},  {0, 2688, 78, 113, 1.0},  {0, 2688, 80, 104, 1.0},
  {0, 2688, 83, 101, 1.0},  {0, 2689, 78, 114, 1.0},  {0, 2689, 81, 102, 1.0},  {0, 2690, 78, 115, 1.0},  {0, 2690, 81, 103, 1.0},  {0, 2690, 82, 102, 1.0},
  {0, 2691, 78, 116, 1.0},  {0, 2691, 81, 104, 1.0},  {0, 2691, 83, 102, 1.0},  {0, 2692, 78, 117, 1.0},  {0, 2692, 82, 103, 1.0},  {0, 2693, 78, 118, 1.0},
  {0, 2693, 82, 104, 1.0},  {0, 2693, 83, 103, 1.0},  {0, 2694, 78, 119, 1.0},  {0, 2694, 83, 104, 1.0},  {0, 2695, 79, 105, 1.0},  {0, 2696, 79, 106, 1.0},
  {0, 2696, 80, 105, 1.0},  {0, 2697, 79, 107, 1.0},  {0, 2697, 81, 105, 1.0},  {0, 2698, 79, 108, 1.0},  {0, 2698, 82, 105, 1.0},  {0, 2699, 79, 109, 1.0},
  {0, 2699, 83, 105, 1.0},  {0, 2700, 79, 110, 1.0},  {0, 2700, 80, 106, 1.0},  {0, 2701, 79, 111, 1.0},  {0, 2701, 80, 107, 1.0},  {0, 2701, 81, 106, 1.0},
  {0, 2702, 79, 112, 1.0},  {0, 2702, 80, 108, 1.0},  {0, 2702, 82, 106, 1.0},  {0, 2703, 79, 113, 1.0},  {0, 2703, 80, 109, 1.0},  {0, 2703, 83, 106, 1.0},
  {0, 2704, 79, 114, 1.0},  {0, 2704, 81, 107, 1.0},  {0, 2705, 79, 115, 1.0},  {0, 2705, 81, 108, 1.0},  {0, 2705, 82, 107, 1.0},  {0, 2706, 79, 116, 1.0},
  {0, 2706, 81, 109, 1.0},  {0, 2706, 83, 107, 1.0},  {0, 2707, 79, 117, 1.0},  {0, 2707, 82, 108, 1.0},  {0, 2708, 79, 118, 1.0},  {0, 2708, 82, 109, 1.0},
  {0, 2708, 83, 108, 1.0},  {0, 2709, 79, 119, 1.0},  {0, 2709, 83, 109, 1.0},  {0, 2710, 80, 110, 1.0},  {0, 2711, 80, 111, 1.0},  {0, 2711, 81, 110, 1.0},
  {0, 2712, 80, 112, 1.0},  {0, 2712, 82, 110, 1.0},  {0, 2713, 80, 113, 1.0},  {0, 2713, 83, 110, 1.0},  {0, 2714, 80, 114, 1.0},  {0, 2714, 81, 111, 1.0},
  {0, 2715, 80, 115, 1.0},  {0, 2715, 81, 112, 1.0},  {0, 2715, 82, 111, 1.0},  {0, 2716, 80, 116, 1.0},  {0, 2716, 81, 113, 1.0},  {0, 2716, 83, 111, 1.0},
  {0, 2717, 80, 117, 1.0},  {0, 2717, 82, 112, 1.0},  {0, 2718, 80, 118, 1.0},  {0, 2718, 82, 113, 1.0},  {0, 2718, 83, 112, 1.0},  {0, 2719, 80, 119, 1.0},
  {0, 2719, 83, 113, 1.0},  {0, 2720, 81, 114, 1.0},  {0, 2721, 81, 115, 1.0},  {0, 2721, 82, 114, 1.0},  {0, 2722, 81, 116, 1.0},  {0, 2722, 83, 114, 1.0},
  {0, 2723, 81, 117, 1.0},  {0, 2723, 82, 115, 1.0},  {0, 2724, 81, 118, 1.0},  {0, 2724, 82, 116, 1.0},  {0, 2724, 83, 115, 1.0},  {0, 2725, 81, 119, 1.0},
  {0, 2725, 83, 116, 1.0},  {0, 2726, 82, 117, 1.0},  {0, 2727, 82, 118, 1.0},  {0, 2727, 83, 117, 1.0},  {0, 2728, 82, 119, 1.0},  {0, 2728, 83, 118, 1.0},
  {0, 2729, 83, 119, 1.0},  {0, 2730, 84, 84, 1.0},  {0, 2731, 84, 85, 1.0},  {0, 2732, 84, 86, 1.0},  {0, 2733, 84, 87, 1.0},  {0, 2734, 84, 88, 1.0},
  {0, 2735, 84, 89, 1.0},  {0, 2736, 84, 90, 1.0},  {0, 2737, 84, 91, 1.0},  {0, 2738, 84, 92, 1.0},  {0, 2738, 85, 85, 1.0},  {0, 2739, 84, 93, 1.0},
  {0, 2739, 85, 86, 1.0},  {0, 2740, 84, 94, 1.0},  {0, 2740, 85, 87, 1.0},  {0, 2741, 84, 95, 1.0},  {0, 2741, 85, 88, 1.0},  {0, 2742, 84, 96, 1.0},
  {0, 2742, 85, 89, 1.0},  {0, 2743, 84, 97, 1.0},  {0, 2743, 85, 90, 1.0},  {0, 2744, 84, 98, 1.0},  {0, 2744, 85, 91, 1.0},  {0, 2745, 84, 99, 1.0},
  {0, 2745, 86, 86, 1.0},  {0, 2746, 84, 100, 1.0},  {0, 2746, 86, 87, 1.0},  {0, 2747, 84, 101, 1.0},  {0, 2747, 86, 88, 1.0},  {0, 2748, 84, 102, 1.0},
  {0, 2748, 86, 89, 1.0},  {0, 2749, 84, 103, 1.0},  {0, 2749, 86, 90, 1.0},  {0, 2750, 84, 104, 1.0},  {0, 2750, 86, 91, 1.0},  {0, 2751, 84, 105, 1.0},
  {0, 2751, 87, 87, 1.0},  {0, 2752, 84, 106, 1.0},  {0, 2752, 87, 88, 1.0},  {0, 2753, 84, 107, 1.0},  {0, 2753, 87, 89, 1.0},  {0, 2754, 84, 108, 1.0},
  {0, 2754, 87, 90, 1.0},  {0, 2755, 84, 109, 1.0},  {0, 2755, 87, 91, 1.0},  {0, 2756, 84, 110, 1.0},  {0, 2756, 88, 88, 1.0},  {0, 2757, 84, 111, 1.0},
  {0, 2757, 88, 89, 1.0},  {0, 2758, 84, 112, 1.0},  {0, 2758, 88, 90, 1.0},  {0, 2759, 84, 113, 1.0},  {0, 2759, 88, 91, 1.0},  {0, 2760, 84, 114, 1.0},
  {0, 2760, 89, 89, 1.0},  {0, 2761, 84, 115, 1.0},  {0, 2761, 89, 90, 1.0},  {0, 2762, 84, 116, 1.0},  {0, 2762, 89, 91, 1.0},  {0, 2763, 84, 117, 1.0},
  {0, 2763, 90, 90, 1.0},  {0, 2764, 84, 118, 1.0},  {0, 2764, 90, 91, 1.0},  {0, 2765, 84, 119, 1.0},  {0, 2765, 91, 91, 1.0},  {0, 2766, 85, 92, 1.0},
  {0, 2767, 85, 93, 1.0},  {0, 2767, 86, 92, 1.0},  {0, 2768, 85, 94, 1.0},  {0, 2768, 87, 92, 1.0},  {0, 2769, 85, 95, 1.0},  {0, 2769, 88, 92, 1.0},
  {0, 2770, 85, 96, 1.0},  {0, 2770, 89, 92, 1.0},  {0, 2771, 85, 97, 1.0},  {0, 2771, 90, 92, 1.0},  {0, 2772, 85, 98, 1.0},  {0, 2772, 91, 92, 1.0},
  {0, 2773, 85, 99, 1.0},  {0, 2773, 86, 93, 1.0},  {0, 2774, 85, 100, 1.0},  {0, 2774, 86, 94, 1.0},  {0, 2774, 87, 93, 1.0},  {0, 2775, 85, 101, 1.0},
  {0, 2775, 86, 95, 1.0},  {0, 2775, 88, 93, 1.0},  {0, 2776, 85, 102, 1.0},  {0, 2776, 86, 96, 1.0},  {0, 2776, 89, 93, 1.0},  {0, 2777, 85, 103, 1.0},
  {0, 2777, 86, 97, 1.0},  {0, 2777, 90, 93, 1.0},  {0, 2778, 85, 104, 1.0},  {0, 2778, 86, 98, 1.0},  {0, 2778, 91, 93, 1.0},  {0, 2779, 85, 105, 1.0},
  {0, 2779, 87, 94, 1.0},  {0, 2780, 85, 106, 1.0},  {0, 2780, 87, 95, 1.0},  {0, 2780, 88, 94, 1.0},  {0, 2781, 85, 107, 1.0},  {0, 2781, 87, 96, 1.0},
  {0, 2781, 89, 94, 1.0},  {0, 2782, 85, 108, 1.0},  {0, 2782, 87, 97, 1.0},  {0, 2782, 90, 94, 1.0},  {0, 2783, 85, 109, 1.0},  {0, 2783, 87, 98, 1.0},
  {0, 2783, 91, 94, 1.0},  {0, 2784, 85, 110, 1.0},  {0, 2784, 88, 95, 1.0},  {0, 2785, 85, 111, 1.0},  {0, 2785, 88, 96, 1.0},  {0, 2785, 89, 95, 1.0},
  {0, 2786, 85, 112, 1.0},  {0, 2786, 88, 97, 1.0},  {0, 2786, 90, 95, 1.0},  {0, 2787, 85, 113, 1.0},  {0, 2787, 88, 98, 1.0},  {0, 2787, 91, 95, 1.0},
  {0, 2788, 85, 114, 1.0},  {0, 2788, 89, 96, 1.0},  {0, 2789, 85, 115, 1.0},  {0, 2789, 89, 97, 1.0},  {0, 2789, 90, 96, 1.0},  {0, 2790, 85, 116, 1.0},
  {0, 2790, 89, 98, 1.0},  {0, 2790, 91, 96, 1.0},  {0, 2791, 85, 117, 1.0},  {0, 2791, 90, 97, 1.0},  {0, 2792, 85, 118, 1.0},  {0, 2792, 90, 98, 1.0},
  {0, 2792, 91, 97, 1.0},  {0, 2793, 85, 119, 1.0},  {0, 2793, 91, 98, 1.0},  {0, 2794, 86, 99, 1.0},  {0, 2795, 86, 100, 1.0},  {0, 2795, 87, 99, 1.0},
  {0, 2796, 86, 101, 1.0},  {0, 2796, 88, 99, 1.0},  {0, 2797, 86, 102, 1.0},  {0, 2797, 89, 99, 1.0},  {0, 2798, 86, 103, 1.0},  {0, 2798, 90, 99, 1.0},
  {0, 2799, 86, 104, 1.0},  {0, 2799, 91, 99, 1.0},  {0, 2800, 86, 105, 1.0},  {0, 2800, 87, 100, 1.0},  {0, 2801, 86, 106, 1.0},  {0, 2801, 87, 101, 1.0},
  {0, 2801, 88, 100, 1.0},  {0, 2802, 86, 107, 1.0},  {0, 2802, 87, 102, 1.0},  {0, 2802, 89, 100, 1.0},  {0, 2803, 86, 108, 1.0},  {0, 2803, 87, 103, 1.0},
  {0, 2803, 90, 100, 1.0},  {0, 2804, 86, 109, 1.0},  {0, 2804, 87, 104, 1.0},  {0, 2804, 91, 100, 1.0},  {0, 2805, 86, 110, 1.0},  {0, 2805, 88, 101, 1.0},
  {0, 2806, 86, 111, 1.0},  {0, 2806, 88, 102, 1.0},  {0, 2806, 89, 101, 1.0},  {0, 2807, 86, 112, 1.0},  {0, 2807, 88, 103, 1.0},  {0, 2807, 90, 101, 1.0},
  {0, 2808, 86, 113, 1.0},  {0, 2808, 88, 104, 1.0},  {0, 2808, 91, 101, 1.0},  {0, 2809, 86, 114, 1.0},  {0, 2809, 89, 102, 1.0},  {0, 2810, 86, 115, 1.0},
  {0, 2810, 89, 103, 1.0},  {0, 2810, 90, 102, 1.0},  {0, 2811, 86, 116, 1.0},  {0, 2811, 89, 104, 1.0},  {0, 2811, 91, 102, 1.0},  {0, 2812, 86, 117, 1.0},
  {0, 2812, 90, 103, 1.0},  {0, 2813, 86, 118, 1.0},  {0, 2813, 90, 104, 1.0},  {0, 2813, 91, 103, 1.0},  {0, 2814, 86, 119, 1.0},  {0, 2814, 91, 104, 1.0},
  {0, 2815, 87, 105, 1.0},  {0, 2816, 87, 106, 1.0},  {0, 2816, 88, 105, 1.0},  {0, 2817, 87, 107, 1.0},  {0, 2817, 89, 105, 1.0},  {0, 2818, 87, 108, 1.0},
  {0, 2818, 90, 105, 1.0},  {0, 2819, 87, 109, 1.0},  {0, 2819, 91, 105, 1.0},  {0, 2820, 87, 110, 1.0},  {0, 2820, 88, 106, 1.0},  {0, 2821, 87, 111, 1.0},
  {0, 2821, 88, 107, 1.0},  {0, 2821, 89, 106, 1.0},  {0, 2822, 87, 112, 1.0},  {0, 2822, 88, 108, 1.0},  {0, 2822, 90, 106, 1.0},  {0, 2823, 87, 113, 1.0},
  {0, 2823, 88, 109, 1.0},  {0, 2823, 91, 106, 1.0},  {0, 2824, 87, 114, 1.0},  {0, 2824, 89, 107, 1.0},  {0, 2825, 87, 115, 1.0},  {0, 2825, 89, 108, 1.0},
  {0, 2825, 90, 107, 1.0},  {0, 2826, 87, 116, 1.0},  {0, 2826, 89, 109, 1.0},  {0, 2826, 91, 107, 1.0},  {0, 2827, 87, 117, 1.0},  {0, 2827, 90, 108, 1.0},
  {0, 2828, 87, 118, 1.0},  {0, 2828, 90, 109, 1.0},  {0, 2828, 91, 108, 1.0},  {0, 2829, 87, 119, 1.0},  {0, 2829, 91, 109, 1.0},  {0, 2830, 88, 110, 1.0},
  {0, 2831, 88, 111, 1.0},  {0, 2831, 89, 110, 1.0},  {0, 2832, 88, 112, 1.0},  {0, 2832, 90, 110, 1.0},  {0, 2833, 88, 113, 1.0},  {0, 2833, 91, 110, 1.0},
  {0, 2834, 88, 114, 1.0},  {0, 2834, 89, 111, 1.0},  {0, 2835, 88, 115, 1.0},  {0, 2835, 89, 112, 1.0},  {0, 2835, 90, 111, 1.0},  {0, 2836, 88, 116, 1.0},
  {0, 2836, 89, 113, 1.0},  {0, 2836, 91, 111, 1.0},  {0, 2837, 88, 117, 1.0},  {0, 2837, 90, 112, 1.0},  {0, 2838, 88, 118, 1.0},  {0, 2838, 90, 113, 1.0},
  {0, 2838, 91, 112, 1.0},  {0, 2839, 88, 119, 1.0},  {0, 2839, 91, 113, 1.0},  {0, 2840, 89, 114, 1.0},  {0, 2841, 89, 115, 1.0},  {0, 2841, 90, 114, 1.0},
  {0, 2842, 89, 116, 1.0},  {0, 2842, 91, 114, 1.0},  {0, 2843, 89, 117, 1.0},  {0, 2843, 90, 115, 1.0},  {0, 2844, 89, 118, 1.0},  {0, 2844, 90, 116, 1.0},
  {0, 2844, 91, 115, 1.0},  {0, 2845, 89, 119, 1.0},  {0, 2845, 91, 116, 1.0},  {0, 2846, 90, 117, 1.0},  {0, 2847, 90, 118, 1.0},  {0, 2847, 91, 117, 1.0},
  {0, 2848, 90, 119, 1.0},  {0, 2848, 91, 118, 1.0},  {0, 2849, 91, 119, 1.0},  {0, 2850, 92, 92, 1.0},  {0, 2851, 92, 93, 1.0},  {0, 2852, 92, 94, 1.0},
  {0, 2853, 92, 95, 1.0},  {0, 2854, 92, 96, 1.0},  {0, 2855, 92, 97, 1.0},  {0, 2856, 92, 98, 1.0},  {0, 2857, 92, 99, 1.0},  {0, 2857, 93, 93, 1.0},
  {0, 2858, 92, 100, 1.0},  {0, 2858, 93, 94, 1.0},  {0, 2859, 92, 101, 1.0},  {0, 2859, 93, 95, 1.0},  {0, 2860, 92, 102, 1.0},  {0, 2860, 93, 96, 1.0},
  {0, 2861, 92, 103, 1.0},  {0, 2861, 93, 97, 1.0},  {0, 2862, 92, 104, 1.0},  {0, 2862, 93, 98, 1.0},  {0, 2863, 92, 105, 1.0},  {0, 2863, 94, 94, 1.0},
  {0, 2864, 92, 106, 1.0},  {0, 2864, 94, 95, 1.0},  {0, 2865, 92, 107, 1.0},  {0, 2865, 94, 96, 1.0},  {0, 2866, 92, 108, 1.0},  {0, 2866, 94, 97, 1.0},
  {0, 2867, 92, 109, 1.0},  {0, 2867, 94, 98, 1.0},  {0, 2868, 92, 110, 1.0},  {0, 2868, 95, 95, 1.0},  {0, 2869, 92, 111, 1.0},  {0, 2869, 95, 96, 1.0},
  {0, 2870, 92, 112, 1.0},  {0, 2870, 95, 97, 1.0},  {0, 2871, 92, 113, 1.0},  {0, 2871, 95, 98, 1.0},  {0, 2872, 92, 114, 1.0},  {0, 2872, 96, 96, 1.0},
  {0, 2873, 92, 115, 1.0},  {0, 2873, 96, 97, 1.0},  {0, 2874, 92, 116, 1.0},  {0, 2874, 96, 98, 1.0},  {0, 2875, 92, 117, 1.0},  {0, 2875, 97, 97, 1.0},
  {0, 2876, 92, 118, 1.0},  {0, 2876, 97, 98, 1.0},  {0, 2877, 92, 119, 1.0},  {0, 2877, 98, 98, 1.0},  {0, 2878, 93, 99, 1.0},  {0, 2879, 93, 100, 1.0},
  {0, 2879, 94, 99, 1.0},  {0, 2880, 93, 101, 1.0},  {0, 2880, 95, 99, 1.0},  {0, 2881, 93, 102, 1.0},  {0, 2881, 96, 99, 1.0},  {0, 2882, 93, 103, 1.0},
  {0, 2882, 97, 99, 1.0},  {0, 2883, 93, 104, 1.0},  {0, 2883, 98, 99, 1.0},  {0, 2884, 93, 105, 1.0},  {0, 2884, 94, 100, 1.0},  {0, 2885, 93, 106, 1.0},
  {0, 2885, 94, 101, 1.0},  {0, 2885, 95, 100, 1.0},  {0, 2886, 93, 107, 1.0},  {0, 2886, 94, 102, 1.0},  {0, 2886, 96, 100, 1.0},  {0, 2887, 93, 108, 1.0},
  {0, 2887, 94, 103, 1.0},  {0, 2887, 97, 100, 1.0},  {0, 2888, 93, 109, 1.0},  {0, 2888, 94, 104, 1.0},  {0, 2888, 98, 100, 1.0},  {0, 2889, 93, 110, 1.0},
  {0, 2889, 95, 101, 1.0},  {0, 2890, 93, 111, 1.0},  {0, 2890, 95, 102, 1.0},  {0, 2890, 96, 101, 1.0},  {0, 2891, 93, 112, 1.0},  {0, 2891, 95, 103, 1.0},
  {0, 2891, 97, 101, 1.0},  {0, 2892, 93, 113, 1.0},  {0, 2892, 95, 104, 1.0},  {0, 2892, 98, 101, 1.0},  {0, 2893, 93, 114, 1.0},  {0, 2893, 96, 102, 1.0},
  {0, 2894, 93, 115, 1.0},  {0, 2894, 96, 103, 1.0},  {0, 2894, 97, 102, 1.0},  {0, 2895, 93, 116, 1.0},  {0, 2895, 96, 104, 1.0},  {0, 2895, 98, 102, 1.0},
  {0, 2896, 93, 117, 1.0},  {0, 2896, 97, 103, 1.0},  {0, 2897, 93, 118, 1.0},  {0, 2897, 97, 104, 1.0},  {0, 2897, 98, 103, 1.0},  {0, 2898, 93, 119, 1.0},
  {0, 2898, 98, 104, 1.0},  {0, 2899, 94, 105, 1.0},  {0, 2900, 94, 106, 1.0},  {0, 2900, 95, 105, 1.0},  {0, 2901, 94, 107, 1.0},  {0, 2901, 96, 105, 1.0},
  {0, 2902, 94, 108, 1.0},  {0, 2902, 97, 105, 1.0},  {0, 2903, 94, 109, 1.0},  {0, 2903, 98, 105, 1.0},  {0, 2904, 94, 110, 1.0},  {0, 2904, 95, 106, 1.0},
  {0, 2905, 94, 111, 1.0},  {0, 2905, 95, 107, 1.0},  {0, 2905, 96, 106, 1.0},  {0, 2906, 94, 112, 1.0},  {0, 2906, 95, 108, 1.0},  {0, 2906, 97, 106, 1.0},
  {0, 2907, 94, 113, 1.0},  {0, 2907, 95, 109, 1.0},  {0, 2907, 98, 106, 1.0},  {0, 2908, 94, 114, 1.0},  {0, 2908, 96, 107, 1.0},  {0, 2909, 94, 115, 1.0},
  {0, 2909, 96, 108, 1.0},  {0, 2909, 97, 107, 1.0},  {0, 2910, 94, 116, 1.0},  {0, 2910, 96, 109, 1.0},  {0, 2910, 98, 107, 1.0},  {0, 2911, 94, 117, 1.0},
  {0, 2911, 97, 108, 1.0},  {0, 2912, 94, 118, 1.0},  {0, 2912, 97, 109, 1.0},  {0, 2912, 98, 108, 1.0},  {0, 2913, 94, 119, 1.0},  {0, 2913, 98, 109, 1.0},
  {0, 2914, 95, 110, 1.0},  {0, 2915, 95, 111, 1.0},  {0, 2915, 96, 110, 1.0},  {0, 2916, 95, 112, 1.0},  {0, 2916, 97, 110, 1.0},  {0, 2917, 95, 113, 1.0},
  {0, 2917, 98, 110, 1.0},  {0, 2918, 95, 114, 1.0},  {0, 2918, 96, 111, 1.0},  {0, 2919, 95, 115, 1.0},  {0, 2919, 96, 112, 1.0},  {0, 2919, 97, 111, 1.0},
  {0, 2920, 95, 116, 1.0},  {0, 2920, 96, 113, 1.0},  {0, 2920, 98, 111, 1.0},  {0, 2921, 95, 117, 1.0},  {0, 2921, 97, 112, 1.0},  {0, 2922, 95, 118, 1.0},
  {0, 2922, 97, 113, 1.0},  {0, 2922, 98, 112, 1.0},  {0, 2923, 95, 119, 1.0},  {0, 2923, 98, 113, 1.0},  {0, 2924, 96, 114, 1.0},  {0, 2925, 96, 115, 1.0},
  {0, 2925, 97, 114, 1.0},  {0, 2926, 96, 116, 1.0},  {0, 2926, 98, 114, 1.0},  {0, 2927, 96, 117, 1.0},  {0, 2927, 97, 115, 1.0},  {0, 2928, 96, 118, 1.0},
  {0, 2928, 97, 116, 1.0},  {0, 2928, 98, 115, 1.0},  {0, 2929, 96, 119, 1.0},  {0, 2929, 98, 116, 1.0},  {0, 2930, 97, 117, 1.0},  {0, 2931, 97, 118, 1.0},
  {0, 2931, 98, 117, 1.0},  {0, 2932, 97, 119, 1.0},  {0, 2932, 98, 118, 1.0},  {0, 2933, 98, 119, 1.0},  {0, 2934, 99, 99, 1.0},  {0, 2935, 99, 100, 1.0},
  {0, 2936, 99, 101, 1.0},  {0, 2937, 99, 102, 1.0},  {0, 2938, 99, 103, 1.0},  {0, 2939, 99, 104, 1.0},  {0, 2940, 99, 105, 1.0},  {0, 2940, 100, 100, 1.0},
  {0, 2941, 99, 106, 1.0},  {0, 2941, 100, 101, 1.0},  {0, 2942, 99, 107, 1.0},  {0, 2942, 100, 102, 1.0},  {0, 2943, 99, 108, 1.0},  {0, 2943, 100, 103, 1.0},
  {0, 2944, 99, 109, 1.0},  {0, 2944, 100, 104, 1.0},  {0, 2945, 99, 110, 1.0},  {0, 2945, 101, 101, 1.0},  {0, 2946, 99, 111, 1.0},  {0, 2946, 101, 102, 1.0},
  {0, 2947, 99, 112, 1.0},  {0, 2947, 101, 103, 1.0},  {0, 2948, 99, 113, 1.0},  {0, 2948, 101, 104, 1.0},  {0, 2949, 99, 114, 1.0},  {0, 2949, 102, 102, 1.0},
  {0, 2950, 99, 115, 1.0},  {0, 2950, 102, 103, 1.0},  {0, 2951, 99, 116, 1.0},  {0, 2951, 102, 104, 1.0},  {0, 2952, 99, 117, 1.0},  {0, 2952, 103, 103, 1.0},
  {0, 2953, 99, 118, 1.0},  {0, 2953, 103, 104, 1.0},  {0, 2954, 99, 119, 1.0},  {0, 2954, 104, 104, 1.0},  {0, 2955, 100, 105, 1.0},  {0, 2956, 100, 106, 1.0},
  {0, 2956, 101, 105, 1.0},  {0, 2957, 100, 107, 1.0},  {0, 2957, 102, 105, 1.0},  {0, 2958, 100, 108, 1.0},  {0, 2958, 103, 105, 1.0},  {0, 2959, 100, 109, 1.0},
  {0, 2959, 104, 105, 1.0},  {0, 2960, 100, 110, 1.0},  {0, 2960, 101, 106, 1.0},  {0, 2961, 100, 111, 1.0},  {0, 2961, 101, 107, 1.0},  {0, 2961, 102, 106, 1.0},
  {0, 2962, 100, 112, 1.0},  {0, 2962, 101, 108, 1.0},  {0, 2962, 103, 106, 1.0},  {0, 2963, 100, 113, 1.0},  {0, 2963, 101, 109, 1.0},  {0, 2963, 104, 106, 1.0},
  {0, 2964, 100, 114, 1.0},  {0, 2964, 102, 107, 1.0},  {0, 2965, 100, 115, 1.0},  {0, 2965, 102, 108, 1.0},  {0, 2965, 103, 107, 1.0},  {0, 2966, 100, 116, 1.0},
  {0, 2966, 102, 109, 1.0},  {0, 2966, 104, 107, 1.0},  {0, 2967, 100, 117, 1.0},  {0, 2967, 103, 108, 1.0},  {0, 2968, 100, 118, 1.0},  {0, 2968, 103, 109, 1.0},
  {0, 2968, 104, 108, 1.0},  {0, 2969, 100, 119, 1.0},  {0, 2969, 104, 109, 1.0},  {0, 2970, 101, 110, 1.0},  {0, 2971, 101, 111, 1.0},  {0, 2971, 102, 110, 1.0},
  {0, 2972, 101, 112, 1.0},  {0, 2972, 103, 110, 1.0},  {0, 2973, 101, 113, 1.0},  {0, 2973, 104, 110, 1.0},  {0, 2974, 101, 114, 1.0},  {0, 2974, 102, 111, 1.0},
  {0, 2975, 101, 115, 1.0},  {0, 2975, 102, 112, 1.0},  {0, 2975, 103, 111, 1.0},  {0, 2976, 101, 116, 1.0},  {0, 2976, 102, 113, 1.0},  {0, 2976, 104, 111, 1.0},
  {0, 2977, 101, 117, 1.0},  {0, 2977, 103, 112, 1.0},  {0, 2978, 101, 118, 1.0},  {0, 2978, 103, 113, 1.0},  {0, 2978, 104, 112, 1.0},  {0, 2979, 101, 119, 1.0},
  {0, 2979, 104, 113, 1.0},  {0, 2980, 102, 114, 1.0},  {0, 2981, 102, 115, 1.0},  {0, 2981, 103, 114, 1.0},  {0, 2982, 102, 116, 1.0},  {0, 2982, 104, 114, 1.0},
  {0, 2983, 102, 117, 1.0},  {0, 2983, 103, 115, 1.0},  {0, 2984, 102, 118, 1.0},  {0, 2984, 103, 116, 1.0},  {0, 2984, 104, 115, 1.0},  {0, 2985, 102, 119, 1.0},
  {0, 2985, 104, 116, 1.0},  {0, 2986, 103, 117, 1.0},  {0, 2987, 103, 118, 1.0},  {0, 2987, 104, 117, 1.0},  {0, 2988, 103, 119, 1.0},  {0, 2988, 104, 118, 1.0},
  {0, 2989, 104, 119, 1.0},  {0, 2990, 105, 105, 1.0},  {0, 2991, 105, 106, 1.0},  {0, 2992, 105, 107, 1.0},  {0, 2993, 105, 108, 1.0},  {0, 2994, 105, 109, 1.0},
  {0, 2995, 105, 110, 1.0},  {0, 2995, 106, 106, 1.0},  {0, 2996, 105, 111, 1.0},  {0, 2996, 106, 107, 1.0},  {0, 2997, 105, 112, 1.0},  {0, 2997, 106, 108, 1.0},
  {0, 2998, 105, 113, 1.0},  {0, 2998, 106, 109, 1.0},  {0, 2999, 105, 114, 1.0},  {0, 2999, 107, 107, 1.0},  {0, 3000, 105, 115, 1.0},  {0, 3000, 107, 108, 1.0},
  {0, 3001, 105, 116, 1.0},  {0, 3001, 107, 109, 1.0},  {0, 3002, 105, 117, 1.0},  {0, 3002, 108, 108, 1.0},  {0, 3003, 105, 118, 1.0},  {0, 3003, 108, 109, 1.0},
  {0, 3004, 105, 119, 1.0},  {0, 3004, 109, 109, 1.0},  {0, 3005, 106, 110, 1.0},  {0, 3006, 106, 111, 1.0},  {0, 3006, 107, 110, 1.0},  {0, 3007, 106, 112, 1.0},
  {0, 3007, 108, 110, 1.0},  {0, 3008, 106, 113, 1.0},  {0, 3008, 109, 110, 1.0},  {0, 3009, 106, 114, 1.0},  {0, 3009, 107, 111, 1.0},  {0, 3010, 106, 115, 1.0},
  {0, 3010, 107, 112, 1.0},  {0, 3010, 108, 111, 1.0},  {0, 3011, 106, 116, 1.0},  {0, 3011, 107, 113, 1.0},  {0, 3011, 109, 111, 1.0},  {0, 3012, 106, 117, 1.0},
  {0, 3012, 108, 112, 1.0},  {0, 3013, 106, 118, 1.0},  {0, 3013, 108, 113, 1.0},  {0, 3013, 109, 112, 1.0},  {0, 3014, 106, 119, 1.0},  {0, 3014, 109, 113, 1.0},
  {0, 3015, 107, 114, 1.0},  {0, 3016, 107, 115, 1.0},  {0, 3016, 108, 114, 1.0},  {0, 3017, 107, 116, 1.0},  {0, 3017, 109, 114, 1.0},  {0, 3018, 107, 117, 1.0},
  {0, 3018, 108, 115, 1.0},  {0, 3019, 107, 118, 1.0},  {0, 3019, 108, 116, 1.0},  {0, 3019, 109, 115, 1.0},  {0, 3020, 107, 119, 1.0},  {0, 3020, 109, 116, 1.0},
  {0, 3021, 108, 117, 1.0},  {0, 3022, 108, 118, 1.0},  {0, 3022, 109, 117, 1.0},  {0, 3023, 108, 119, 1.0},  {0, 3023, 109, 118, 1.0},  {0, 3024, 109, 119, 1.0},
  {0, 3025, 110, 110, 1.0},  {0, 3026, 110, 111, 1.0},  {0, 3027, 110, 112, 1.0},  {0, 3028, 110, 113, 1.0},  {0, 3029, 110, 114, 1.0},  {0, 3029, 111, 111, 1.0},
  {0, 3030, 110, 115, 1.0},  {0, 3030, 111, 112, 1.0},  {0, 3031, 110, 116, 1.0},  {0, 3031, 111, 113, 1.0},  {0, 3032, 110, 117, 1.0},  {0, 3032, 112, 112, 1.0},
  {0, 3033, 110, 118, 1.0},  {0, 3033, 112, 113, 1.0},  {0, 3034, 110, 119, 1.0},  {0, 3034, 113, 113, 1.0},  {0, 3035, 111, 114, 1.0},  {0, 3036, 111, 115, 1.0},
  {0, 3036, 112, 114, 1.0},  {0, 3037, 111, 116, 1.0},  {0, 3037, 113, 114, 1.0},  {0, 3038, 111, 117, 1.0},  {0, 3038, 112, 115, 1.0},  {0, 3039, 111, 118, 1.0},
  {0, 3039, 112, 116, 1.0},  {0, 3039, 113, 115, 1.0},  {0, 3040, 111, 119, 1.0},  {0, 3040, 113, 116, 1.0},  {0, 3041, 112, 117, 1.0},  {0, 3042, 112, 118, 1.0},
  {0, 3042, 113, 117, 1.0},  {0, 3043, 112, 119, 1.0},  {0, 3043, 113, 118, 1.0},  {0, 3044, 113, 119, 1.0},  {0, 3045, 114, 114, 1.0},  {0, 3046, 114, 115, 1.0},
  {0, 3047, 114, 116, 1.0},  {0, 3048, 114, 117, 1.0},  {0, 3048, 115, 115, 1.0},  {0, 3049, 114, 118, 1.0},  {0, 3049, 115, 116, 1.0},  {0, 3050, 114, 119, 1.0},
  {0, 3050, 116, 116, 1.0},  {0, 3051, 115, 117, 1.0},  {0, 3052, 115, 118, 1.0},  {0, 3052, 116, 117, 1.0},  {0, 3053, 115, 119, 1.0},  {0, 3053, 116, 118, 1.0},
  {0, 3054, 116, 119, 1.0},  {0, 3055, 117, 117, 1.0},  {0, 3056, 117, 118, 1.0},  {0, 3057, 117, 119, 1.0},  {0, 3057, 118, 118, 1.0},  {0, 3058, 118, 119, 1.0},
  {0, 3059, 119, 119, 1.0},  {1, 0, 0, 0, -1.0},  {1, 1, 0, 1, -1.0},  {1, 2, 0, 2, -1.0},  {1, 3, 0, 3, -1.0},  {1, 4, 0, 4, -1.0},
  {1, 5, 0, 5, -1.0},  {1, 6, 0, 6, -1.0},  {1, 7, 0, 7, -1.0},  {1, 8, 0, 8, -1.0},  {1, 9, 0, 9, -1.0},  {1, 10, 0, 10, -1.0},
  {1, 11, 0, 11, -1.0},  {1, 12, 0, 12, -1.0},  {1, 13, 0, 13, -1.0},  {1, 14, 0, 14, -1.0},  {1, 15, 0, 0, 1.0},  {1, 15, 1, 1, -1.0},
  {1, 16, 1, 2, -1.0},  {1, 17, 1, 3, -1.0},  {1, 18, 1, 4, -1.0},  {1, 19, 1, 5, -1.0},  {1, 20, 1, 6, -1.0},  {1, 21, 1, 7, -1.0},
  {1, 22, 1, 8, -1.0},  {1, 23, 1, 9, -1.0},  {1, 24, 1, 10, -1.0},  {1, 25, 1, 11, -1.0},  {1, 26, 1, 12, -1.0},  {1, 27, 1, 13, -1.0},
  {1, 28, 1, 14, -1.0},  {1, 29, 0, 0, 1.0},  {1, 29, 2, 2, -1.0},  {1, 30, 2, 3, -1.0},  {1, 31, 2, 4, -1.0},  {1, 32, 2, 5, -1.0},
  {1, 33, 2, 6, -1.0},  {1, 34, 2, 7, -1.0},  {1, 35, 2, 8, -1.0},  {1, 36, 2, 9, -1.0},  {1, 37, 2, 10, -1.0},  {1, 38, 2, 11, -1.0},
  {1, 39, 2, 12, -1.0},  {1, 40, 2, 13, -1.0},  {1, 41, 2, 14, -1.0},  {1, 42, 0, 0, 1.0},  {1, 42, 3, 3, -1.0},  {1, 43, 3, 4, -1.0},
  {1, 44, 3, 5, -1.0},  {1, 45, 3, 6, -1.0},  {1, 46, 3, 7, -1.0},  {1, 47, 3, 8, -1.0},  {1, 48, 3, 9, -1.0},  {1, 49, 3, 10, -1.0},
  {1, 50, 3, 11, -1.0},  {1, 51, 3, 12, -1.0},  {1, 52, 3, 13, -1.0},  {1, 53, 3, 14, -1.0},  {1, 54, 0, 0, 1.0},  {1, 54, 4, 4, -1.0},
  {1, 55, 4, 5, -1.0},  {1, 56, 4, 6, -1.0},  {1, 57, 4, 7, -1.0},  {1, 58, 4, 8, -1.0},  {1, 59, 4, 9, -1.0},  {1, 60, 4, 10, -1.0},
  {1, 61, 4, 11, -1.0},  {1, 62, 4, 12, -1.0},  {1, 63, 4, 13, -1.0},  {1, 64, 4, 14, -1.0},  {1, 65, 5, 5, -1.0},  {1, 66, 5, 6, -1.0},
  {1, 67, 5, 7, -1.0},  {1, 68, 5, 8, -1.0},  {1, 69, 5, 9, -1.0},  {1, 70, 5, 10, -1.0},  {1, 71, 5, 11, -1.0},  {1, 72, 5, 12, -1.0},
  {1, 73, 5, 13, -1.0},  {1, 74, 5, 14, -1.0},  {1, 75, 6, 6, -1.0},  {1, 76, 6, 7, -1.0},  {1, 77, 6, 8, -1.0},  {1, 78, 6, 9, -1.0},
  {1, 79, 6, 10, -1.0},  {1, 80, 6, 11, -1.0},  {1, 81, 6, 12, -1.0},  {1, 82, 6, 13, -1.0},  {1, 83, 6, 14, -1.0},  {1, 84, 7, 7, -1.0},
  {1, 85, 7, 8, -1.0},  {1, 86, 7, 9, -1.0},  {1, 87, 7, 10, -1.0},  {1, 88, 7, 11, -1.0},  {1, 89, 7, 12, -1.0},  {1, 90, 7, 13, -1.0},
  {1, 91, 7, 14, -1.0},  {1, 92, 8, 8, -1.0},  {1, 93, 8, 9, -1.0},  {1, 94, 8, 10, -1.0},  {1, 95, 8, 11, -1.0},  {1, 96, 8, 12, -1.0},
  {1, 97, 8, 13, -1.0},  {1, 98, 8, 14, -1.0},  {1, 99, 9, 9, -1.0},  {1, 100, 9, 10, -1.0},  {1, 101, 9, 11, -1.0},  {1, 102, 9, 12, -1.0},
  {1, 103, 9, 13, -1.0},  {1, 104, 9, 14, -1.0},  {1, 105, 10, 10, -1.0},  {1, 106, 10, 11, -1.0},  {1, 107, 10, 12, -1.0},  {1, 108, 10, 13, -1.0},
  {1, 109, 10, 14, -1.0},  {1, 110, 11, 11, -1.0},  {1, 111, 11, 12, -1.0},  {1, 112, 11, 13, -1.0},  {1, 113, 11, 14, -1.0},  {1, 114, 12, 12, -1.0},
  {1, 115, 12, 13, -1.0},  {1, 116, 12, 14, -1.0},  {1, 117, 13, 13, -1.0},  {1, 118, 13, 14, -1.0},  {1, 119, 14, 14, -1.0},  {1, 120, 0, 1, 1.0},
  {1, 121, 0, 2, 1.0},  {1, 122, 0, 3, 1.0},  {1, 123, 0, 4, 1.0},  {1, 124, 0, 5, 1.0},  {1, 125, 0, 6, 1.0},  {1, 126, 0, 7, 1.0},
  {1, 127, 0, 8, 1.0},  {1, 128, 0, 9, 1.0},  {1, 129, 0, 10, 1.0},  {1, 130, 0, 11, 1.0},  {1, 131, 0, 12, 1.0},  {1, 132, 0, 13, 1.0},
  {1, 133, 0, 14, 1.0},  {1, 134, 0, 1, 1.0},  {1, 147, 0, 1, 1.0},  {1, 159, 0, 1, 1.0},  {1, 225, 0, 2, 1.0},  {1, 226, 0, 3, 1.0},
  {1, 227, 0, 4, 1.0},  {1, 228, 0, 5, 1.0},  {1, 229, 0, 6, 1.0},  {1, 230, 0, 7, 1.0},  {1, 231, 0, 8, 1.0},  {1, 232, 0, 9, 1.0},
  {1, 233, 0, 10, 1.0},  {1, 234, 0, 11, 1.0},  {1, 235, 0, 12, 1.0},  {1, 236, 0, 13, 1.0},  {1, 237, 0, 14, 1.0},  {1, 238, 0, 2, 1.0},
  {1, 250, 0, 2, 1.0},  {1, 316, 0, 3, 1.0},  {1, 317, 0, 4, 1.0},  {1, 318, 0, 5, 1.0},  {1, 319, 0, 6, 1.0},  {1, 320, 0, 7, 1.0},
  {1, 321, 0, 8, 1.0},  {1, 322, 0, 9, 1.0},  {1, 323, 0, 10, 1.0},  {1, 324, 0, 11, 1.0},  {1, 325, 0, 12, 1.0},  {1, 326, 0, 13, 1.0},
  {1, 327, 0, 14, 1.0},  {1, 328, 0, 3, 1.0},  {1, 394, 0, 4, 1.0},  {1, 395, 0, 5, 1.0},  {1, 396, 0, 6, 1.0},  {1, 397, 0, 7, 1.0},
  {1, 398, 0, 8, 1.0},  {1, 399, 0, 9, 1.0},  {1, 400, 0, 10, 1.0},  {1, 401, 0, 11, 1.0},  {1, 402, 0, 12, 1.0},  {1, 403, 0, 13, 1.0},
  {1, 404, 0, 14, 1.0},  {1, 680, 1, 1, 1.0},  {1, 681, 1, 2, 1.0},  {1, 682, 1, 3, 1.0},  {1, 683, 1, 4, 1.0},  {1, 684, 1, 5, 1.0},
  {1, 685, 1, 6, 1.0},  {1, 686, 1, 7, 1.0},  {1, 687, 1, 8, 1.0},  {1, 688, 1, 9, 1.0},  {1, 689, 1, 10, 1.0},  {1, 690, 1, 11, 1.0},
  {1, 691, 1, 12, 1.0},  {1, 692, 1, 13, 1.0},  {1, 693, 1, 14, 1.0},  {1, 694, 1, 1, 1.0},  {1, 694, 2, 2, 1.0},  {1, 695, 2, 3, 1.0},
  {1, 696, 2, 4, 1.0},  {1, 697, 2, 5, 1.0},  {1, 698, 2, 6, 1.0},  {1, 699, 2, 7, 1.0},  {1, 700, 2, 8, 1.0},  {1, 701, 2, 9, 1.0},
  {1, 702, 2, 10, 1.0},  {1, 703, 2, 11, 1.0},  {1, 704, 2, 12, 1.0},  {1, 705, 2, 13, 1.0},  {1, 706, 2, 14, 1.0},  {1, 707, 1, 1, 1.0},
  {1, 707, 3, 3, 1.0},  {1, 708, 3, 4, 1.0},  {1, 709, 3, 5, 1.0},  {1, 710, 3, 6, 1.0},  {1, 711, 3, 7, 1.0},  {1, 712, 3, 8, 1.0},
  {1, 713, 3, 9, 1.0},  {1, 714, 3, 10, 1.0},  {1, 715, 3, 11, 1.0},  {1, 716, 3, 12, 1.0},  {1, 717, 3, 13, 1.0},  {1, 718, 3, 14, 1.0},
  {1, 719, 1, 1, 1.0},  {1, 719, 4, 4, 1.0},  {1, 720, 4, 5, 1.0},  {1, 721, 4, 6, 1.0},  {1, 722, 4, 7, 1.0},  {1, 723, 4, 8, 1.0},
  {1, 724, 4, 9, 1.0},  {1, 725, 4, 10, 1.0},  {1, 726, 4, 11, 1.0},  {1, 727, 4, 12, 1.0},  {1, 728, 4, 13, 1.0},  {1, 729, 4, 14, 1.0},
  {1, 730, 5, 5, 1.0},  {1, 731, 5, 6, 1.0},  {1, 732, 5, 7, 1.0},  {1, 733, 5, 8, 1.0},  {1, 734, 5, 9, 1.0},  {1, 735, 5, 10, 1.0},
  {1, 736, 5, 11, 1.0},  {1, 737, 5, 12, 1.0},  {1, 738, 5, 13, 1.0},  {1, 739, 5, 14, 1.0},  {1, 740, 6, 6, 1.0},  {1, 741, 6, 7, 1.0},
  {1, 742, 6, 8, 1.0},  {1, 743, 6, 9, 1.0},  {1, 744, 6, 10, 1.0},  {1, 745, 6, 11, 1.0},  {1, 746, 6, 12, 1.0},  {1, 747, 6, 13, 1.0},
  {1, 748, 6, 14, 1.0},  {1, 749, 7, 7, 1.0},  {1, 750, 7, 8, 1.0},  {1, 751, 7, 9, 1.0},  {1, 752, 7, 10, 1.0},  {1, 753, 7, 11, 1.0},
  {1, 754, 7, 12, 1.0},  {1, 755, 7, 13, 1.0},  {1, 756, 7, 14, 1.0},  {1, 757, 8, 8, 1.0},  {1, 758, 8, 9, 1.0},  {1, 759, 8, 10, 1.0},
  {1, 760, 8, 11, 1.0},  {1, 761, 8, 12, 1.0},  {1, 762, 8, 13, 1.0},  {1, 763, 8, 14, 1.0},  {1, 764, 9, 9, 1.0},  {1, 765, 9, 10, 1.0},
  {1, 766, 9, 11, 1.0},  {1, 767, 9, 12, 1.0},  {1, 768, 9, 13, 1.0},  {1, 769, 9, 14, 1.0},  {1, 770, 10, 10, 1.0},  {1, 771, 10, 11, 1.0},
  {1, 772, 10, 12, 1.0},  {1, 773, 10, 13, 1.0},  {1, 774, 10, 14, 1.0},  {1, 775, 11, 11, 1.0},  {1, 776, 11, 12, 1.0},  {1, 777, 11, 13, 1.0},
  {1, 778, 11, 14, 1.0},  {1, 779, 12, 12, 1.0},  {1, 780, 12, 13, 1.0},  {1, 781, 12, 14, 1.0},  {1, 782, 13, 13, 1.0},  {1, 783, 13, 14, 1.0},
  {1, 784, 14, 14, 1.0},  {1, 785, 1, 2, 1.0},  {1, 786, 1, 3, 1.0},  {1, 787, 1, 4, 1.0},  {1, 788, 1, 5, 1.0},  {1, 789, 1, 6, 1.0},
  {1, 790, 1, 7, 1.0},  {1, 791, 1, 8, 1.0},  {1, 792, 1, 9, 1.0},  {1, 793, 1, 10, 1.0},  {1, 794, 1, 11, 1.0},  {1, 795, 1, 12, 1.0},
  {1, 796, 1, 13, 1.0},  {1, 797, 1, 14, 1.0},  {1, 798, 1, 2, 1.0},  {1, 810, 1, 2, 1.0},  {1, 876, 1, 3, 1.0},  {1, 877, 1, 4, 1.0},
  {1, 878, 1, 5, 1.0},  {1, 879, 1, 6, 1.0},  {1, 880, 1, 7, 1.0},  {1, 881, 1, 8, 1.0},  {1, 882, 1, 9, 1.0},  {1, 883, 1, 10, 1.0},
  {1, 884, 1, 11, 1.0},  {1, 885, 1, 12, 1.0},  {1, 886, 1, 13, 1.0},  {1, 887, 1, 14, 1.0},  {1, 888, 1, 3, 1.0},  {1, 954, 1, 4, 1.0},
  {1, 955, 1, 5, 1.0},  {1, 956, 1, 6, 1.0},  {1, 957, 1, 7, 1.0},  {1, 958, 1, 8, 1.0},  {1, 959, 1, 9, 1.0},  {1, 960, 1, 10, 1.0},
  {1, 961, 1, 11, 1.0},  {1, 962, 1, 12, 1.0},  {1, 963, 1, 13, 1.0},  {1, 964, 1, 14, 1.0},  {1, 1240, 2, 2, 1.0},  {1, 1241, 2, 3, 1.0},
  {1, 1242, 2, 4, 1.0},  {1, 1243, 2, 5, 1.0},  {1, 1244, 2, 6, 1.0},  {1, 1245, 2, 7, 1.0},  {1, 1246, 2, 8, 1.0},  {1, 1247, 2, 9, 1.0},
  {1, 1248, 2, 10, 1.0},  {1, 1249, 2, 11, 1.0},  {1, 1250, 2, 12, 1.0},  {1, 1251, 2, 13, 1.0},  {1, 1252, 2, 14, 1.0},  {1, 1253, 2, 2, 1.0},
  {1, 1253, 3, 3, 1.0},  {1, 1254, 3, 4, 1.0},  {1, 1255, 3, 5, 1.0},  {1, 1256, 3, 6, 1.0},  {1, 1257, 3, 7, 1.0},  {1, 1258, 3, 8, 1.0},
  {1, 1259, 3, 9, 1.0},  {1, 1260, 3, 10, 1.0},  {1, 1261, 3, 11, 1.0},  {1, 1262, 3, 12, 1.0},  {1, 1263, 3, 13, 1.0},  {1, 1264, 3, 14, 1.0},
  {1, 1265, 2, 2, 1.0},  {1, 1265, 4, 4, 1.0},  {1, 1266, 4, 5, 1.0},  {1, 1267, 4, 6, 1.0},  {1, 1268, 4, 7, 1.0},  {1, 1269, 4, 8, 1.0},
  {1, 1270, 4, 9, 1.0},  {1, 1271, 4, 10, 1.0},  {1, 1272, 4, 11, 1.0},  {1, 1273, 4, 12, 1.0},  {1, 1274, 4, 13, 1.0},  {1, 1275, 4, 14, 1.0},
  {1, 1276, 5, 5, 1.0},  {1, 1277, 5, 6, 1.0},  {1, 1278, 5, 7, 1.0},  {1, 1279, 5, 8, 1.0},  {1, 1280, 5, 9, 1.0},  {1, 1281, 5, 10, 1.0},
  {1, 1282, 5, 11, 1.0},  {1, 1283, 5, 12, 1.0},  {1, 1284, 5, 13, 1.0},  {1, 1285, 5, 14, 1.0},  {1, 1286, 6, 6, 1.0},  {1, 1287, 6, 7, 1.0},
  {1, 1288, 6, 8, 1.0},  {1, 1289, 6, 9, 1.0},  {1, 1290, 6, 10, 1.0},  {1, 1291, 6, 11, 1.0},  {1, 1292, 6, 12, 1.0},  {1, 1293, 6, 13, 1.0},
  {1, 1294, 6, 14, 1.0},  {1, 1295, 7, 7, 1.0},  {1, 1296, 7, 8, 1.0},  {1, 1297, 7, 9, 1.0},  {1, 1298, 7, 10, 1.0},  {1, 1299, 7, 11, 1.0},
  {1, 1300, 7, 12, 1.0},  {1, 1301, 7, 13, 1.0},  {1, 1302, 7, 14, 1.0},  {1, 1303, 8, 8, 1.0},  {1, 1304, 8, 9, 1.0},  {1, 1305, 8, 10, 1.0},
  {1, 1306, 8, 11, 1.0},  {1, 1307, 8, 12, 1.0},  {1, 1308, 8, 13, 1.0},  {1, 1309, 8, 14, 1.0},  {1, 1310, 9, 9, 1.0},  {1, 1311, 9, 10, 1.0},
  {1, 1312, 9, 11, 1.0},  {1, 1313, 9, 12, 1.0},  {1, 1314, 9, 13, 1.0},  {1, 1315, 9, 14, 1.0},  {1, 1316, 10, 10, 1.0},  {1, 1317, 10, 11, 1.0},
  {1, 1318, 10, 12, 1.0},  {1, 1319, 10, 13, 1.0},  {1, 1320, 10, 14, 1.0},  {1, 1321, 11, 11, 1.0},  {1, 1322, 11, 12, 1.0},  {1, 1323, 11, 13, 1.0},
  {1, 1324, 11, 14, 1.0},  {1, 1325, 12, 12, 1.0},  {1, 1326, 12, 13, 1.0},  {1, 1327, 12, 14, 1.0},  {1, 1328, 13, 13, 1.0},  {1, 1329, 13, 14, 1.0},
  {1, 1330, 14, 14, 1.0},  {1, 1331, 2, 3, 1.0},  {1, 1332, 2, 4, 1.0},  {1, 1333, 2, 5, 1.0},  {1, 1334, 2, 6, 1.0},  {1, 1335, 2, 7, 1.0},
  {1, 1336, 2, 8, 1.0},  {1, 1337, 2, 9, 1.0},  {1, 1338, 2, 10, 1.0},  {1, 1339, 2, 11, 1.0},  {1, 1340, 2, 12, 1.0},  {1, 1341, 2, 13, 1.0},
  {1, 1342, 2, 14, 1.0},  {1, 1343, 2, 3, 1.0},  {1, 1409, 2, 4, 1.0},  {1, 1410, 2, 5, 1.0},  {1, 1411, 2, 6, 1.0},  {1, 1412, 2, 7, 1.0},
  {1, 1413, 2, 8, 1.0},  {1, 1414, 2, 9, 1.0},  {1, 1415, 2, 10, 1.0},  {1, 1416, 2, 11, 1.0},  {1, 1417, 2, 12, 1.0},  {1, 1418, 2, 13, 1.0},
  {1, 1419, 2, 14, 1.0},  {1, 1695, 3, 3, 1.0},  {1, 1696, 3, 4, 1.0},  {1, 1697, 3, 5, 1.0},  {1, 1698, 3, 6, 1.0},  {1, 1699, 3, 7, 1.0},
  {1, 1700, 3, 8, 1.0},  {1, 1701, 3, 9, 1.0},  {1, 1702, 3, 10, 1.0},  {1, 1703, 3, 11, 1.0},  {1, 1704, 3, 12, 1.0},  {1, 1705, 3, 13, 1.0},
  {1, 1706, 3, 14, 1.0},  {1, 1707, 3, 3, 1.0},  {1, 1707, 4, 4, 1.0},  {1, 1708, 4, 5, 1.0},  {1, 1709, 4, 6, 1.0},  {1, 1710, 4, 7, 1.0},
  {1, 1711, 4, 8, 1.0},  {1, 1712, 4, 9, 1.0},  {1, 1713, 4, 10, 1.0},  {1, 1714, 4, 11, 1.0},  {1, 1715, 4, 12, 1.0},  {1, 1716, 4, 13, 1.0},
  {1, 1717, 4, 14, 1.0},  {1, 1718, 5, 5, 1.0},  {1, 1719, 5, 6, 1.0},  {1, 1720, 5, 7, 1.0},  {1, 1721, 5, 8, 1.0},  {1, 1722, 5, 9, 1.0},
  {1, 1723, 5, 10, 1.0},  {1, 1724, 5, 11, 1.0},  {1, 1725, 5, 12, 1.0},  {1, 1726, 5, 13, 1.0},  {1, 1727, 5, 14, 1.0},  {1, 1728, 6, 6, 1.0},
  {1, 1729, 6, 7, 1.0},  {1, 1730, 6, 8, 1.0},  {1, 1731, 6, 9, 1.0},  {1, 1732, 6, 10, 1.0},  {1, 1733, 6, 11, 1.0},  {1, 1734, 6, 12, 1.0},
  {1, 1735, 6, 13, 1.0},  {1, 1736, 6, 14, 1.0},  {1, 1737, 7, 7, 1.0},  {1, 1738, 7, 8, 1.0},  {1, 1739, 7, 9, 1.0},  {1, 1740, 7, 10, 1.0},
  {1, 1741, 7, 11, 1.0},  {1, 1742, 7, 12, 1.0},  {1, 1743, 7, 13, 1.0},  {1, 1744, 7, 14, 1.0},  {1, 1745, 8, 8, 1.0},  {1, 1746, 8, 9, 1.0},
  {1, 1747, 8, 10, 1.0},  {1, 1748, 8, 11, 1.0},  {1, 1749, 8, 12, 1.0},  {1, 1750, 8, 13, 1.0},  {1, 1751, 8, 14, 1.0},  {1, 1752, 9, 9, 1.0},
  {1, 1753, 9, 10, 1.0},  {1, 1754, 9, 11, 1.0},  {1, 1755, 9, 12, 1.0},  {1, 1756, 9, 13, 1.0},  {1, 1757, 9, 14, 1.0},  {1, 1758, 10, 10, 1.0},
  {1, 1759, 10, 11, 1.0},  {1, 1760, 10, 12, 1.0},  {1, 1761, 10, 13, 1.0},  {1, 1762, 10, 14, 1.0},  {1, 1763, 11, 11, 1.0},  {1, 1764, 11, 12, 1.0},
  {1, 1765, 11, 13, 1.0},  {1, 1766, 11, 14, 1.0},  {1, 1767, 12, 12, 1.0},  {1, 1768, 12, 13, 1.0},  {1, 1769, 12, 14, 1.0},  {1, 1770, 13, 13, 1.0},
  {1, 1771, 13, 14, 1.0},  {1, 1772, 14, 14, 1.0},  {1, 1773, 3, 4, 1.0},  {1, 1774, 3, 5, 1.0},  {1, 1775, 3, 6, 1.0},  {1, 1776, 3, 7, 1.0},
  {1, 1777, 3, 8, 1.0},  {1, 1778, 3, 9, 1.0},  {1, 1779, 3, 10, 1.0},  {1, 1780, 3, 11, 1.0},  {1, 1781, 3, 12, 1.0},  {1, 1782, 3, 13, 1.0},
  {1, 1783, 3, 14, 1.0},  {1, 2059, 4, 4, 1.0},  {1, 2060, 4, 5, 1.0},  {1, 2061, 4, 6, 1.0},  {1, 2062, 4, 7, 1.0},  {1, 2063, 4, 8, 1.0},
  {1, 2064, 4, 9, 1.0},  {1, 2065, 4, 10, 1.0},  {1, 2066, 4, 11, 1.0},  {1, 2067, 4, 12, 1.0},  {1, 2068, 4, 13, 1.0},  {1, 2069, 4, 14, 1.0},
  {1, 2070, 5, 5, 1.0},  {1, 2071, 5, 6, 1.0},  {1, 2072, 5, 7, 1.0},  {1, 2073, 5, 8, 1.0},  {1, 2074, 5, 9, 1.0},  {1, 2075, 5, 10, 1.0},
  {1, 2076, 5, 11, 1.0},  {1, 2077, 5, 12, 1.0},  {1, 2078, 5, 13, 1.0},  {1, 2079, 5, 14, 1.0},  {1, 2080, 6, 6, 1.0},  {1, 2081, 6, 7, 1.0},
  {1, 2082, 6, 8, 1.0},  {1, 2083, 6, 9, 1.0},  {1, 2084, 6, 10, 1.0},  {1, 2085, 6, 11, 1.0},  {1, 2086, 6, 12, 1.0},  {1, 2087, 6, 13, 1.0},
  {1, 2088, 6, 14, 1.0},  {1, 2089, 7, 7, 1.0},  {1, 2090, 7, 8, 1.0},  {1, 2091, 7, 9, 1.0},  {1, 2092, 7, 10, 1.0},  {1, 2093, 7, 11, 1.0},
  {1, 2094, 7, 12, 1.0},  {1, 2095, 7, 13, 1.0},  {1, 2096, 7, 14, 1.0},  {1, 2097, 8, 8, 1.0},  {1, 2098, 8, 9, 1.0},  {1, 2099, 8, 10, 1.0},
  {1, 2100, 8, 11, 1.0},  {1, 2101, 8, 12, 1.0},  {1, 2102, 8, 13, 1.0},  {1, 2103, 8, 14, 1.0},  {1, 2104, 9, 9, 1.0},  {1, 2105, 9, 10, 1.0},
  {1, 2106, 9, 11, 1.0},  {1, 2107, 9, 12, 1.0},  {1, 2108, 9, 13, 1.0},  {1, 2109, 9, 14, 1.0},  {1, 2110, 10, 10, 1.0},  {1, 2111, 10, 11, 1.0},
  {1, 2112, 10, 12, 1.0},  {1, 2113, 10, 13, 1.0},  {1, 2114, 10, 14, 1.0},  {1, 2115, 11, 11, 1.0},  {1, 2116, 11, 12, 1.0},  {1, 2117, 11, 13, 1.0},
  {1, 2118, 11, 14, 1.0},  {1, 2119, 12, 12, 1.0},  {1, 2120, 12, 13, 1.0},  {1, 2121, 12, 14, 1.0},  {1, 2122, 13, 13, 1.0},  {1, 2123, 13, 14, 1.0},
  {1, 2124, 14, 14, 1.0},  {2, 0, 0, 0, 1.0},  {2, 1, 0, 1, 1.0},  {2, 2, 0, 2, 1.0},  {2, 3, 0, 3, 1.0},  {2, 4, 0, 4, 1.0},
  {2, 5, 0, 5, 1.0},  {2, 6, 0, 6, 1.0},  {2, 7, 0, 7, 1.0},  {2, 8, 0, 8, 1.0},  {2, 9, 0, 9, 1.0},  {2, 10, 0, 10, 1.0},
  {2, 11, 0, 11, 1.0},  {2, 12, 0, 12, 1.0},  {2, 13, 0, 13, 1.0},  {2, 14, 0, 14, 1.0},  {2, 15, 0, 0, -1.0},  {2, 15, 1, 1, 1.0},
  {2, 16, 1, 2, 1.0},  {2, 17, 1, 3, 1.0},  {2, 18, 1, 4, 1.0},  {2, 19, 1, 5, 1.0},  {2, 20, 1, 6, 1.0},  {2, 21, 1, 7, 1.0},
  {2, 22, 1, 8, 1.0},  {2, 23, 1, 9, 1.0},  {2, 24, 1, 10, 1.0},  {2, 25, 1, 11, 1.0},  {2, 26, 1, 12, 1.0},  {2, 27, 1, 13, 1.0},
  {2, 28, 1, 14, 1.0},  {2, 29, 0, 0, -1.0},  {2, 29, 2, 2, 1.0},  {2, 30, 2, 3, 1.0},  {2, 31, 2, 4, 1.0},  {2, 32, 2, 5, 1.0},
  {2, 33, 2, 6, 1.0},  {2, 34, 2, 7, 1.0},  {2, 35, 2, 8, 1.0},  {2, 36, 2, 9, 1.0},  {2, 37, 2, 10, 1.0},  {2, 38, 2, 11, 1.0},
  {2, 39, 2, 12, 1.0},  {2, 40, 2, 13, 1.0},  {2, 41, 2, 14, 1.0},  {2, 42, 0, 0, -1.0},  {2, 42, 3, 3, 1.0},  {2, 43, 3, 4, 1.0},
  {2, 44, 3, 5, 1.0},  {2, 45, 3, 6, 1.0},  {2, 46, 3, 7, 1.0},  {2, 47, 3, 8, 1.0},  {2, 48, 3, 9, 1.0},  {2, 49, 3, 10, 1.0},
  {2, 50, 3, 11, 1.0},  {2, 51, 3, 12, 1.0},  {2, 52, 3, 13, 1.0},  {2, 53, 3, 14, 1.0},  {2, 54, 0, 0, -1.0},  {2, 54, 4, 4, 1.0},
  {2, 55, 4, 5, 1.0},  {2, 56, 4, 6, 1.0},  {2, 57, 4, 7, 1.0},  {2, 58, 4, 8, 1.0},  {2, 59, 4, 9, 1.0},  {2, 60, 4, 10, 1.0},
  {2, 61, 4, 11, 1.0},  {2, 62, 4, 12, 1.0},  {2, 63, 4, 13, 1.0},  {2, 64, 4, 14, 1.0},  {2, 65, 5, 5, 1.0},  {2, 66, 5, 6, 1.0},
  {2, 67, 5, 7, 1.0},  {2, 68, 5, 8, 1.0},  {2, 69, 5, 9, 1.0},  {2, 70, 5, 10, 1.0},  {2, 71, 5, 11, 1.0},  {2, 72, 5, 12, 1.0},
  {2, 73, 5, 13, 1.0},  {2, 74, 5, 14, 1.0},  {2, 75, 6, 6, 1.0},  {2, 76, 6, 7, 1.0},  {2, 77, 6, 8, 1.0},  {2, 78, 6, 9, 1.0},
  {2, 79, 6, 10, 1.0},  {2, 80, 6, 11, 1.0},  {2, 81, 6, 12, 1.0},  {2, 82, 6, 13, 1.0},  {2, 83, 6, 14, 1.0},  {2, 84, 7, 7, 1.0},
  {2, 85, 7, 8, 1.0},  {2, 86, 7, 9, 1.0},  {2, 87, 7, 10, 1.0},  {2, 88, 7, 11, 1.0},  {2, 89, 7, 12, 1.0},  {2, 90, 7, 13, 1.0},
  {2, 91, 7, 14, 1.0},  {2, 92, 8, 8, 1.0},  {2, 93, 8, 9, 1.0},  {2, 94, 8, 10, 1.0},  {2, 95, 8, 11, 1.0},  {2, 96, 8, 12, 1.0},
  {2, 97, 8, 13, 1.0},  {2, 98, 8, 14, 1.0},  {2, 99, 9, 9, 1.0},  {2, 100, 9, 10, 1.0},  {2, 101, 9, 11, 1.0},  {2, 102, 9, 12, 1.0},
  {2, 103, 9, 13, 1.0},  {2, 104, 9, 14, 1.0},  {2, 105, 10, 10, 1.0},  {2, 106, 10, 11, 1.0},  {2, 107, 10, 12, 1.0},  {2, 108, 10, 13, 1.0},
  {2, 109, 10, 14, 1.0},  {2, 110, 11, 11, 1.0},  {2, 111, 11, 12, 1.0},  {2, 112, 11, 13, 1.0},  {2, 113, 11, 14, 1.0},  {2, 114, 12, 12, 1.0},
  {2, 115, 12, 13, 1.0},  {2, 116, 12, 14, 1.0},  {2, 117, 13, 13, 1.0},  {2, 118, 13, 14, 1.0},  {2, 119, 14, 14, 1.0},  {2, 120, 0, 1, -1.0},
  {2, 121, 0, 2, -1.0},  {2, 122, 0, 3, -1.0},  {2, 123, 0, 4, -1.0},  {2, 124, 0, 5, -1.0},  {2, 125, 0, 6, -1.0},  {2, 126, 0, 7, -1.0},
  {2, 127, 0, 8, -1.0},  {2, 128, 0, 9, -1.0},  {2, 129, 0, 10, -1.0},  {2, 130, 0, 11, -1.0},  {2, 131, 0, 12, -1.0},  {2, 132, 0, 13, -1.0},
  {2, 133, 0, 14, -1.0},  {2, 134, 0, 1, -1.0},  {2, 147, 0, 1, -1.0},  {2, 159, 0, 1, -1.0},  {2, 225, 0, 2, -1.0},  {2, 226, 0, 3, -1.0},
  {2, 227, 0, 4, -1.0},  {2, 228, 0, 5, -1.0},  {2, 229, 0, 6, -1.0},  {2, 230, 0, 7, -1.0},  {2, 231, 0, 8, -1.0},  {2, 232, 0, 9, -1.0},
  {2, 233, 0, 10, -1.0},  {2, 234, 0, 11, -1.0},  {2, 235, 0, 12, -1.0},  {2, 236, 0, 13, -1.0},  {2, 237, 0, 14, -1.0},  {2, 238, 0, 2, -1.0},
  {2, 250, 0, 2, -1.0},  {2, 316, 0, 3, -1.0},  {2, 317, 0, 4, -1.0},  {2, 318, 0, 5, -1.0},  {2, 319, 0, 6, -1.0},  {2, 320, 0, 7, -1.0},
  {2, 321, 0, 8, -1.0},  {2, 322, 0, 9, -1.0},  {2, 323, 0, 10, -1.0},  {2, 324, 0, 11, -1.0},  {2, 325, 0, 12, -1.0},  {2, 326, 0, 13, -1.0},
  {2, 327, 0, 14, -1.0},  {2, 328, 0, 3, -1.0},  {2, 394, 0, 4, -1.0},  {2, 395, 0, 5, -1.0},  {2, 396, 0, 6, -1.0},  {2, 397, 0, 7, -1.0},
  {2, 398, 0, 8, -1.0},  {2, 399, 0, 9, -1.0},  {2, 400, 0, 10, -1.0},  {2, 401, 0, 11, -1.0},  {2, 402, 0, 12, -1.0},  {2, 403, 0, 13, -1.0},
  {2, 404, 0, 14, -1.0},  {2, 680, 1, 1, -1.0},  {2, 681, 1, 2, -1.0},  {2, 682, 1, 3, -1.0},  {2, 683, 1, 4, -1.0},  {2, 684, 1, 5, -1.0},
  {2, 685, 1, 6, -1.0},  {2, 686, 1, 7, -1.0},  {2, 687, 1, 8, -1.0},  {2, 688, 1, 9, -1.0},  {2, 689, 1, 10, -1.0},  {2, 690, 1, 11, -1.0},
  {2, 691, 1, 12, -1.0},  {2, 692, 1, 13, -1.0},  {2, 693, 1, 14, -1.0},  {2, 694, 1, 1, -1.0},  {2, 694, 2, 2, -1.0},  {2, 695, 2, 3, -1.0},
  {2, 696, 2, 4, -1.0},  {2, 697, 2, 5, -1.0},  {2, 698, 2, 6, -1.0},  {2, 699, 2, 7, -1.0},  {2, 700, 2, 8, -1.0},  {2, 701, 2, 9, -1.0},
  {2, 702, 2, 10, -1.0},  {2, 703, 2, 11, -1.0},  {2, 704, 2, 12, -1.0},  {2, 705, 2, 13, -1.0},  {2, 706, 2, 14, -1.0},  {2, 707, 1, 1, -1.0},
  {2, 707, 3, 3, -1.0},  {2, 708, 3, 4, -1.0},  {2, 709, 3, 5, -1.0},  {2, 710, 3, 6, -1.0},  {2, 711, 3, 7, -1.0},  {2, 712, 3, 8, -1.0},
  {2, 713, 3, 9, -1.0},  {2, 714, 3, 10, -1.0},  {2, 715, 3, 11, -1.0},  {2, 716, 3, 12, -1.0},  {2, 717, 3, 13, -1.0},  {2, 718, 3, 14, -1.0},
  {2, 719, 1, 1, -1.0},  {2, 719, 4, 4, -1.0},  {2, 720, 4, 5, -1.0},  {2, 721, 4, 6, -1.0},  {2, 722, 4, 7, -1.0},  {2, 723, 4, 8, -1.0},
  {2, 724, 4, 9, -1.0},  {2, 725, 4, 10, -1.0},  {2, 726, 4, 11, -1.0},  {2, 727, 4, 12, -1.0},  {2, 728, 4, 13, -1.0},  {2, 729, 4, 14, -1.0},
  {2, 730, 5, 5, -1.0},  {2, 731, 5, 6, -1.0},  {2, 732, 5, 7, -1.0},  {2, 733, 5, 8, -1.0},  {2, 734, 5, 9, -1.0},  {2, 735, 5, 10, -1.0},
  {2, 736, 5, 11, -1.0},  {2, 737, 5, 12, -1.0},  {2, 738, 5, 13, -1.0},  {2, 739, 5, 14, -1.0},  {2, 740, 6, 6, -1.0},  {2, 741, 6, 7, -1.0},
  {2, 742, 6, 8, -1.0},  {2, 743, 6, 9, -1.0},  {2, 744, 6, 10, -1.0},  {2, 745, 6, 11, -1.0},  {2, 746, 6, 12, -1.0},  {2, 747, 6, 13, -1.0},
  {2, 748, 6, 14, -1.0},  {2, 749, 7, 7, -1.0},  {2, 750, 7, 8, -1.0},  {2, 751, 7, 9, -1.0},  {2, 752, 7, 10, -1.0},  {2, 753, 7, 11, -1.0},
  {2, 754, 7, 12, -1.0},  {2, 755, 7, 13, -1.0},  {2, 756, 7, 14, -1.0},  {2, 757, 8, 8, -1.0},  {2, 758, 8, 9, -1.0},  {2, 759, 8, 10, -1.0},
  {2, 760, 8, 11, -1.0},  {2, 761, 8, 12, -1.0},  {2, 762, 8, 13, -1.0},  {2, 763, 8, 14, -1.0},  {2, 764, 9, 9, -1.0},  {2, 765, 9, 10, -1.0},
  {2, 766, 9, 11, -1.0},  {2, 767, 9, 12, -1.0},  {2, 768, 9, 13, -1.0},  {2, 769, 9, 14, -1.0},  {2, 770, 10, 10, -1.0},  {2, 771, 10, 11, -1.0},
  {2, 772, 10, 12, -1.0},  {2, 773, 10, 13, -1.0},  {2, 774, 10, 14, -1.0},  {2, 775, 11, 11, -1.0},  {2, 776, 11, 12, -1.0},  {2, 777, 11, 13, -1.0},
  {2, 778, 11, 14, -1.0},  {2, 779, 12, 12, -1.0},  {2, 780, 12, 13, -1.0},  {2, 781, 12, 14, -1.0},  {2, 782, 13, 13, -1.0},  {2, 783, 13, 14, -1.0},
  {2, 784, 14, 14, -1.0},  {2, 785, 1, 2, -1.0},  {2, 786, 1, 3, -1.0},  {2, 787, 1, 4, -1.0},  {2, 788, 1, 5, -1.0},  {2, 789, 1, 6, -1.0},
  {2, 790, 1, 7, -1.0},  {2, 791, 1, 8, -1.0},  {2, 792, 1, 9, -1.0},  {2, 793, 1, 10, -1.0},  {2, 794, 1, 11, -1.0},  {2, 795, 1, 12, -1.0},
  {2, 796, 1, 13, -1.0},  {2, 797, 1, 14, -1.0},  {2, 798, 1, 2, -1.0},  {2, 810, 1, 2, -1.0},  {2, 876, 1, 3, -1.0},  {2, 877, 1, 4, -1.0},
  {2, 878, 1, 5, -1.0},  {2, 879, 1, 6, -1.0},  {2, 880, 1, 7, -1.0},  {2, 881, 1, 8, -1.0},  {2, 882, 1, 9, -1.0},  {2, 883, 1, 10, -1.0},
  {2, 884, 1, 11, -1.0},  {2, 885, 1, 12, -1.0},  {2, 886, 1, 13, -1.0},  {2, 887, 1, 14, -1.0},  {2, 888, 1, 3, -1.0},  {2, 954, 1, 4, -1.0},
  {2, 955, 1, 5, -1.0},  {2, 956, 1, 6, -1.0},  {2, 957, 1, 7, -1.0},  {2, 958, 1, 8, -1.0},  {2, 959, 1, 9, -1.0},  {2, 960, 1, 10, -1.0},
  {2, 961, 1, 11, -1.0},  {2, 962, 1, 12, -1.0},  {2, 963, 1, 13, -1.0},  {2, 964, 1, 14, -1.0},  {2, 1240, 2, 2, -1.0},  {2, 1241, 2, 3, -1.0},
  {2, 1242, 2, 4, -1.0},  {2, 1243, 2, 5, -1.0},  {2, 1244, 2, 6, -1.0},  {2, 1245, 2, 7, -1.0},  {2, 1246, 2, 8, -1.0},  {2, 1247, 2, 9, -1.0},
  {2, 1248, 2, 10, -1.0},  {2, 1249, 2, 11, -1.0},  {2, 1250, 2, 12, -1.0},  {2, 1251, 2, 13, -1.0},  {2, 1252, 2, 14, -1.0},  {2, 1253, 2, 2, -1.0},
  {2, 1253, 3, 3, -1.0},  {2, 1254, 3, 4, -1.0},  {2, 1255, 3, 5, -1.0},  {2, 1256, 3, 6, -1.0},  {2, 1257, 3, 7, -1.0},  {2, 1258, 3, 8, -1.0},
  {2, 1259, 3, 9, -1.0},  {2, 1260, 3, 10, -1.0},  {2, 1261, 3, 11, -1.0},  {2, 1262, 3, 12, -1.0},  {2, 1263, 3, 13, -1.0},  {2, 1264, 3, 14, -1.0},
  {2, 1265, 2, 2, -1.0},  {2, 1265, 4, 4, -1.0},  {2, 1266, 4, 5, -1.0},  {2, 1267, 4, 6, -1.0},  {2, 1268, 4, 7, -1.0},  {2, 1269, 4, 8, -1.0},
  {2, 1270, 4, 9, -1.0},  {2, 1271, 4, 10, -1.0},  {2, 1272, 4, 11, -1.0},  {2, 1273, 4, 12, -1.0},  {2, 1274, 4, 13, -1.0},  {2, 1275, 4, 14, -1.0},
  {2, 1276, 5, 5, -1.0},  {2, 1277, 5, 6, -1.0},  {2, 1278, 5, 7, -1.0},  {2, 1279, 5, 8, -1.0},  {2, 1280, 5, 9, -1.0},  {2, 1281, 5, 10, -1.0},
  {2, 1282, 5, 11, -1.0},  {2, 1283, 5, 12, -1.0},  {2, 1284, 5, 13, -1.0},  {2, 1285, 5, 14, -1.0},  {2, 1286, 6, 6, -1.0},  {2, 1287, 6, 7, -1.0},
  {2, 1288, 6, 8, -1.0},  {2, 1289, 6, 9, -1.0},  {2, 1290, 6, 10, -1.0},  {2, 1291, 6, 11, -1.0},  {2, 1292, 6, 12, -1.0},  {2, 1293, 6, 13, -1.0},
  {2, 1294, 6, 14, -1.0},  {2, 1295, 7, 7, -1.0},  {2, 1296, 7, 8, -1.0},  {2, 1297, 7, 9, -1.0},  {2, 1298, 7, 10, -1.0},  {2, 1299, 7, 11, -1.0},
  {2, 1300, 7, 12, -1.0},  {2, 1301, 7, 13, -1.0},  {2, 1302, 7, 14, -1.0},  {2, 1303, 8, 8, -1.0},  {2, 1304, 8, 9, -1.0},  {2, 1305, 8, 10, -1.0},
  {2, 1306, 8, 11, -1.0},  {2, 1307, 8, 12, -1.0},  {2, 1308, 8, 13, -1.0},  {2, 1309, 8, 14, -1.0},  {2, 1310, 9, 9, -1.0},  {2, 1311, 9, 10, -1.0},
  {2, 1312, 9, 11, -1.0},  {2, 1313, 9, 12, -1.0},  {2, 1314, 9, 13, -1.0},  {2, 1315, 9, 14, -1.0},  {2, 1316, 10, 10, -1.0},  {2, 1317, 10, 11, -1.0},
  {2, 1318, 10, 12, -1.0},  {2, 1319, 10, 13, -1.0},  {2, 1320, 10, 14, -1.0},  {2, 1321, 11, 11, -1.0},  {2, 1322, 11, 12, -1.0},  {2, 1323, 11, 13, -1.0},
  {2, 1324, 11, 14, -1.0},  {2, 1325, 12, 12, -1.0},  {2, 1326, 12, 13, -1.0},  {2, 1327, 12, 14, -1.0},  {2, 1328, 13, 13, -1.0},  {2, 1329, 13, 14, -1.0},
  {2, 1330, 14, 14, -1.0},  {2, 1331, 2, 3, -1.0},  {2, 1332, 2, 4, -1.0},  {2, 1333, 2, 5, -1.0},  {2, 1334, 2, 6, -1.0},  {2, 1335, 2, 7, -1.0},
  {2, 1336, 2, 8, -1.0},  {2, 1337, 2, 9, -1.0},  {2, 1338, 2, 10, -1.0},  {2, 1339, 2, 11, -1.0},  {2, 1340, 2, 12, -1.0},  {2, 1341, 2, 13, -1.0},
  {2, 1342, 2, 14, -1.0},  {2, 1343, 2, 3, -1.0},  {2, 1409, 2, 4, -1.0},  {2, 1410, 2, 5, -1.0},  {2, 1411, 2, 6, -1.0},  {2, 1412, 2, 7, -1.0},
  {2, 1413, 2, 8, -1.0},  {2, 1414, 2, 9, -1.0},  {2, 1415, 2, 10, -1.0},  {2, 1416, 2, 11, -1.0},  {2, 1417, 2, 12, -1.0},  {2, 1418, 2, 13, -1.0},
  {2, 1419, 2, 14, -1.0},  {2, 1695, 3, 3, -1.0},  {2, 1696, 3, 4, -1.0},  {2, 1697, 3, 5, -1.0},  {2, 1698, 3, 6, -1.0},  {2, 1699, 3, 7, -1.0},
  {2, 1700, 3, 8, -1.0},  {2, 1701, 3, 9, -1.0},  {2, 1702, 3, 10, -1.0},  {2, 1703, 3, 11, -1.0},  {2, 1704, 3, 12, -1.0},  {2, 1705, 3, 13, -1.0},
  {2, 1706, 3, 14, -1.0},  {2, 1707, 3, 3, -1.0},  {2, 1707, 4, 4, -1.0},  {2, 1708, 4, 5, -1.0},  {2, 1709, 4, 6, -1.0},  {2, 1710, 4, 7, -1.0},
  {2, 1711, 4, 8, -1.0},  {2, 1712, 4, 9, -1.0},  {2, 1713, 4, 10, -1.0},  {2, 1714, 4, 11, -1.0},  {2, 1715, 4, 12, -1.0},  {2, 1716, 4, 13, -1.0},
  {2, 1717, 4, 14, -1.0},  {2, 1718, 5, 5, -1.0},  {2, 1719, 5, 6, -1.0},  {2, 1720, 5, 7, -1.0},  {2, 1721, 5, 8, -1.0},  {2, 1722, 5, 9, -1.0},
  {2, 1723, 5, 10, -1.0},  {2, 1724, 5, 11, -1.0},  {2, 1725, 5, 12, -1.0},  {2, 1726, 5, 13, -1.0},  {2, 1727, 5, 14, -1.0},  {2, 1728, 6, 6, -1.0},
  {2, 1729, 6, 7, -1.0},  {2, 1730, 6, 8, -1.0},  {2, 1731, 6, 9, -1.0},  {2, 1732, 6, 10, -1.0},  {2, 1733, 6, 11, -1.0},  {2, 1734, 6, 12, -1.0},
  {2, 1735, 6, 13, -1.0},  {2, 1736, 6, 14, -1.0},  {2, 1737, 7, 7, -1.0},  {2, 1738, 7, 8, -1.0},  {2, 1739, 7, 9, -1.0},  {2, 1740, 7, 10, -1.0},
  {2, 1741, 7, 11, -1.0},  {2, 1742, 7, 12, -1.0},  {2, 1743, 7, 13, -1.0},  {2, 1744, 7, 14, -1.0},  {2, 1745, 8, 8, -1.0},  {2, 1746, 8, 9, -1.0},
  {2, 1747, 8, 10, -1.0},  {2, 1748, 8, 11, -1.0},  {2, 1749, 8, 12, -1.0},  {2, 1750, 8, 13, -1.0},  {2, 1751, 8, 14, -1.0},  {2, 1752, 9, 9, -1.0},
  {2, 1753, 9, 10, -1.0},  {2, 1754, 9, 11, -1.0},  {2, 1755, 9, 12, -1.0},  {2, 1756, 9, 13, -1.0},  {2, 1757, 9, 14, -1.0},  {2, 1758, 10, 10, -1.0},
  {2, 1759, 10, 11, -1.0},  {2, 1760, 10, 12, -1.0},  {2, 1761, 10, 13, -1.0},  {2, 1762, 10, 14, -1.0},  {2, 1763, 11, 11, -1.0},  {2, 1764, 11, 12, -1.0},
  {2, 1765, 11, 13, -1.0},  {2, 1766, 11, 14, -1.0},  {2, 1767, 12, 12, -1.0},  {2, 1768, 12, 13, -1.0},  {2, 1769, 12, 14, -1.0},  {2, 1770, 13, 13, -1.0},
  {2, 1771, 13, 14, -1.0},  {2, 1772, 14, 14, -1.0},  {2, 1773, 3, 4, -1.0},  {2, 1774, 3, 5, -1.0},  {2, 1775, 3, 6, -1.0},  {2, 1776, 3, 7, -1.0},
  {2, 1777, 3, 8, -1.0},  {2, 1778, 3, 9, -1.0},  {2, 1779, 3, 10, -1.0},  {2, 1780, 3, 11, -1.0},  {2, 1781, 3, 12, -1.0},  {2, 1782, 3, 13, -1.0},
  {2, 1783, 3, 14, -1.0},  {2, 2059, 4, 4, -1.0},  {2, 2060, 4, 5, -1.0},  {2, 2061, 4, 6, -1.0},  {2, 2062, 4, 7, -1.0},  {2, 2063, 4, 8, -1.0},
  {2, 2064, 4, 9, -1.0},  {2, 2065, 4, 10, -1.0},  {2, 2066, 4, 11, -1.0},  {2, 2067, 4, 12, -1.0},  {2, 2068, 4, 13, -1.0},  {2, 2069, 4, 14, -1.0},
  {2, 2070, 5, 5, -1.0},  {2, 2071, 5, 6, -1.0},  {2, 2072, 5, 7, -1.0},  {2, 2073, 5, 8, -1.0},  {2, 2074, 5, 9, -1.0},  {2, 2075, 5, 10, -1.0},
  {2, 2076, 5, 11, -1.0},  {2, 2077, 5, 12, -1.0},  {2, 2078, 5, 13, -1.0},  {2, 2079, 5, 14, -1.0},  {2, 2080, 6, 6, -1.0},  {2, 2081, 6, 7, -1.0},
  {2, 2082, 6, 8, -1.0},  {2, 2083, 6, 9, -1.0},  {2, 2084, 6, 10, -1.0},  {2, 2085, 6, 11, -1.0},  {2, 2086, 6, 12, -1.0},  {2, 2087, 6, 13, -1.0},
  {2, 2088, 6, 14, -1.0},  {2, 2089, 7, 7, -1.0},  {2, 2090, 7, 8, -1.0},  {2, 2091, 7, 9, -1.0},  {2, 2092, 7, 10, -1.0},  {2, 2093, 7, 11, -1.0},
  {2, 2094, 7, 12, -1.0},  {2, 2095, 7, 13, -1.0},  {2, 2096, 7, 14, -1.0},  {2, 2097, 8, 8, -1.0},  {2, 2098, 8, 9, -1.0},  {2, 2099, 8, 10, -1.0},
  {2, 2100, 8, 11, -1.0},  {2, 2101, 8, 12, -1.0},  {2, 2102, 8, 13, -1.0},  {2, 2103, 8, 14, -1.0},  {2, 2104, 9, 9, -1.0},  {2, 2105, 9, 10, -1.0},
  {2, 2106, 9, 11, -1.0},  {2, 2107, 9, 12, -1.0},  {2, 2108, 9, 13, -1.0},  {2, 2109, 9, 14, -1.0},  {2, 2110, 10, 10, -1.0},  {2, 2111, 10, 11, -1.0},
  {2, 2112, 10, 12, -1.0},  {2, 2113, 10, 13, -1.0},  {2, 2114, 10, 14, -1.0},  {2, 2115, 11, 11, -1.0},  {2, 2116, 11, 12, -1.0},  {2, 2117, 11, 13, -1.0},
  {2, 2118, 11, 14, -1.0},  {2, 2119, 12, 12, -1.0},  {2, 2120, 12, 13, -1.0},  {2, 2121, 12, 14, -1.0},  {2, 2122, 13, 13, -1.0},  {2, 2123, 13, 14, -1.0},
  {2, 2124, 14, 14, -1.0},  {3, 1, 0, 0, 1.0},  {3, 15, 0, 1, 1.0},  {3, 16, 0, 2, 1.0},  {3, 17, 0, 3, 1.0},  {3, 18, 0, 4, 1.0},
  {3, 19, 0, 5, 1.0},  {3, 20, 0, 6, 1.0},  {3, 21, 0, 7, 1.0},  {3, 22, 0, 8, 1.0},  {3, 23, 0, 9, 1.0},  {3, 24, 0, 10, 1.0},
  {3, 25, 0, 11, 1.0},  {3, 26, 0, 12, 1.0},  {3, 27, 0, 13, 1.0},  {3, 28, 0, 14, 1.0},  {3, 120, 1, 1, 1.0},  {3, 121, 1, 2, 1.0},
  {3, 122, 1, 3, 1.0},  {3, 123, 1, 4, 1.0},  {3, 124, 1, 5, 1.0},  {3, 125, 1, 6, 1.0},  {3, 126, 1, 7, 1.0},  {3, 127, 1, 8, 1.0},
  {3, 128, 1, 9, 1.0},  {3, 129, 1, 10, 1.0},  {3, 130, 1, 11, 1.0},  {3, 131, 1, 12, 1.0},  {3, 132, 1, 13, 1.0},  {3, 133, 1, 14, 1.0},
  {3, 134, 2, 2, 1.0},  {3, 135, 2, 3, 1.0},  {3, 136, 2, 4, 1.0},  {3, 137, 2, 5, 1.0},  {3, 138, 2, 6, 1.0},  {3, 139, 2, 7, 1.0},
  {3, 140, 2, 8, 1.0},  {3, 141, 2, 9, 1.0},  {3, 142, 2, 10, 1.0},  {3, 143, 2, 11, 1.0},  {3, 144, 2, 12, 1.0},  {3, 145, 2, 13, 1.0},
  {3, 146, 2, 14, 1.0},  {3, 147, 3, 3, 1.0},  {3, 148, 3, 4, 1.0},  {3, 149, 3, 5, 1.0},  {3, 150, 3, 6, 1.0},  {3, 151, 3, 7, 1.0},
  {3, 152, 3, 8, 1.0},  {3, 153, 3, 9, 1.0},  {3, 154, 3, 10, 1.0},  {3, 155, 3, 11, 1.0},  {3, 156, 3, 12, 1.0},  {3, 157, 3, 13, 1.0},
  {3, 158, 3, 14, 1.0},  {3, 159, 4, 4, 1.0},  {3, 160, 4, 5, 1.0},  {3, 161, 4, 6, 1.0},  {3, 162, 4, 7, 1.0},  {3, 163, 4, 8, 1.0},
  {3, 164, 4, 9, 1.0},  {3, 165, 4, 10, 1.0},  {3, 166, 4, 11, 1.0},  {3, 167, 4, 12, 1.0},  {3, 168, 4, 13, 1.0},  {3, 169, 4, 14, 1.0},
  {3, 170, 5, 5, 1.0},  {3, 171, 5, 6, 1.0},  {3, 172, 5, 7, 1.0},  {3, 173, 5, 8, 1.0},  {3, 174, 5, 9, 1.0},  {3, 175, 5, 10, 1.0},
  {3, 176, 5, 11, 1.0},  {3, 177, 5, 12, 1.0},  {3, 178, 5, 13, 1.0},  {3, 179, 5, 14, 1.0},  {3, 180, 6, 6, 1.0},  {3, 181, 6, 7, 1.0},
  {3, 182, 6, 8, 1.0},  {3, 183, 6, 9, 1.0},  {3, 184, 6, 10, 1.0},  {3, 185, 6, 11, 1.0},  {3, 186, 6, 12, 1.0},  {3, 187, 6, 13, 1.0},
  {3, 188, 6, 14, 1.0},  {3, 189, 7, 7, 1.0},  {3, 190, 7, 8, 1.0},  {3, 191, 7, 9, 1.0},  {3, 192, 7, 10, 1.0},  {3, 193, 7, 11, 1.0},
  {3, 194, 7, 12, 1.0},  {3, 195, 7, 13, 1.0},  {3, 196, 7, 14, 1.0},  {3, 197, 8, 8, 1.0},  {3, 198, 8, 9, 1.0},  {3, 199, 8, 10, 1.0},
  {3, 200, 8, 11, 1.0},  {3, 201, 8, 12, 1.0},  {3, 202, 8, 13, 1.0},  {3, 203, 8, 14, 1.0},  {3, 204, 9, 9, 1.0},  {3, 205, 9, 10, 1.0},
  {3, 206, 9, 11, 1.0},  {3, 207, 9, 12, 1.0},  {3, 208, 9, 13, 1.0},  {3, 209, 9, 14, 1.0},  {3, 210, 10, 10, 1.0},  {3, 211, 10, 11, 1.0},
  {3, 212, 10, 12, 1.0},  {3, 213, 10, 13, 1.0},  {3, 214, 10, 14, 1.0},  {3, 215, 11, 11, 1.0},  {3, 216, 11, 12, 1.0},  {3, 217, 11, 13, 1.0},
  {3, 218, 11, 14, 1.0},  {3, 219, 12, 12, 1.0},  {3, 220, 12, 13, 1.0},  {3, 221, 12, 14, 1.0},  {3, 222, 13, 13, 1.0},  {3, 223, 13, 14, 1.0},
  {3, 224, 14, 14, 1.0},  {4, 0, 0, 0, 0.0},  {4, 1, 0, 1, 0.0},  {4, 2, 0, 2, 0.0},  {4, 3, 0, 3, 0.0},  {4, 4, 0, 4, 0.0},
  {4, 5, 0, 5, 0.0},  {4, 6, 0, 6, 0.0},  {4, 7, 0, 7, 0.0},  {4, 8, 0, 8, 0.0},  {4, 9, 0, 9, 0.0},  {4, 10, 0, 10, 0.0},
  {4, 11, 0, 11, 0.0},  {4, 12, 0, 12, 0.0},  {4, 13, 0, 13, 0.0},  {4, 14, 0, 14, 0.0},  {4, 15, 1, 1, 0.0},  {4, 16, 1, 2, 0.0},
  {4, 17, 1, 3, 0.0},  {4, 18, 1, 4, 0.0},  {4, 19, 1, 5, 0.0},  {4, 20, 1, 6, 0.0},  {4, 21, 1, 7, 0.0},  {4, 22, 1, 8, 0.0},
  {4, 23, 1, 9, 0.0},  {4, 24, 1, 10, 0.0},  {4, 25, 1, 11, 0.0},  {4, 26, 1, 12, 0.0},  {4, 27, 1, 13, 0.0},  {4, 28, 1, 14, 0.0},
  {4, 29, 2, 2, 0.0},  {4, 30, 2, 3, 0.0},  {4, 31, 2, 4, 0.0},  {4, 32, 2, 5, 0.0},  {4, 33, 2, 6, 0.0},  {4, 34, 2, 7, 0.0},
  {4, 35, 2, 8, 0.0},  {4, 36, 2, 9, 0.0},  {4, 37, 2, 10, 0.0},  {4, 38, 2, 11, 0.0},  {4, 39, 2, 12, 0.0},  {4, 40, 2, 13, 0.0},
  {4, 41, 2, 14, 0.0},  {4, 42, 3, 3, 0.0},  {4, 43, 3, 4, 0.0},  {4, 44, 3, 5, 0.0},  {4, 45, 3, 6, 0.0},  {4, 46, 3, 7, 0.0},
  {4, 47, 3, 8, 0.0},  {4, 48, 3, 9, 0.0},  {4, 49, 3, 10, 0.0},  {4, 50, 3, 11, 0.0},  {4, 51, 3, 12, 0.0},  {4, 52, 3, 13, 0.0},
  {4, 53, 3, 14, 0.0},  {4, 54, 4, 4, 0.0},  {4, 55, 4, 5, 0.0},  {4, 56, 4, 6, 0.0},  {4, 57, 4, 7, 0.0},  {4, 58, 4, 8, 0.0},
  {4, 59, 4, 9, 0.0},  {4, 60, 4, 10, 0.0},  {4, 61, 4, 11, 0.0},  {4, 62, 4, 12, 0.0},  {4, 63, 4, 13, 0.0},  {4, 64, 4, 14, 0.0},
  {4, 65, 0, 0, -1.0},  {4, 65, 5, 5, 0.0},  {4, 66, 5, 6, 0.0},  {4, 67, 5, 7, 0.0},  {4, 68, 5, 8, 0.0},  {4, 69, 5, 9, 0.0},
  {4, 70, 5, 10, 0.0},  {4, 71, 5, 11, 0.0},  {4, 72, 5, 12, 0.0},  {4, 73, 5, 13, 0.0},  {4, 74, 5, 14, 0.0},  {4, 75, 0, 0, -1.0},
  {4, 75, 6, 6, 0.0},  {4, 76, 6, 7, 0.0},  {4, 77, 6, 8, 0.0},  {4, 78, 6, 9, 0.0},  {4, 79, 6, 10, 0.0},  {4, 80, 6, 11, 0.0},
  {4, 81, 6, 12, 0.0},  {4, 82, 6, 13, 0.0},  {4, 83, 6, 14, 0.0},  {4, 84, 0, 0, -1.0},  {4, 84, 7, 7, 0.0},  {4, 85, 7, 8, 0.0},
  {4, 86, 7, 9, 0.0},  {4, 87, 7, 10, 0.0},  {4, 88, 7, 11, 0.0},  {4, 89, 7, 12, 0.0},  {4, 90, 7, 13, 0.0},  {4, 91, 7, 14, 0.0},
  {4, 92, 8, 8, 0.0},  {4, 93, 8, 9, 0.0},  {4, 94, 8, 10, 0.0},  {4, 95, 8, 11, 0.0},  {4, 96, 8, 12, 0.0},  {4, 97, 8, 13, 0.0},
  {4, 98, 8, 14, 0.0},  {4, 99, 9, 9, 0.0},  {4, 100, 9, 10, 0.0},  {4, 101, 9, 11, 0.0},  {4, 102, 9, 12, 0.0},  {4, 103, 9, 13, 0.0},
  {4, 104, 9, 14, 0.0},  {4, 105, 10, 10, 0.0},  {4, 106, 10, 11, 0.0},  {4, 107, 10, 12, 0.0},  {4, 108, 10, 13, 0.0},  {4, 109, 10, 14, 0.0},
  {4, 110, 11, 11, 0.0},  {4, 111, 11, 12, 0.0},  {4, 112, 11, 13, 0.0},  {4, 113, 11, 14, 0.0},  {4, 114, 12, 12, 0.0},  {4, 115, 12, 13, 0.0},
  {4, 116, 12, 14, 0.0},  {4, 117, 13, 13, 0.0},  {4, 118, 13, 14, 0.0},  {4, 119, 14, 14, 0.0},  {4, 170, 0, 1, -1.0},  {4, 180, 0, 1, -1.0},
  {4, 189, 0, 1, -1.0},  {4, 261, 0, 2, -1.0},  {4, 271, 0, 2, -1.0},  {4, 280, 0, 2, -1.0},  {4, 339, 0, 3, -1.0},  {4, 349, 0, 3, -1.0},
  {4, 358, 0, 3, -1.0},  {4, 405, 0, 4, -1.0},  {4, 415, 0, 4, -1.0},  {4, 424, 0, 4, -1.0},  {4, 460, 0, 5, -1.0},  {4, 461, 0, 6, -1.0},
  {4, 462, 0, 7, -1.0},  {4, 463, 0, 8, -1.0},  {4, 464, 0, 9, -1.0},  {4, 465, 0, 10, -1.0},  {4, 466, 0, 11, -1.0},  {4, 467, 0, 12, -1.0},
  {4, 468, 0, 13, -1.0},  {4, 469, 0, 14, -1.0},  {4, 470, 0, 5, -1.0},  {4, 479, 0, 5, -1.0},  {4, 515, 0, 6, -1.0},  {4, 516, 0, 7, -1.0},
  {4, 517, 0, 8, -1.0},  {4, 518, 0, 9, -1.0},  {4, 519, 0, 10, -1.0},  {4, 520, 0, 11, -1.0},  {4, 521, 0, 12, -1.0},  {4, 522, 0, 13, -1.0},
  {4, 523, 0, 14, -1.0},  {4, 524, 0, 6, -1.0},  {4, 560, 0, 7, -1.0},  {4, 561, 0, 8, -1.0},  {4, 562, 0, 9, -1.0},  {4, 563, 0, 10, -1.0},
  {4, 564, 0, 11, -1.0},  {4, 565, 0, 12, -1.0},  {4, 566, 0, 13, -1.0},  {4, 567, 0, 14, -1.0},  {4, 730, 1, 1, -1.0},  {4, 740, 1, 1, -1.0},
  {4, 749, 1, 1, -1.0},  {4, 821, 1, 2, -1.0},  {4, 831, 1, 2, -1.0},  {4, 840, 1, 2, -1.0},  {4, 899, 1, 3, -1.0},  {4, 909, 1, 3, -1.0},
  {4, 918, 1, 3, -1.0},  {4, 965, 1, 4, -1.0},  {4, 975, 1, 4, -1.0},  {4, 984, 1, 4, -1.0},  {4, 1020, 1, 5, -1.0},  {4, 1021, 1, 6, -1.0},
  {4, 1022, 1, 7, -1.0},  {4, 1023, 1, 8, -1.0},  {4, 1024, 1, 9, -1.0},  {4, 1025, 1, 10, -1.0},  {4, 1026, 1, 11, -1.0},  {4, 1027, 1, 12, -1.0},
  {4, 1028, 1, 13, -1.0},  {4, 1029, 1, 14, -1.0},  {4, 1030, 1, 5, -1.0},  {4, 1039, 1, 5, -1.0},  {4, 1075, 1, 6, -1.0},  {4, 1076, 1, 7, -1.0},
  {4, 1077, 1, 8, -1.0},  {4, 1078, 1, 9, -1.0},  {4, 1079, 1, 10, -1.0},  {4, 1080, 1, 11, -1.0},  {4, 1081, 1, 12, -1.0},  {4, 1082, 1, 13, -1.0},
  {4, 1083, 1, 14, -1.0},  {4, 1084, 1, 6, -1.0},  {4, 1120, 1, 7, -1.0},  {4, 1121, 1, 8, -1.0},  {4, 1122, 1, 9, -1.0},  {4, 1123, 1, 10, -1.0},
  {4, 1124, 1, 11, -1.0},  {4, 1125, 1, 12, -1.0},  {4, 1126, 1, 13, -1.0},  {4, 1127, 1, 14, -1.0},  {4, 1276, 2, 2, -1.0},  {4, 1286, 2, 2, -1.0},
  {4, 1295, 2, 2, -1.0},  {4, 1354, 2, 3, -1.0},  {4, 1364, 2, 3, -1.0},  {4, 1373, 2, 3, -1.0},  {4, 1420, 2, 4, -1.0},  {4, 1430, 2, 4, -1.0},
  {4, 1439, 2, 4, -1.0},  {4, 1475, 2, 5, -1.0},  {4, 1476, 2, 6, -1.0},  {4, 1477, 2, 7, -1.0},  {4, 1478, 2, 8, -1.0},  {4, 1479, 2, 9, -1.0},
  {4, 1480, 2, 10, -1.0},  {4, 1481, 2, 11, -1.0},  {4, 1482, 2, 12, -1.0},  {4, 1483, 2, 13, -1.0},  {4, 1484, 2, 14, -1.0},  {4, 1485, 2, 5, -1.0},
  {4, 1494, 2, 5, -1.0},  {4, 1530, 2, 6, -1.0},  {4, 1531, 2, 7, -1.0},  {4, 1532, 2, 8, -1.0},  {4, 1533, 2, 9, -1.0},  {4, 1534, 2, 10, -1.0},
  {4, 1535, 2, 11, -1.0},  {4, 1536, 2, 12, -1.0},  {4, 1537, 2, 13, -1.0},  {4, 1538, 2, 14, -1.0},  {4, 1539, 2, 6, -1.0},  {4, 1575, 2, 7, -1.0},
  {4, 1576, 2, 8, -1.0},  {4, 1577, 2, 9, -1.0},  {4, 1578, 2, 10, -1.0},  {4, 1579, 2, 11, -1.0},  {4, 1580, 2, 12, -1.0},  {4, 1581, 2, 13, -1.0},
  {4, 1582, 2, 14, -1.0},  {4, 1718, 3, 3, -1.0},  {4, 1728, 3, 3, -1.0},  {4, 1737, 3, 3, -1.0},  {4, 1784, 3, 4, -1.0},  {4, 1794, 3, 4, -1.0},
  {4, 1803, 3, 4, -1.0},  {4, 1839, 3, 5, -1.0},  {4, 1840, 3, 6, -1.0},  {4, 1841, 3, 7, -1.0},  {4, 1842, 3, 8, -1.0},  {4, 1843, 3, 9, -1.0},
  {4, 1844, 3, 10, -1.0},  {4, 1845, 3, 11, -1.0},  {4, 1846, 3, 12, -1.0},  {4, 1847, 3, 13, -1.0},  {4, 1848, 3, 14, -1.0},  {4, 1849, 3, 5, -1.0},
  {4, 1858, 3, 5, -1.0},  {4, 1894, 3, 6, -1.0},  {4, 1895, 3, 7, -1.0},  {4, 1896, 3, 8, -1.0},  {4, 1897, 3, 9, -1.0},  {4, 1898, 3, 10, -1.0},
  {4, 1899, 3, 11, -1.0},  {4, 1900, 3, 12, -1.0},  {4, 1901, 3, 13, -1.0},  {4, 1902, 3, 14, -1.0},  {4, 1903, 3, 6, -1.0},  {4, 1939, 3, 7, -1.0},
  {4, 1940, 3, 8, -1.0},  {4, 1941, 3, 9, -1.0},  {4, 1942, 3, 10, -1.0},  {4, 1943, 3, 11, -1.0},  {4, 1944, 3, 12, -1.0},  {4, 1945, 3, 13, -1.0},
  {4, 1946, 3, 14, -1.0},  {4, 2070, 4, 4, -1.0},  {4, 2080, 4, 4, -1.0},  {4, 2089, 4, 4, -1.0},  {4, 2125, 4, 5, -1.0},  {4, 2126, 4, 6, -1.0},
  {4, 2127, 4, 7, -1.0},  {4, 2128, 4, 8, -1.0},  {4, 2129, 4, 9, -1.0},  {4, 2130, 4, 10, -1.0},  {4, 2131, 4, 11, -1.0},  {4, 2132, 4, 12, -1.0},
  {4, 2133, 4, 13, -1.0},  {4, 2134, 4, 14, -1.0},  {4, 2135, 4, 5, -1.0},  {4, 2144, 4, 5, -1.0},  {4, 2180, 4, 6, -1.0},  {4, 2181, 4, 7, -1.0},
  {4, 2182, 4, 8, -1.0},  {4, 2183, 4, 9, -1.0},  {4, 2184, 4, 10, -1.0},  {4, 2185, 4, 11, -1.0},  {4, 2186, 4, 12, -1.0},  {4, 2187, 4, 13, -1.0},
  {4, 2188, 4, 14, -1.0},  {4, 2189, 4, 6, -1.0},  {4, 2225, 4, 7, -1.0},  {4, 2226, 4, 8, -1.0},  {4, 2227, 4, 9, -1.0},  {4, 2228, 4, 10, -1.0},
  {4, 2229, 4, 11, -1.0},  {4, 2230, 4, 12, -1.0},  {4, 2231, 4, 13, -1.0},  {4, 2232, 4, 14, -1.0},  {4, 2345, 5, 5, -1.0},  {4, 2346, 5, 6, -1.0},
  {4, 2347, 5, 7, -1.0},  {4, 2348, 5, 8, -1.0},  {4, 2349, 5, 9, -1.0},  {4, 2350, 5, 10, -1.0},  {4, 2351, 5, 11, -1.0},  {4, 2352, 5, 12, -1.0},
  {4, 2353, 5, 13, -1.0},  {4, 2354, 5, 14, -1.0},  {4, 2355, 5, 5, -1.0},  {4, 2355, 6, 6, -1.0},  {4, 2356, 6, 7, -1.0},  {4, 2357, 6, 8, -1.0},
  {4, 2358, 6, 9, -1.0},  {4, 2359, 6, 10, -1.0},  {4, 2360, 6, 11, -1.0},  {4, 2361, 6, 12, -1.0},  {4, 2362, 6, 13, -1.0},  {4, 2363, 6, 14, -1.0},
  {4, 2364, 5, 5, -1.0},  {4, 2364, 7, 7, -1.0},  {4, 2365, 7, 8, -1.0},  {4, 2366, 7, 9, -1.0},  {4, 2367, 7, 10, -1.0},  {4, 2368, 7, 11, -1.0},
  {4, 2369, 7, 12, -1.0},  {4, 2370, 7, 13, -1.0},  {4, 2371, 7, 14, -1.0},  {4, 2372, 8, 8, -1.0},  {4, 2373, 8, 9, -1.0},  {4, 2374, 8, 10, -1.0},
  {4, 2375, 8, 11, -1.0},  {4, 2376, 8, 12, -1.0},  {4, 2377, 8, 13, -1.0},  {4, 2378, 8, 14, -1.0},  {4, 2379, 9, 9, -1.0},  {4, 2380, 9, 10, -1.0},
  {4, 2381, 9, 11, -1.0},  {4, 2382, 9, 12, -1.0},  {4, 2383, 9, 13, -1.0},  {4, 2384, 9, 14, -1.0},  {4, 2385, 10, 10, -1.0},  {4, 2386, 10, 11, -1.0},
  {4, 2387, 10, 12, -1.0},  {4, 2388, 10, 13, -1.0},  {4, 2389, 10, 14, -1.0},  {4, 2390, 11, 11, -1.0},  {4, 2391, 11, 12, -1.0},  {4, 2392, 11, 13, -1.0},
  {4, 2393, 11, 14, -1.0},  {4, 2394, 12, 12, -1.0},  {4, 2395, 12, 13, -1.0},  {4, 2396, 12, 14, -1.0},  {4, 2397, 13, 13, -1.0},  {4, 2398, 13, 14, -1.0},
  {4, 2399, 14, 14, -1.0},  {4, 2400, 5, 6, -1.0},  {4, 2401, 5, 7, -1.0},  {4, 2402, 5, 8, -1.0},  {4, 2403, 5, 9, -1.0},  {4, 2404, 5, 10, -1.0},
  {4, 2405, 5, 11, -1.0},  {4, 2406, 5, 12, -1.0},  {4, 2407, 5, 13, -1.0},  {4, 2408, 5, 14, -1.0},  {4, 2409, 5, 6, -1.0},  {4, 2445, 5, 7, -1.0},
  {4, 2446, 5, 8, -1.0},  {4, 2447, 5, 9, -1.0},  {4, 2448, 5, 10, -1.0},  {4, 2449, 5, 11, -1.0},  {4, 2450, 5, 12, -1.0},  {4, 2451, 5, 13, -1.0},
  {4, 2452, 5, 14, -1.0},  {4, 2565, 6, 6, -1.0},  {4, 2566, 6, 7, -1.0},  {4, 2567, 6, 8, -1.0},  {4, 2568, 6, 9, -1.0},  {4, 2569, 6, 10, -1.0},
  {4, 2570, 6, 11, -1.0},  {4, 2571, 6, 12, -1.0},  {4, 2572, 6, 13, -1.0},  {4, 2573, 6, 14, -1.0},  {4, 2574, 6, 6, -1.0},  {4, 2574, 7, 7, -1.0},
  {4, 2575, 7, 8, -1.0},  {4, 2576, 7, 9, -1.0},  {4, 2577, 7, 10, -1.0},  {4, 2578, 7, 11, -1.0},  {4, 2579, 7, 12, -1.0},  {4, 2580, 7, 13, -1.0},
  {4, 2581, 7, 14, -1.0},  {4, 2582, 8, 8, -1.0},  {4, 2583, 8, 9, -1.0},  {4, 2584, 8, 10, -1.0},  {4, 2585, 8, 11, -1.0},  {4, 2586, 8, 12, -1.0},
  {4, 2587, 8, 13, -1.0},  {4, 2588, 8, 14, -1.0},  {4, 2589, 9, 9, -1.0},  {4, 2590, 9, 10, -1.0},  {4, 2591, 9, 11, -1.0},  {4, 2592, 9, 12, -1.0},
  {4, 2593, 9, 13, -1.0},  {4, 2594, 9, 14, -1.0},  {4, 2595, 10, 10, -1.0},  {4, 2596, 10, 11, -1.0},  {4, 2597, 10, 12, -1.0},  {4, 2598, 10, 13, -1.0},
  {4, 2599, 10, 14, -1.0},  {4, 2600, 11, 11, -1.0},  {4, 2601, 11, 12, -1.0},  {4, 2602, 11, 13, -1.0},  {4, 2603, 11, 14, -1.0},  {4, 2604, 12, 12, -1.0},
  {4, 2605, 12, 13, -1.0},  {4, 2606, 12, 14, -1.0},  {4, 2607, 13, 13, -1.0},  {4, 2608, 13, 14, -1.0},  {4, 2609, 14, 14, -1.0},  {4, 2610, 6, 7, -1.0},
  {4, 2611, 6, 8, -1.0},  {4, 2612, 6, 9, -1.0},  {4, 2613, 6, 10, -1.0},  {4, 2614, 6, 11, -1.0},  {4, 2615, 6, 12, -1.0},  {4, 2616, 6, 13, -1.0},
  {4, 2617, 6, 14, -1.0},  {4, 2730, 7, 7, -1.0},  {4, 2731, 7, 8, -1.0},  {4, 2732, 7, 9, -1.0},  {4, 2733, 7, 10, -1.0},  {4, 2734, 7, 11, -1.0},
  {4, 2735, 7, 12, -1.0},  {4, 2736, 7, 13, -1.0},  {4, 2737, 7, 14, -1.0},  {4, 2738, 8, 8, -1.0},  {4, 2739, 8, 9, -1.0},  {4, 2740, 8, 10, -1.0},
  {4, 2741, 8, 11, -1.0},  {4, 2742, 8, 12, -1.0},  {4, 2743, 8, 13, -1.0},  {4, 2744, 8, 14, -1.0},  {4, 2745, 9, 9, -1.0},  {4, 2746, 9, 10, -1.0},
  {4, 2747, 9, 11, -1.0},  {4, 2748, 9, 12, -1.0},  {4, 2749, 9, 13, -1.0},  {4, 2750, 9, 14, -1.0},  {4, 2751, 10, 10, -1.0},  {4, 2752, 10, 11, -1.0},
  {4, 2753, 10, 12, -1.0},  {4, 2754, 10, 13, -1.0},  {4, 2755, 10, 14, -1.0},  {4, 2756, 11, 11, -1.0},  {4, 2757, 11, 12, -1.0},  {4, 2758, 11, 13, -1.0},
  {4, 2759, 11, 14, -1.0},  {4, 2760, 12, 12, -1.0},  {4, 2761, 12, 13, -1.0},  {4, 2762, 12, 14, -1.0},  {4, 2763, 13, 13, -1.0},  {4, 2764, 13, 14, -1.0},
  {4, 2765, 14, 14, -1.0},  {5, 0, 0, 0, -1.0},  {5, 1, 0, 1, -1.0},  {5, 2, 0, 2, -1.0},  {5, 3, 0, 3, -1.0},  {5, 4, 0, 4, -1.0},
  {5, 5, 0, 5, -1.0},  {5, 6, 0, 6, -1.0},  {5, 7, 0, 7, -1.0},  {5, 8, 0, 8, -1.0},  {5, 9, 0, 9, -1.0},  {5, 10, 0, 10, -1.0},
  {5, 11, 0, 11, -1.0},  {5, 12, 0, 12, -1.0},  {5, 13, 0, 13, -1.0},  {5, 14, 0, 14, -1.0},  {5, 15, 1, 1, -1.0},  {5, 16, 1, 2, -1.0},
  {5, 17, 1, 3, -1.0},  {5, 18, 1, 4, -1.0},  {5, 19, 1, 5, -1.0},  {5, 20, 1, 6, -1.0},  {5, 21, 1, 7, -1.0},  {5, 22, 1, 8, -1.0},
  {5, 23, 1, 9, -1.0},  {5, 24, 1, 10, -1.0},  {5, 25, 1, 11, -1.0},  {5, 26, 1, 12, -1.0},  {5, 27, 1, 13, -1.0},  {5, 28, 1, 14, -1.0},
  {5, 29, 2, 2, -1.0},  {5, 30, 2, 3, -1.0},  {5, 31, 2, 4, -1.0},  {5, 32, 2, 5, -1.0},  {5, 33, 2, 6, -1.0},  {5, 34, 2, 7, -1.0},
  {5, 35, 2, 8, -1.0},  {5, 36, 2, 9, -1.0},  {5, 37, 2, 10, -1.0},  {5, 38, 2, 11, -1.0},  {5, 39, 2, 12, -1.0},  {5, 40, 2, 13, -1.0},
  {5, 41, 2, 14, -1.0},  {5, 42, 3, 3, -1.0},  {5, 43, 3, 4, -1.0},  {5, 44, 3, 5, -1.0},  {5, 45, 3, 6, -1.0},  {5, 46, 3, 7, -1.0},
  {5, 47, 3, 8, -1.0},  {5, 48, 3, 9, -1.0},  {5, 49, 3, 10, -1.0},  {5, 50, 3, 11, -1.0},  {5, 51, 3, 12, -1.0},  {5, 52, 3, 13, -1.0},
  {5, 53, 3, 14, -1.0},  {5, 54, 4, 4, -1.0},  {5, 55, 4, 5, -1.0},  {5, 56, 4, 6, -1.0},  {5, 57, 4, 7, -1.0},  {5, 58, 4, 8, -1.0},
  {5, 59, 4, 9, -1.0},  {5, 60, 4, 10, -1.0},  {5, 61, 4, 11, -1.0},  {5, 62, 4, 12, -1.0},  {5, 63, 4, 13, -1.0},  {5, 64, 4, 14, -1.0},
  {5, 65, 5, 5, -1.0},  {5, 66, 5, 6, -1.0},  {5, 67, 5, 7, -1.0},  {5, 68, 5, 8, -1.0},  {5, 69, 5, 9, -1.0},  {5, 70, 5, 10, -1.0},
  {5, 71, 5, 11, -1.0},  {5, 72, 5, 12, -1.0},  {5, 73, 5, 13, -1.0},  {5, 74, 5, 14, -1.0},  {5, 75, 6, 6, -1.0},  {5, 76, 6, 7, -1.0},
  {5, 77, 6, 8, -1.0},  {5, 78, 6, 9, -1.0},  {5, 79, 6, 10, -1.0},  {5, 80, 6, 11, -1.0},  {5, 81, 6, 12, -1.0},  {5, 82, 6, 13, -1.0},
  {5, 83, 6, 14, -1.0},  {5, 84, 7, 7, -1.0},  {5, 85, 7, 8, -1.0},  {5, 86, 7, 9, -1.0},  {5, 87, 7, 10, -1.0},  {5, 88, 7, 11, -1.0},
  {5, 89, 7, 12, -1.0},  {5, 90, 7, 13, -1.0},  {5, 91, 7, 14, -1.0},  {5, 92, 0, 0, 1.0},  {5, 92, 8, 8, -1.0},  {5, 93, 8, 9, -1.0},
  {5, 94, 8, 10, -1.0},  {5, 95, 8, 11, -1.0},  {5, 96, 8, 12, -1.0},  {5, 97, 8, 13, -1.0},  {5, 98, 8, 14, -1.0},  {5, 99, 0, 0, 1.0},
  {5, 99, 9, 9, -1.0},  {5, 100, 9, 10, -1.0},  {5, 101, 9, 11, -1.0},  {5, 102, 9, 12, -1.0},  {5, 103, 9, 13, -1.0},  {5, 104, 9, 14, -1.0},
  {5, 105, 0, 0, 1.0},  {5, 105, 10, 10, -1.0},  {5, 106, 10, 11, -1.0},  {5, 107, 10, 12, -1.0},  {5, 108, 10, 13, -1.0},  {5, 109, 10, 14, -1.0},
  {5, 110, 0, 0, 1.0},  {5, 110, 11, 11, -1.0},  {5, 111, 11, 12, -1.0},  {5, 112, 11, 13, -1.0},  {5, 113, 11, 14, -1.0},  {5, 114, 12, 12, -1.0},
  {5, 115, 12, 13, -1.0},  {5, 116, 12, 14, -1.0},  {5, 117, 13, 13, -1.0},  {5, 118, 13, 14, -1.0},  {5, 119, 14, 14, -1.0},  {5, 197, 0, 1, 1.0},
  {5, 204, 0, 1, 1.0},  {5, 210, 0, 1, 1.0},  {5, 215, 0, 1, 1.0},  {5, 288, 0, 2, 1.0},  {5, 295, 0, 2, 1.0},  {5, 301, 0, 2, 1.0},
  {5, 306, 0, 2, 1.0},  {5, 366, 0, 3, 1.0},  {5, 373, 0, 3, 1.0},  {5, 379, 0, 3, 1.0},  {5, 384, 0, 3, 1.0},  {5, 432, 0, 4, 1.0},
  {5, 439, 0, 4, 1.0},  {5, 445, 0, 4, 1.0},  {5, 450, 0, 4, 1.0},  {5, 487, 0, 5, 1.0},  {5, 494, 0, 5, 1.0},  {5, 500, 0, 5, 1.0},
  {5, 505, 0, 5, 1.0},  {5, 532, 0, 6, 1.0},  {5, 539, 0, 6, 1.0},  {5, 545, 0, 6, 1.0},  {5, 550, 0, 6, 1.0},  {5, 568, 0, 7, 1.0},
  {5, 575, 0, 7, 1.0},  {5, 581, 0, 7, 1.0},  {5, 586, 0, 7, 1.0},  {5, 596, 0, 8, 1.0},  {5, 597, 0, 9, 1.0},  {5, 598, 0, 10, 1.0},
  {5, 599, 0, 11, 1.0},  {5, 600, 0, 12, 1.0},  {5, 601, 0, 13, 1.0},  {5, 602, 0, 14, 1.0},  {5, 603, 0, 8, 1.0},  {5, 609, 0, 8, 1.0},
  {5, 614, 0, 8, 1.0},  {5, 624, 0, 9, 1.0},  {5, 625, 0, 10, 1.0},  {5, 626, 0, 11, 1.0},  {5, 627, 0, 12, 1.0},  {5, 628, 0, 13, 1.0},
  {5, 629, 0, 14, 1.0},  {5, 630, 0, 9, 1.0},  {5, 635, 0, 9, 1.0},  {5, 645, 0, 10, 1.0},  {5, 646, 0, 11, 1.0},  {5, 647, 0, 12, 1.0},
  {5, 648, 0, 13, 1.0},  {5, 649, 0, 14, 1.0},  {5, 650, 0, 10, 1.0},  {5, 660, 0, 11, 1.0},  {5, 661, 0, 12, 1.0},  {5, 662, 0, 13, 1.0},
  {5, 663, 0, 14, 1.0},  {5, 757, 1, 1, 1.0},  {5, 764, 1, 1, 1.0},  {5, 770, 1, 1, 1.0},  {5, 775, 1, 1, 1.0},  {5, 848, 1, 2, 1.0},
  {5, 855, 1, 2, 1.0},  {5, 861, 1, 2, 1.0},  {5, 866, 1, 2, 1.0},  {5, 926, 1, 3, 1.0},  {5, 933, 1, 3, 1.0},  {5, 939, 1, 3, 1.0},
  {5, 944, 1, 3, 1.0},  {5, 992, 1, 4, 1.0},  {5, 999, 1, 4, 1.0},  {5, 1005, 1, 4, 1.0},  {5, 1010, 1, 4, 1.0},  {5, 1047, 1, 5, 1.0},
  {5, 1054, 1, 5, 1.0},  {5, 1060, 1, 5, 1.0},  {5, 1065, 1, 5, 1.0},  {5, 1092, 1, 6, 1.0},  {5, 1099, 1, 6, 1.0},  {5, 1105, 1, 6, 1.0},
  {5, 1110, 1, 6, 1.0},  {5, 1128, 1, 7, 1.0},  {5, 1135, 1, 7, 1.0},  {5, 1141, 1, 7, 1.0},  {5, 1146, 1, 7, 1.0},  {5, 1156, 1, 8, 1.0},
  {5, 1157, 1, 9, 1.0},  {5, 1158, 1, 10, 1.0},  {5, 1159, 1, 11, 1.0},  {5, 1160, 1, 12, 1.0},  {5, 1161, 1, 13, 1.0},  {5, 1162, 1, 14, 1.0},
  {5, 1163, 1, 8, 1.0},  {5, 1169, 1, 8, 1.0},  {5, 1174, 1, 8, 1.0},  {5, 1184, 1, 9, 1.0},  {5, 1185, 1, 10, 1.0},  {5, 1186, 1, 11, 1.0},
  {5, 1187, 1, 12, 1.0},  {5, 1188, 1, 13, 1.0},  {5, 1189, 1, 14, 1.0},  {5, 1190, 1, 9, 1.0},  {5, 1195, 1, 9, 1.0},  {5, 1205, 1, 10, 1.0},
  {5, 1206, 1, 11, 1.0},  {5, 1207, 1, 12, 1.0},  {5, 1208, 1, 13, 1.0},  {5, 1209, 1, 14, 1.0},  {5, 1210, 1, 10, 1.0},  {5, 1220, 1, 11, 1.0},
  {5, 1221, 1, 12, 1.0},  {5, 1222, 1, 13, 1.0},  {5, 1223, 1, 14, 1.0},  {5, 1303, 2, 2, 1.0},  {5, 1310, 2, 2, 1.0},  {5, 1316, 2, 2, 1.0},
  {5, 1321, 2, 2, 1.0},  {5, 1381, 2, 3, 1.0},  {5, 1388, 2, 3, 1.0},  {5, 1394, 2, 3, 1.0},  {5, 1399, 2, 3, 1.0},  {5, 1447, 2, 4, 1.0},
  {5, 1454, 2, 4, 1.0},  {5, 1460, 2, 4, 1.0},  {5, 1465, 2, 4, 1.0},  {5, 1502, 2, 5, 1.0},  {5, 1509, 2, 5, 1.0},  {5, 1515, 2, 5, 1.0},
  {5, 1520, 2, 5, 1.0},  {5, 1547, 2, 6, 1.0},  {5, 1554, 2, 6, 1.0},  {5, 1560, 2, 6, 1.0},  {5, 1565, 2, 6, 1.0},  {5, 1583, 2, 7, 1.0},
  {5, 1590, 2, 7, 1.0},  {5, 1596, 2, 7, 1.0},  {5, 1601, 2, 7, 1.0},  {5, 1611, 2, 8, 1.0},  {5, 1612, 2, 9, 1.0},  {5, 1613, 2, 10, 1.0},
  {5, 1614, 2, 11, 1.0},  {5, 1615, 2, 12, 1.0},  {5, 1616, 2, 13, 1.0},  {5, 1617, 2, 14, 1.0},  {5, 1618, 2, 8, 1.0},  {5, 1624, 2, 8, 1.0},
  {5, 1629, 2, 8, 1.0},  {5, 1639, 2, 9, 1.0},  {5, 1640, 2, 10, 1.0},  {5, 1641, 2, 11, 1.0},  {5, 1642, 2, 12, 1.0},  {5, 1643, 2, 13, 1.0},
  {5, 1644, 2, 14, 1.0},  {5, 1645, 2, 9, 1.0},  {5, 1650, 2, 9, 1.0},  {5, 1660, 2, 10, 1.0},  {5, 1661, 2, 11, 1.0},  {5, 1662, 2, 12, 1.0},
  {5, 1663, 2, 13, 1.0},  {5, 1664, 2, 14, 1.0},  {5, 1665, 2, 10, 1.0},  {5, 1675, 2, 11, 1.0},  {5, 1676, 2, 12, 1.0},  {5, 1677, 2, 13, 1.0},
  {5, 1678, 2, 14, 1.0},  {5, 1745, 3, 3, 1.0},  {5, 1752, 3, 3, 1.0},  {5, 1758, 3, 3, 1.0},  {5, 1763, 3, 3, 1.0},  {5, 1811, 3, 4, 1.0},
  {5, 1818, 3, 4, 1.0},  {5, 1824, 3, 4, 1.0},  {5, 1829, 3, 4, 1.0},  {5, 1866, 3, 5, 1.0},  {5, 1873, 3, 5, 1.0},  {5, 1879, 3, 5, 1.0},
  {5, 1884, 3, 5, 1.0},  {5, 1911, 3, 6, 1.0},  {5, 1918, 3, 6, 1.0},  {5, 1924, 3, 6, 1.0},  {5, 1929, 3, 6, 1.0},  {5, 1947, 3, 7, 1.0},
  {5, 1954, 3, 7, 1.0},  {5, 1960, 3, 7, 1.0},  {5, 1965, 3, 7, 1.0},  {5, 1975, 3, 8, 1.0},  {5, 1976, 3, 9, 1.0},  {5, 1977, 3, 10, 1.0},
  {5, 1978, 3, 11, 1.0},  {5, 1979, 3, 12, 1.0},  {5, 1980, 3, 13, 1.0},  {5, 1981, 3, 14, 1.0},  {5, 1982, 3, 8, 1.0},  {5, 1988, 3, 8, 1.0},
  {5, 1993, 3, 8, 1.0},  {5, 2003, 3, 9, 1.0},  {5, 2004, 3, 10, 1.0},  {5, 2005, 3, 11, 1.0},  {5, 2006, 3, 12, 1.0},  {5, 2007, 3, 13, 1.0},
  {5, 2008, 3, 14, 1.0},  {5, 2009, 3, 9, 1.0},  {5, 2014, 3, 9, 1.0},  {5, 2024, 3, 10, 1.0},  {5, 2025, 3, 11, 1.0},  {5, 2026, 3, 12, 1.0},
  {5, 2027, 3, 13, 1.0},  {5, 2028, 3, 14, 1.0},  {5, 2029, 3, 10, 1.0},  {5, 2039, 3, 11, 1.0},  {5, 2040, 3, 12, 1.0},  {5, 2041, 3, 13, 1.0},
  {5, 2042, 3, 14, 1.0},  {5, 2097, 4, 4, 1.0},  {5, 2104, 4, 4, 1.0},  {5, 2110, 4, 4, 1.0},  {5, 2115, 4, 4, 1.0},  {5, 2152, 4, 5, 1.0},
  {5, 2159, 4, 5, 1.0},  {5, 2165, 4, 5, 1.0},  {5, 2170, 4, 5, 1.0},  {5, 2197, 4, 6, 1.0},  {5, 2204, 4, 6, 1.0},  {5, 2210, 4, 6, 1.0},
  {5, 2215, 4, 6, 1.0},  {5, 2233, 4, 7, 1.0},  {5, 2240, 4, 7, 1.0},  {5, 2246, 4, 7, 1.0},  {5, 2251, 4, 7, 1.0},  {5, 2261, 4, 8, 1.0},
  {5, 2262, 4, 9, 1.0},  {5, 2263, 4, 10, 1.0},  {5, 2264, 4, 11, 1.0},  {5, 2265, 4, 12, 1.0},  {5, 2266, 4, 13, 1.0},  {5, 2267, 4, 14, 1.0},
  {5, 2268, 4, 8, 1.0},  {5, 2274, 4, 8, 1.0},  {5, 2279, 4, 8, 1.0},  {5, 2289, 4, 9, 1.0},  {5, 2290, 4, 10, 1.0},  {5, 2291, 4, 11, 1.0},
  {5, 2292, 4, 12, 1.0},  {5, 2293, 4, 13, 1.0},  {5, 2294, 4, 14, 1.0},  {5, 2295, 4, 9, 1.0},  {5, 2300, 4, 9, 1.0},  {5, 2310, 4, 10, 1.0},
  {5, 2311, 4, 11, 1.0},  {5, 2312, 4, 12, 1.0},  {5, 2313, 4, 13, 1.0},  {5, 2314, 4, 14, 1.0},  {5, 2315, 4, 10, 1.0},  {5, 2325, 4, 11, 1.0},
  {5, 2326, 4, 12, 1.0},  {5, 2327, 4, 13, 1.0},  {5, 2328, 4, 14, 1.0},  {5, 2372, 5, 5, 1.0},  {5, 2379, 5, 5, 1.0},  {5, 2385, 5, 5, 1.0},
  {5, 2390, 5, 5, 1.0},  {5, 2417, 5, 6, 1.0},  {5, 2424, 5, 6, 1.0},  {5, 2430, 5, 6, 1.0},  {5, 2435, 5, 6, 1.0},  {5, 2453, 5, 7, 1.0},
  {5, 2460, 5, 7, 1.0},  {5, 2466, 5, 7, 1.0},  {5, 2471, 5, 7, 1.0},  {5, 2481, 5, 8, 1.0},  {5, 2482, 5, 9, 1.0},  {5, 2483, 5, 10, 1.0},
  {5, 2484, 5, 11, 1.0},  {5, 2485, 5, 12, 1.0},  {5, 2486, 5, 13, 1.0},  {5, 2487, 5, 14, 1.0},  {5, 2488, 5, 8, 1.0},  {5, 2494, 5, 8, 1.0},
  {5, 2499, 5, 8, 1.0},  {5, 2509, 5, 9, 1.0},  {5, 2510, 5, 10, 1.0},  {5, 2511, 5, 11, 1.0},  {5, 2512, 5, 12, 1.0},  {5, 2513, 5, 13, 1.0},
  {5, 2514, 5, 14, 1.0},  {5, 2515, 5, 9, 1.0},  {5, 2520, 5, 9, 1.0},  {5, 2530, 5, 10, 1.0},  {5, 2531, 5, 11, 1.0},  {5, 2532, 5, 12, 1.0},
  {5, 2533, 5, 13, 1.0},  {5, 2534, 5, 14, 1.0},  {5, 2535, 5, 10, 1.0},  {5, 2545, 5, 11, 1.0},  {5, 2546, 5, 12, 1.0},  {5, 2547, 5, 13, 1.0},
  {5, 2548, 5, 14, 1.0},  {5, 2582, 6, 6, 1.0},  {5, 2589, 6, 6, 1.0},  {5, 2595, 6, 6, 1.0},  {5, 2600, 6, 6, 1.0},  {5, 2618, 6, 7, 1.0},
  {5, 2625, 6, 7, 1.0},  {5, 2631, 6, 7, 1.0},  {5, 2636, 6, 7, 1.0},  {5, 2646, 6, 8, 1.0},  {5, 2647, 6, 9, 1.0},  {5, 2648, 6, 10, 1.0},
  {5, 2649, 6, 11, 1.0},  {5, 2650, 6, 12, 1.0},  {5, 2651, 6, 13, 1.0},  {5, 2652, 6, 14, 1.0},  {5, 2653, 6, 8, 1.0},  {5, 2659, 6, 8, 1.0},
  {5, 2664, 6, 8, 1.0},  {5, 2674, 6, 9, 1.0},  {5, 2675, 6, 10, 1.0},  {5, 2676, 6, 11, 1.0},  {5, 2677, 6, 12, 1.0},  {5, 2678, 6, 13, 1.0},
  {5, 2679, 6, 14, 1.0},  {5, 2680, 6, 9, 1.0},  {5, 2685, 6, 9, 1.0},  {5, 2695, 6, 10, 1.0},  {5, 2696, 6, 11, 1.0},  {5, 2697, 6, 12, 1.0},
  {5, 2698, 6, 13, 1.0},  {5, 2699, 6, 14, 1.0},  {5, 2700, 6, 10, 1.0},  {5, 2710, 6, 11, 1.0},  {5, 2711, 6, 12, 1.0},  {5, 2712, 6, 13, 1.0},
  {5, 2713, 6, 14, 1.0},  {5, 2738, 7, 7, 1.0},  {5, 2745, 7, 7, 1.0},  {5, 2751, 7, 7, 1.0},  {5, 2756, 7, 7, 1.0},  {5, 2766, 7, 8, 1.0},
  {5, 2767, 7, 9, 1.0},  {5, 2768, 7, 10, 1.0},  {5, 2769, 7, 11, 1.0},  {5, 2770, 7, 12, 1.0},  {5, 2771, 7, 13, 1.0},  {5, 2772, 7, 14, 1.0},
  {5, 2773, 7, 8, 1.0},  {5, 2779, 7, 8, 1.0},  {5, 2784, 7, 8, 1.0},  {5, 2794, 7, 9, 1.0},  {5, 2795, 7, 10, 1.0},  {5, 2796, 7, 11, 1.0},
  {5, 2797, 7, 12, 1.0},  {5, 2798, 7, 13, 1.0},  {5, 2799, 7, 14, 1.0},  {5, 2800, 7, 9, 1.0},  {5, 2805, 7, 9, 1.0},  {5, 2815, 7, 10, 1.0},
  {5, 2816, 7, 11, 1.0},  {5, 2817, 7, 12, 1.0},  {5, 2818, 7, 13, 1.0},  {5, 2819, 7, 14, 1.0},  {5, 2820, 7, 10, 1.0},  {5, 2830, 7, 11, 1.0},
  {5, 2831, 7, 12, 1.0},  {5, 2832, 7, 13, 1.0},  {5, 2833, 7, 14, 1.0},  {5, 2850, 8, 8, 1.0},  {5, 2851, 8, 9, 1.0},  {5, 2852, 8, 10, 1.0},
  {5, 2853, 8, 11, 1.0},  {5, 2854, 8, 12, 1.0},  {5, 2855, 8, 13, 1.0},  {5, 2856, 8, 14, 1.0},  {5, 2857, 8, 8, 1.0},  {5, 2857, 9, 9, 1.0},
  {5, 2858, 9, 10, 1.0},  {5, 2859, 9, 11, 1.0},  {5, 2860, 9, 12, 1.0},  {5, 2861, 9, 13, 1.0},  {5, 2862, 9, 14, 1.0},  {5, 2863, 8, 8, 1.0},
  {5, 2863, 10, 10, 1.0},  {5, 2864, 10, 11, 1.0},  {5, 2865, 10, 12, 1.0},  {5, 2866, 10, 13, 1.0},  {5, 2867, 10, 14, 1.0},  {5, 2868, 8, 8, 1.0},
  {5, 2868, 11, 11, 1.0},  {5, 2869, 11, 12, 1.0},  {5, 2870, 11, 13, 1.0},  {5, 2871, 11, 14, 1.0},  {5, 2872, 12, 12, 1.0},  {5, 2873, 12, 13, 1.0},
  {5, 2874, 12, 14, 1.0},  {5, 2875, 13, 13, 1.0},  {5, 2876, 13, 14, 1.0},  {5, 2877, 14, 14, 1.0},  {5, 2878, 8, 9, 1.0},  {5, 2879, 8, 10, 1.0},
  {5, 2880, 8, 11, 1.0},  {5, 2881, 8, 12, 1.0},  {5, 2882, 8, 13, 1.0},  {5, 2883, 8, 14, 1.0},  {5, 2884, 8, 9, 1.0},  {5, 2889, 8, 9, 1.0},
  {5, 2899, 8, 10, 1.0},  {5, 2900, 8, 11, 1.0},  {5, 2901, 8, 12, 1.0},  {5, 2902, 8, 13, 1.0},  {5, 2903, 8, 14, 1.0},  {5, 2904, 8, 10, 1.0},
  {5, 2914, 8, 11, 1.0},  {5, 2915, 8, 12, 1.0},  {5, 2916, 8, 13, 1.0},  {5, 2917, 8, 14, 1.0},  {5, 2934, 9, 9, 1.0},  {5, 2935, 9, 10, 1.0},
  {5, 2936, 9, 11, 1.0},  {5, 2937, 9, 12, 1.0},  {5, 2938, 9, 13, 1.0},  {5, 2939, 9, 14, 1.0},  {5, 2940, 9, 9, 1.0},  {5, 2940, 10, 10, 1.0},
  {5, 2941, 10, 11, 1.0},  {5, 2942, 10, 12, 1.0},  {5, 2943, 10, 13, 1.0},  {5, 2944, 10, 14, 1.0},  {5, 2945, 9, 9, 1.0},  {5, 2945, 11, 11, 1.0},
  {5, 2946, 11, 12, 1.0},  {5, 2947, 11, 13, 1.0},  {5, 2948, 11, 14, 1.0},  {5, 2949, 12, 12, 1.0},  {5, 2950, 12, 13, 1.0},  {5, 2951, 12, 14, 1.0},
  {5, 2952, 13, 13, 1.0},  {5, 2953, 13, 14, 1.0},  {5, 2954, 14, 14, 1.0},  {5, 2955, 9, 10, 1.0},  {5, 2956, 9, 11, 1.0},  {5, 2957, 9, 12, 1.0},
  {5, 2958, 9, 13, 1.0},  {5, 2959, 9, 14, 1.0},  {5, 2960, 9, 10, 1.0},  {5, 2970, 9, 11, 1.0},  {5, 2971, 9, 12, 1.0},  {5, 2972, 9, 13, 1.0},
  {5, 2973, 9, 14, 1.0},  {5, 2990, 10, 10, 1.0},  {5, 2991, 10, 11, 1.0},  {5, 2992, 10, 12, 1.0},  {5, 2993, 10, 13, 1.0},  {5, 2994, 10, 14, 1.0},
  {5, 2995, 10, 10, 1.0},  {5, 2995, 11, 11, 1.0},  {5, 2996, 11, 12, 1.0},  {5, 2997, 11, 13, 1.0},  {5, 2998, 11, 14, 1.0},  {5, 2999, 12, 12, 1.0},
  {5, 3000, 12, 13, 1.0},  {5, 3001, 12, 14, 1.0},  {5, 3002, 13, 13, 1.0},  {5, 3003, 13, 14, 1.0},  {5, 3004, 14, 14, 1.0},  {5, 3005, 10, 11, 1.0},
  {5, 3006, 10, 12, 1.0},  {5, 3007, 10, 13, 1.0},  {5, 3008, 10, 14, 1.0},  {5, 3025, 11, 11, 1.0},  {5, 3026, 11, 12, 1.0},  {5, 3027, 11, 13, 1.0},
  {5, 3028, 11, 14, 1.0},  {5, 3029, 12, 12, 1.0},  {5, 3030, 12, 13, 1.0},  {5, 3031, 12, 14, 1.0},  {5, 3032, 13, 13, 1.0},  {5, 3033, 13, 14, 1.0},
  {5, 3034, 14, 14, 1.0},  {6, 0, 0, 0, 1.0},  {6, 1, 0, 1, 1.0},  {6, 2, 0, 2, 1.0},  {6, 3, 0, 3, 1.0},  {6, 4, 0, 4, 1.0},
  {6, 5, 0, 5, 1.0},  {6, 6, 0, 6, 1.0},  {6, 7, 0, 7, 1.0},  {6, 8, 0, 8, 1.0},  {6, 9, 0, 9, 1.0},  {6, 10, 0, 10, 1.0},
  {6, 11, 0, 11, 1.0},  {6, 12, 0, 12, 1.0},  {6, 13, 0, 13, 1.0},  {6, 14, 0, 14, 1.0},  {6, 15, 1, 1, 1.0},  {6, 16, 1, 2, 1.0},
  {6, 17, 1, 3, 1.0},  {6, 18, 1, 4, 1.0},  {6, 19, 1, 5, 1.0},  {6, 20, 1, 6, 1.0},  {6, 21, 1, 7, 1.0},  {6, 22, 1, 8, 1.0},
  {6, 23, 1, 9, 1.0},  {6, 24, 1, 10, 1.0},  {6, 25, 1, 11, 1.0},  {6, 26, 1, 12, 1.0},  {6, 27, 1, 13, 1.0},  {6, 28, 1, 14, 1.0},
  {6, 29, 2, 2, 1.0},  {6, 30, 2, 3, 1.0},  {6, 31, 2, 4, 1.0},  {6, 32, 2, 5, 1.0},  {6, 33, 2, 6, 1.0},  {6, 34, 2, 7, 1.0},
  {6, 35, 2, 8, 1.0},  {6, 36, 2, 9, 1.0},  {6, 37, 2, 10, 1.0},  {6, 38, 2, 11, 1.0},  {6, 39, 2, 12, 1.0},  {6, 40, 2, 13, 1.0},
  {6, 41, 2, 14, 1.0},  {6, 42, 3, 3, 1.0},  {6, 43, 3, 4, 1.0},  {6, 44, 3, 5, 1.0},  {6, 45, 3, 6, 1.0},  {6, 46, 3, 7, 1.0},
  {6, 47, 3, 8, 1.0},  {6, 48, 3, 9, 1.0},  {6, 49, 3, 10, 1.0},  {6, 50, 3, 11, 1.0},  {6, 51, 3, 12, 1.0},  {6, 52, 3, 13, 1.0},
  {6, 53, 3, 14, 1.0},  {6, 54, 4, 4, 1.0},  {6, 55, 4, 5, 1.0},  {6, 56, 4, 6, 1.0},  {6, 57, 4, 7, 1.0},  {6, 58, 4, 8, 1.0},
  {6, 59, 4, 9, 1.0},  {6, 60, 4, 10, 1.0},  {6, 61, 4, 11, 1.0},  {6, 62, 4, 12, 1.0},  {6, 63, 4, 13, 1.0},  {6, 64, 4, 14, 1.0},
  {6, 65, 5, 5, 1.0},  {6, 66, 5, 6, 1.0},  {6, 67, 5, 7, 1.0},  {6, 68, 5, 8, 1.0},  {6, 69, 5, 9, 1.0},  {6, 70, 5, 10, 1.0},
  {6, 71, 5, 11, 1.0},  {6, 72, 5, 12, 1.0},  {6, 73, 5, 13, 1.0},  {6, 74, 5, 14, 1.0},  {6, 75, 6, 6, 1.0},  {6, 76, 6, 7, 1.0},
  {6, 77, 6, 8, 1.0},  {6, 78, 6, 9, 1.0},  {6, 79, 6, 10, 1.0},  {6, 80, 6, 11, 1.0},  {6, 81, 6, 12, 1.0},  {6, 82, 6, 13, 1.0},
  {6, 83, 6, 14, 1.0},  {6, 84, 7, 7, 1.0},  {6, 85, 7, 8, 1.0},  {6, 86, 7, 9, 1.0},  {6, 87, 7, 10, 1.0},  {6, 88, 7, 11, 1.0},
  {6, 89, 7, 12, 1.0},  {6, 90, 7, 13, 1.0},  {6, 91, 7, 14, 1.0},  {6, 92, 0, 0, -1.0},  {6, 92, 8, 8, 1.0},  {6, 93, 8, 9, 1.0},
  {6, 94, 8, 10, 1.0},  {6, 95, 8, 11, 1.0},  {6, 96, 8, 12, 1.0},  {6, 97, 8, 13, 1.0},  {6, 98, 8, 14, 1.0},  {6, 99, 0, 0, -1.0},
  {6, 99, 9, 9, 1.0},  {6, 100, 9, 10, 1.0},  {6, 101, 9, 11, 1.0},  {6, 102, 9, 12, 1.0},  {6, 103, 9, 13, 1.0},  {6, 104, 9, 14, 1.0},
  {6, 105, 0, 0, -1.0},  {6, 105, 10, 10, 1.0},  {6, 106, 10, 11, 1.0},  {6, 107, 10, 12, 1.0},  {6, 108, 10, 13, 1.0},  {6, 109, 10, 14, 1.0},
  {6, 110, 0, 0, -1.0},  {6, 110, 11, 11, 1.0},  {6, 111, 11, 12, 1.0},  {6, 112, 11, 13, 1.0},  {6, 113, 11, 14, 1.0},  {6, 114, 12, 12, 1.0},
  {6, 115, 12, 13, 1.0},  {6, 116, 12, 14, 1.0},  {6, 117, 13, 13, 1.0},  {6, 118, 13, 14, 1.0},  {6, 119, 14, 14, 1.0},  {6, 197, 0, 1, -1.0},
  {6, 204, 0, 1, -1.0},  {6, 210, 0, 1, -1.0},  {6, 215, 0, 1, -1.0},  {6, 288, 0, 2, -1.0},  {6, 295, 0, 2, -1.0},  {6, 301, 0, 2, -1.0},
  {6, 306, 0, 2, -1.0},  {6, 366, 0, 3, -1.0},  {6, 373, 0, 3, -1.0},  {6, 379, 0, 3, -1.0},  {6, 384, 0, 3, -1.0},  {6, 432, 0, 4, -1.0},
  {6, 439, 0, 4, -1.0},  {6, 445, 0, 4, -1.0},  {6, 450, 0, 4, -1.0},  {6, 487, 0, 5, -1.0},  {6, 494, 0, 5, -1.0},  {6, 500, 0, 5, -1.0},
  {6, 505, 0, 5, -1.0},  {6, 532, 0, 6, -1.0},  {6, 539, 0, 6, -1.0},  {6, 545, 0, 6, -1.0},  {6, 550, 0, 6, -1.0},  {6, 568, 0, 7, -1.0},
  {6, 575, 0, 7, -1.0},  {6, 581, 0, 7, -1.0},  {6, 586, 0, 7, -1.0},  {6, 596, 0, 8, -1.0},  {6, 597, 0, 9, -1.0},  {6, 598, 0, 10, -1.0},
  {6, 599, 0, 11, -1.0},  {6, 600, 0, 12, -1.0},  {6, 601, 0, 13, -1.0},  {6, 602, 0, 14, -1.0},  {6, 603, 0, 8, -1.0},  {6, 609, 0, 8, -1.0},
  {6, 614, 0, 8, -1.0},  {6, 624, 0, 9, -1.0},  {6, 625, 0, 10, -1.0},  {6, 626, 0, 11, -1.0},  {6, 627, 0, 12, -1.0},  {6, 628, 0, 13, -1.0},
  {6, 629, 0, 14, -1.0},  {6, 630, 0, 9, -1.0},  {6, 635, 0, 9, -1.0},  {6, 645, 0, 10, -1.0},  {6, 646, 0, 11, -1.0},  {6, 647, 0, 12, -1.0},
  {6, 648, 0, 13, -1.0},  {6, 649, 0, 14, -1.0},  {6, 650, 0, 10, -1.0},  {6, 660, 0, 11, -1.0},  {6, 661, 0, 12, -1.0},  {6, 662, 0, 13, -1.0},
  {6, 663, 0, 14, -1.0},  {6, 757, 1, 1, -1.0},  {6, 764, 1, 1, -1.0},  {6, 770, 1, 1, -1.0},  {6, 775, 1, 1, -1.0},  {6, 848, 1, 2, -1.0},
  {6, 855, 1, 2, -1.0},  {6, 861, 1, 2, -1.0},  {6, 866, 1, 2, -1.0},  {6, 926, 1, 3, -1.0},  {6, 933, 1, 3, -1.0},  {6, 939, 1, 3, -1.0},
  {6, 944, 1, 3, -1.0},  {6, 992, 1, 4, -1.0},  {6, 999, 1, 4, -1.0},  {6, 1005, 1, 4, -1.0},  {6, 1010, 1, 4, -1.0},  {6, 1047, 1, 5, -1.0},
  {6, 1054, 1, 5, -1.0},  {6, 1060, 1, 5, -1.0},  {6, 1065, 1, 5, -1.0},  {6, 1092, 1, 6, -1.0},  {6, 1099, 1, 6, -1.0},  {6, 1105, 1, 6, -1.0},
  {6, 1110, 1, 6, -1.0},  {6, 1128, 1, 7, -1.0},  {6, 1135, 1, 7, -1.0},  {6, 1141, 1, 7, -1.0},  {6, 1146, 1, 7, -1.0},  {6, 1156, 1, 8, -1.0},
  {6, 1157, 1, 9, -1.0},  {6, 1158, 1, 10, -1.0},  {6, 1159, 1, 11, -1.0},  {6, 1160, 1, 12, -1.0},  {6, 1161, 1, 13, -1.0},  {6, 1162, 1, 14, -1.0},
  {6, 1163, 1, 8, -1.0},  {6, 1169, 1, 8, -1.0},  {6, 1174, 1, 8, -1.0},  {6, 1184, 1, 9, -1.0},  {6, 1185, 1, 10, -1.0},  {6, 1186, 1, 11, -1.0},
  {6, 1187, 1, 12, -1.0},  {6, 1188, 1, 13, -1.0},  {6, 1189, 1, 14, -1.0},  {6, 1190, 1, 9, -1.0},  {6, 1195, 1, 9, -1.0},  {6, 1205, 1, 10, -1.0},
  {6, 1206, 1, 11, -1.0},  {6, 1207, 1, 12, -1.0},  {6, 1208, 1, 13, -1.0},  {6, 1209, 1, 14, -1.0},  {6, 1210, 1, 10, -1.0},  {6, 1220, 1, 11, -1.0},
  {6, 1221, 1, 12, -1.0},  {6, 1222, 1, 13, -1.0},  {6, 1223, 1, 14, -1.0},  {6, 1303, 2, 2, -1.0},  {6, 1310, 2, 2, -1.0},  {6, 1316, 2, 2, -1.0},
  {6, 1321, 2, 2, -1.0},  {6, 1381, 2, 3, -1.0},  {6, 1388, 2, 3, -1.0},  {6, 1394, 2, 3, -1.0},  {6, 1399, 2, 3, -1.0},  {6, 1447, 2, 4, -1.0},
  {6, 1454, 2, 4, -1.0},  {6, 1460, 2, 4, -1.0},  {6, 1465, 2, 4, -1.0},  {6, 1502, 2, 5, -1.0},  {6, 1509, 2, 5, -1.0},  {6, 1515, 2, 5, -1.0},
  {6, 1520, 2, 5, -1.0},  {6, 1547, 2, 6, -1.0},  {6, 1554, 2, 6, -1.0},  {6, 1560, 2, 6, -1.0},  {6, 1565, 2, 6, -1.0},  {6, 1583, 2, 7, -1.0},
  {6, 1590, 2, 7, -1.0},  {6, 1596, 2, 7, -1.0},  {6, 1601, 2, 7, -1.0},  {6, 1611, 2, 8, -1.0},  {6, 1612, 2, 9, -1.0},  {6, 1613, 2, 10, -1.0},
  {6, 1614, 2, 11, -1.0},  {6, 1615, 2, 12, -1.0},  {6, 1616, 2, 13, -1.0},  {6, 1617, 2, 14, -1.0},  {6, 1618, 2, 8, -1.0},  {6, 1624, 2, 8, -1.0},
  {6, 1629, 2, 8, -1.0},  {6, 1639, 2, 9, -1.0},  {6, 1640, 2, 10, -1.0},  {6, 1641, 2, 11, -1.0},  {6, 1642, 2, 12, -1.0},  {6, 1643, 2, 13, -1.0},
  {6, 1644, 2, 14, -1.0},  {6, 1645, 2, 9, -1.0},  {6, 1650, 2, 9, -1.0},  {6, 1660, 2, 10, -1.0},  {6, 1661, 2, 11, -1.0},  {6, 1662, 2, 12, -1.0},
  {6, 1663, 2, 13, -1.0},  {6, 1664, 2, 14, -1.0},  {6, 1665, 2, 10, -1.0},  {6, 1675, 2, 11, -1.0},  {6, 1676, 2, 12, -1.0},  {6, 1677, 2, 13, -1.0},
  {6, 1678, 2, 14, -1.0},  {6, 1745, 3, 3, -1.0},  {6, 1752, 3, 3, -1.0},  {6, 1758, 3, 3, -1.0},  {6, 1763, 3, 3, -1.0},  {6, 1811, 3, 4, -1.0},
  {6, 1818, 3, 4, -1.0},  {6, 1824, 3, 4, -1.0},  {6, 1829, 3, 4, -1.0},  {6, 1866, 3, 5, -1.0},  {6, 1873, 3, 5, -1.0},  {6, 1879, 3, 5, -1.0},
  {6, 1884, 3, 5, -1.0},  {6, 1911, 3, 6, -1.0},  {6, 1918, 3, 6, -1.0},  {6, 1924, 3, 6, -1.0},  {6, 1929, 3, 6, -1.0},  {6, 1947, 3, 7, -1.0},
  {6, 1954, 3, 7, -1.0},  {6, 1960, 3, 7, -1.0},  {6, 1965, 3, 7, -1.0},  {6, 1975, 3, 8, -1.0},  {6, 1976, 3, 9, -1.0},  {6, 1977, 3, 10, -1.0},
  {6, 1978, 3, 11, -1.0},  {6, 1979, 3, 12, -1.0},  {6, 1980, 3, 13, -1.0},  {6, 1981, 3, 14, -1.0},  {6, 1982, 3, 8, -1.0},  {6, 1988, 3, 8, -1.0},
  {6, 1993, 3, 8, -1.0},  {6, 2003, 3, 9, -1.0},  {6, 2004, 3, 10, -1.0},  {6, 2005, 3, 11, -1.0},  {6, 2006, 3, 12, -1.0},  {6, 2007, 3, 13, -1.0},
  {6, 2008, 3, 14, -1.0},  {6, 2009, 3, 9, -1.0},  {6, 2014, 3, 9, -1.0},  {6, 2024, 3, 10, -1.0},  {6, 2025, 3, 11, -1.0},  {6, 2026, 3, 12, -1.0},
  {6, 2027, 3, 13, -1.0},  {6, 2028, 3, 14, -1.0},  {6, 2029, 3, 10, -1.0},  {6, 2039, 3, 11, -1.0},  {6, 2040, 3, 12, -1.0},  {6, 2041, 3, 13, -1.0},
  {6, 2042, 3, 14, -1.0},  {6, 2097, 4, 4, -1.0},  {6, 2104, 4, 4, -1.0},  {6, 2110, 4, 4, -1.0},  {6, 2115, 4, 4, -1.0},  {6, 2152, 4, 5, -1.0},
  {6, 2159, 4, 5, -1.0},  {6, 2165, 4, 5, -1.0},  {6, 2170, 4, 5, -1.0},  {6, 2197, 4, 6, -1.0},  {6, 2204, 4, 6, -1.0},  {6, 2210, 4, 6, -1.0},
  {6, 2215, 4, 6, -1.0},  {6, 2233, 4, 7, -1.0},  {6, 2240, 4, 7, -1.0},  {6, 2246, 4, 7, -1.0},  {6, 2251, 4, 7, -1.0},  {6, 2261, 4, 8, -1.0},
  {6, 2262, 4, 9, -1.0},  {6, 2263, 4, 10, -1.0},  {6, 2264, 4, 11, -1.0},  {6, 2265, 4, 12, -1.0},  {6, 2266, 4, 13, -1.0},  {6, 2267, 4, 14, -1.0},
  {6, 2268, 4, 8, -1.0},  {6, 2274, 4, 8, -1.0},  {6, 2279, 4, 8, -1.0},  {6, 2289, 4, 9, -1.0},  {6, 2290, 4, 10, -1.0},  {6, 2291, 4, 11, -1.0},
  {6, 2292, 4, 12, -1.0},  {6, 2293, 4, 13, -1.0},  {6, 2294, 4, 14, -1.0},  {6, 2295, 4, 9, -1.0},  {6, 2300, 4, 9, -1.0},  {6, 2310, 4, 10, -1.0},
  {6, 2311, 4, 11, -1.0},  {6, 2312, 4, 12, -1.0},  {6, 2313, 4, 13, -1.0},  {6, 2314, 4, 14, -1.0},  {6, 2315, 4, 10, -1.0},  {6, 2325, 4, 11, -1.0},
  {6, 2326, 4, 12, -1.0},  {6, 2327, 4, 13, -1.0},  {6, 2328, 4, 14, -1.0},  {6, 2372, 5, 5, -1.0},  {6, 2379, 5, 5, -1.0},  {6, 2385, 5, 5, -1.0},
  {6, 2390, 5, 5, -1.0},  {6, 2417, 5, 6, -1.0},  {6, 2424, 5, 6, -1.0},  {6, 2430, 5, 6, -1.0},  {6, 2435, 5, 6, -1.0},  {6, 2453, 5, 7, -1.0},
  {6, 2460, 5, 7, -1.0},  {6, 2466, 5, 7, -1.0},  {6, 2471, 5, 7, -1.0},  {6, 2481, 5, 8, -1.0},  {6, 2482, 5, 9, -1.0},  {6, 2483, 5, 10, -1.0},
  {6, 2484, 5, 11, -1.0},  {6, 2485, 5, 12, -1.0},  {6, 2486, 5, 13, -1.0},  {6, 2487, 5, 14, -1.0},  {6, 2488, 5, 8, -1.0},  {6, 2494, 5, 8, -1.0},
  {6, 2499, 5, 8, -1.0},  {6, 2509, 5, 9, -1.0},  {6, 2510, 5, 10, -1.0},  {6, 2511, 5, 11, -1.0},  {6, 2512, 5, 12, -1.0},  {6, 2513, 5, 13, -1.0},
  {6, 2514, 5, 14, -1.0},  {6, 2515, 5, 9, -1.0},  {6, 2520, 5, 9, -1.0},  {6, 2530, 5, 10, -1.0},  {6, 2531, 5, 11, -1.0},  {6, 2532, 5, 12, -1.0},
  {6, 2533, 5, 13, -1.0},  {6, 2534, 5, 14, -1.0},  {6, 2535, 5, 10, -1.0},  {6, 2545, 5, 11, -1.0},  {6, 2546, 5, 12, -1.0},  {6, 2547, 5, 13, -1.0},
  {6, 2548, 5, 14, -1.0},  {6, 2582, 6, 6, -1.0},  {6, 2589, 6, 6, -1.0},  {6, 2595, 6, 6, -1.0},  {6, 2600, 6, 6, -1.0},  {6, 2618, 6, 7, -1.0},
  {6, 2625, 6, 7, -1.0},  {6, 2631, 6, 7, -1.0},  {6, 2636, 6, 7, -1.0},  {6, 2646, 6, 8, -1.0},  {6, 2647, 6, 9, -1.0},  {6, 2648, 6, 10, -1.0},
  {6, 2649, 6, 11, -1.0},  {6, 2650, 6, 12, -1.0},  {6, 2651, 6, 13, -1.0},  {6, 2652, 6, 14, -1.0},  {6, 2653, 6, 8, -1.0},  {6, 2659, 6, 8, -1.0},
  {6, 2664, 6, 8, -1.0},  {6, 2674, 6, 9, -1.0},  {6, 2675, 6, 10, -1.0},  {6, 2676, 6, 11, -1.0},  {6, 2677, 6, 12, -1.0},  {6, 2678, 6, 13, -1.0},
  {6, 2679, 6, 14, -1.0},  {6, 2680, 6, 9, -1.0},  {6, 2685, 6, 9, -1.0},  {6, 2695, 6, 10, -1.0},  {6, 2696, 6, 11, -1.0},  {6, 2697, 6, 12, -1.0},
  {6, 2698, 6, 13, -1.0},  {6, 2699, 6, 14, -1.0},  {6, 2700, 6, 10, -1.0},  {6, 2710, 6, 11, -1.0},  {6, 2711, 6, 12, -1.0},  {6, 2712, 6, 13, -1.0},
  {6, 2713, 6, 14, -1.0},  {6, 2738, 7, 7, -1.0},  {6, 2745, 7, 7, -1.0},  {6, 2751, 7, 7, -1.0},  {6, 2756, 7, 7, -1.0},  {6, 2766, 7, 8, -1.0},
  {6, 2767, 7, 9, -1.0},  {6, 2768, 7, 10, -1.0},  {6, 2769, 7, 11, -1.0},  {6, 2770, 7, 12, -1.0},  {6, 2771, 7, 13, -1.0},  {6, 2772, 7, 14, -1.0},
  {6, 2773, 7, 8, -1.0},  {6, 2779, 7, 8, -1.0},  {6, 2784, 7, 8, -1.0},  {6, 2794, 7, 9, -1.0},  {6, 2795, 7, 10, -1.0},  {6, 2796, 7, 11, -1.0},
  {6, 2797, 7, 12, -1.0},  {6, 2798, 7, 13, -1.0},  {6, 2799, 7, 14, -1.0},  {6, 2800, 7, 9, -1.0},  {6, 2805, 7, 9, -1.0},  {6, 2815, 7, 10, -1.0},
  {6, 2816, 7, 11, -1.0},  {6, 2817, 7, 12, -1.0},  {6, 2818, 7, 13, -1.0},  {6, 2819, 7, 14, -1.0},  {6, 2820, 7, 10, -1.0},  {6, 2830, 7, 11, -1.0},
  {6, 2831, 7, 12, -1.0},  {6, 2832, 7, 13, -1.0},  {6, 2833, 7, 14, -1.0},  {6, 2850, 8, 8, -1.0},  {6, 2851, 8, 9, -1.0},  {6, 2852, 8, 10, -1.0},
  {6, 2853, 8, 11, -1.0},  {6, 2854, 8, 12, -1.0},  {6, 2855, 8, 13, -1.0},  {6, 2856, 8, 14, -1.0},  {6, 2857, 8, 8, -1.0},  {6, 2857, 9, 9, -1.0},
  {6, 2858, 9, 10, -1.0},  {6, 2859, 9, 11, -1.0},  {6, 2860, 9, 12, -1.0},  {6, 2861, 9, 13, -1.0},  {6, 2862, 9, 14, -1.0},  {6, 2863, 8, 8, -1.0},
  {6, 2863, 10, 10, -1.0},  {6, 2864, 10, 11, -1.0},  {6, 2865, 10, 12, -1.0},  {6, 2866, 10, 13, -1.0},  {6, 2867, 10, 14, -1.0},  {6, 2868, 8, 8, -1.0},
  {6, 2868, 11, 11, -1.0},  {6, 2869, 11, 12, -1.0},  {6, 2870, 11, 13, -1.0},  {6, 2871, 11, 14, -1.0},  {6, 2872, 12, 12, -1.0},  {6, 2873, 12, 13, -1.0},
  {6, 2874, 12, 14, -1.0},  {6, 2875, 13, 13, -1.0},  {6, 2876, 13, 14, -1.0},  {6, 2877, 14, 14, -1.0},  {6, 2878, 8, 9, -1.0},  {6, 2879, 8, 10, -1.0},
  {6, 2880, 8, 11, -1.0},  {6, 2881, 8, 12, -1.0},  {6, 2882, 8, 13, -1.0},  {6, 2883, 8, 14, -1.0},  {6, 2884, 8, 9, -1.0},  {6, 2889, 8, 9, -1.0},
  {6, 2899, 8, 10, -1.0},  {6, 2900, 8, 11, -1.0},  {6, 2901, 8, 12, -1.0},  {6, 2902, 8, 13, -1.0},  {6, 2903, 8, 14, -1.0},  {6, 2904, 8, 10, -1.0},
  {6, 2914, 8, 11, -1.0},  {6, 2915, 8, 12, -1.0},  {6, 2916, 8, 13, -1.0},  {6, 2917, 8, 14, -1.0},  {6, 2934, 9, 9, -1.0},  {6, 2935, 9, 10, -1.0},
  {6, 2936, 9, 11, -1.0},  {6, 2937, 9, 12, -1.0},  {6, 2938, 9, 13, -1.0},  {6, 2939, 9, 14, -1.0},  {6, 2940, 9, 9, -1.0},  {6, 2940, 10, 10, -1.0},
  {6, 2941, 10, 11, -1.0},  {6, 2942, 10, 12, -1.0},  {6, 2943, 10, 13, -1.0},  {6, 2944, 10, 14, -1.0},  {6, 2945, 9, 9, -1.0},  {6, 2945, 11, 11, -1.0},
  {6, 2946, 11, 12, -1.0},  {6, 2947, 11, 13, -1.0},  {6, 2948, 11, 14, -1.0},  {6, 2949, 12, 12, -1.0},  {6, 2950, 12, 13, -1.0},  {6, 2951, 12, 14, -1.0},
  {6, 2952, 13, 13, -1.0},  {6, 2953, 13, 14, -1.0},  {6, 2954, 14, 14, -1.0},  {6, 2955, 9, 10, -1.0},  {6, 2956, 9, 11, -1.0},  {6, 2957, 9, 12, -1.0},
  {6, 2958, 9, 13, -1.0},  {6, 2959, 9, 14, -1.0},  {6, 2960, 9, 10, -1.0},  {6, 2970, 9, 11, -1.0},  {6, 2971, 9, 12, -1.0},  {6, 2972, 9, 13, -1.0},
  {6, 2973, 9, 14, -1.0},  {6, 2990, 10, 10, -1.0},  {6, 2991, 10, 11, -1.0},  {6, 2992, 10, 12, -1.0},  {6, 2993, 10, 13, -1.0},  {6, 2994, 10, 14, -1.0},
  {6, 2995, 10, 10, -1.0},  {6, 2995, 11, 11, -1.0},  {6, 2996, 11, 12, -1.0},  {6, 2997, 11, 13, -1.0},  {6, 2998, 11, 14, -1.0},  {6, 2999, 12, 12, -1.0},
  {6, 3000, 12, 13, -1.0},  {6, 3001, 12, 14, -1.0},  {6, 3002, 13, 13, -1.0},  {6, 3003, 13, 14, -1.0},  {6, 3004, 14, 14, -1.0},  {6, 3005, 10, 11, -1.0},
  {6, 3006, 10, 12, -1.0},  {6, 3007, 10, 13, -1.0},  {6, 3008, 10, 14, -1.0},  {6, 3025, 11, 11, -1.0},  {6, 3026, 11, 12, -1.0},  {6, 3027, 11, 13, -1.0},
  {6, 3028, 11, 14, -1.0},  {6, 3029, 12, 12, -1.0},  {6, 3030, 12, 13, -1.0},  {6, 3031, 12, 14, -1.0},  {6, 3032, 13, 13, -1.0},  {6, 3033, 13, 14, -1.0},
  {6, 3034, 14, 14, -1.0},  {7, 8, 0, 0, 1.0},  {7, 22, 0, 1, 1.0},  {7, 35, 0, 2, 1.0},  {7, 47, 0, 3, 1.0},  {7, 58, 0, 4, 1.0},
  {7, 68, 0, 5, 1.0},  {7, 77, 0, 6, 1.0},  {7, 85, 0, 7, 1.0},  {7, 92, 0, 8, 1.0},  {7, 93, 0, 9, 1.0},  {7, 94, 0, 10, 1.0},
  {7, 95, 0, 11, 1.0},  {7, 96, 0, 12, 1.0},  {7, 97, 0, 13, 1.0},  {7, 98, 0, 14, 1.0},  {7, 127, 1, 1, 1.0},  {7, 140, 1, 2, 1.0},
  {7, 152, 1, 3, 1.0},  {7, 163, 1, 4, 1.0},  {7, 173, 1, 5, 1.0},  {7, 182, 1, 6, 1.0},  {7, 190, 1, 7, 1.0},  {7, 197, 1, 8, 1.0},
  {7, 198, 1, 9, 1.0},  {7, 199, 1, 10, 1.0},  {7, 200, 1, 11, 1.0},  {7, 201, 1, 12, 1.0},  {7, 202, 1, 13, 1.0},  {7, 203, 1, 14, 1.0},
  {7, 231, 2, 2, 1.0},  {7, 243, 2, 3, 1.0},  {7, 254, 2, 4, 1.0},  {7, 264, 2, 5, 1.0},  {7, 273, 2, 6, 1.0},  {7, 281, 2, 7, 1.0},
  {7, 288, 2, 8, 1.0},  {7, 289, 2, 9, 1.0},  {7, 290, 2, 10, 1.0},  {7, 291, 2, 11, 1.0},  {7, 292, 2, 12, 1.0},  {7, 293, 2, 13, 1.0},
  {7, 294, 2, 14, 1.0},  {7, 321, 3, 3, 1.0},  {7, 332, 3, 4, 1.0},  {7, 342, 3, 5, 1.0},  {7, 351, 3, 6, 1.0},  {7, 359, 3, 7, 1.0},
  {7, 366, 3, 8, 1.0},  {7, 367, 3, 9, 1.0},  {7, 368, 3, 10, 1.0},  {7, 369, 3, 11, 1.0},  {7, 370, 3, 12, 1.0},  {7, 371, 3, 13, 1.0},
  {7, 372, 3, 14, 1.0},  {7, 398, 4, 4, 1.0},  {7, 408, 4, 5, 1.0},  {7, 417, 4, 6, 1.0},  {7, 425, 4, 7, 1.0},  {7, 432, 4, 8, 1.0},
  {7, 433, 4, 9, 1.0},  {7, 434, 4, 10, 1.0},  {7, 435, 4, 11, 1.0},  {7, 436, 4, 12, 1.0},  {7, 437, 4, 13, 1.0},  {7, 438, 4, 14, 1.0},
  {7, 463, 5, 5, 1.0},  {7, 472, 5, 6, 1.0},  {7, 480, 5, 7, 1.0},  {7, 487, 5, 8, 1.0},  {7, 488, 5, 9, 1.0},  {7, 489, 5, 10, 1.0},
  {7, 490, 5, 11, 1.0},  {7, 491, 5, 12, 1.0},  {7, 492, 5, 13, 1.0},  {7, 493, 5, 14, 1.0},  {7, 517, 6, 6, 1.0},  {7, 525, 6, 7, 1.0},
  {7, 532, 6, 8, 1.0},  {7, 533, 6, 9, 1.0},  {7, 534, 6, 10, 1.0},  {7, 535, 6, 11, 1.0},  {7, 536, 6, 12, 1.0},  {7, 537, 6, 13, 1.0},
  {7, 538, 6, 14, 1.0},  {7, 561, 7, 7, 1.0},  {7, 568, 7, 8, 1.0},  {7, 569, 7, 9, 1.0},  {7, 570, 7, 10, 1.0},  {7, 571, 7, 11, 1.0},
  {7, 572, 7, 12, 1.0},  {7, 573, 7, 13, 1.0},  {7, 574, 7, 14, 1.0},  {7, 596, 8, 8, 1.0},  {7, 597, 8, 9, 1.0},  {7, 598, 8, 10, 1.0},
  {7, 599, 8, 11, 1.0},  {7, 600, 8, 12, 1.0},  {7, 601, 8, 13, 1.0},  {7, 602, 8, 14, 1.0},  {7, 603, 9, 9, 1.0},  {7, 604, 9, 10, 1.0},
  {7, 605, 9, 11, 1.0},  {7, 606, 9, 12, 1.0},  {7, 607, 9, 13, 1.0},  {7, 608, 9, 14, 1.0},  {7, 609, 10, 10, 1.0},  {7, 610, 10, 11, 1.0},
  {7, 611, 10, 12, 1.0},  {7, 612, 10, 13, 1.0},  {7, 613, 10, 14, 1.0},  {7, 614, 11, 11, 1.0},  {7, 615, 11, 12, 1.0},  {7, 616, 11, 13, 1.0},
  {7, 617, 11, 14, 1.0},  {7, 618, 12, 12, 1.0},  {7, 619, 12, 13, 1.0},  {7, 620, 12, 14, 1.0},  {7, 621, 13, 13, 1.0},  {7, 622, 13, 14, 1.0},
  {7, 623, 14, 14, 1.0},  {8, 0, 0, 0, 0.0},  {8, 1, 0, 1, 0.0},  {8, 2, 0, 2, 0.0},  {8, 3, 0, 3, 0.0},  {8, 4, 0, 4, 0.0},
  {8, 5, 0, 5, 0.0},  {8, 6, 0, 6, 0.0},  {8, 7, 0, 7, 0.0},  {8, 8, 0, 8, 0.0},  {8, 9, 0, 9, 0.0},  {8, 10, 0, 10, 0.0},
  {8, 11, 0, 11, 0.0},  {8, 12, 0, 12, 0.0},  {8, 13, 0, 13, 0.0},  {8, 14, 0, 14, 0.0},  {8, 15, 1, 1, 0.0},  {8, 16, 1, 2, 0.0},
  {8, 17, 1, 3, 0.0},  {8, 18, 1, 4, 0.0},  {8, 19, 1, 5, 0.0},  {8, 20, 1, 6, 0.0},  {8, 21, 1, 7, 0.0},  {8, 22, 1, 8, 0.0},
  {8, 23, 1, 9, 0.0},  {8, 24, 1, 10, 0.0},  {8, 25, 1, 11, 0.0},  {8, 26, 1, 12, 0.0},  {8, 27, 1, 13, 0.0},  {8, 28, 1, 14, 0.0},
  {8, 29, 2, 2, 0.0},  {8, 30, 2, 3, 0.0},  {8, 31, 2, 4, 0.0},  {8, 32, 2, 5, 0.0},  {8, 33, 2, 6, 0.0},  {8, 34, 2, 7, 0.0},
  {8, 35, 2, 8, 0.0},  {8, 36, 2, 9, 0.0},  {8, 37, 2, 10, 0.0},  {8, 38, 2, 11, 0.0},  {8, 39, 2, 12, 0.0},  {8, 40, 2, 13, 0.0},
  {8, 41, 2, 14, 0.0},  {8, 42, 3, 3, 0.0},  {8, 43, 3, 4, 0.0},  {8, 44, 3, 5, 0.0},  {8, 45, 3, 6, 0.0},  {8, 46, 3, 7, 0.0},
  {8, 47, 3, 8, 0.0},  {8, 48, 3, 9, 0.0},  {8, 49, 3, 10, 0.0},  {8, 50, 3, 11, 0.0},  {8, 51, 3, 12, 0.0},  {8, 52, 3, 13, 0.0},
  {8, 53, 3, 14, 0.0},  {8, 54, 4, 4, 0.0},  {8, 55, 4, 5, 0.0},  {8, 56, 4, 6, 0.0},  {8, 57, 4, 7, 0.0},  {8, 58, 4, 8, 0.0},
  {8, 59, 4, 9, 0.0},  {8, 60, 4, 10, 0.0},  {8, 61, 4, 11, 0.0},  {8, 62, 4, 12, 0.0},  {8, 63, 4, 13, 0.0},  {8, 64, 4, 14, 0.0},
  {8, 65, 5, 5, 0.0},  {8, 66, 5, 6, 0.0},  {8, 67, 5, 7, 0.0},  {8, 68, 5, 8, 0.0},  {8, 69, 5, 9, 0.0},  {8, 70, 5, 10, 0.0},
  {8, 71, 5, 11, 0.0},  {8, 72, 5, 12, 0.0},  {8, 73, 5, 13, 0.0},  {8, 74, 5, 14, 0.0},  {8, 75, 6, 6, 0.0},  {8, 76, 6, 7, 0.0},
  {8, 77, 6, 8, 0.0},  {8, 78, 6, 9, 0.0},  {8, 79, 6, 10, 0.0},  {8, 80, 6, 11, 0.0},  {8, 81, 6, 12, 0.0},  {8, 82, 6, 13, 0.0},
  {8, 83, 6, 14, 0.0},  {8, 84, 7, 7, 0.0},  {8, 85, 7, 8, 0.0},  {8, 86, 7, 9, 0.0},  {8, 87, 7, 10, 0.0},  {8, 88, 7, 11, 0.0},
  {8, 89, 7, 12, 0.0},  {8, 90, 7, 13, 0.0},  {8, 91, 7, 14, 0.0},  {8, 92, 8, 8, 0.0},  {8, 93, 8, 9, 0.0},  {8, 94, 8, 10, 0.0},
  {8, 95, 8, 11, 0.0},  {8, 96, 8, 12, 0.0},  {8, 97, 8, 13, 0.0},  {8, 98, 8, 14, 0.0},  {8, 99, 9, 9, 0.0},  {8, 100, 9, 10, 0.0},
  {8, 101, 9, 11, 0.0},  {8, 102, 9, 12, 0.0},  {8, 103, 9, 13, 0.0},  {8, 104, 9, 14, 0.0},  {8, 105, 10, 10, 0.0},  {8, 106, 10, 11, 0.0},
  {8, 107, 10, 12, 0.0},  {8, 108, 10, 13, 0.0},  {8, 109, 10, 14, 0.0},  {8, 110, 11, 11, 0.0},  {8, 111, 11, 12, 0.0},  {8, 112, 11, 13, 0.0},
  {8, 113, 11, 14, 0.0},  {8, 114, 0, 0, -1.0},  {8, 114, 12, 12, 0.0},  {8, 115, 12, 13, 0.0},  {8, 116, 12, 14, 0.0},  {8, 117, 0, 0, -1.0},
  {8, 117, 13, 13, 0.0},  {8, 118, 13, 14, 0.0},  {8, 119, 0, 0, -1.0},  {8, 119, 14, 14, 0.0},  {8, 219, 0, 1, -1.0},  {8, 222, 0, 1, -1.0},
  {8, 224, 0, 1, -1.0},  {8, 310, 0, 2, -1.0},  {8, 313, 0, 2, -1.0},  {8, 315, 0, 2, -1.0},  {8, 388, 0, 3, -1.0},  {8, 391, 0, 3, -1.0},
  {8, 393, 0, 3, -1.0},  {8, 454, 0, 4, -1.0},  {8, 457, 0, 4, -1.0},  {8, 459, 0, 4, -1.0},  {8, 509, 0, 5, -1.0},  {8, 512, 0, 5, -1.0},
  {8, 514, 0, 5, -1.0},  {8, 554, 0, 6, -1.0},  {8, 557, 0, 6, -1.0},  {8, 559, 0, 6, -1.0},  {8, 590, 0, 7, -1.0},  {8, 593, 0, 7, -1.0},
  {8, 595, 0, 7, -1.0},  {8, 618, 0, 8, -1.0},  {8, 621, 0, 8, -1.0},  {8, 623, 0, 8, -1.0},  {8, 639, 0, 9, -1.0},  {8, 642, 0, 9, -1.0},
  {8, 644, 0, 9, -1.0},  {8, 654, 0, 10, -1.0},  {8, 657, 0, 10, -1.0},  {8, 659, 0, 10, -1.0},  {8, 664, 0, 11, -1.0},  {8, 667, 0, 11, -1.0},
  {8, 669, 0, 11, -1.0},  {8, 670, 0, 12, -1.0},  {8, 671, 0, 13, -1.0},  {8, 672, 0, 14, -1.0},  {8, 673, 0, 12, -1.0},  {8, 675, 0, 12, -1.0},
  {8, 676, 0, 13, -1.0},  {8, 677, 0, 14, -1.0},  {8, 678, 0, 13, -1.0},  {8, 679, 0, 14, -1.0},  {8, 779, 1, 1, -1.0},  {8, 782, 1, 1, -1.0},
  {8, 784, 1, 1, -1.0},  {8, 870, 1, 2, -1.0},  {8, 873, 1, 2, -1.0},  {8, 875, 1, 2, -1.0},  {8, 948, 1, 3, -1.0},  {8, 951, 1, 3, -1.0},
  {8, 953, 1, 3, -1.0},  {8, 1014, 1, 4, -1.0},  {8, 1017, 1, 4, -1.0},  {8, 1019, 1, 4, -1.0},  {8, 1069, 1, 5, -1.0},  {8, 1072, 1, 5, -1.0},
  {8, 1074, 1, 5, -1.0},  {8, 1114, 1, 6, -1.0},  {8, 1117, 1, 6, -1.0},  {8, 1119, 1, 6, -1.0},  {8, 1150, 1, 7, -1.0},  {8, 1153, 1, 7, -1.0},
  {8, 1155, 1, 7, -1.0},  {8, 1178, 1, 8, -1.0},  {8, 1181, 1, 8, -1.0},  {8, 1183, 1, 8, -1.0},  {8, 1199, 1, 9, -1.0},  {8, 1202, 1, 9, -1.0},
  {8, 1204, 1, 9, -1.0},  {8, 1214, 1, 10, -1.0},  {8, 1217, 1, 10, -1.0},  {8, 1219, 1, 10, -1.0},  {8, 1224, 1, 11, -1.0},  {8, 1227, 1, 11, -1.0},
  {8, 1229, 1, 11, -1.0},  {8, 1230, 1, 12, -1.0},  {8, 1231, 1, 13, -1.0},  {8, 1232, 1, 14, -1.0},  {8, 1233, 1, 12, -1.0},  {8, 1235, 1, 12, -1.0},
  {8, 1236, 1, 13, -1.0},  {8, 1237, 1, 14, -1.0},  {8, 1238, 1, 13, -1.0},  {8, 1239, 1, 14, -1.0},  {8, 1325, 2, 2, -1.0},  {8, 1328, 2, 2, -1.0},
  {8, 1330, 2, 2, -1.0},  {8, 1403, 2, 3, -1.0},  {8, 1406, 2, 3, -1.0},  {8, 1408, 2, 3, -1.0},  {8, 1469, 2, 4, -1.0},  {8, 1472, 2, 4, -1.0},
  {8, 1474, 2, 4, -1.0},  {8, 1524, 2, 5, -1.0},  {8, 1527, 2, 5, -1.0},  {8, 1529, 2, 5, -1.0},  {8, 1569, 2, 6, -1.0},  {8, 1572, 2, 6, -1.0},
  {8, 1574, 2, 6, -1.0},  {8, 1605, 2, 7, -1.0},  {8, 1608, 2, 7, -1.0},  {8, 1610, 2, 7, -1.0},  {8, 1633, 2, 8, -1.0},  {8, 1636, 2, 8, -1.0},
  {8, 1638, 2, 8, -1.0},  {8, 1654, 2, 9, -1.0},  {8, 1657, 2, 9, -1.0},  {8, 1659, 2, 9, -1.0},  {8, 1669, 2, 10, -1.0},  {8, 1672, 2, 10, -1.0},
  {8, 1674, 2, 10, -1.0},  {8, 1679, 2, 11, -1.0},  {8, 1682, 2, 11, -1.0},  {8, 1684, 2, 11, -1.0},  {8, 1685, 2, 12, -1.0},  {8, 1686, 2, 13, -1.0},
  {8, 1687, 2, 14, -1.0},  {8, 1688, 2, 12, -1.0},  {8, 1690, 2, 12, -1.0},  {8, 1691, 2, 13, -1.0},  {8, 1692, 2, 14, -1.0},  {8, 1693, 2, 13, -1.0},
  {8, 1694, 2, 14, -1.0},  {8, 1767, 3, 3, -1.0},  {8, 1770, 3, 3, -1.0},  {8, 1772, 3, 3, -1.0},  {8, 1833, 3, 4, -1.0},  {8, 1836, 3, 4, -1.0},
  {8, 1838, 3, 4, -1.0},  {8, 1888, 3, 5, -1.0},  {8, 1891, 3, 5, -1.0},  {8, 1893, 3, 5, -1.0},  {8, 1933, 3, 6, -1.0},  {8, 1936, 3, 6, -1.0},
  {8, 1938, 3, 6, -1.0},  {8, 1969, 3, 7, -1.0},  {8, 1972, 3, 7, -1.0},  {8, 1974, 3, 7, -1.0},  {8, 1997, 3, 8, -1.0},  {8, 2000, 3, 8, -1.0},
  {8, 2002, 3, 8, -1.0},  {8, 2018, 3, 9, -1.0},  {8, 2021, 3, 9, -1.0},  {8, 2023, 3, 9, -1.0},  {8, 2033, 3, 10, -1.0},  {8, 2036, 3, 10, -1.0},
  {8, 2038, 3, 10, -1.0},  {8, 2043, 3, 11, -1.0},  {8, 2046, 3, 11, -1.0},  {8, 2048, 3, 11, -1.0},  {8, 2049, 3, 12, -1.0},  {8, 2050, 3, 13, -1.0},
  {8, 2051, 3, 14, -1.0},  {8, 2052, 3, 12, -1.0},  {8, 2054, 3, 12, -1.0},  {8, 2055, 3, 13, -1.0},  {8, 2056, 3, 14, -1.0},  {8, 2057, 3, 13, -1.0},
  {8, 2058, 3, 14, -1.0},  {8, 2119, 4, 4, -1.0},  {8, 2122, 4, 4, -1.0},  {8, 2124, 4, 4, -1.0},  {8, 2174, 4, 5, -1.0},  {8, 2177, 4, 5, -1.0},
  {8, 2179, 4, 5, -1.0},  {8, 2219, 4, 6, -1.0},  {8, 2222, 4, 6, -1.0},  {8, 2224, 4, 6, -1.0},  {8, 2255, 4, 7, -1.0},  {8, 2258, 4, 7, -1.0},
  {8, 2260, 4, 7, -1.0},  {8, 2283, 4, 8, -1.0},  {8, 2286, 4, 8, -1.0},  {8, 2288, 4, 8, -1.0},  {8, 2304, 4, 9, -1.0},  {8, 2307, 4, 9, -1.0},
  {8, 2309, 4, 9, -1.0},  {8, 2319, 4, 10, -1.0},  {8, 2322, 4, 10, -1.0},  {8, 2324, 4, 10, -1.0},  {8, 2329, 4, 11, -1.0},  {8, 2332, 4, 11, -1.0},
  {8, 2334, 4, 11, -1.0},  {8, 2335, 4, 12, -1.0},  {8, 2336, 4, 13, -1.0},  {8, 2337, 4, 14, -1.0},  {8, 2338, 4, 12, -1.0},  {8, 2340, 4, 12, -1.0},
  {8, 2341, 4, 13, -1.0},  {8, 2342, 4, 14, -1.0},  {8, 2343, 4, 13, -1.0},  {8, 2344, 4, 14, -1.0},  {8, 2394, 5, 5, -1.0},  {8, 2397, 5, 5, -1.0},
  {8, 2399, 5, 5, -1.0},  {8, 2439, 5, 6, -1.0},  {8, 2442, 5, 6, -1.0},  {8, 2444, 5, 6, -1.0},  {8, 2475, 5, 7, -1.0},  {8, 2478, 5, 7, -1.0},
  {8, 2480, 5, 7, -1.0},  {8, 2503, 5, 8, -1.0},  {8, 2506, 5, 8, -1.0},  {8, 2508, 5, 8, -1.0},  {8, 2524, 5, 9, -1.0},  {8, 2527, 5, 9, -1.0},
  {8, 2529, 5, 9, -1.0},  {8, 2539, 5, 10, -1.0},  {8, 2542, 5, 10, -1.0},  {8, 2544, 5, 10, -1.0},  {8, 2549, 5, 11, -1.0},  {8, 2552, 5, 11, -1.0},
  {8, 2554, 5, 11, -1.0},  {8, 2555, 5, 12, -1.0},  {8, 2556, 5, 13, -1.0},  {8, 2557, 5, 14, -1.0},  {8, 2558, 5, 12, -1.0},  {8, 2560, 5, 12, -1.0},
  {8, 2561, 5, 13, -1.0},  {8, 2562, 5, 14, -1.0},  {8, 2563, 5, 13, -1.0},  {8, 2564, 5, 14, -1.0},  {8, 2604, 6, 6, -1.0},  {8, 2607, 6, 6, -1.0},
  {8, 2609, 6, 6, -1.0},  {8, 2640, 6, 7, -1.0},  {8, 2643, 6, 7, -1.0},  {8, 2645, 6, 7, -1.0},  {8, 2668, 6, 8, -1.0},  {8, 2671, 6, 8, -1.0},
  {8, 2673, 6, 8, -1.0},  {8, 2689, 6, 9, -1.0},  {8, 2692, 6, 9, -1.0},  {8, 2694, 6, 9, -1.0},  {8, 2704, 6, 10, -1.0},  {8, 2707, 6, 10, -1.0},
  {8, 2709, 6, 10, -1.0},  {8, 2714, 6, 11, -1.0},  {8, 2717, 6, 11, -1.0},  {8, 2719, 6, 11, -1.0},  {8, 2720, 6, 12, -1.0},  {8, 2721, 6, 13, -1.0},
  {8, 2722, 6, 14, -1.0},  {8, 2723, 6, 12, -1.0},  {8, 2725, 6, 12, -1.0},  {8, 2726, 6, 13, -1.0},  {8, 2727, 6, 14, -1.0},  {8, 2728, 6, 13, -1.0},
  {8, 2729, 6, 14, -1.0},  {8, 2760, 7, 7, -1.0},  {8, 2763, 7, 7, -1.0},  {8, 2765, 7, 7, -1.0},  {8, 2788, 7, 8, -1.0},  {8, 2791, 7, 8, -1.0},
  {8, 2793, 7, 8, -1.0},  {8, 2809, 7, 9, -1.0},  {8, 2812, 7, 9, -1.0},  {8, 2814, 7, 9, -1.0},  {8, 2824, 7, 10, -1.0},  {8, 2827, 7, 10, -1.0},
  {8, 2829, 7, 10, -1.0},  {8, 2834, 7, 11, -1.0},  {8, 2837, 7, 11, -1.0},  {8, 2839, 7, 11, -1.0},  {8, 2840, 7, 12, -1.0},  {8, 2841, 7, 13, -1.0},
  {8, 2842, 7, 14, -1.0},  {8, 2843, 7, 12, -1.0},  {8, 2845, 7, 12, -1.0},  {8, 2846, 7, 13, -1.0},  {8, 2847, 7, 14, -1.0},  {8, 2848, 7, 13, -1.0},
  {8, 2849, 7, 14, -1.0},  {8, 2872, 8, 8, -1.0},  {8, 2875, 8, 8, -1.0},  {8, 2877, 8, 8, -1.0},  {8, 2893, 8, 9, -1.0},  {8, 2896, 8, 9, -1.0},
  {8, 2898, 8, 9, -1.0},  {8, 2908, 8, 10, -1.0},  {8, 2911, 8, 10, -1.0},  {8, 2913, 8, 10, -1.0},  {8, 2918, 8, 11, -1.0},  {8, 2921, 8, 11, -1.0},
  {8, 2923, 8, 11, -1.0},  {8, 2924, 8, 12, -1.0},  {8, 2925, 8, 13, -1.0},  {8, 2926, 8, 14, -1.0},  {8, 2927, 8, 12, -1.0},  {8, 2929, 8, 12, -1.0},
  {8, 2930, 8, 13, -1.0},  {8, 2931, 8, 14, -1.0},  {8, 2932, 8, 13, -1.0},  {8, 2933, 8, 14, -1.0},  {8, 2949, 9, 9, -1.0},  {8, 2952, 9, 9, -1.0},
  {8, 2954, 9, 9, -1.0},  {8, 2964, 9, 10, -1.0},  {8, 2967, 9, 10, -1.0},  {8, 2969, 9, 10, -1.0},  {8, 2974, 9, 11, -1.0},  {8, 2977, 9, 11, -1.0},
  {8, 2979, 9, 11, -1.0},  {8, 2980, 9, 12, -1.0},  {8, 2981, 9, 13, -1.0},  {8, 2982, 9, 14, -1.0},  {8, 2983, 9, 12, -1.0},  {8, 2985, 9, 12, -1.0},
  {8, 2986, 9, 13, -1.0},  {8, 2987, 9, 14, -1.0},  {8, 2988, 9, 13, -1.0},  {8, 2989, 9, 14, -1.0},  {8, 2999, 10, 10, -1.0},  {8, 3002, 10, 10, -1.0},
  {8, 3004, 10, 10, -1.0},  {8, 3009, 10, 11, -1.0},  {8, 3012, 10, 11, -1.0},  {8, 3014, 10, 11, -1.0},  {8, 3015, 10, 12, -1.0},  {8, 3016, 10, 13, -1.0},
  {8, 3017, 10, 14, -1.0},  {8, 3018, 10, 12, -1.0},  {8, 3020, 10, 12, -1.0},  {8, 3021, 10, 13, -1.0},  {8, 3022, 10, 14, -1.0},  {8, 3023, 10, 13, -1.0},
  {8, 3024, 10, 14, -1.0},  {8, 3029, 11, 11, -1.0},  {8, 3032, 11, 11, -1.0},  {8, 3034, 11, 11, -1.0},  {8, 3035, 11, 12, -1.0},  {8, 3036, 11, 13, -1.0},
  {8, 3037, 11, 14, -1.0},  {8, 3038, 11, 12, -1.0},  {8, 3040, 11, 12, -1.0},  {8, 3041, 11, 13, -1.0},  {8, 3042, 11, 14, -1.0},  {8, 3043, 11, 13, -1.0},
  {8, 3044, 11, 14, -1.0},  {8, 3045, 12, 12, -1.0},  {8, 3046, 12, 13, -1.0},  {8, 3047, 12, 14, -1.0},  {8, 3048, 12, 12, -1.0},  {8, 3048, 13, 13, -1.0},
  {8, 3049, 13, 14, -1.0},  {8, 3050, 12, 12, -1.0},  {8, 3050, 14, 14, -1.0},  {8, 3051, 12, 13, -1.0},  {8, 3052, 12, 14, -1.0},  {8, 3053, 12, 13, -1.0},
  {8, 3054, 12, 14, -1.0},  {8, 3055, 13, 13, -1.0},  {8, 3056, 13, 14, -1.0},  {8, 3057, 13, 13, -1.0},  {8, 3057, 14, 14, -1.0},  {8, 3058, 13, 14, -1.0},
  {8, 3059, 14, 14, -1.0}
};
const HerwcProblem::conVar HerwcProblem::con_vars[] = {
  {4, 0, 0},  {4, 1, 0},  {4, 2, 0},  {4, 3, 0},  {4, 4, 0},
  {4, 5, 0},  {4, 6, 0},  {4, 7, 0},  {4, 8, 0},  {4, 9, 0},  {4, 10, 0},
  {4, 11, 0},  {4, 12, 0},  {4, 13, 0},  {4, 14, 0},  {4, 15, 0},  {4, 16, 0},
  {4, 17, 0},  {4, 18, 0},  {4, 19, 0},  {4, 20, 0},  {4, 21, 0},  {4, 22, 0},
  {4, 23, 0},  {4, 24, 0},  {4, 25, 0},  {4, 26, 0},  {4, 27, 0},  {4, 28, 0},
  {4, 29, 0},  {4, 30, 0},  {4, 31, 0},  {4, 32, 0},  {4, 33, 0},  {4, 34, 0},
  {4, 35, 0},  {4, 36, 0},  {4, 37, 0},  {4, 38, 0},  {4, 39, 0},  {4, 40, 0},
  {4, 41, 0},  {4, 42, 0},  {4, 43, 0},  {4, 44, 0},  {4, 45, 0},  {4, 46, 0},
  {4, 47, 0},  {4, 48, 0},  {4, 49, 0},  {4, 50, 0},  {4, 51, 0},  {4, 52, 0},
  {4, 53, 0},  {4, 54, 0},  {4, 55, 0},  {4, 56, 0},  {4, 57, 0},  {4, 58, 0},
  {4, 59, 0},  {4, 60, 0},  {4, 61, 0},  {4, 62, 0},  {4, 63, 0},  {4, 64, 0},
  {4, 65, 1},  {4, 66, 0},  {4, 67, 0},  {4, 68, 0},  {4, 69, 0},  {4, 70, 0},
  {4, 71, 0},  {4, 72, 0},  {4, 73, 0},  {4, 74, 0},  {4, 75, 1},  {4, 76, 0},
  {4, 77, 0},  {4, 78, 0},  {4, 79, 0},  {4, 80, 0},  {4, 81, 0},  {4, 82, 0},
  {4, 83, 0},  {4, 84, 1},  {4, 85, 0},  {4, 86, 0},  {4, 87, 0},  {4, 88, 0},
  {4, 89, 0},  {4, 90, 0},  {4, 91, 0},  {4, 92, 0},  {4, 93, 0},  {4, 94, 0},
  {4, 95, 0},  {4, 96, 0},  {4, 97, 0},  {4, 98, 0},  {4, 99, 0},  {4, 100, 0},
  {4, 101, 0},  {4, 102, 0},  {4, 103, 0},  {4, 104, 0},  {4, 105, 0},  {4, 106, 0},
  {4, 107, 0},  {4, 108, 0},  {4, 109, 0},  {4, 110, 0},  {4, 111, 0},  {4, 112, 0},
  {4, 113, 0},  {4, 114, 0},  {4, 115, 0},  {4, 116, 0},  {4, 117, 0},  {4, 118, 0},
  {4, 119, 0},  {8, 0, 0},  {8, 1, 0},  {8, 2, 0},  {8, 3, 0},  {8, 4, 0},
  {8, 5, 0},  {8, 6, 0},  {8, 7, 0},  {8, 8, 0},  {8, 9, 0},  {8, 10, 0},
  {8, 11, 0},  {8, 12, 0},  {8, 13, 0},  {8, 14, 0},  {8, 15, 0},  {8, 16, 0},
  {8, 17, 0},  {8, 18, 0},  {8, 19, 0},  {8, 20, 0},  {8, 21, 0},  {8, 22, 0},
  {8, 23, 0},  {8, 24, 0},  {8, 25, 0},  {8, 26, 0},  {8, 27, 0},  {8, 28, 0},
  {8, 29, 0},  {8, 30, 0},  {8, 31, 0},  {8, 32, 0},  {8, 33, 0},  {8, 34, 0},
  {8, 35, 0},  {8, 36, 0},  {8, 37, 0},  {8, 38, 0},  {8, 39, 0},  {8, 40, 0},
  {8, 41, 0},  {8, 42, 0},  {8, 43, 0},  {8, 44, 0},  {8, 45, 0},  {8, 46, 0},
  {8, 47, 0},  {8, 48, 0},  {8, 49, 0},  {8, 50, 0},  {8, 51, 0},  {8, 52, 0},
  {8, 53, 0},  {8, 54, 0},  {8, 55, 0},  {8, 56, 0},  {8, 57, 0},  {8, 58, 0},
  {8, 59, 0},  {8, 60, 0},  {8, 61, 0},  {8, 62, 0},  {8, 63, 0},  {8, 64, 0},
  {8, 65, 0},  {8, 66, 0},  {8, 67, 0},  {8, 68, 0},  {8, 69, 0},  {8, 70, 0},
  {8, 71, 0},  {8, 72, 0},  {8, 73, 0},  {8, 74, 0},  {8, 75, 0},  {8, 76, 0},
  {8, 77, 0},  {8, 78, 0},  {8, 79, 0},  {8, 80, 0},  {8, 81, 0},  {8, 82, 0},
  {8, 83, 0},  {8, 84, 0},  {8, 85, 0},  {8, 86, 0},  {8, 87, 0},  {8, 88, 0},
  {8, 89, 0},  {8, 90, 0},  {8, 91, 0},  {8, 92, 0},  {8, 93, 0},  {8, 94, 0},
  {8, 95, 0},  {8, 96, 0},  {8, 97, 0},  {8, 98, 0},  {8, 99, 0},  {8, 100, 0},
  {8, 101, 0},  {8, 102, 0},  {8, 103, 0},  {8, 104, 0},  {8, 105, 0},  {8, 106, 0},
  {8, 107, 0},  {8, 108, 0},  {8, 109, 0},  {8, 110, 0},  {8, 111, 0},  {8, 112, 0},
  {8, 113, 0},  {8, 114, 1},  {8, 115, 0},  {8, 116, 0},  {8, 117, 1},  {8, 118, 0},
  {8, 119, 1}
};
