/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: EP2C.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 11-May-2024 14:40:32
 */

/* Include Files */
#include "EP2C.h"

/* Function Definitions */
/*
 * EP2C
 *
 *  C = EP2C(Q) returns the direction cosine
 *  matrix in terms of the 4x1 Euler parameter vector
 *  Q.  The first element is the non-dimensional Euler
 *  parameter, while the remain three elements form
 *  the Eulerparameter vector.
 *  PP: Takes a vector from NED to Body
 *
 *
 * Arguments    : double qw
 *                double qx
 *                double qy
 *                double qz
 *                double C[9]
 * Return Type  : void
 */
void EP2C(double qw, double qx, double qy, double qz, double C[9])
{
  double C_tmp;
  double b_C_tmp;
  double c_C_tmp;
  double d_C_tmp;
  double e_C_tmp;
  double f_C_tmp;
  double g_C_tmp;
  double h_C_tmp;
  /*  q = q/norm(q); */
  C_tmp = qw * qw;
  b_C_tmp = qx * qx;
  c_C_tmp = qy * qy;
  d_C_tmp = qz * qz;
  C[0] = ((C_tmp + b_C_tmp) - c_C_tmp) - d_C_tmp;
  e_C_tmp = qx * qy;
  f_C_tmp = qw * qz;
  C[3] = 2.0 * (e_C_tmp + f_C_tmp);
  g_C_tmp = qx * qz;
  h_C_tmp = qw * qy;
  C[6] = 2.0 * (g_C_tmp - h_C_tmp);
  C[1] = 2.0 * (e_C_tmp - f_C_tmp);
  C_tmp -= b_C_tmp;
  C[4] = (C_tmp + c_C_tmp) - d_C_tmp;
  b_C_tmp = qy * qz;
  e_C_tmp = qw * qx;
  C[7] = 2.0 * (b_C_tmp + e_C_tmp);
  C[2] = 2.0 * (g_C_tmp + h_C_tmp);
  C[5] = 2.0 * (b_C_tmp - e_C_tmp);
  C[8] = (C_tmp - c_C_tmp) + d_C_tmp;
}

/*
 * File trailer for EP2C.c
 *
 * [EOF]
 */
