/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: BmatEP.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 11-May-2024 15:06:41
 */

/* Include Files */
#include "BmatEP.h"

/* Function Definitions */
/*
 * BmatEP(Q)
 *
 *  B = BmatEP(Q) returns the 4x3 matrix which relates the
 *  body angular velocity vector w to the derivative of
 *  Euler parameter vector Q.
 *
 *   dQ/dt = 1/2 [B(Q)] w
 *
 *
 * Arguments    : double qw
 *                double qx
 *                double qy
 *                double qz
 *                double B[12]
 * Return Type  : void
 */
void BmatEP(double qw, double qx, double qy, double qz, double B[12])
{
  B[0] = -qx;
  B[4] = -qy;
  B[8] = -qz;
  B[1] = qw;
  B[5] = -qz;
  B[9] = qy;
  B[2] = qz;
  B[6] = qw;
  B[10] = -qx;
  B[3] = -qy;
  B[7] = qx;
  B[11] = qw;
}

/*
 * File trailer for BmatEP.c
 *
 * [EOF]
 */
