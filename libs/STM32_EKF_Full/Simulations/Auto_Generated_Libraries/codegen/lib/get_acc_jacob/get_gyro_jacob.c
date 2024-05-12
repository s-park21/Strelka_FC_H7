/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: get_gyro_jacob.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 11-May-2024 14:18:01
 */

/* Include Files */
#include "get_gyro_jacob.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Declarations */
static double rt_powd_snf(double u0, double u1);

/* Function Definitions */
/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_powd_snf(double u0, double u1)
{
  double y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else {
    double d;
    double d1;
    d = fabs(u0);
    d1 = fabs(u1);
    if (rtIsInf(u1)) {
      if (d == 1.0) {
        y = 1.0;
      } else if (d > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d1 == 0.0) {
      y = 1.0;
    } else if (d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = rtNaN;
    } else {
      y = pow(u0, u1);
    }
  }
  return y;
}

/*
 * Arguments    : double w1
 *                double w2
 *                double w3
 *                double qw
 *                double qx
 *                double qy
 *                double qz
 *                double dt
 *                double jacob_w_pred[100]
 * Return Type  : void
 */
void get_gyro_jacob(double w1, double w2, double w3, double qw, double qx,
                    double qy, double qz, double dt, double jacob_w_pred[100])
{
  static const signed char iv[10] = {0, 0, 0, 0, 1, 0, 0, 0, 0, 0};
  static const signed char iv1[10] = {0, 0, 0, 0, 0, 1, 0, 0, 0, 0};
  static const signed char iv2[10] = {0, 0, 0, 0, 0, 0, 1, 0, 0, 0};
  static const signed char iv3[10] = {0, 0, 0, 0, 0, 0, 0, 1, 0, 0};
  static const signed char iv4[10] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 0};
  static const signed char iv5[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 1};
  double a_tmp;
  double a_tmp_tmp;
  double b_a_tmp;
  double b_a_tmp_tmp;
  double b_jacob_w_pred_tmp;
  double c_a_tmp;
  double c_a_tmp_tmp;
  double c_jacob_w_pred_tmp;
  double d_a_tmp;
  double d_a_tmp_tmp;
  double d_jacob_w_pred_tmp;
  double e_jacob_w_pred_tmp;
  double f_jacob_w_pred_tmp;
  double g_jacob_w_pred_tmp;
  double h_jacob_w_pred_tmp;
  double i_jacob_w_pred_tmp;
  double j_jacob_w_pred_tmp;
  double jacob_w_pred_tmp;
  int i;
  a_tmp_tmp = (qw * w1 / 2.0 + qy * w3 / 2.0) - qz * w2 / 2.0;
  a_tmp = qx + dt * a_tmp_tmp;
  b_a_tmp_tmp = (qw * w2 / 2.0 - qx * w3 / 2.0) + qz * w1 / 2.0;
  b_a_tmp = qy + dt * b_a_tmp_tmp;
  c_a_tmp_tmp = (qw * w3 / 2.0 + qx * w2 / 2.0) - qy * w1 / 2.0;
  c_a_tmp = qz + dt * c_a_tmp_tmp;
  d_a_tmp_tmp = (qx * w1 / 2.0 + qy * w2 / 2.0) + qz * w3 / 2.0;
  d_a_tmp = qw - dt * d_a_tmp_tmp;
  jacob_w_pred_tmp = ((a_tmp * a_tmp + b_a_tmp * b_a_tmp) + c_a_tmp * c_a_tmp) +
                     d_a_tmp * d_a_tmp;
  b_jacob_w_pred_tmp = dt * w1;
  c_jacob_w_pred_tmp = sqrt(jacob_w_pred_tmp);
  d_jacob_w_pred_tmp = dt * w2;
  e_jacob_w_pred_tmp = dt * w3;
  jacob_w_pred_tmp = 2.0 * rt_powd_snf(jacob_w_pred_tmp, 1.5);
  f_jacob_w_pred_tmp =
      (((2.0 * qw - 2.0 * dt * d_a_tmp_tmp) + b_jacob_w_pred_tmp * a_tmp) +
       d_jacob_w_pred_tmp * b_a_tmp) +
      e_jacob_w_pred_tmp * c_a_tmp;
  g_jacob_w_pred_tmp = 1.0 / c_jacob_w_pred_tmp;
  jacob_w_pred[0] =
      g_jacob_w_pred_tmp - d_a_tmp * f_jacob_w_pred_tmp / jacob_w_pred_tmp;
  c_jacob_w_pred_tmp *= 2.0;
  h_jacob_w_pred_tmp =
      (((2.0 * qx + 2.0 * dt * a_tmp_tmp) - b_jacob_w_pred_tmp * d_a_tmp) +
       d_jacob_w_pred_tmp * c_a_tmp) -
      e_jacob_w_pred_tmp * b_a_tmp;
  i_jacob_w_pred_tmp = -b_jacob_w_pred_tmp / c_jacob_w_pred_tmp;
  jacob_w_pred[10] =
      i_jacob_w_pred_tmp - d_a_tmp * h_jacob_w_pred_tmp / jacob_w_pred_tmp;
  b_a_tmp_tmp =
      (((2.0 * qy + 2.0 * dt * b_a_tmp_tmp) - b_jacob_w_pred_tmp * c_a_tmp) -
       d_jacob_w_pred_tmp * d_a_tmp) +
      e_jacob_w_pred_tmp * a_tmp;
  j_jacob_w_pred_tmp = -d_jacob_w_pred_tmp / c_jacob_w_pred_tmp;
  jacob_w_pred[20] =
      j_jacob_w_pred_tmp - d_a_tmp * b_a_tmp_tmp / jacob_w_pred_tmp;
  d_a_tmp_tmp =
      (((2.0 * qz + 2.0 * dt * c_a_tmp_tmp) + b_jacob_w_pred_tmp * b_a_tmp) -
       d_jacob_w_pred_tmp * a_tmp) -
      e_jacob_w_pred_tmp * d_a_tmp;
  a_tmp_tmp = -e_jacob_w_pred_tmp / c_jacob_w_pred_tmp;
  jacob_w_pred[30] = a_tmp_tmp - d_a_tmp * d_a_tmp_tmp / jacob_w_pred_tmp;
  jacob_w_pred[40] = 0.0;
  jacob_w_pred[50] = 0.0;
  jacob_w_pred[60] = 0.0;
  jacob_w_pred[70] = 0.0;
  jacob_w_pred[80] = 0.0;
  jacob_w_pred[90] = 0.0;
  b_jacob_w_pred_tmp /= c_jacob_w_pred_tmp;
  jacob_w_pred[1] =
      b_jacob_w_pred_tmp - a_tmp * f_jacob_w_pred_tmp / jacob_w_pred_tmp;
  jacob_w_pred[11] =
      g_jacob_w_pred_tmp - a_tmp * h_jacob_w_pred_tmp / jacob_w_pred_tmp;
  e_jacob_w_pred_tmp /= c_jacob_w_pred_tmp;
  jacob_w_pred[21] =
      e_jacob_w_pred_tmp - a_tmp * b_a_tmp_tmp / jacob_w_pred_tmp;
  jacob_w_pred[31] =
      j_jacob_w_pred_tmp - a_tmp * d_a_tmp_tmp / jacob_w_pred_tmp;
  jacob_w_pred[41] = 0.0;
  jacob_w_pred[51] = 0.0;
  jacob_w_pred[61] = 0.0;
  jacob_w_pred[71] = 0.0;
  jacob_w_pred[81] = 0.0;
  jacob_w_pred[91] = 0.0;
  c_jacob_w_pred_tmp = d_jacob_w_pred_tmp / c_jacob_w_pred_tmp;
  jacob_w_pred[2] =
      c_jacob_w_pred_tmp - b_a_tmp * f_jacob_w_pred_tmp / jacob_w_pred_tmp;
  jacob_w_pred[12] =
      a_tmp_tmp - b_a_tmp * h_jacob_w_pred_tmp / jacob_w_pred_tmp;
  jacob_w_pred[22] =
      g_jacob_w_pred_tmp - b_a_tmp * b_a_tmp_tmp / jacob_w_pred_tmp;
  jacob_w_pred[32] =
      b_jacob_w_pred_tmp - b_a_tmp * d_a_tmp_tmp / jacob_w_pred_tmp;
  jacob_w_pred[42] = 0.0;
  jacob_w_pred[52] = 0.0;
  jacob_w_pred[62] = 0.0;
  jacob_w_pred[72] = 0.0;
  jacob_w_pred[82] = 0.0;
  jacob_w_pred[92] = 0.0;
  jacob_w_pred[3] =
      e_jacob_w_pred_tmp - c_a_tmp * f_jacob_w_pred_tmp / jacob_w_pred_tmp;
  jacob_w_pred[13] =
      c_jacob_w_pred_tmp - c_a_tmp * h_jacob_w_pred_tmp / jacob_w_pred_tmp;
  jacob_w_pred[23] =
      i_jacob_w_pred_tmp - c_a_tmp * b_a_tmp_tmp / jacob_w_pred_tmp;
  jacob_w_pred[33] =
      g_jacob_w_pred_tmp - c_a_tmp * d_a_tmp_tmp / jacob_w_pred_tmp;
  jacob_w_pred[43] = 0.0;
  jacob_w_pred[53] = 0.0;
  jacob_w_pred[63] = 0.0;
  jacob_w_pred[73] = 0.0;
  jacob_w_pred[83] = 0.0;
  jacob_w_pred[93] = 0.0;
  for (i = 0; i < 10; i++) {
    jacob_w_pred[10 * i + 4] = iv[i];
    jacob_w_pred[10 * i + 5] = iv1[i];
    jacob_w_pred[10 * i + 6] = iv2[i];
    jacob_w_pred[10 * i + 7] = iv3[i];
    jacob_w_pred[10 * i + 8] = iv4[i];
    jacob_w_pred[10 * i + 9] = iv5[i];
  }
}

/*
 * File trailer for get_gyro_jacob.c
 *
 * [EOF]
 */
