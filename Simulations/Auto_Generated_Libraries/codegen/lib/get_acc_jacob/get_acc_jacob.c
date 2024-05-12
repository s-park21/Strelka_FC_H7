/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: get_acc_jacob.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 11-May-2024 14:18:01
 */

/* Include Files */
#include "get_acc_jacob.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : double qw
 *                double qx
 *                double qy
 *                double qz
 *                double a1
 *                double a2
 *                double a3
 *                double dt
 *                double jacob_a_pred[100]
 * Return Type  : void
 */
void get_acc_jacob(double qw, double qx, double qy, double qz, double a1,
                   double a2, double a3, double dt, double jacob_a_pred[100])
{
  static const signed char iv[10] = {1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  static const signed char iv1[10] = {0, 1, 0, 0, 0, 0, 0, 0, 0, 0};
  static const signed char iv2[10] = {0, 0, 1, 0, 0, 0, 0, 0, 0, 0};
  static const signed char iv3[10] = {0, 0, 0, 1, 0, 0, 0, 0, 0, 0};
  int i;
  for (i = 0; i < 10; i++) {
    jacob_a_pred[10 * i] = iv[i];
    jacob_a_pred[10 * i + 1] = iv1[i];
    jacob_a_pred[10 * i + 2] = iv2[i];
    jacob_a_pred[10 * i + 3] = iv3[i];
  }
  double b_jacob_a_pred_tmp;
  double b_jacob_a_pred_tmp_tmp;
  double b_jacob_a_pred_tmp_tmp_tmp;
  double c_jacob_a_pred_tmp;
  double c_jacob_a_pred_tmp_tmp;
  double c_jacob_a_pred_tmp_tmp_tmp;
  double d_jacob_a_pred_tmp;
  double d_jacob_a_pred_tmp_tmp;
  double e_jacob_a_pred_tmp;
  double e_jacob_a_pred_tmp_tmp;
  double f_jacob_a_pred_tmp_tmp;
  double g_jacob_a_pred_tmp_tmp;
  double h_jacob_a_pred_tmp_tmp;
  double i_jacob_a_pred_tmp_tmp;
  double j_jacob_a_pred_tmp_tmp;
  double jacob_a_pred_tmp;
  double jacob_a_pred_tmp_tmp;
  double jacob_a_pred_tmp_tmp_tmp;
  jacob_a_pred_tmp = dt * dt;
  jacob_a_pred_tmp_tmp_tmp = 981.0 * a1 * qw;
  b_jacob_a_pred_tmp_tmp_tmp = 981.0 * a3 * qy;
  c_jacob_a_pred_tmp_tmp_tmp = 981.0 * a2 * qz;
  jacob_a_pred_tmp_tmp =
      jacob_a_pred_tmp *
      ((jacob_a_pred_tmp_tmp_tmp / 100.0 + b_jacob_a_pred_tmp_tmp_tmp / 100.0) -
       c_jacob_a_pred_tmp_tmp_tmp / 100.0);
  b_jacob_a_pred_tmp = jacob_a_pred_tmp_tmp / 2.0;
  jacob_a_pred[4] = b_jacob_a_pred_tmp;
  b_jacob_a_pred_tmp_tmp = 981.0 * a1 * qx;
  c_jacob_a_pred_tmp_tmp = 981.0 * a2 * qy;
  d_jacob_a_pred_tmp_tmp = 981.0 * a3 * qz;
  c_jacob_a_pred_tmp =
      jacob_a_pred_tmp *
      ((b_jacob_a_pred_tmp_tmp / 100.0 + c_jacob_a_pred_tmp_tmp / 100.0) +
       d_jacob_a_pred_tmp_tmp / 100.0) /
      2.0;
  jacob_a_pred[14] = c_jacob_a_pred_tmp;
  e_jacob_a_pred_tmp_tmp = 981.0 * a3 * qw;
  f_jacob_a_pred_tmp_tmp = 981.0 * a2 * qx;
  g_jacob_a_pred_tmp_tmp = 981.0 * a1 * qy;
  d_jacob_a_pred_tmp =
      jacob_a_pred_tmp *
      ((e_jacob_a_pred_tmp_tmp / 100.0 + f_jacob_a_pred_tmp_tmp / 100.0) -
       g_jacob_a_pred_tmp_tmp / 100.0);
  e_jacob_a_pred_tmp = d_jacob_a_pred_tmp / 2.0;
  jacob_a_pred[24] = e_jacob_a_pred_tmp;
  h_jacob_a_pred_tmp_tmp = 981.0 * a2 * qw;
  i_jacob_a_pred_tmp_tmp = 981.0 * a3 * qx;
  j_jacob_a_pred_tmp_tmp = 981.0 * a1 * qz;
  jacob_a_pred_tmp *=
      (h_jacob_a_pred_tmp_tmp / 100.0 - i_jacob_a_pred_tmp_tmp / 100.0) +
      j_jacob_a_pred_tmp_tmp / 100.0;
  jacob_a_pred[34] = -jacob_a_pred_tmp / 2.0;
  jacob_a_pred[44] = 1.0;
  jacob_a_pred[54] = 0.0;
  jacob_a_pred[64] = 0.0;
  jacob_a_pred[74] = dt;
  jacob_a_pred[84] = 0.0;
  jacob_a_pred[94] = 0.0;
  jacob_a_pred_tmp /= 2.0;
  jacob_a_pred[5] = jacob_a_pred_tmp;
  jacob_a_pred[15] = -d_jacob_a_pred_tmp / 2.0;
  jacob_a_pred[25] = c_jacob_a_pred_tmp;
  jacob_a_pred[35] = b_jacob_a_pred_tmp;
  jacob_a_pred[45] = 0.0;
  jacob_a_pred[55] = 1.0;
  jacob_a_pred[65] = 0.0;
  jacob_a_pred[75] = 0.0;
  jacob_a_pred[85] = dt;
  jacob_a_pred[95] = 0.0;
  jacob_a_pred[6] = e_jacob_a_pred_tmp;
  jacob_a_pred[16] = jacob_a_pred_tmp;
  jacob_a_pred[26] = -jacob_a_pred_tmp_tmp / 2.0;
  jacob_a_pred[36] = c_jacob_a_pred_tmp;
  jacob_a_pred[46] = 0.0;
  jacob_a_pred[56] = 0.0;
  jacob_a_pred[66] = 1.0;
  jacob_a_pred[76] = 0.0;
  jacob_a_pred[86] = 0.0;
  jacob_a_pred[96] = dt;
  jacob_a_pred_tmp_tmp =
      (jacob_a_pred_tmp_tmp_tmp / 50.0 + b_jacob_a_pred_tmp_tmp_tmp / 50.0) -
      c_jacob_a_pred_tmp_tmp_tmp / 50.0;
  jacob_a_pred_tmp = dt * jacob_a_pred_tmp_tmp;
  jacob_a_pred[7] = jacob_a_pred_tmp;
  b_jacob_a_pred_tmp =
      dt * ((b_jacob_a_pred_tmp_tmp / 50.0 + c_jacob_a_pred_tmp_tmp / 50.0) +
            d_jacob_a_pred_tmp_tmp / 50.0);
  jacob_a_pred[17] = b_jacob_a_pred_tmp;
  c_jacob_a_pred_tmp =
      (e_jacob_a_pred_tmp_tmp / 50.0 + f_jacob_a_pred_tmp_tmp / 50.0) -
      g_jacob_a_pred_tmp_tmp / 50.0;
  d_jacob_a_pred_tmp = dt * c_jacob_a_pred_tmp;
  jacob_a_pred[27] = d_jacob_a_pred_tmp;
  e_jacob_a_pred_tmp =
      (h_jacob_a_pred_tmp_tmp / 50.0 - i_jacob_a_pred_tmp_tmp / 50.0) +
      j_jacob_a_pred_tmp_tmp / 50.0;
  jacob_a_pred[37] = -dt * e_jacob_a_pred_tmp;
  jacob_a_pred[47] = 0.0;
  jacob_a_pred[57] = 0.0;
  jacob_a_pred[67] = 0.0;
  jacob_a_pred[77] = 1.0;
  jacob_a_pred[87] = 0.0;
  jacob_a_pred[97] = 0.0;
  e_jacob_a_pred_tmp *= dt;
  jacob_a_pred[8] = e_jacob_a_pred_tmp;
  jacob_a_pred[18] = -dt * c_jacob_a_pred_tmp;
  jacob_a_pred[28] = b_jacob_a_pred_tmp;
  jacob_a_pred[38] = jacob_a_pred_tmp;
  jacob_a_pred[48] = 0.0;
  jacob_a_pred[58] = 0.0;
  jacob_a_pred[68] = 0.0;
  jacob_a_pred[78] = 0.0;
  jacob_a_pred[88] = 1.0;
  jacob_a_pred[98] = 0.0;
  jacob_a_pred[9] = d_jacob_a_pred_tmp;
  jacob_a_pred[19] = e_jacob_a_pred_tmp;
  jacob_a_pred[29] = -dt * jacob_a_pred_tmp_tmp;
  jacob_a_pred[39] = b_jacob_a_pred_tmp;
  jacob_a_pred[49] = 0.0;
  jacob_a_pred[59] = 0.0;
  jacob_a_pred[69] = 0.0;
  jacob_a_pred[79] = 0.0;
  jacob_a_pred[89] = 0.0;
  jacob_a_pred[99] = 1.0;
}

/*
 * File trailer for get_acc_jacob.c
 *
 * [EOF]
 */
