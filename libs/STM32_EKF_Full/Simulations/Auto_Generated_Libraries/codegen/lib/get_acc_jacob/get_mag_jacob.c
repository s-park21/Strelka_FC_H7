/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: get_mag_jacob.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 11-May-2024 14:18:01
 */

/* Include Files */
#include "get_mag_jacob.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : double qw
 *                double qx
 *                double qy
 *                double qz
 *                double jacob_mag[30]
 * Return Type  : void
 */
void get_mag_jacob(double qw, double qx, double qy, double qz,
                   double jacob_mag[30])
{
  double b_jacob_mag_tmp;
  double b_jacob_mag_tmp_tmp;
  double c_jacob_mag_tmp;
  double c_jacob_mag_tmp_tmp;
  double d_jacob_mag_tmp;
  double e_jacob_mag_tmp;
  double f_jacob_mag_tmp;
  double g_jacob_mag_tmp;
  double h_jacob_mag_tmp;
  double jacob_mag_tmp;
  double jacob_mag_tmp_tmp;
  jacob_mag_tmp = 5.90515027096472E+14 * qy / 3.16848112446871E+14;
  b_jacob_mag_tmp = 4.880578184069775E+15 * qz / 3.2952203694474584E+16;
  c_jacob_mag_tmp =
      (5.852057180349399E+15 * qw / 8.238050923618646E+15 + jacob_mag_tmp) +
      b_jacob_mag_tmp;
  jacob_mag[0] = c_jacob_mag_tmp;
  d_jacob_mag_tmp = (5.852057180349399E+15 * qx / 8.238050923618646E+15 +
                     4.880578184069775E+15 * qy / 3.2952203694474584E+16) -
                    5.90515027096472E+14 * qz / 3.16848112446871E+14;
  jacob_mag[3] = d_jacob_mag_tmp;
  e_jacob_mag_tmp = 5.852057180349399E+15 * qy / 8.238050923618646E+15;
  f_jacob_mag_tmp = 4.880578184069775E+15 * qx / 3.2952203694474584E+16;
  g_jacob_mag_tmp = 5.90515027096472E+14 * qw / 3.16848112446871E+14;
  jacob_mag[6] = (g_jacob_mag_tmp + f_jacob_mag_tmp) - e_jacob_mag_tmp;
  jacob_mag_tmp_tmp = 5.90515027096472E+14 * qx / 3.16848112446871E+14;
  b_jacob_mag_tmp_tmp = 4.880578184069775E+15 * qw / 3.2952203694474584E+16;
  c_jacob_mag_tmp_tmp = 5.852057180349399E+15 * qz / 8.238050923618646E+15;
  h_jacob_mag_tmp =
      (b_jacob_mag_tmp_tmp - jacob_mag_tmp_tmp) - c_jacob_mag_tmp_tmp;
  jacob_mag[9] = h_jacob_mag_tmp;
  jacob_mag[12] = 0.0;
  jacob_mag[15] = 0.0;
  jacob_mag[18] = 0.0;
  jacob_mag[21] = 0.0;
  jacob_mag[24] = 0.0;
  jacob_mag[27] = 0.0;
  jacob_mag[1] = h_jacob_mag_tmp;
  e_jacob_mag_tmp = (e_jacob_mag_tmp - f_jacob_mag_tmp) - g_jacob_mag_tmp;
  jacob_mag[4] = e_jacob_mag_tmp;
  jacob_mag[7] = d_jacob_mag_tmp;
  jacob_mag[10] =
      (-(5.852057180349399E+15 * qw) / 8.238050923618646E+15 - jacob_mag_tmp) -
      b_jacob_mag_tmp;
  jacob_mag[13] = 0.0;
  jacob_mag[16] = 0.0;
  jacob_mag[19] = 0.0;
  jacob_mag[22] = 0.0;
  jacob_mag[25] = 0.0;
  jacob_mag[28] = 0.0;
  jacob_mag[2] = e_jacob_mag_tmp;
  jacob_mag[5] =
      (jacob_mag_tmp_tmp - b_jacob_mag_tmp_tmp) + c_jacob_mag_tmp_tmp;
  jacob_mag[8] = c_jacob_mag_tmp;
  jacob_mag[11] = d_jacob_mag_tmp;
  jacob_mag[14] = 0.0;
  jacob_mag[17] = 0.0;
  jacob_mag[20] = 0.0;
  jacob_mag[23] = 0.0;
  jacob_mag[26] = 0.0;
  jacob_mag[29] = 0.0;
}

/*
 * File trailer for get_mag_jacob.c
 *
 * [EOF]
 */
