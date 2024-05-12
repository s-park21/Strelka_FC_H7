/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: get_gyro_jacob.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 11-May-2024 14:18:01
 */

#ifndef GET_GYRO_JACOB_H
#define GET_GYRO_JACOB_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void get_gyro_jacob(double w1, double w2, double w3, double qw,
                           double qx, double qy, double qz, double dt,
                           double jacob_w_pred[100]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for get_gyro_jacob.h
 *
 * [EOF]
 */
