/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: get_acc_jacob.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 11-May-2024 14:18:01
 */

#ifndef GET_ACC_JACOB_H
#define GET_ACC_JACOB_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void get_acc_jacob(double qw, double qx, double qy, double qz, double a1,
                          double a2, double a3, double dt,
                          double jacob_a_pred[100]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for get_acc_jacob.h
 *
 * [EOF]
 */
