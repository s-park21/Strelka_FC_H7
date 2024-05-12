/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_get_acc_jacob_api.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 11-May-2024 14:18:01
 */

#ifndef _CODER_GET_ACC_JACOB_API_H
#define _CODER_GET_ACC_JACOB_API_H

/* Include Files */
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"
#include <string.h>

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void get_acc_jacob(real_T qw, real_T qx, real_T qy, real_T qz, real_T a1,
                   real_T a2, real_T a3, real_T dt, real_T jacob_a_pred[100]);

void get_acc_jacob_api(const mxArray *const prhs[8], const mxArray **plhs);

void get_acc_jacob_atexit(void);

void get_acc_jacob_initialize(void);

void get_acc_jacob_terminate(void);

void get_acc_jacob_xil_shutdown(void);

void get_acc_jacob_xil_terminate(void);

void get_gyro_jacob(real_T w1, real_T w2, real_T w3, real_T qw, real_T qx,
                    real_T qy, real_T qz, real_T dt, real_T jacob_w_pred[100]);

void get_gyro_jacob_api(const mxArray *const prhs[8], const mxArray **plhs);

void get_mag_jacob(real_T qw, real_T qx, real_T qy, real_T qz,
                   real_T jacob_mag[30]);

void get_mag_jacob_api(const mxArray *const prhs[4], const mxArray **plhs);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_get_acc_jacob_api.h
 *
 * [EOF]
 */
