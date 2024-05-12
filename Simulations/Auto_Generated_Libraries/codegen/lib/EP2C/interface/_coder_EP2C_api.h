/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_EP2C_api.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 11-May-2024 14:40:32
 */

#ifndef _CODER_EP2C_API_H
#define _CODER_EP2C_API_H

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
void EP2C(real_T qw, real_T qx, real_T qy, real_T qz, real_T C[9]);

void EP2C_api(const mxArray *const prhs[4], const mxArray **plhs);

void EP2C_atexit(void);

void EP2C_initialize(void);

void EP2C_terminate(void);

void EP2C_xil_shutdown(void);

void EP2C_xil_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_EP2C_api.h
 *
 * [EOF]
 */
