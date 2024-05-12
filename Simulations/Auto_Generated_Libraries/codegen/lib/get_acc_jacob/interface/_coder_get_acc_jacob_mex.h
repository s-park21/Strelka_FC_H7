/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_get_acc_jacob_mex.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 11-May-2024 14:18:01
 */

#ifndef _CODER_GET_ACC_JACOB_MEX_H
#define _CODER_GET_ACC_JACOB_MEX_H

/* Include Files */
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
MEXFUNCTION_LINKAGE void mexFunction(int32_T nlhs, mxArray *plhs[],
                                     int32_T nrhs, const mxArray *prhs[]);

emlrtCTX mexFunctionCreateRootTLS(void);

void unsafe_get_acc_jacob_mexFunction(int32_T nlhs, mxArray *plhs[1],
                                      int32_T nrhs, const mxArray *prhs[8]);

void unsafe_get_gyro_jacob_mexFunction(int32_T nlhs, mxArray *plhs[1],
                                       int32_T nrhs, const mxArray *prhs[8]);

void unsafe_get_mag_jacob_mexFunction(int32_T nlhs, mxArray *plhs[1],
                                      int32_T nrhs, const mxArray *prhs[4]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_get_acc_jacob_mex.h
 *
 * [EOF]
 */
