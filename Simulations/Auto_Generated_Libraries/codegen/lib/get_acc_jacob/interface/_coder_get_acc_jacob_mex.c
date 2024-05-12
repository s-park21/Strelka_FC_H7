/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_get_acc_jacob_mex.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 11-May-2024 14:18:01
 */

/* Include Files */
#include "_coder_get_acc_jacob_mex.h"
#include "_coder_get_acc_jacob_api.h"

/* Function Definitions */
/*
 * Arguments    : int32_T nlhs
 *                mxArray *plhs[]
 *                int32_T nrhs
 *                const mxArray *prhs[]
 * Return Type  : void
 */
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  static const char_T *emlrtEntryPoints[3] = {"get_acc_jacob", "get_gyro_jacob",
                                              "get_mag_jacob"};
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexAtExit(&get_acc_jacob_atexit);
  /* Module initialization. */
  get_acc_jacob_initialize();
  st.tls = emlrtRootTLSGlobal;
  /* Dispatch the entry-point. */
  switch (emlrtGetEntryPointIndexR2016a(
      &st, nrhs, &prhs[0], (const char_T **)&emlrtEntryPoints[0], 3)) {
  case 0:
    unsafe_get_acc_jacob_mexFunction(nlhs, plhs, nrhs - 1, &prhs[1]);
    break;
  case 1:
    unsafe_get_gyro_jacob_mexFunction(nlhs, plhs, nrhs - 1, &prhs[1]);
    break;
  case 2:
    unsafe_get_mag_jacob_mexFunction(nlhs, plhs, nrhs - 1, &prhs[1]);
    break;
  }
  /* Module termination. */
  get_acc_jacob_terminate();
}

/*
 * Arguments    : void
 * Return Type  : emlrtCTX
 */
emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1,
                           NULL, "windows-1252", true);
  return emlrtRootTLSGlobal;
}

/*
 * Arguments    : int32_T nlhs
 *                mxArray *plhs[1]
 *                int32_T nrhs
 *                const mxArray *prhs[8]
 * Return Type  : void
 */
void unsafe_get_acc_jacob_mexFunction(int32_T nlhs, mxArray *plhs[1],
                                      int32_T nrhs, const mxArray *prhs[8])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const mxArray *b_prhs[8];
  const mxArray *outputs;
  int32_T i;
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs != 8) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 8, 4,
                        13, "get_acc_jacob");
  }
  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 13,
                        "get_acc_jacob");
  }
  /* Call the function. */
  for (i = 0; i < 8; i++) {
    b_prhs[i] = prhs[i];
  }
  get_acc_jacob_api(b_prhs, &outputs);
  /* Copy over outputs to the caller. */
  emlrtReturnArrays(1, &plhs[0], &outputs);
}

/*
 * Arguments    : int32_T nlhs
 *                mxArray *plhs[1]
 *                int32_T nrhs
 *                const mxArray *prhs[8]
 * Return Type  : void
 */
void unsafe_get_gyro_jacob_mexFunction(int32_T nlhs, mxArray *plhs[1],
                                       int32_T nrhs, const mxArray *prhs[8])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const mxArray *b_prhs[8];
  const mxArray *outputs;
  int32_T i;
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs != 8) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 8, 4,
                        14, "get_gyro_jacob");
  }
  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 14,
                        "get_gyro_jacob");
  }
  /* Call the function. */
  for (i = 0; i < 8; i++) {
    b_prhs[i] = prhs[i];
  }
  get_gyro_jacob_api(b_prhs, &outputs);
  /* Copy over outputs to the caller. */
  emlrtReturnArrays(1, &plhs[0], &outputs);
}

/*
 * Arguments    : int32_T nlhs
 *                mxArray *plhs[1]
 *                int32_T nrhs
 *                const mxArray *prhs[4]
 * Return Type  : void
 */
void unsafe_get_mag_jacob_mexFunction(int32_T nlhs, mxArray *plhs[1],
                                      int32_T nrhs, const mxArray *prhs[4])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const mxArray *b_prhs[4];
  const mxArray *outputs;
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs != 4) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 4, 4,
                        13, "get_mag_jacob");
  }
  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 13,
                        "get_mag_jacob");
  }
  /* Call the function. */
  b_prhs[0] = prhs[0];
  b_prhs[1] = prhs[1];
  b_prhs[2] = prhs[2];
  b_prhs[3] = prhs[3];
  get_mag_jacob_api(b_prhs, &outputs);
  /* Copy over outputs to the caller. */
  emlrtReturnArrays(1, &plhs[0], &outputs);
}

/*
 * File trailer for _coder_get_acc_jacob_mex.c
 *
 * [EOF]
 */
