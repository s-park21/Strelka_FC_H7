/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_get_acc_jacob_api.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 11-May-2024 14:18:01
 */

/* Include Files */
#include "_coder_get_acc_jacob_api.h"
#include "_coder_get_acc_jacob_mex.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;

emlrtContext emlrtContextGlobal = {
    true,                                                 /* bFirstTime */
    false,                                                /* bInitialized */
    131643U,                                              /* fVersionInfo */
    NULL,                                                 /* fErrorFunction */
    "get_acc_jacob",                                      /* fFunctionName */
    NULL,                                                 /* fRTCallStack */
    false,                                                /* bDebugMode */
    {2045744189U, 2170104910U, 2743257031U, 4284093946U}, /* fSigWrd */
    NULL                                                  /* fSigMem */
};

/* Function Declarations */
static real_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId);

static const mxArray *b_emlrt_marshallOut(const real_T u[30]);

static real_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId);

static real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                               const char_T *identifier);

static const mxArray *emlrt_marshallOut(const real_T u[100]);

/* Function Definitions */
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T
 */
static real_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = c_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const real_T u[30]
 * Return Type  : const mxArray *
 */
static const mxArray *b_emlrt_marshallOut(const real_T u[30])
{
  static const int32_T iv[2] = {0, 0};
  static const int32_T iv1[2] = {3, 10};
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m, &iv1[0], 2);
  emlrtAssign(&y, m);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T
 */
static real_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims = 0;
  real_T ret;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 0U,
                          (const void *)&dims);
  ret = *(real_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *nullptr
 *                const char_T *identifier
 * Return Type  : real_T
 */
static real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                               const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  real_T y;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId);
  emlrtDestroyArray(&nullptr);
  return y;
}

/*
 * Arguments    : const real_T u[100]
 * Return Type  : const mxArray *
 */
static const mxArray *emlrt_marshallOut(const real_T u[100])
{
  static const int32_T iv[2] = {0, 0};
  static const int32_T iv1[2] = {10, 10};
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m, &iv1[0], 2);
  emlrtAssign(&y, m);
  return y;
}

/*
 * Arguments    : const mxArray * const prhs[8]
 *                const mxArray **plhs
 * Return Type  : void
 */
void get_acc_jacob_api(const mxArray *const prhs[8], const mxArray **plhs)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  real_T(*jacob_a_pred)[100];
  real_T a1;
  real_T a2;
  real_T a3;
  real_T dt;
  real_T qw;
  real_T qx;
  real_T qy;
  real_T qz;
  st.tls = emlrtRootTLSGlobal;
  jacob_a_pred = (real_T(*)[100])mxMalloc(sizeof(real_T[100]));
  /* Marshall function inputs */
  qw = emlrt_marshallIn(&st, emlrtAliasP(prhs[0]), "qw");
  qx = emlrt_marshallIn(&st, emlrtAliasP(prhs[1]), "qx");
  qy = emlrt_marshallIn(&st, emlrtAliasP(prhs[2]), "qy");
  qz = emlrt_marshallIn(&st, emlrtAliasP(prhs[3]), "qz");
  a1 = emlrt_marshallIn(&st, emlrtAliasP(prhs[4]), "a1");
  a2 = emlrt_marshallIn(&st, emlrtAliasP(prhs[5]), "a2");
  a3 = emlrt_marshallIn(&st, emlrtAliasP(prhs[6]), "a3");
  dt = emlrt_marshallIn(&st, emlrtAliasP(prhs[7]), "dt");
  /* Invoke the target function */
  get_acc_jacob(qw, qx, qy, qz, a1, a2, a3, dt, *jacob_a_pred);
  /* Marshall function outputs */
  *plhs = emlrt_marshallOut(*jacob_a_pred);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void get_acc_jacob_atexit(void)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  get_acc_jacob_xil_terminate();
  get_acc_jacob_xil_shutdown();
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void get_acc_jacob_initialize(void)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, NULL);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void get_acc_jacob_terminate(void)
{
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/*
 * Arguments    : const mxArray * const prhs[8]
 *                const mxArray **plhs
 * Return Type  : void
 */
void get_gyro_jacob_api(const mxArray *const prhs[8], const mxArray **plhs)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  real_T(*jacob_w_pred)[100];
  real_T dt;
  real_T qw;
  real_T qx;
  real_T qy;
  real_T qz;
  real_T w1;
  real_T w2;
  real_T w3;
  st.tls = emlrtRootTLSGlobal;
  jacob_w_pred = (real_T(*)[100])mxMalloc(sizeof(real_T[100]));
  /* Marshall function inputs */
  w1 = emlrt_marshallIn(&st, emlrtAliasP(prhs[0]), "w1");
  w2 = emlrt_marshallIn(&st, emlrtAliasP(prhs[1]), "w2");
  w3 = emlrt_marshallIn(&st, emlrtAliasP(prhs[2]), "w3");
  qw = emlrt_marshallIn(&st, emlrtAliasP(prhs[3]), "qw");
  qx = emlrt_marshallIn(&st, emlrtAliasP(prhs[4]), "qx");
  qy = emlrt_marshallIn(&st, emlrtAliasP(prhs[5]), "qy");
  qz = emlrt_marshallIn(&st, emlrtAliasP(prhs[6]), "qz");
  dt = emlrt_marshallIn(&st, emlrtAliasP(prhs[7]), "dt");
  /* Invoke the target function */
  get_gyro_jacob(w1, w2, w3, qw, qx, qy, qz, dt, *jacob_w_pred);
  /* Marshall function outputs */
  *plhs = emlrt_marshallOut(*jacob_w_pred);
}

/*
 * Arguments    : const mxArray * const prhs[4]
 *                const mxArray **plhs
 * Return Type  : void
 */
void get_mag_jacob_api(const mxArray *const prhs[4], const mxArray **plhs)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  real_T(*jacob_mag)[30];
  real_T qw;
  real_T qx;
  real_T qy;
  real_T qz;
  st.tls = emlrtRootTLSGlobal;
  jacob_mag = (real_T(*)[30])mxMalloc(sizeof(real_T[30]));
  /* Marshall function inputs */
  qw = emlrt_marshallIn(&st, emlrtAliasP(prhs[0]), "qw");
  qx = emlrt_marshallIn(&st, emlrtAliasP(prhs[1]), "qx");
  qy = emlrt_marshallIn(&st, emlrtAliasP(prhs[2]), "qy");
  qz = emlrt_marshallIn(&st, emlrtAliasP(prhs[3]), "qz");
  /* Invoke the target function */
  get_mag_jacob(qw, qx, qy, qz, *jacob_mag);
  /* Marshall function outputs */
  *plhs = b_emlrt_marshallOut(*jacob_mag);
}

/*
 * File trailer for _coder_get_acc_jacob_api.c
 *
 * [EOF]
 */
