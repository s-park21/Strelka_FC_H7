/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: main.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 11-May-2024 14:18:01
 */

/*************************************************************************/
/* This automatically generated example C main file shows how to call    */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/

/* Include Files */
#include "main.h"
#include "get_acc_jacob.h"
#include "get_acc_jacob_terminate.h"
#include "get_gyro_jacob.h"
#include "get_mag_jacob.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static double argInit_real_T(void);

/* Function Definitions */
/*
 * Arguments    : void
 * Return Type  : double
 */
static double argInit_real_T(void)
{
  return 0.0;
}

/*
 * Arguments    : int argc
 *                char **argv
 * Return Type  : int
 */
int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;
  /* The initialize function is being called automatically from your entry-point
   * function. So, a call to initialize is not included here. */
  /* Invoke the entry-point functions.
You can call entry-point functions multiple times. */
  main_get_acc_jacob();
  main_get_gyro_jacob();
  main_get_mag_jacob();
  /* Terminate the application.
You do not need to do this more than one time. */
  get_acc_jacob_terminate();
  return 0;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void main_get_acc_jacob(void)
{
  double jacob_a_pred[100];
  double qw_tmp;
  /* Initialize function 'get_acc_jacob' input arguments. */
  qw_tmp = argInit_real_T();
  /* Call the entry-point 'get_acc_jacob'. */
  get_acc_jacob(qw_tmp, qw_tmp, qw_tmp, qw_tmp, qw_tmp, qw_tmp, qw_tmp, qw_tmp,
                jacob_a_pred);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void main_get_gyro_jacob(void)
{
  double jacob_w_pred[100];
  double w1_tmp;
  /* Initialize function 'get_gyro_jacob' input arguments. */
  w1_tmp = argInit_real_T();
  /* Call the entry-point 'get_gyro_jacob'. */
  get_gyro_jacob(w1_tmp, w1_tmp, w1_tmp, w1_tmp, w1_tmp, w1_tmp, w1_tmp, w1_tmp,
                 jacob_w_pred);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void main_get_mag_jacob(void)
{
  double jacob_mag[30];
  double qw_tmp;
  /* Initialize function 'get_mag_jacob' input arguments. */
  qw_tmp = argInit_real_T();
  /* Call the entry-point 'get_mag_jacob'. */
  get_mag_jacob(qw_tmp, qw_tmp, qw_tmp, qw_tmp, jacob_mag);
}

/*
 * File trailer for main.c
 *
 * [EOF]
 */
