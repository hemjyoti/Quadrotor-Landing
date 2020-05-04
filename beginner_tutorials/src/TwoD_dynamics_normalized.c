/*
 * TwoD_dynamics_normalized.c
 *
 * Code generation for function 'TwoD_dynamics_normalized'
 *
 * C source code generated on: Tue Mar 29 13:33:27 2016
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "add1.h"
#include "dynamics_TEST.h"
#include "TwoD_dynamics_normalized.h"

/* Function Definitions */
void TwoD_dynamics_normalized(const real_T y[5], real_T z[5])
{
  /* state */
  /*  control */
  /*  constant parameters */
  z[0] = y[3];
  z[1] = y[2] / y[0];
  z[2] = -(y[2] * y[3] / y[0]);
  z[3] = y[2] * y[2] / y[0] - 4.90278E+12 / (y[0] * y[0]);
  z[4] = 0.0;
}

/* End of code generation (TwoD_dynamics_normalized.c) */
