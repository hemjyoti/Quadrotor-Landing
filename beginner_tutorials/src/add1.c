/*
 * add.c
 *
 * Code generation for function 'add'
 *
 * C source code generated on: Tue Mar 29 13:33:27 2016
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "add1.h"
#include "dynamics_TEST.h"

/* Function Definitions */
int8_T add1(int8_T A, int8_T B)
{
  int32_T i0;
  i0 = A + B;
  if (i0 > 127) {
    i0 = 127;
  } else {
    if (i0 < -128) {
      i0 = -128;
    }
  }

  return (int8_T)i0;
}

/* End of code generation (add.c) */
