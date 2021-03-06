/*
 * dynamics_TEST.c
 *
 * Code generation for function 'dynamics_TEST'
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
void dynamics_TEST(const real_T x0[5], real_T X_end[5])
{
  real_T X_state[2150];
  int32_T i1;
  int32_T k;
  real_T K1[5];
  real_T b_X_state[5];
  real_T K2[5];
  real_T K3[5];
  real_T K4[5];

  /*  nonoptimal final time; */
  memset(&X_state[0], 0, 2150U * sizeof(real_T));
  for (i1 = 0; i1 < 5; i1++) {
    X_state[i1] = x0[i1];
  }

  for (k = 0; k < 430; k++) {
    TwoD_dynamics_normalized(*(real_T (*)[5])&X_state[5 * k], K1);
    for (i1 = 0; i1 < 5; i1++) {
      b_X_state[i1] = X_state[i1 + 5 * k] + K1[i1] / 2.0;
    }

    TwoD_dynamics_normalized(b_X_state, K2);
    for (i1 = 0; i1 < 5; i1++) {
      b_X_state[i1] = X_state[i1 + 5 * k] + 0.5 * K2[i1];
    }

    TwoD_dynamics_normalized(b_X_state, K3);
    for (i1 = 0; i1 < 5; i1++) {
      b_X_state[i1] = X_state[i1 + 5 * k] + K3[i1];
    }

    TwoD_dynamics_normalized(b_X_state, K4);
    for (i1 = 0; i1 < 5; i1++) {
      X_state[i1 + 5 * (k + 1)] = X_state[i1 + 5 * k] + (((K1[i1] / 6.0 + K2[i1]
        / 3.0) + K3[i1] / 3.0) + K4[i1] / 6.0);
    }
  }

  for (i1 = 0; i1 < 5; i1++) {
    X_end[i1] = X_state[2145 + i1];
  }

  /* %%%%%%%%%%%%%%%%%%%%%%%% */
}

/* End of code generation (dynamics_TEST.c) */
