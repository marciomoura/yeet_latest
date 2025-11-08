/*
 * rt_look1d.cpp
 *
 * Code generation for model "LCITopology6x6Minimal".
 *
 * Model version              : 12.152
 * Simulink Coder version : 9.4 (R2020b) 29-Jul-2020
 * C++ source code generated on : Fri Jun  9 15:14:25 2023
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: 32-bit Generic
 * Code generation objective: Debugging
 * Validation result: Not run
 */

#include "rt_look1d.h"
#ifdef __cplusplus

extern "C" {

#endif

  /* 1D lookup routine for data type of real_T. */
  real_T rt_Lookup(const real_T *x, int_T xlen, real_T u, const real_T *y)
  {
    int_T idx = rt_GetLookupIndex(x, xlen, u);
    real_T num = y[idx+1] - y[idx];
    real_T den = x[idx+1] - x[idx];

    /* Due to the way the binary search is implemented
       in rt_look.c (rt_GetLookupIndex), den cannot be
       0.  Equivalently, m cannot be inf or nan. */
    real_T m = num/den;
    return (y[idx] + (m * (u - x[idx])));
  }

#ifdef __cplusplus

}                                      /* extern "C" */
#endif
