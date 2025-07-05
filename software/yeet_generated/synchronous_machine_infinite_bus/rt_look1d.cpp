//
// File: rt_look1d.cpp
//
// Code generated for Simulink model 'plant_schematic'.
//
// Model version                  : 6.5
// Simulink Coder version         : 9.9 (R2023a) 19-Nov-2022
// C/C++ source code generated on : Sun Aug 11 15:12:16 2024
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Windows64)
// Code generation objectives: Unspecified
// Validation result: Not run
//

#include "rt_look.h"
#include "rtwtypes.h"
#include "rt_look1d.h"
#ifdef __cplusplus

extern "C"
{

#endif

  // 1D lookup routine for data type of real_T.
  real_T rt_Lookup(const real_T *x, int_T xlen, real_T u, const real_T *y)
  {
    int_T idx{ rt_GetLookupIndex(x, xlen, u) };

    real_T num{ y[idx+1] - y[idx] };

    real_T den{ x[idx+1] - x[idx] };

    // Due to the way the binary search is implemented
    // in rt_look.c (rt_GetLookupIndex), den cannot be
    // 0.  Equivalently, m cannot be inf or nan.
    real_T m{ num/den };

    return (y[idx] + (m * (u - x[idx])));
  }

#ifdef __cplusplus

}                                      // extern "C"

#endif

//
// File trailer for generated code.
//
// [EOF]
//
