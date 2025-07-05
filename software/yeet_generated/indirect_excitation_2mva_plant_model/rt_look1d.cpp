//
// File: rt_look1d.cpp
//
// Code generated for Simulink model 'indirect_excitation_2mva_plant_model'.
//
// Model version                  : 10.19
// Simulink Coder version         : 23.2 (R2023b) 01-Aug-2023
// C/C++ source code generated on : Fri Dec 20 00:34:19 2024
//
// Target selection: ert.tlc
// Embedded hardware selection: 32-bit Generic
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
