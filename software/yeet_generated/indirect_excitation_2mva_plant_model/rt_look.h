//
// File: rt_look.h
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
#ifndef RTW_HEADER_rt_look_h_
#define RTW_HEADER_rt_look_h_
#include "rtwtypes.h"
#ifdef DOINTERPSEARCH
#include <float.h>
#endif

#ifdef __cplusplus

extern "C"
{

#endif

#ifndef INTERP
#define INTERP(x,x1,x2,y1,y2)          ( (y1)+(((y2) - (y1))/((x2) - (x1)))*((x)-(x1)) )
#endif

#ifndef ZEROTECHNIQUE
#define ZEROTECHNIQUE

  enum class ZeroTechnique : int_T {
    NORMAL_INTERP,
    AVERAGE_VALUE,
    MIDDLE_VALUE
  };

#endif

  extern int_T rt_GetLookupIndex(const real_T *x, int_T xlen, real_T u) ;

#ifdef __cplusplus

}                                      // extern "C"

#endif
#endif                                 // RTW_HEADER_rt_look_h_

//
// File trailer for generated code.
//
// [EOF]
//
