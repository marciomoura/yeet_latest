//
// File: rt_look.h
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
