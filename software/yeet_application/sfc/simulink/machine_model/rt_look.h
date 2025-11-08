/*
 * rt_look.h
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

#ifndef RTW_HEADER_rt_look_h_
#define RTW_HEADER_rt_look_h_
#include "rtwtypes.h"
#ifdef DOINTERPSEARCH
#include <float.h>
#endif

#ifdef __cplusplus

extern "C" {

#endif

#ifndef INTERP
#define INTERP(x,x1,x2,y1,y2)          ( (y1)+(((y2) - (y1))/((x2) - (x1)))*((x)-(x1)) )
#endif

#ifndef ZEROTECHNIQUE
#define ZEROTECHNIQUE

  typedef enum {
    NORMAL_INTERP,
    AVERAGE_VALUE,
    MIDDLE_VALUE
  } ZeroTechnique;

#endif

  extern int_T rt_GetLookupIndex(const real_T *x, int_T xlen, real_T u) ;

#ifdef __cplusplus

}                                      /* extern "C" */
#endif
#endif                                 /* RTW_HEADER_rt_look_h_ */
