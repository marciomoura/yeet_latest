/*
 * LCITopology6x6Minimal.h
 *
 * Code generation for model "LCITopology6x6Minimal".
 *
 * Model version              : 12.196
 * Simulink Coder version : 9.4 (R2020b) 29-Jul-2020
 * C++ source code generated on : Mon Jul 28 10:36:17 2025
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: 32-bit Generic
 * Code generation objective: Execution efficiency
 * Validation result: Not run
 */

#ifndef RTW_HEADER_LCITopology6x6Minimal_h_
#define RTW_HEADER_LCITopology6x6Minimal_h_
#include <cfloat>
#include <cmath>
#include <cstring>
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "LCITopology6x6Minimal_types.h"

/* Shared type includes */
#include "multiword_types.h"
#include "rtGetInf.h"
#include "rt_nonfinite.h"
#include "rt_defines.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetContStateDisabled
#define rtmGetContStateDisabled(rtm)   ((rtm)->contStateDisabled)
#endif

#ifndef rtmSetContStateDisabled
#define rtmSetContStateDisabled(rtm, val) ((rtm)->contStateDisabled = (val))
#endif

#ifndef rtmGetContStates
#define rtmGetContStates(rtm)          ((rtm)->contStates)
#endif

#ifndef rtmSetContStates
#define rtmSetContStates(rtm, val)     ((rtm)->contStates = (val))
#endif

#ifndef rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag
#define rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm) ((rtm)->CTOutputIncnstWithState)
#endif

#ifndef rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag
#define rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm, val) ((rtm)->CTOutputIncnstWithState = (val))
#endif

#ifndef rtmGetDerivCacheNeedsReset
#define rtmGetDerivCacheNeedsReset(rtm) ((rtm)->derivCacheNeedsReset)
#endif

#ifndef rtmSetDerivCacheNeedsReset
#define rtmSetDerivCacheNeedsReset(rtm, val) ((rtm)->derivCacheNeedsReset = (val))
#endif

#ifndef rtmGetIntgData
#define rtmGetIntgData(rtm)            ((rtm)->intgData)
#endif

#ifndef rtmSetIntgData
#define rtmSetIntgData(rtm, val)       ((rtm)->intgData = (val))
#endif

#ifndef rtmGetOdeF
#define rtmGetOdeF(rtm)                ((rtm)->odeF)
#endif

#ifndef rtmSetOdeF
#define rtmSetOdeF(rtm, val)           ((rtm)->odeF = (val))
#endif

#ifndef rtmGetPeriodicContStateIndices
#define rtmGetPeriodicContStateIndices(rtm) ((rtm)->periodicContStateIndices)
#endif

#ifndef rtmSetPeriodicContStateIndices
#define rtmSetPeriodicContStateIndices(rtm, val) ((rtm)->periodicContStateIndices = (val))
#endif

#ifndef rtmGetPeriodicContStateRanges
#define rtmGetPeriodicContStateRanges(rtm) ((rtm)->periodicContStateRanges)
#endif

#ifndef rtmSetPeriodicContStateRanges
#define rtmSetPeriodicContStateRanges(rtm, val) ((rtm)->periodicContStateRanges = (val))
#endif

#ifndef rtmGetZCCacheNeedsReset
#define rtmGetZCCacheNeedsReset(rtm)   ((rtm)->zCCacheNeedsReset)
#endif

#ifndef rtmSetZCCacheNeedsReset
#define rtmSetZCCacheNeedsReset(rtm, val) ((rtm)->zCCacheNeedsReset = (val))
#endif

#ifndef rtmGetdX
#define rtmGetdX(rtm)                  ((rtm)->derivs)
#endif

#ifndef rtmSetdX
#define rtmSetdX(rtm, val)             ((rtm)->derivs = (val))
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

/* Block signals for system '<S54>/Subsystem - pi//2 delay' */
typedef struct {
  real_T Fcn;                          /* '<S58>/Fcn' */
  real_T Fcn1;                         /* '<S58>/Fcn1' */
} B_Subsystempi2delay_LCITopolo_T;

/* Block states (default storage) for system '<S54>/Subsystem - pi//2 delay' */
typedef struct {
  boolean_T Subsystempi2delay_MODE;    /* '<S54>/Subsystem - pi//2 delay' */
} DW_Subsystempi2delay_LCITopol_T;

/* Block signals for system '<S54>/Subsystem1' */
typedef struct {
  real_T Fcn;                          /* '<S59>/Fcn' */
  real_T Fcn1;                         /* '<S59>/Fcn1' */
} B_Subsystem1_LCITopology6x6Mi_T;

/* Block states (default storage) for system '<S54>/Subsystem1' */
typedef struct {
  boolean_T Subsystem1_MODE;           /* '<S54>/Subsystem1' */
} DW_Subsystem1_LCITopology6x6M_T;

/* Block signals (default storage) */
typedef struct {
  creal_T RealImagtoComplex;           /* '<S48>/Real-Imag to Complex' */
  real_T xk1[5];                       /* '<S86>/fluxes' */
  real_T Linv[25];                     /* '<S75>/Switch' */
  real_T Ik[5];                        /* '<S75>/Product3' */
  real_T changeIqIdcurrentsigns[5];    /* '<S75>/change Iq Id  current signs' */
  real_T d_thetarad;                   /* '<S73>/Rotor angle dthetae' */
  real_T DigitalClock;                 /* '<S73>/Digital Clock' */
  real_T web2;                         /* '<S73>/web2' */
  real_T Sum3;                         /* '<S73>/Sum3' */
  real_T Switch[2];                    /* '<S104>/Switch' */
  real_T TmpSignalConversionAtGain3Inpor[3];
  real_T Gain3[3];                     /* '<S103>/Gain3' */
  real_T iabc[3];                      /* '<S77>/ib' */
  real_T Constant4;                    /* '<S2>/Constant4' */
  real_T Integrator;                   /* '<S36>/Integrator' */
  real_T Saturation;                   /* '<S36>/Saturation' */
  real_T Sum;                          /* '<S31>/Sum' */
  real_T Mod;                          /* '<S31>/Mod' */
  real_T Gain2[3];                     /* '<S2>/Gain2' */
  real_T Add1[3];                      /* '<S2>/Add1' */
  real_T TrigonometricFunction1[3];    /* '<S2>/Trigonometric Function1' */
  real_T Gain1[3];                     /* '<S2>/Gain1' */
  real_T Product5;                     /* '<S2>/Product5' */
  real_T Product6;                     /* '<S2>/Product6' */
  real_T Product7;                     /* '<S2>/Product7' */
  real_T StateSpace_o1[37];            /* '<S164>/State-Space' */
  real_T StateSpace_o2[14];            /* '<S164>/State-Space' */
  real_T donotdeletethisgain;          /* '<S27>/do not delete this gain' */
  real_T donotdeletethisgain_o;        /* '<S3>/do not delete this gain' */
  real_T donotdeletethisgain_c;        /* '<S133>/do not delete this gain' */
  real_T donotdeletethisgain_p;        /* '<S134>/do not delete this gain' */
  real_T donotdeletethisgain_cx;       /* '<S135>/do not delete this gain' */
  real_T Kv[3];                        /* '<S22>/Kv' */
  real_T donotdeletethisgain_f;        /* '<S136>/do not delete this gain' */
  real_T donotdeletethisgain_k;        /* '<S137>/do not delete this gain' */
  real_T donotdeletethisgain_n;        /* '<S138>/do not delete this gain' */
  real_T Kv1[3];                       /* '<S22>/Kv1' */
  real_T Subtract;                     /* '<S42>/Subtract' */
  real_T Initial;                      /* '<S42>/Initial' */
  real_T Integrator_d;                 /* '<S42>/Integrator' */
  real_T Memory;                       /* '<S42>/Memory' */
  real_T donotdeletethisgain_c2;       /* '<S151>/do not delete this gain' */
  real_T donotdeletethisgain_g;        /* '<S152>/do not delete this gain' */
  real_T donotdeletethisgain_f1;       /* '<S153>/do not delete this gain' */
  real_T Kv1_j[3];                     /* '<S23>/Kv1' */
  real_T Integrator_o;                 /* '<S44>/Integrator' */
  real_T VariableTransportDelay;       /* '<S64>/Variable Transport Delay' */
  real_T integrator;                   /* '<S64>/integrator' */
  real_T Clock;                        /* '<S64>/Clock' */
  real_T Memory_l;                     /* '<S64>/Memory' */
  real_T Switch_g;                     /* '<S64>/Switch' */
  real_T Divide_g;                     /* '<S42>/Divide' */
  real_T Kp4;                          /* '<S44>/Kp4' */
  real_T Kp5;                          /* '<S44>/Kp5' */
  real_T Kp6;                          /* '<S44>/Kp6' */
  real_T TransferFcn;                  /* '<S44>/Transfer Fcn' */
  real_T Sum6;                         /* '<S44>/Sum6' */
  real_T Saturation2;                  /* '<S44>/Saturation2' */
  real_T Gain10;                       /* '<S42>/Gain10' */
  real_T RateLimiter;                  /* '<S42>/Rate Limiter' */
  real_T x1;                           /* '<S60>/Integrator_x1' */
  real_T A11;                          /* '<S61>/A11' */
  real_T x2;                           /* '<S60>/Integrator_x2' */
  real_T A12;                          /* '<S61>/A12' */
  real_T A21;                          /* '<S61>/A21' */
  real_T A22;                          /* '<S61>/A22' */
  real_T sum2;                         /* '<S61>/sum2' */
  real_T sum3;                         /* '<S61>/sum3' */
  real_T B11;                          /* '<S62>/B11' */
  real_T x1_i;                         /* '<S60>/A*x1+ B* u1' */
  real_T B21;                          /* '<S62>/B21' */
  real_T x2_m;                         /* '<S60>/A*x2+ B*u2' */
  real_T C11;                          /* '<S63>/C11' */
  real_T C12;                          /* '<S63>/C12' */
  real_T sum2_a;                       /* '<S63>/sum2' */
  real_T Du;                           /* '<S60>/D*u' */
  real_T y;                            /* '<S60>/C*x + D*u' */
  real_T Freq;                         /* '<S64>/To avoid division  by zero' */
  real_T period;                       /* '<S64>/period' */
  real_T Gain3_o[3];                   /* '<S66>/Gain3' */
  real_T Gain1_f[3];                   /* '<S66>/Gain1' */
  real_T Switch_a[2];                  /* '<S65>/Switch' */
  real_T donotdeletethisgain_h;        /* '<S148>/do not delete this gain' */
  real_T donotdeletethisgain_d;        /* '<S149>/do not delete this gain' */
  real_T donotdeletethisgain_kk;       /* '<S150>/do not delete this gain' */
  real_T Kv_n[3];                      /* '<S23>/Kv' */
  real_T donotdeletethisgain_m;        /* '<S118>/do not delete this gain' */
  real_T donotdeletethisgain_b;        /* '<S119>/do not delete this gain' */
  real_T donotdeletethisgain_n2;       /* '<S120>/do not delete this gain' */
  real_T Kv_n3[3];                     /* '<S21>/Kv' */
  real_T donotdeletethisgain_de;       /* '<S121>/do not delete this gain' */
  real_T donotdeletethisgain_kv;       /* '<S122>/do not delete this gain' */
  real_T donotdeletethisgain_g0;       /* '<S123>/do not delete this gain' */
  real_T Kv1_l[3];                     /* '<S21>/Kv1' */
  real_T dw_delay;                     /* '<S110>/dw_delay' */
  real_T F2;                           /* '<S110>/F2' */
  real_T dw_predict;                   /* '<S110>/dw_predict' */
  real_T Sum1_h;                       /* '<S110>/Sum1' */
  real_T n;                            /* '<S73>/Sum' */
  real_T Switch_e;                     /* '<S73>/Switch' */
  real_T iqsids[2];                    /* '<S82>/1-1' */
  real_T Mult1[2];                     /* '<S82>/Mult1' */
  real_T Sum2;                         /* '<S82>/Sum2' */
  real_T theta;                        /* '<S73>/theta' */
  real_T Fcn;                          /* '<S73>/Fcn' */
  real_T Sign;                         /* '<S7>/Sign' */
  real_T Abs;                          /* '<S7>/Abs' */
  real_T Product;                      /* '<S7>/Product' */
  real_T Sum_c;                        /* '<S7>/Sum' */
  real_T Product1;                     /* '<S7>/Product1' */
  real_T Sum1_l;                       /* '<S7>/Sum1' */
  real_T Pe;                           /* '<S73>/Product' */
  real_T ikq2;                         /* '<S74>/Switch' */
  real_T uLl_q[2];                     /* '<S84>/1//Ll_q' */
  real_T Switch3[2];                   /* '<S75>/Switch3' */
  real_T Product_k[2];                 /* '<S84>/Product' */
  real_T phimq;                        /* '<S84>/sum2' */
  real_T uLl_d[3];                     /* '<S84>/1//Ll_d' */
  real_T Product1_h[3];                /* '<S84>/Product1' */
  real_T phimd;                        /* '<S84>/sum1' */
  real_T u_Vb[2];                      /* '<S76>/1_Vb' */
  real_T Sum_h;                        /* '<S76>/Sum' */
  real_T B;                            /* '<S76>/Gain' */
  real_T A;                            /* '<S76>/Sum1' */
  real_T C;                            /* '<S76>/Sum2' */
  real_T TmpSignalConversionAtGain3Inp_a[3];
  real_T Gain3_d[3];                   /* '<S97>/Gain3' */
  real_T Gain1_p[3];                   /* '<S97>/Gain1' */
  real_T Switch_i[2];                  /* '<S96>/Switch' */
  real_T Switch2[2];                   /* '<S75>/Switch2' */
  real_T xtheta;                       /* '<S80>/x->theta' */
  real_T Gain;                         /* '<S81>/Gain' */
  real_T Product2_e;                   /* '<S79>/Product2' */
  real_T Product3;                     /* '<S79>/Product3' */
  real_T Peo;                          /* '<S79>/Sum' */
  real_T Gain_d;                       /* '<S79>/Gain' */
  real_T Product4;                     /* '<S79>/Product4' */
  real_T Product5_d;                   /* '<S79>/Product5' */
  real_T Qeo;                          /* '<S79>/Sum1' */
  real_T Gain1_j;                      /* '<S79>/Gain1' */
  real_T outputformatting[18];         /* '<S74>/output formatting' */
  real_T Vfd;                          /* '<S71>/N' */
  real_T voltages[5];                  /* '<S86>/voltages' */
  real_T DigitalClock_g;               /* '<S86>/Digital Clock' */
  real_T IC[5];                        /* '<S86>/IC' */
  real_T u_Pb;                         /* '<S73>/1_Pb' */
  real_T div;                          /* '<S73>/div' */
  real_T F;                            /* '<S73>/F' */
  real_T u_Pb1;                        /* '<S73>/1_Pb1' */
  real_T Sum2_o;                       /* '<S73>/Sum2' */
  real_T u2H;                          /* '<S73>/1 ----- 2H' */
  real_T Sum1_p;                       /* '<S73>/Sum1' */
  real_T dw;                          /* '<S109>/Rotor spee<d deviation (dw)' */
  real_T webase;                       /* '<S73>/we base' */
  real_T web3;                         /* '<S73>/web3' */
  real_T donotdeletethisgain_mj;       /* '<S26>/do not delete this gain' */
  real_T Product4_m;                   /* '<S2>/Product4' */
  real_T DataTypeConversion;           /* '<S38>/Data Type Conversion' */
  real_T DataTypeConversion_m;         /* '<S40>/Data Type Conversion' */
  real_T Fcn_m;                        /* '<S108>/Fcn' */
  real_T Fcn1;                         /* '<S108>/Fcn1' */
  real_T Fcn_p;                        /* '<S107>/Fcn' */
  real_T Fcn1_h;                       /* '<S107>/Fcn1' */
  real_T Fcn_b;                        /* '<S101>/Fcn' */
  real_T Fcn1_j;                       /* '<S101>/Fcn1' */
  real_T Fcn_bn;                       /* '<S100>/Fcn' */
  real_T Fcn1_p;                       /* '<S100>/Fcn1' */
  real_T ukuk1[5];                     /* '<S86>/sum' */
  real_T Switch1[25];                  /* '<S75>/Switch1' */
  real_T W12wr[25];                    /* '<S83>/W(1,2)=wr' */
  real_T Gain1_e;                      /* '<S83>/Gain1' */
  real_T W21wr[25];                    /* '<S83>/W(2,1)=-wr' */
  real_T A_m[25];                      /* '<S75>/Sum1' */
  real_T wbaseTs2[25];                 /* '<S94>/wbase*Ts//2' */
  real_T Sum1_b[25];                   /* '<S94>/Sum1' */
  real_T inversion[25];                /* '<S94>/inversion' */
  real_T wbaseTs2_c[25];               /* '<S94>/wbase*Ts//2 ' */
  real_T Product1_b[5];                /* '<S86>/Product1' */
  real_T Sum5[25];                     /* '<S94>/Sum5' */
  real_T Product4_e[25];               /* '<S94>/Product4' */
  real_T Product2_a[5];                /* '<S86>/Product2' */
  real_T xk[5];                    /* '<S86>/Ad*x(k-1) + Bd*( u(k-1) + u(k))' */
  real_T uLl_d_h[3];                   /* '<S91>/1//Ll_d' */
  real_T phimd_g;                      /* '<S91>/sum2' */
  real_T Lmd_sat;                      /* '<S87>/Lmd_sat' */
  real_T MathFunction2;                /* '<S90>/Math Function2' */
  real_T Sum1_bq;                      /* '<S90>/Sum1' */
  real_T MathFunction3;                /* '<S90>/Math Function3' */
  real_T phimd_n;                      /* '<S91>/Product4' */
  real_T Phisat;                       /* '<S87>/Abs' */
  real_T Ifdsat;                       /* '<S87>/1-D Lookup Table' */
  real_T Switch_d;                     /* '<S87>/Switch' */
  real_T Lmsatd;                       /* '<S87>/Lmd' */
  real_T Switch1_j;                    /* '<S85>/Switch1' */
  real_T UpdateLmqrows156col156[25];
                          /* '<S89>/Update Lmq rows[1,5, (6)] col[1,5, (6)] ' */
  real_T UpdateLmdrows234col234[25];
                                 /* '<S89>/Update Lmd rows[2,3,4] col[2,3,4]' */
  real_T Sum2_h[25];                   /* '<S89>/Sum2' */
  real_T Linv_d[25];                   /* '<S85>/inversion' */
  real_T RLinv[25];                    /* '<S85>/Product1' */
  real_T Switch2_a;                    /* '<S85>/Switch2' */
  real_T uLl_q_i[2];                   /* '<S93>/1//Ll_q' */
  real_T phimq_p;                      /* '<S93>/sum1' */
  real_T Lmq_sat;                      /* '<S88>/Lmq_sat' */
  real_T MathFunction;                 /* '<S92>/Math Function' */
  real_T Sum2_d;                       /* '<S92>/Sum2' */
  real_T MathFunction1;                /* '<S92>/Math Function1' */
  real_T phimq_i;                      /* '<S93>/Product3' */
  real_T Phisat_a;                     /* '<S88>/Abs' */
  real_T Ifqsat;                       /* '<S88>/1-D Lookup Table' */
  real_T Switch_b;                     /* '<S88>/Switch' */
  real_T Lmsatq;                       /* '<S88>/Lmq' */
  real_T Product_i;                    /* '<S88>/Product' */
  real_T Product_n;                    /* '<S87>/Product' */
  real_T Sum7;                         /* '<S64>/Sum7' */
  real_T Meanvalue;                    /* '<S64>/Product' */
  real_T VariableTransportDelay_p;     /* '<S52>/Variable Transport Delay' */
  real_T integrator_d;                 /* '<S52>/integrator' */
  real_T Clock_c;                      /* '<S52>/Clock' */
  real_T Memory_f;                     /* '<S52>/Memory' */
  real_T Switch_n;                     /* '<S52>/Switch' */
  real_T VariableTransportDelay_e;     /* '<S53>/Variable Transport Delay' */
  real_T integrator_de;                /* '<S53>/integrator' */
  real_T Clock_b;                      /* '<S53>/Clock' */
  real_T Memory_c;                     /* '<S53>/Memory' */
  real_T Switch_m;                     /* '<S53>/Switch' */
  real_T ComplextoMagnitudeAngle_o1;   /* '<S48>/Complex to Magnitude-Angle' */
  real_T ComplextoMagnitudeAngle_o2;   /* '<S48>/Complex to Magnitude-Angle' */
  real_T Saturation_a;                 /* '<S43>/Saturation' */
  real_T MathFunction_c;               /* '<S43>/Math Function' */
  real_T Freq_o;                       /* '<S52>/To avoid division  by zero' */
  real_T period_i;                     /* '<S52>/period' */
  real_T Freq_l;                       /* '<S53>/To avoid division  by zero' */
  real_T period_a;                     /* '<S53>/period' */
  real_T Gain3_o4[3];                  /* '<S55>/Gain3' */
  real_T Gain1_c[3];                   /* '<S55>/Gain1' */
  real_T Switch_dq[2];                 /* '<S54>/Switch' */
  real_T Sum7_i;                       /* '<S53>/Sum7' */
  real_T Meanvalue_j;                  /* '<S53>/Product' */
  real_T Sum7_h;                       /* '<S52>/Sum7' */
  real_T Meanvalue_o;                  /* '<S52>/Product' */
  uint8_T Compare;                     /* '<S105>/Compare' */
  uint8_T Compare_n;                   /* '<S106>/Compare' */
  uint8_T Compare_e;                   /* '<S67>/Compare' */
  uint8_T Compare_j;                   /* '<S68>/Compare' */
  uint8_T Compare_l;                   /* '<S98>/Compare' */
  uint8_T Compare_ld;                  /* '<S99>/Compare' */
  uint8_T Compare_j0;                  /* '<S56>/Compare' */
  uint8_T Compare_p;                   /* '<S57>/Compare' */
  boolean_T RelationalOperator;        /* '<S42>/Relational Operator' */
  boolean_T RelationalOperator_g;      /* '<S64>/Relational Operator' */
  boolean_T LogicalOperator;           /* '<Root>/Logical Operator' */
  boolean_T RelationalOperator_h;      /* '<S52>/Relational Operator' */
  boolean_T RelationalOperator_g1;     /* '<S53>/Relational Operator' */
  B_Subsystem1_LCITopology6x6Mi_T Subsystem1;/* '<S65>/Subsystem1' */
  B_Subsystempi2delay_LCITopolo_T Subsystempi2delay;/* '<S65>/Subsystem - pi//2 delay' */
  B_Subsystem1_LCITopology6x6Mi_T Subsystem1_ka;/* '<S54>/Subsystem1' */
  B_Subsystempi2delay_LCITopolo_T Subsystempi2delay_k;/* '<S54>/Subsystem - pi//2 delay' */
} B_LCITopology6x6Minimal_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T fluxes_DSTATE[5];             /* '<S86>/fluxes' */
  real_T Rotorangledthetae_DSTATE;     /* '<S73>/Rotor angle dthetae' */
  real_T Integrator_DSTATE;            /* '<S36>/Integrator' */
  real_T StateSpace_DSTATE[21];        /* '<S164>/State-Space' */
  real_T dw_delay_DSTATE;              /* '<S110>/dw_delay' */
  real_T dw_predict_DSTATE;            /* '<S110>/dw_predict' */
  real_T theta_DSTATE;                 /* '<S73>/theta' */
  real_T voltages_DSTATE[5];           /* '<S86>/voltages' */
  real_T Rotorspeeddeviationdw_DSTATE;/* '<S109>/Rotor spee<d deviation (dw)' */
  real_T Lmd_sat_DSTATE;               /* '<S87>/Lmd_sat' */
  real_T Lmq_sat_DSTATE;               /* '<S88>/Lmq_sat' */
  real_T Initial_FirstOutputTime;      /* '<S42>/Initial' */
  real_T Memory_PreviousInput;         /* '<S42>/Memory' */
  real_T Memory_PreviousInput_p;       /* '<S64>/Memory' */
  real_T PrevY;                        /* '<S42>/Rate Limiter' */
  real_T LastMajorTime;                /* '<S42>/Rate Limiter' */
  real_T inversion_DWORK1[25];         /* '<S94>/inversion' */
  real_T inversion_DWORK3[25];         /* '<S94>/inversion' */
  real_T inversion_DWORK4[25];         /* '<S94>/inversion' */
  real_T inversion_DWORK4_b[25];       /* '<S85>/inversion' */
  real_T Memory_PreviousInput_d;       /* '<S52>/Memory' */
  real_T Memory_PreviousInput_h;       /* '<S53>/Memory' */
  struct {
    real_T modelTStart;
    real_T TUbufferArea[24576];
  } VariableTransportDelay_RWORK;      /* '<S64>/Variable Transport Delay' */

  struct {
    real_T modelTStart;
    real_T TUbufferArea[24576];
  } VariableTransportDelay_RWORK_b;    /* '<S52>/Variable Transport Delay' */

  struct {
    real_T modelTStart;
    real_T TUbufferArea[24576];
  } VariableTransportDelay_RWORK_p;    /* '<S53>/Variable Transport Delay' */

  struct {
    void *AS;
    void *BS;
    void *CS;
    void *DS;
    void *DX_COL;
    void *BD_COL;
    void *TMP1;
    void *TMP2;
    void *XTMP;
    void *SWITCH_STATUS;
    void *SWITCH_STATUS_INIT;
    void *SW_CHG;
    void *G_STATE;
    void *USWLAST;
    void *XKM12;
    void *XKP12;
    void *XLAST;
    void *ULAST;
    void *IDX_SW_CHG;
    void *Y_SWITCH;
    void *SWITCH_TYPES;
    void *IDX_OUT_SW;
    void *SWITCH_TOPO_SAVED_IDX;
    void *SWITCH_MAP;
  } StateSpace_PWORK;                  /* '<S164>/State-Space' */

  struct {
    void *TUbufferPtrs[3];
  } VariableTransportDelay_PWORK;      /* '<S64>/Variable Transport Delay' */

  struct {
    void *TUbufferPtrs[3];
  } VariableTransportDelay_PWORK_g;    /* '<S52>/Variable Transport Delay' */

  struct {
    void *TUbufferPtrs[3];
  } VariableTransportDelay_PWORK_j;    /* '<S53>/Variable Transport Delay' */

  int32_T inversion_DWORK2[5];         /* '<S94>/inversion' */
  uint32_T m_bpIndex;                  /* '<S87>/1-D Lookup Table' */
  uint32_T m_bpIndex_i;                /* '<S88>/1-D Lookup Table' */
  int_T StateSpace_IWORK[11];          /* '<S164>/State-Space' */
  int_T Integrator_IWORK;              /* '<S42>/Integrator' */
  struct {
    int_T Tail;
    int_T Head;
    int_T Last;
    int_T CircularBufSize;
  } VariableTransportDelay_IWORK;      /* '<S64>/Variable Transport Delay' */

  struct {
    int_T Tail;
    int_T Head;
    int_T Last;
    int_T CircularBufSize;
  } VariableTransportDelay_IWORK_g;    /* '<S52>/Variable Transport Delay' */

  struct {
    int_T Tail;
    int_T Head;
    int_T Last;
    int_T CircularBufSize;
  } VariableTransportDelay_IWORK_a;    /* '<S53>/Variable Transport Delay' */

  int8_T Integrator_PrevResetState;    /* '<S36>/Integrator' */
  int8_T Rotorspeeddeviationdw_PrevReset;
                                      /* '<S109>/Rotor spee<d deviation (dw)' */
  uint8_T Rotorspeeddeviationdw_IC_LOADIN;
                                      /* '<S109>/Rotor spee<d deviation (dw)' */
  boolean_T AutomaticGainControl_MODE; /* '<S42>/Automatic Gain Control' */
  DW_Subsystem1_LCITopology6x6M_T Subsystem1;/* '<S65>/Subsystem1' */
  DW_Subsystempi2delay_LCITopol_T Subsystempi2delay;/* '<S65>/Subsystem - pi//2 delay' */
  DW_Subsystem1_LCITopology6x6M_T Subsystem1_ka;/* '<S54>/Subsystem1' */
  DW_Subsystempi2delay_LCITopol_T Subsystempi2delay_k;/* '<S54>/Subsystem - pi//2 delay' */
} DW_LCITopology6x6Minimal_T;

/* Continuous states (default storage) */
typedef struct {
  real_T Integrator_CSTATE;            /* '<S42>/Integrator' */
  real_T Integrator_CSTATE_i;          /* '<S44>/Integrator' */
  real_T VariableTransportDelay_CSTATE;/* '<S64>/Variable Transport Delay' */
  real_T integrator_CSTATE;            /* '<S64>/integrator' */
  real_T TransferFcn_CSTATE;           /* '<S44>/Transfer Fcn' */
  real_T Integrator_x1_CSTATE;         /* '<S60>/Integrator_x1' */
  real_T Integrator_x2_CSTATE;         /* '<S60>/Integrator_x2' */
  real_T VariableTransportDelay_CSTATE_c;/* '<S52>/Variable Transport Delay' */
  real_T integrator_CSTATE_g;          /* '<S52>/integrator' */
  real_T VariableTransportDelay_CSTATE_l;/* '<S53>/Variable Transport Delay' */
  real_T integrator_CSTATE_b;          /* '<S53>/integrator' */
} X_LCITopology6x6Minimal_T;

/* State derivatives (default storage) */
typedef struct {
  real_T Integrator_CSTATE;            /* '<S42>/Integrator' */
  real_T Integrator_CSTATE_i;          /* '<S44>/Integrator' */
  real_T VariableTransportDelay_CSTATE;/* '<S64>/Variable Transport Delay' */
  real_T integrator_CSTATE;            /* '<S64>/integrator' */
  real_T TransferFcn_CSTATE;           /* '<S44>/Transfer Fcn' */
  real_T Integrator_x1_CSTATE;         /* '<S60>/Integrator_x1' */
  real_T Integrator_x2_CSTATE;         /* '<S60>/Integrator_x2' */
  real_T VariableTransportDelay_CSTATE_c;/* '<S52>/Variable Transport Delay' */
  real_T integrator_CSTATE_g;          /* '<S52>/integrator' */
  real_T VariableTransportDelay_CSTATE_l;/* '<S53>/Variable Transport Delay' */
  real_T integrator_CSTATE_b;          /* '<S53>/integrator' */
} XDot_LCITopology6x6Minimal_T;

/* State disabled  */
typedef struct {
  boolean_T Integrator_CSTATE;         /* '<S42>/Integrator' */
  boolean_T Integrator_CSTATE_i;       /* '<S44>/Integrator' */
  boolean_T VariableTransportDelay_CSTATE;/* '<S64>/Variable Transport Delay' */
  boolean_T integrator_CSTATE;         /* '<S64>/integrator' */
  boolean_T TransferFcn_CSTATE;        /* '<S44>/Transfer Fcn' */
  boolean_T Integrator_x1_CSTATE;      /* '<S60>/Integrator_x1' */
  boolean_T Integrator_x2_CSTATE;      /* '<S60>/Integrator_x2' */
  boolean_T VariableTransportDelay_CSTATE_c;/* '<S52>/Variable Transport Delay' */
  boolean_T integrator_CSTATE_g;       /* '<S52>/integrator' */
  boolean_T VariableTransportDelay_CSTATE_l;/* '<S53>/Variable Transport Delay' */
  boolean_T integrator_CSTATE_b;       /* '<S53>/integrator' */
} XDis_LCITopology6x6Minimal_T;

/* Zero-crossing (trigger) state */
typedef struct {
  ZCSigState Integrator_Reset_ZCE;     /* '<S42>/Integrator' */
} PrevZCX_LCITopology6x6Minimal_T;

#ifndef ODE1_INTG
#define ODE1_INTG

/* ODE1 Integration Data */
typedef struct {
  real_T *f[1];                        /* derivatives */
} ODE1_IntgData;

#endif

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T pulses_in_grid[6];            /* '<Root>/pulses_in_grid' */
  real_T pulses_in_motor[6];           /* '<Root>/pulses_in_motor' */
  real_T voltage_magnitude_a;          /* '<Root>/voltage_magnitude_a' */
  real_T voltage_magnitude_b;          /* '<Root>/voltage_magnitude_b' */
  real_T voltage_magnitude_c;          /* '<Root>/voltage_magnitude_c' */
  real_T voltage_frequency;            /* '<Root>/voltage_frequency' */
  real_T voltage_excitation;           /* '<Root>/voltage_excitation' */
  real_T load_coefficients[3];         /* '<Root>/load_coefficients' */
  real_T grid_pt_ratio;                /* '<Root>/grid_pt_ratio' */
  real_T grid_ct_ratio;                /* '<Root>/grid_ct_ratio' */
  real_T forced_speed_pu;              /* '<Root>/forced_speed_pu' */
  real_T enable_forced_speed;          /* '<Root>/enable_forced_speed' */
  boolean_T close_dc_isolator;         /* '<Root>/close_dc_isolator' */
  real_T motor_pt_ratio;               /* '<Root>/motor_pt_ratio' */
  real_T motor_ct_ratio;               /* '<Root>/motor_ct_ratio' */
} ExtU_LCITopology6x6Minimal_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T Udc;                          /* '<Root>/Udc' */
  real_T Idc;                          /* '<Root>/Idc' */
  real_T Iph_grid[3];                  /* '<Root>/Iph_grid' */
  real_T Uph_ph_grid[3];               /* '<Root>/Uph_ph_grid' */
  real_T wt;                           /* '<Root>/wt' */
  real_T Uph_n_source[3];              /* '<Root>/Uph_n_source' */
  real_T Iph_source[3];                /* '<Root>/Iph_source' */
  real_T Iph_motor[3];                 /* '<Root>/Iph_motor' */
  real_T Uph_ph_motor[3];              /* '<Root>/Uph_ph_motor' */
  real_T machine_speed;                /* '<Root>/machine_speed' */
  real_T machine_electrical_torque;    /* '<Root>/machine_electrical_torque' */
  real_T machine_mechanical_angle;     /* '<Root>/machine_mechanical_angle' */
  real_T machine_load;                 /* '<Root>/machine_load' */
  real_T machine_electrical_power;     /* '<Root>/machine_electrical_power' */
  real_T Udc_machine;                  /* '<Root>/Udc_machine' */
  real_T machine_flux_d;               /* '<Root>/machine_flux_d' */
  real_T machine_flux_q;               /* '<Root>/machine_flux_q' */
  real_T machine_stator_current_d;     /* '<Root>/machine_stator_current_d' */
  real_T machine_stator_current_q;     /* '<Root>/machine_stator_current_q' */
  real_T machine_field_current_d;      /* '<Root>/machine_field_current_d' */
  real_T machine_damper_current_q1;    /* '<Root>/machine_damper_current_q1' */
  real_T machine_damper_current_q2;    /* '<Root>/machine_damper_current_q2' */
  real_T machine_damper_current_d;     /* '<Root>/machine_damper_current_d' */
  real_T machine_stator_voltage_q;     /* '<Root>/machine_stator_voltage_q' */
  real_T machine_stator_voltage_d;     /* '<Root>/machine_stator_voltage_d' */
  real_T machine_phase_current_a;      /* '<Root>/machine_phase_current_a' */
  real_T machine_phase_current_b;      /* '<Root>/machine_phase_current_b' */
  real_T machine_phase_current_c;      /* '<Root>/machine_phase_current_c' */
  real_T source_voltage_phase;         /* '<Root>/source_voltage_phase' */
} ExtY_LCITopology6x6Minimal_T;

/* Parameters for system: '<S54>/Subsystem - pi//2 delay' */
struct P_Subsystempi2delay_LCITopolo_T_ {
  real_T dq_Y0[2];                     /* Expression: [0,0]
                                        * Referenced by: '<S58>/dq'
                                        */
};

/* Parameters for system: '<S54>/Subsystem1' */
struct P_Subsystem1_LCITopology6x6Mi_T_ {
  real_T dq_Y0[2];                     /* Expression: [0,0]
                                        * Referenced by: '<S59>/dq'
                                        */
};

/* Parameters (default storage) */
struct P_LCITopology6x6Minimal_T_ {
  real_T AlphaBetaZerotodq0_Alignment;
                                 /* Mask Parameter: AlphaBetaZerotodq0_Alignment
                                  * Referenced by: '<S54>/Constant'
                                  */
  real_T dq0toAlphaBetaZero_Alignment;
                                 /* Mask Parameter: dq0toAlphaBetaZero_Alignment
                                  * Referenced by: '<S104>/Constant'
                                  */
  real_T AlphaBetaZerotodq0_Alignment_n;
                               /* Mask Parameter: AlphaBetaZerotodq0_Alignment_n
                                * Referenced by: '<S65>/Constant'
                                */
  real_T AlphaBetaZerotodq0_Alignment_b;
                               /* Mask Parameter: AlphaBetaZerotodq0_Alignment_b
                                * Referenced by: '<S96>/Constant'
                                */
  real_T Continuous_Init;              /* Mask Parameter: Continuous_Init
                                        * Referenced by: '<S44>/Integrator'
                                        */
  real_T Continuous_Kd;                /* Mask Parameter: Continuous_Kd
                                        * Referenced by: '<S44>/Kp6'
                                        */
  real_T Continuous_Ki;                /* Mask Parameter: Continuous_Ki
                                        * Referenced by: '<S44>/Kp5'
                                        */
  real_T Continuous_Kp;                /* Mask Parameter: Continuous_Kp
                                        * Referenced by: '<S44>/Kp4'
                                        */
  real_T CompareToConstant_const;     /* Mask Parameter: CompareToConstant_const
                                       * Referenced by: '<S56>/Constant'
                                       */
  real_T CompareToConstant1_const;   /* Mask Parameter: CompareToConstant1_const
                                      * Referenced by: '<S57>/Constant'
                                      */
  real_T CompareToConstant_const_b; /* Mask Parameter: CompareToConstant_const_b
                                     * Referenced by: '<S105>/Constant'
                                     */
  real_T CompareToConstant1_const_d;
                                   /* Mask Parameter: CompareToConstant1_const_d
                                    * Referenced by: '<S106>/Constant'
                                    */
  real_T CompareToConstant_const_c; /* Mask Parameter: CompareToConstant_const_c
                                     * Referenced by: '<S67>/Constant'
                                     */
  real_T CompareToConstant1_const_g;
                                   /* Mask Parameter: CompareToConstant1_const_g
                                    * Referenced by: '<S68>/Constant'
                                    */
  real_T CompareToConstant_const_d; /* Mask Parameter: CompareToConstant_const_d
                                     * Referenced by: '<S98>/Constant'
                                     */
  real_T CompareToConstant1_const_o;
                                   /* Mask Parameter: CompareToConstant1_const_o
                                    * Referenced by: '<S99>/Constant'
                                    */
  real_T Gain_Y0;                      /* Expression: [1]
                                        * Referenced by: '<S43>/Gain'
                                        */
  real_T VariableTransportDelay_MaxDelay;/* Expression: 1/sps.Fmin+eps
                                          * Referenced by: '<S52>/Variable Transport Delay'
                                          */
  real_T VariableTransportDelay_InitOutp;/* Expression: 0
                                          * Referenced by: '<S52>/Variable Transport Delay'
                                          */
  real_T integrator_IC;                /* Expression: 0
                                        * Referenced by: '<S52>/integrator'
                                        */
  real_T Constant_Value;               /* Expression: 1/sps.Finit
                                        * Referenced by: '<S52>/Constant'
                                        */
  real_T Memory_InitialCondition;      /* Expression: sps.Vinit
                                        * Referenced by: '<S52>/Memory'
                                        */
  real_T VariableTransportDelay_MaxDel_i;/* Expression: 1/sps.Fmin+eps
                                          * Referenced by: '<S53>/Variable Transport Delay'
                                          */
  real_T VariableTransportDelay_InitOu_g;/* Expression: 0
                                          * Referenced by: '<S53>/Variable Transport Delay'
                                          */
  real_T integrator_IC_m;              /* Expression: 0
                                        * Referenced by: '<S53>/integrator'
                                        */
  real_T Constant_Value_p;             /* Expression: 1/sps.Finit
                                        * Referenced by: '<S53>/Constant'
                                        */
  real_T Memory_InitialCondition_i;    /* Expression: sps.Vinit
                                        * Referenced by: '<S53>/Memory'
                                        */
  real_T Saturation_UpperSat;          /* Expression: inf
                                        * Referenced by: '<S43>/Saturation'
                                        */
  real_T Saturation_LowerSat;          /* Expression: eps
                                        * Referenced by: '<S43>/Saturation'
                                        */
  real_T Toavoiddivisionbyzero_UpperSat;/* Expression: 1e6
                                         * Referenced by: '<S52>/To avoid division  by zero'
                                         */
  real_T Toavoiddivisionbyzero_LowerSat;/* Expression: eps
                                         * Referenced by: '<S52>/To avoid division  by zero'
                                         */
  real_T Toavoiddivisionbyzero_UpperSa_h;/* Expression: 1e6
                                          * Referenced by: '<S53>/To avoid division  by zero'
                                          */
  real_T Toavoiddivisionbyzero_LowerSa_l;/* Expression: eps
                                          * Referenced by: '<S53>/To avoid division  by zero'
                                          */
  real_T Gain3_Gain[9];
  /* Expression: [ 1   -1/2   -1/2; 0   sqrt(3)/2   -sqrt(3)/2; 1/2  1/2  1/2 ]
   * Referenced by: '<S55>/Gain3'
   */
  real_T Gain1_Gain;                   /* Expression: 2/3
                                        * Referenced by: '<S55>/Gain1'
                                        */
  real_T Constant2_Value;              /* Expression: 0
                                        * Referenced by: '<S74>/Constant2'
                                        */
  real_T Constant1_Value;              /* Expression: 1
                                        * Referenced by: '<S87>/Constant1'
                                        */
  real_T Constant1_Value_i;            /* Expression: 1
                                        * Referenced by: '<S88>/Constant1'
                                        */
  real_T Lmq_sat_Y0;                   /* Computed Parameter: Lmq_sat_Y0
                                        * Referenced by: '<S88>/Lmq_sat '
                                        */
  real_T Laq_Y0;                       /* Computed Parameter: Laq_Y0
                                        * Referenced by: '<S88>/Laq_'
                                        */
  real_T uLl_q_Gain[2];                /* Expression: SM.One_Llq
                                        * Referenced by: '<S93>/1//Ll_q'
                                        */
  real_T u2_Value[3];            /* Expression: [ 1/SM.Ll 1/SM.Llkq1 1/SM.Llkq2]
                                  * Referenced by: '<S92>/u2'
                                  */
  real_T Lmq_sat_InitialCondition;     /* Expression: SM.Lmsatq(1)
                                        * Referenced by: '<S88>/Lmq_sat'
                                        */
  real_T uDLookupTable_tableData[2];
                  /* Expression: [ 0 SM.Phisat(2:end)./SM.Lmsatq(2:end)*SM.Lmq ]
                   * Referenced by: '<S88>/1-D Lookup Table'
                   */
  real_T uDLookupTable_bp01Data[2];    /* Expression: SM.Phisat
                                        * Referenced by: '<S88>/1-D Lookup Table'
                                        */
  real_T Lmq_Gain;                     /* Expression: SM.Lmq
                                        * Referenced by: '<S88>/Lmq'
                                        */
  real_T u3_Value;                     /* Expression: SM.Lmq
                                        * Referenced by: '<S85>/u3'
                                        */
  real_T u2_Value_n;                   /* Expression: SM.Laq
                                        * Referenced by: '<S85>/u2'
                                        */
  real_T Linv_Y0;                      /* Computed Parameter: Linv_Y0
                                        * Referenced by: '<S85>/Linv'
                                        */
  real_T RLinv_Y0;                     /* Computed Parameter: RLinv_Y0
                                        * Referenced by: '<S85>/R*Linv'
                                        */
  real_T Lmqd_Y0;                      /* Computed Parameter: Lmqd_Y0
                                        * Referenced by: '<S85>/Lmqd'
                                        */
  real_T Laqd_Y0;                      /* Computed Parameter: Laqd_Y0
                                        * Referenced by: '<S85>/Laqd'
                                        */
  real_T uLl_d_Gain[3];
  /* Expression: [ 1/SM.Ll   SM.Llkd/(SM.Llfd*SM.Llkd+(SM.Llfd+SM.Llkd)*SM.LC)   SM.Llfd/(SM.Llfd*SM.Llkd+(SM.Llfd+SM.Llkd)*SM.LC)]
   * Referenced by: '<S91>/1//Ll_d'
   */
  real_T u1_Value[2];
            /* Expression: [1/SM.Ll 1/(SM.Llkd*SM.Llfd/(SM.Llkd+SM.Llfd)+SM.LC)]
             * Referenced by: '<S90>/u1'
             */
  real_T Lmd_sat_InitialCondition;     /* Expression: SM.Lmsatd(1)
                                        * Referenced by: '<S87>/Lmd_sat'
                                        */
  real_T uDLookupTable_tableData_a[2];
                  /* Expression: [ 0 SM.Phisat(2:end)./SM.Lmsatd(2:end)*SM.Lmd ]
                   * Referenced by: '<S87>/1-D Lookup Table'
                   */
  real_T uDLookupTable_bp01Data_i[2];  /* Expression: SM.Phisat
                                        * Referenced by: '<S87>/1-D Lookup Table'
                                        */
  real_T Lmd_Gain;                     /* Expression: SM.Lmd
                                        * Referenced by: '<S87>/Lmd'
                                        */
  real_T u1_Value_a[25];               /* Expression: SM.R
                                        * Referenced by: '<S85>/u1'
                                        */
  real_T u1_Value_l[25];               /* Expression: zeros(SM.nState,SM.nState)
                                        * Referenced by: '<S89>/u1'
                                        */
  real_T u5_Value[25];                 /* Expression: SM.Llqd
                                        * Referenced by: '<S89>/u5'
                                        */
  real_T Lmqd_nosat_Value[2];          /* Expression: [SM.Lmq SM.Lmd]
                                        * Referenced by: '<S75>/Lmqd_nosat'
                                        */
  real_T Laqd_nosat_Value[2];          /* Expression: [SM.Laq  SM.Lad]
                                        * Referenced by: '<S75>/Laqd_nosat'
                                        */
  real_T Constant4_Value[25];          /* Expression: SM.Linv
                                        * Referenced by: '<S75>/Constant4'
                                        */
  real_T Constant6_Value[25];          /* Expression: SM.RLinv
                                        * Referenced by: '<S75>/Constant6'
                                        */
  real_T Constant2_Value_i;            /* Expression: SM.Sat
                                        * Referenced by: '<S75>/Constant2'
                                        */
  real_T Switch1_Threshold;            /* Expression: 0.5
                                        * Referenced by: '<S75>/Switch1'
                                        */
  real_T u1_Value_p[25];               /* Expression: zeros(SM.nState,SM.nState)
                                        * Referenced by: '<S83>/u1'
                                        */
  real_T Gain1_Gain_g;                 /* Expression: -1
                                        * Referenced by: '<S83>/Gain1'
                                        */
  real_T wbaseTs2_Gain;                /* Expression: SM.web*Ts/2
                                        * Referenced by: '<S94>/wbase*Ts//2'
                                        */
  real_T u5_Value_o[25];               /* Expression: eye(SM.nState,SM.nState)
                                        * Referenced by: '<S94>/u5'
                                        */
  real_T wbaseTs2_Gain_g;              /* Expression: SM.web*Ts/2
                                        * Referenced by: '<S94>/wbase*Ts//2 '
                                        */
  real_T dq_Y0[2];                     /* Expression: [0,0]
                                        * Referenced by: '<S100>/dq'
                                        */
  real_T dq_Y0_m[2];                   /* Expression: [0,0]
                                        * Referenced by: '<S101>/dq'
                                        */
  real_T alpha_beta_Y0[2];             /* Expression: [0,0]
                                        * Referenced by: '<S107>/alpha_beta'
                                        */
  real_T alpha_beta_Y0_l[2];           /* Expression: [0,0]
                                        * Referenced by: '<S108>/alpha_beta'
                                        */
  real_T SwitchCurrents_Value[14];     /* Expression: zeros(14,1)
                                        * Referenced by: '<S166>/SwitchCurrents'
                                        */
  real_T fluxes_InitialCondition[5];   /* Expression: SM.phiqd0
                                        * Referenced by: '<S86>/fluxes'
                                        */
  real_T Constant1_Value_l;            /* Expression: SM.Sat
                                        * Referenced by: '<S75>/Constant1'
                                        */
  real_T Constant3_Value;              /* Expression: SM.Sat
                                        * Referenced by: '<S75>/Constant3'
                                        */
  real_T Switch_Threshold;             /* Expression: 0.5
                                        * Referenced by: '<S75>/Switch'
                                        */
  real_T changeIqIdcurrentsigns_Gain[5];/* Expression: SM.IqdSign
                                         * Referenced by: '<S75>/change Iq Id  current signs'
                                         */
  real_T Rotorangledthetae_gainval;
                                /* Computed Parameter: Rotorangledthetae_gainval
                                 * Referenced by: '<S73>/Rotor angle dthetae'
                                 */
  real_T Rotorangledthetae_IC;         /* Expression: SM.tho
                                        * Referenced by: '<S73>/Rotor angle dthetae'
                                        */
  real_T web2_Gain;                    /* Expression: SM.web
                                        * Referenced by: '<S73>/web2'
                                        */
  real_T Constant_Value_k;             /* Expression: 0
                                        * Referenced by: '<S77>/Constant'
                                        */
  real_T Gain3_Gain_l[9];
          /* Expression: [ 1   0   1; -1/2  sqrt(3)/2   1; -1/2  -sqrt(3)/2  1 ]
           * Referenced by: '<S103>/Gain3'
           */
  real_T ib_Gain;                      /* Expression: SM.ib
                                        * Referenced by: '<S77>/ib'
                                        */
  real_T Constant4_Value_d;            /* Expression: 0
                                        * Referenced by: '<S2>/Constant4'
                                        */
  real_T Integrator_gainval;           /* Computed Parameter: Integrator_gainval
                                        * Referenced by: '<S36>/Integrator'
                                        */
  real_T Integrator_UpperSat;          /* Expression: antiwindupUpperLimit
                                        * Referenced by: '<S36>/Integrator'
                                        */
  real_T Integrator_LowerSat;          /* Expression: antiwindupLowerLimit
                                        * Referenced by: '<S36>/Integrator'
                                        */
  real_T Saturation_UpperSat_e;        /* Expression: windupUpperLimit
                                        * Referenced by: '<S36>/Saturation'
                                        */
  real_T Saturation_LowerSat_f;        /* Expression: windupLowerLimit
                                        * Referenced by: '<S36>/Saturation'
                                        */
  real_T Constant5_Value;              /* Expression: pi
                                        * Referenced by: '<S31>/Constant5'
                                        */
  real_T Constant6_Value_k;            /* Expression: 2*pi
                                        * Referenced by: '<S31>/Constant6'
                                        */
  real_T Constant1_Value_e;            /* Expression: sim_source_phase_a_deg
                                        * Referenced by: '<Root>/Constant1'
                                        */
  real_T Constant2_Value_e;            /* Expression: sim_source_phase_b_deg
                                        * Referenced by: '<Root>/Constant2'
                                        */
  real_T Constant3_Value_k;            /* Expression: sim_source_phase_c_deg
                                        * Referenced by: '<Root>/Constant3'
                                        */
  real_T Gain2_Gain;                   /* Expression: pi/180
                                        * Referenced by: '<S2>/Gain2'
                                        */
  real_T Gain1_Gain_j;                 /* Expression: sqrt(2)
                                        * Referenced by: '<S2>/Gain1'
                                        */
  real_T StateSpace_AS_param[441];     /* Expression: S.A
                                        * Referenced by: '<S164>/State-Space'
                                        */
  real_T StateSpace_BS_param[399];     /* Expression: S.B
                                        * Referenced by: '<S164>/State-Space'
                                        */
  real_T StateSpace_CS_param[777];     /* Expression: S.C
                                        * Referenced by: '<S164>/State-Space'
                                        */
  real_T StateSpace_DS_param[703];     /* Expression: S.D
                                        * Referenced by: '<S164>/State-Space'
                                        */
  real_T StateSpace_X0_param[21];      /* Expression: S.x0
                                        * Referenced by: '<S164>/State-Space'
                                        */
  real_T donotdeletethisgain_Gain;     /* Expression: 1
                                        * Referenced by: '<S27>/do not delete this gain'
                                        */
  real_T donotdeletethisgain_Gain_d;   /* Expression: 1
                                        * Referenced by: '<S3>/do not delete this gain'
                                        */
  real_T donotdeletethisgain_Gain_i;   /* Expression: 1
                                        * Referenced by: '<S133>/do not delete this gain'
                                        */
  real_T donotdeletethisgain_Gain_o;   /* Expression: 1
                                        * Referenced by: '<S134>/do not delete this gain'
                                        */
  real_T donotdeletethisgain_Gain_m;   /* Expression: 1
                                        * Referenced by: '<S135>/do not delete this gain'
                                        */
  real_T Kv_Gain;                      /* Expression: Ki
                                        * Referenced by: '<S22>/Kv'
                                        */
  real_T donotdeletethisgain_Gain_k;   /* Expression: 1
                                        * Referenced by: '<S136>/do not delete this gain'
                                        */
  real_T donotdeletethisgain_Gain_h;   /* Expression: 1
                                        * Referenced by: '<S137>/do not delete this gain'
                                        */
  real_T donotdeletethisgain_Gain_n;   /* Expression: 1
                                        * Referenced by: '<S138>/do not delete this gain'
                                        */
  real_T Kv1_Gain;                     /* Expression: Kv
                                        * Referenced by: '<S22>/Kv1'
                                        */
  real_T Constant2_Value_ez;           /* Expression: 2*pi
                                        * Referenced by: '<S42>/Constant2'
                                        */
  real_T Initial_Value;                /* Expression: sps.Phase_Init*pi/180
                                        * Referenced by: '<S42>/Initial'
                                        */
  real_T Integrator_UpperSat_l;        /* Expression: inf
                                        * Referenced by: '<S42>/Integrator'
                                        */
  real_T Integrator_LowerSat_n;        /* Expression: -inf
                                        * Referenced by: '<S42>/Integrator'
                                        */
  real_T HitCrossing_Offset;           /* Expression: pi
                                        * Referenced by: '<S42>/Hit  Crossing'
                                        */
  real_T Memory_InitialCondition_a;    /* Expression: sps.Finit
                                        * Referenced by: '<S42>/Memory'
                                        */
  real_T donotdeletethisgain_Gain_m0;  /* Expression: 1
                                        * Referenced by: '<S151>/do not delete this gain'
                                        */
  real_T donotdeletethisgain_Gain_ip;  /* Expression: 1
                                        * Referenced by: '<S152>/do not delete this gain'
                                        */
  real_T donotdeletethisgain_Gain_c;   /* Expression: 1
                                        * Referenced by: '<S153>/do not delete this gain'
                                        */
  real_T Kv1_Gain_a;                   /* Expression: Kv
                                        * Referenced by: '<S23>/Kv1'
                                        */
  real_T Constant1_Value_a;            /* Expression: sps.AGC
                                        * Referenced by: '<S42>/Constant1'
                                        */
  real_T Integrator_UpperSat_j;        /* Expression: Par_Limits(1)
                                        * Referenced by: '<S44>/Integrator'
                                        */
  real_T Integrator_LowerSat_c;        /* Expression: Par_Limits(2)
                                        * Referenced by: '<S44>/Integrator'
                                        */
  real_T VariableTransportDelay_MaxDel_l;/* Expression: 1/sps.Fmin+eps
                                          * Referenced by: '<S64>/Variable Transport Delay'
                                          */
  real_T VariableTransportDelay_InitOu_b;/* Expression: 0
                                          * Referenced by: '<S64>/Variable Transport Delay'
                                          */
  real_T integrator_IC_n;              /* Expression: 0
                                        * Referenced by: '<S64>/integrator'
                                        */
  real_T Constant_Value_f;             /* Expression: 1/sps.Finit
                                        * Referenced by: '<S64>/Constant'
                                        */
  real_T Memory_InitialCondition_b;    /* Expression: sps.Vinit
                                        * Referenced by: '<S64>/Memory'
                                        */
  real_T TransferFcn_A;                /* Computed Parameter: TransferFcn_A
                                        * Referenced by: '<S44>/Transfer Fcn'
                                        */
  real_T TransferFcn_C;                /* Computed Parameter: TransferFcn_C
                                        * Referenced by: '<S44>/Transfer Fcn'
                                        */
  real_T TransferFcn_D;                /* Computed Parameter: TransferFcn_D
                                        * Referenced by: '<S44>/Transfer Fcn'
                                        */
  real_T Saturation2_UpperSat;         /* Expression: Par_Limits(1)
                                        * Referenced by: '<S44>/Saturation2'
                                        */
  real_T Saturation2_LowerSat;         /* Expression: Par_Limits(2)
                                        * Referenced by: '<S44>/Saturation2'
                                        */
  real_T Gain10_Gain;                  /* Expression: 1/2/pi
                                        * Referenced by: '<S42>/Gain10'
                                        */
  real_T RateLimiter_RisingLim;        /* Expression: sps.MaxRateChangeFreq
                                        * Referenced by: '<S42>/Rate Limiter'
                                        */
  real_T RateLimiter_FallingLim;       /* Expression: -sps.MaxRateChangeFreq
                                        * Referenced by: '<S42>/Rate Limiter'
                                        */
  real_T Integrator_x1_IC;             /* Expression: sps.x0(1,:)
                                        * Referenced by: '<S60>/Integrator_x1'
                                        */
  real_T A11_Gain;                     /* Expression: sps.A11
                                        * Referenced by: '<S61>/A11'
                                        */
  real_T Integrator_x2_IC;             /* Expression: sps.x0(2,:)
                                        * Referenced by: '<S60>/Integrator_x2'
                                        */
  real_T A12_Gain;                     /* Expression: sps.A12
                                        * Referenced by: '<S61>/A12'
                                        */
  real_T A21_Gain;                     /* Expression: sps.A21
                                        * Referenced by: '<S61>/A21'
                                        */
  real_T A22_Gain;                     /* Expression: sps.A22
                                        * Referenced by: '<S61>/A22'
                                        */
  real_T B11_Gain;                     /* Expression: sps.B11
                                        * Referenced by: '<S62>/B11'
                                        */
  real_T B21_Gain;                     /* Expression: sps.B21
                                        * Referenced by: '<S62>/B21'
                                        */
  real_T C11_Gain;                     /* Expression: sps.C11
                                        * Referenced by: '<S63>/C11'
                                        */
  real_T C12_Gain;                     /* Expression: sps.C12
                                        * Referenced by: '<S63>/C12'
                                        */
  real_T Du_Gain;                      /* Expression: sps.D
                                        * Referenced by: '<S60>/D*u'
                                        */
  real_T Toavoiddivisionbyzero_UpperSa_o;/* Expression: 1e6
                                          * Referenced by: '<S64>/To avoid division  by zero'
                                          */
  real_T Toavoiddivisionbyzero_LowerSa_n;/* Expression: eps
                                          * Referenced by: '<S64>/To avoid division  by zero'
                                          */
  real_T Gain3_Gain_j[9];
  /* Expression: [ 1   -1/2   -1/2; 0   sqrt(3)/2   -sqrt(3)/2; 1/2  1/2  1/2 ]
   * Referenced by: '<S66>/Gain3'
   */
  real_T Gain1_Gain_o;                 /* Expression: 2/3
                                        * Referenced by: '<S66>/Gain1'
                                        */
  real_T donotdeletethisgain_Gain_b;   /* Expression: 1
                                        * Referenced by: '<S148>/do not delete this gain'
                                        */
  real_T donotdeletethisgain_Gain_km;  /* Expression: 1
                                        * Referenced by: '<S149>/do not delete this gain'
                                        */
  real_T donotdeletethisgain_Gain_p;   /* Expression: 1
                                        * Referenced by: '<S150>/do not delete this gain'
                                        */
  real_T Kv_Gain_e;                    /* Expression: Ki
                                        * Referenced by: '<S23>/Kv'
                                        */
  real_T donotdeletethisgain_Gain_f;   /* Expression: 1
                                        * Referenced by: '<S118>/do not delete this gain'
                                        */
  real_T donotdeletethisgain_Gain_hv;  /* Expression: 1
                                        * Referenced by: '<S119>/do not delete this gain'
                                        */
  real_T donotdeletethisgain_Gain_j;   /* Expression: 1
                                        * Referenced by: '<S120>/do not delete this gain'
                                        */
  real_T Kv_Gain_ej;                   /* Expression: Ki
                                        * Referenced by: '<S21>/Kv'
                                        */
  real_T donotdeletethisgain_Gain_fx;  /* Expression: 1
                                        * Referenced by: '<S121>/do not delete this gain'
                                        */
  real_T donotdeletethisgain_Gain_ka;  /* Expression: 1
                                        * Referenced by: '<S122>/do not delete this gain'
                                        */
  real_T donotdeletethisgain_Gain_fm;  /* Expression: 1
                                        * Referenced by: '<S123>/do not delete this gain'
                                        */
  real_T Kv1_Gain_k;                   /* Expression: Kv
                                        * Referenced by: '<S21>/Kv1'
                                        */
  real_T nominalspeed_Value;           /* Expression: 1
                                        * Referenced by: '<S73>/nominal speed'
                                        */
  real_T dw_delay_InitialCondition;    /* Expression: SM.dwo
                                        * Referenced by: '<S110>/dw_delay'
                                        */
  real_T F2_Gain;                      /* Expression: 2
                                        * Referenced by: '<S110>/F2'
                                        */
  real_T dw_predict_InitialCondition;  /* Expression: SM.dwo
                                        * Referenced by: '<S110>/dw_predict'
                                        */
  real_T Switch_Threshold_k;           /* Expression: 0
                                        * Referenced by: '<S73>/Switch'
                                        */
  real_T units_Gain;                   /* Expression: SM.Nb
                                        * Referenced by: '<S73>/units'
                                        */
  real_T u1_Gain[2];                   /* Expression: [1 -1]
                                        * Referenced by: '<S82>/1-1'
                                        */
  real_T units2_Gain;                  /* Expression: SM.Pb/SM.Nb
                                        * Referenced by: '<S73>/units2'
                                        */
  real_T theta_gainval;                /* Computed Parameter: theta_gainval
                                        * Referenced by: '<S73>/theta'
                                        */
  real_T theta_IC;                     /* Expression: SM.tho/SM.p
                                        * Referenced by: '<S73>/theta'
                                        */
  real_T t_Gain;                       /* Expression: 180/pi
                                        * Referenced by: '<S73>/t'
                                        */
  real_T units1_Gain;                  /* Expression: SM.Pb
                                        * Referenced by: '<S73>/units1'
                                        */
  real_T uLl_q_Gain_h[2];              /* Expression: SM.One_Llq
                                        * Referenced by: '<S84>/1//Ll_q'
                                        */
  real_T Constant8_Value;              /* Expression: SM.Sat
                                        * Referenced by: '<S75>/Constant8'
                                        */
  real_T Switch3_Threshold;            /* Expression: 0.5
                                        * Referenced by: '<S75>/Switch3'
                                        */
  real_T uLl_d_Gain_p[3];
  /* Expression: [ 1/SM.Ll   SM.Llkd/(SM.Llfd*SM.Llkd+(SM.Llfd+SM.Llkd)*SM.LC)   SM.Llfd/(SM.Llfd*SM.Llkd+(SM.Llfd+SM.Llkd)*SM.LC)]
   * Referenced by: '<S84>/1//Ll_d'
   */
  real_T u_Vb_Gain;                    /* Expression: 1/SM.Vb
                                        * Referenced by: '<S76>/1_Vb'
                                        */
  real_T Gain_Gain;                    /* Expression: 1/3
                                        * Referenced by: '<S76>/Gain'
                                        */
  real_T Gain3_Gain_o[9];
  /* Expression: [ 1   -1/2   -1/2; 0   sqrt(3)/2   -sqrt(3)/2; 1/2  1/2  1/2 ]
   * Referenced by: '<S97>/Gain3'
   */
  real_T Gain1_Gain_gd;                /* Expression: 2/3
                                        * Referenced by: '<S97>/Gain1'
                                        */
  real_T Constant5_Value_c;            /* Expression: SM.Sat
                                        * Referenced by: '<S75>/Constant5'
                                        */
  real_T Switch2_Threshold;            /* Expression: 0.5
                                        * Referenced by: '<S75>/Switch2'
                                        */
  real_T Gain_Gain_d;                  /* Expression: 180/pi
                                        * Referenced by: '<S81>/Gain'
                                        */
  real_T Gain_Gain_m;                  /* Expression: SM.Gain1
                                        * Referenced by: '<S79>/Gain'
                                        */
  real_T Gain1_Gain_f;                 /* Expression: SM.Gain1
                                        * Referenced by: '<S79>/Gain1'
                                        */
  real_T outputformatting_Gain[18];
  /* Expression: [SM.Ib2*ones(1,5),SM.N2,SM.Ib2,SM.Ib2,SM.Ib2,SM.phib,SM.phib,SM.Vb2,SM.Vb2,(SM.phib/SM.Ib2)*ones(1,2),1,1,1]
   * Referenced by: '<S74>/output formatting'
   */
  real_T Constant6_Value_m;            /* Expression: 0
                                        * Referenced by: '<Root>/Constant6'
                                        */
  real_T N_Gain;                       /* Expression: SM.N
                                        * Referenced by: '<S71>/N'
                                        */
  real_T Vkd0Vkq10Vkq20_Value[2];      /* Expression: zeros(1, SM.nState-3)
                                        * Referenced by: '<S71>/[ Vkd =0 Vkq1=0  Vkq2=0 ]'
                                        */
  real_T voltages_InitialCondition;    /* Expression: 0
                                        * Referenced by: '<S86>/voltages'
                                        */
  real_T IC_Threshold;                 /* Expression: Ts
                                        * Referenced by: '<S86>/IC'
                                        */
  real_T u_Pb_Gain;                    /* Expression: 1/SM.Pb
                                        * Referenced by: '<S73>/1_Pb'
                                        */
  real_T F_Gain;                       /* Expression: SM.F
                                        * Referenced by: '<S73>/F'
                                        */
  real_T u_Pb1_Gain;                   /* Expression: SM.Nb/SM.Pb
                                        * Referenced by: '<S73>/1_Pb1'
                                        */
  real_T u2H_Gain;                     /* Expression: 1/(2*SM.H)
                                        * Referenced by: '<S73>/1 ----- 2H'
                                        */
  real_T Rotorspeeddeviationdw_gainval;
                            /* Computed Parameter: Rotorspeeddeviationdw_gainval
                             * Referenced by: '<S109>/Rotor spee<d deviation (dw)'
                             */
  real_T webase_Gain;                  /* Expression: SM.web
                                        * Referenced by: '<S73>/we base'
                                        */
  real_T web3_Gain;                    /* Expression: SM.web/SM.p
                                        * Referenced by: '<S73>/web3'
                                        */
  real_T donotdeletethisgain_Gain_on;  /* Expression: 1
                                        * Referenced by: '<S26>/do not delete this gain'
                                        */
  real_T Constant9_Value;              /* Expression: 2*pi
                                        * Referenced by: '<S2>/Constant9'
                                        */
  boolean_T Constant1_Value_n;         /* Expression: SM.nState==6
                                        * Referenced by: '<S85>/Constant1'
                                        */
  boolean_T Constant2_Value_io;        /* Expression: SM.nState==6
                                        * Referenced by: '<S85>/Constant2'
                                        */
  boolean_T Constant3_Value_p;         /* Expression: SM.nState==6
                                        * Referenced by: '<S85>/Constant3'
                                        */
  boolean_T Constant1_Value_c;         /* Expression: SM.nState==6
                                        * Referenced by: '<S74>/Constant1'
                                        */
  P_Subsystem1_LCITopology6x6Mi_T Subsystem1;/* '<S65>/Subsystem1' */
  P_Subsystempi2delay_LCITopolo_T Subsystempi2delay;/* '<S65>/Subsystem - pi//2 delay' */
  P_Subsystem1_LCITopology6x6Mi_T Subsystem1_ka;/* '<S54>/Subsystem1' */
  P_Subsystempi2delay_LCITopolo_T Subsystempi2delay_k;/* '<S54>/Subsystem - pi//2 delay' */
};

/* Real-time Model Data Structure */
struct tag_RTM_LCITopology6x6Minimal_T {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  X_LCITopology6x6Minimal_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  boolean_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeF[1][11];
  ODE1_IntgData intgData;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  } Sizes;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    uint32_T clockTick1;
    uint32_T clockTickH1;
    boolean_T firstInitCondFlag;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

/* Class declaration for model LCITopology6x6Minimal */
class LCITopology6x6MinimalModelClass {
  /* public data and function members */
 public:
  /* External inputs */
  ExtU_LCITopology6x6Minimal_T LCITopology6x6Minimal_U;

  /* External outputs */
  ExtY_LCITopology6x6Minimal_T LCITopology6x6Minimal_Y;

  /* model initialize function */
  void initialize();

  /* model step function */
  void step();

  /* model terminate function */
  void terminate();

  /* Constructor */
  LCITopology6x6MinimalModelClass();

  /* Destructor */
  ~LCITopology6x6MinimalModelClass();

  /* Real-Time Model get method */
  RT_MODEL_LCITopology6x6Minima_T * getRTM();

  /* private data and function members */
 private:
  /* Tunable parameters */
  static P_LCITopology6x6Minimal_T LCITopology6x6Minimal_P;

  /* Block signals */
  B_LCITopology6x6Minimal_T LCITopology6x6Minimal_B;

  /* Block states */
  DW_LCITopology6x6Minimal_T LCITopology6x6Minimal_DW;
  X_LCITopology6x6Minimal_T LCITopology6x6Minimal_X;/* Block continuous states */
  PrevZCX_LCITopology6x6Minimal_T LCITopology6x6Minimal_PrevZCX;/* Triggered events */

  /* Real-Time Model */
  RT_MODEL_LCITopology6x6Minima_T LCITopology6x6Minimal_M;

  /* private member function(s) for subsystem '<S54>/Subsystem - pi//2 delay'*/
  static void LCITopol_Subsystempi2delay_Init(B_Subsystempi2delay_LCITopolo_T
    *localB, P_Subsystempi2delay_LCITopolo_T *localP);
  static void LCITo_Subsystempi2delay_Disable(DW_Subsystempi2delay_LCITopol_T
    *localDW);
  void LCITopology6x_Subsystempi2delay(uint8_T rtu_Enable, const real_T
    rtu_alpha_beta[2], real_T rtu_wt, B_Subsystempi2delay_LCITopolo_T *localB,
    DW_Subsystempi2delay_LCITopol_T *localDW);

  /* private member function(s) for subsystem '<S54>/Subsystem1'*/
  static void LCITopology6x6M_Subsystem1_Init(B_Subsystem1_LCITopology6x6Mi_T
    *localB, P_Subsystem1_LCITopology6x6Mi_T *localP);
  static void LCITopology6_Subsystem1_Disable(DW_Subsystem1_LCITopology6x6M_T
    *localDW);
  void LCITopology6x6Minima_Subsystem1(uint8_T rtu_Enable, const real_T
    rtu_alpha_beta[2], real_T rtu_wt, B_Subsystem1_LCITopology6x6Mi_T *localB,
    DW_Subsystem1_LCITopology6x6M_T *localDW);

  /* Continuous states update member function*/
  void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si );

  /* Derivatives member function */
  void LCITopology6x6Minimal_derivatives();
};

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<Root>/Constant' : Unused code path elimination
 * Block '<Root>/Constant4' : Unused code path elimination
 * Block '<Root>/Constant5' : Unused code path elimination
 * Block '<S38>/0 1' : Unused code path elimination
 * Block '<S38>/1//Rsw' : Unused code path elimination
 * Block '<S38>/Switch' : Unused code path elimination
 * Block '<S40>/0 1' : Unused code path elimination
 * Block '<S40>/1//Rsw' : Unused code path elimination
 * Block '<S40>/Switch' : Unused code path elimination
 * Block '<S48>/Rad->Deg.' : Unused code path elimination
 * Block '<S80>/x->r' : Unused code path elimination
 * Block '<S160>/Constant' : Unused code path elimination
 * Block '<S161>/Constant' : Unused code path elimination
 * Block '<S28>/Divide' : Unused code path elimination
 * Block '<S28>/Divide1' : Unused code path elimination
 * Block '<S28>/Gain' : Unused code path elimination
 * Block '<S71>/Rate Transition1' : Eliminated since input and output rates are identical
 * Block '<S71>/Rate Transition2' : Eliminated since input and output rates are identical
 * Block '<S71>/Rate Transition3' : Eliminated since input and output rates are identical
 * Block '<S73>/Rate Transition1' : Eliminated since input and output rates are identical
 * Block '<S73>/Rate Transition2' : Eliminated since input and output rates are identical
 * Block '<S160>/Data Type Conversion' : Eliminate redundant data type conversion
 * Block '<S161>/Data Type Conversion' : Eliminate redundant data type conversion
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'LCITopology6x6Minimal'
 * '<S1>'   : 'LCITopology6x6Minimal/CBA to ABC'
 * '<S2>'   : 'LCITopology6x6Minimal/Controlled_three_phase_source1'
 * '<S3>'   : 'LCITopology6x6Minimal/Current Measurement2'
 * '<S4>'   : 'LCITopology6x6Minimal/Ideal Switch'
 * '<S5>'   : 'LCITopology6x6Minimal/Ideal Switch1'
 * '<S6>'   : 'LCITopology6x6Minimal/PLL (3ph)1'
 * '<S7>'   : 'LCITopology6x6Minimal/Quadratic_load'
 * '<S8>'   : 'LCITopology6x6Minimal/Reorder pulses from {positive, negative} to bridge'
 * '<S9>'   : 'LCITopology6x6Minimal/Reorder pulses from {positive, negative} to bridge1'
 * '<S10>'  : 'LCITopology6x6Minimal/Subsystem'
 * '<S11>'  : 'LCITopology6x6Minimal/Subsystem1'
 * '<S12>'  : 'LCITopology6x6Minimal/Subsystem2'
 * '<S13>'  : 'LCITopology6x6Minimal/Subsystem3'
 * '<S14>'  : 'LCITopology6x6Minimal/Subsystem4'
 * '<S15>'  : 'LCITopology6x6Minimal/Subsystem5'
 * '<S16>'  : 'LCITopology6x6Minimal/Subsystem6'
 * '<S17>'  : 'LCITopology6x6Minimal/Subsystem7'
 * '<S18>'  : 'LCITopology6x6Minimal/Subsystem8'
 * '<S19>'  : 'LCITopology6x6Minimal/Synchronous Machine SI Fundamental (custom)1'
 * '<S20>'  : 'LCITopology6x6Minimal/Three-Phase Transformer (Two Windings)'
 * '<S21>'  : 'LCITopology6x6Minimal/Three-Phase V-I Measurement'
 * '<S22>'  : 'LCITopology6x6Minimal/Three-Phase V-I Measurement1'
 * '<S23>'  : 'LCITopology6x6Minimal/Three-Phase V-I Measurement2'
 * '<S24>'  : 'LCITopology6x6Minimal/Universal Bridge1'
 * '<S25>'  : 'LCITopology6x6Minimal/Universal Bridge2'
 * '<S26>'  : 'LCITopology6x6Minimal/Voltage Measurement1'
 * '<S27>'  : 'LCITopology6x6Minimal/Voltage Measurement8'
 * '<S28>'  : 'LCITopology6x6Minimal/base_values_calculation'
 * '<S29>'  : 'LCITopology6x6Minimal/convert_machine_outputs_to_si'
 * '<S30>'  : 'LCITopology6x6Minimal/powergui1'
 * '<S31>'  : 'LCITopology6x6Minimal/Controlled_three_phase_source1/-pi..pi_range'
 * '<S32>'  : 'LCITopology6x6Minimal/Controlled_three_phase_source1/Controlled Voltage Source3'
 * '<S33>'  : 'LCITopology6x6Minimal/Controlled_three_phase_source1/Controlled Voltage Source4'
 * '<S34>'  : 'LCITopology6x6Minimal/Controlled_three_phase_source1/Controlled Voltage Source5'
 * '<S35>'  : 'LCITopology6x6Minimal/Controlled_three_phase_source1/Integrator (Discrete or Continuous)'
 * '<S36>'  : 'LCITopology6x6Minimal/Controlled_three_phase_source1/Integrator (Discrete or Continuous)/Discrete'
 * '<S37>'  : 'LCITopology6x6Minimal/Current Measurement2/Model'
 * '<S38>'  : 'LCITopology6x6Minimal/Ideal Switch/Model'
 * '<S39>'  : 'LCITopology6x6Minimal/Ideal Switch/Model/Measurement list'
 * '<S40>'  : 'LCITopology6x6Minimal/Ideal Switch1/Model'
 * '<S41>'  : 'LCITopology6x6Minimal/Ideal Switch1/Model/Measurement list'
 * '<S42>'  : 'LCITopology6x6Minimal/PLL (3ph)1/Model'
 * '<S43>'  : 'LCITopology6x6Minimal/PLL (3ph)1/Model/Automatic Gain Control'
 * '<S44>'  : 'LCITopology6x6Minimal/PLL (3ph)1/Model/Continuous'
 * '<S45>'  : 'LCITopology6x6Minimal/PLL (3ph)1/Model/Second-Order Filter'
 * '<S46>'  : 'LCITopology6x6Minimal/PLL (3ph)1/Model/Variable Frequency Mean value'
 * '<S47>'  : 'LCITopology6x6Minimal/PLL (3ph)1/Model/abc to dq0'
 * '<S48>'  : 'LCITopology6x6Minimal/PLL (3ph)1/Model/Automatic Gain Control/Positive-Sequence (PLL-Driven)'
 * '<S49>'  : 'LCITopology6x6Minimal/PLL (3ph)1/Model/Automatic Gain Control/Positive-Sequence (PLL-Driven)/Mean (Variable Frequency)1'
 * '<S50>'  : 'LCITopology6x6Minimal/PLL (3ph)1/Model/Automatic Gain Control/Positive-Sequence (PLL-Driven)/Mean (Variable Frequency)2'
 * '<S51>'  : 'LCITopology6x6Minimal/PLL (3ph)1/Model/Automatic Gain Control/Positive-Sequence (PLL-Driven)/abc ->dq0'
 * '<S52>'  : 'LCITopology6x6Minimal/PLL (3ph)1/Model/Automatic Gain Control/Positive-Sequence (PLL-Driven)/Mean (Variable Frequency)1/Model'
 * '<S53>'  : 'LCITopology6x6Minimal/PLL (3ph)1/Model/Automatic Gain Control/Positive-Sequence (PLL-Driven)/Mean (Variable Frequency)2/Model'
 * '<S54>'  : 'LCITopology6x6Minimal/PLL (3ph)1/Model/Automatic Gain Control/Positive-Sequence (PLL-Driven)/abc ->dq0/Alpha-Beta-Zero to dq0'
 * '<S55>'  : 'LCITopology6x6Minimal/PLL (3ph)1/Model/Automatic Gain Control/Positive-Sequence (PLL-Driven)/abc ->dq0/abc to Alpha-Beta-Zero'
 * '<S56>'  : 'LCITopology6x6Minimal/PLL (3ph)1/Model/Automatic Gain Control/Positive-Sequence (PLL-Driven)/abc ->dq0/Alpha-Beta-Zero to dq0/Compare To Constant'
 * '<S57>'  : 'LCITopology6x6Minimal/PLL (3ph)1/Model/Automatic Gain Control/Positive-Sequence (PLL-Driven)/abc ->dq0/Alpha-Beta-Zero to dq0/Compare To Constant1'
 * '<S58>'  : 'LCITopology6x6Minimal/PLL (3ph)1/Model/Automatic Gain Control/Positive-Sequence (PLL-Driven)/abc ->dq0/Alpha-Beta-Zero to dq0/Subsystem - pi//2 delay'
 * '<S59>'  : 'LCITopology6x6Minimal/PLL (3ph)1/Model/Automatic Gain Control/Positive-Sequence (PLL-Driven)/abc ->dq0/Alpha-Beta-Zero to dq0/Subsystem1'
 * '<S60>'  : 'LCITopology6x6Minimal/PLL (3ph)1/Model/Second-Order Filter/Model'
 * '<S61>'  : 'LCITopology6x6Minimal/PLL (3ph)1/Model/Second-Order Filter/Model/A*x'
 * '<S62>'  : 'LCITopology6x6Minimal/PLL (3ph)1/Model/Second-Order Filter/Model/B*u'
 * '<S63>'  : 'LCITopology6x6Minimal/PLL (3ph)1/Model/Second-Order Filter/Model/C*x'
 * '<S64>'  : 'LCITopology6x6Minimal/PLL (3ph)1/Model/Variable Frequency Mean value/Model'
 * '<S65>'  : 'LCITopology6x6Minimal/PLL (3ph)1/Model/abc to dq0/Alpha-Beta-Zero to dq0'
 * '<S66>'  : 'LCITopology6x6Minimal/PLL (3ph)1/Model/abc to dq0/abc to Alpha-Beta-Zero'
 * '<S67>'  : 'LCITopology6x6Minimal/PLL (3ph)1/Model/abc to dq0/Alpha-Beta-Zero to dq0/Compare To Constant'
 * '<S68>'  : 'LCITopology6x6Minimal/PLL (3ph)1/Model/abc to dq0/Alpha-Beta-Zero to dq0/Compare To Constant1'
 * '<S69>'  : 'LCITopology6x6Minimal/PLL (3ph)1/Model/abc to dq0/Alpha-Beta-Zero to dq0/Subsystem - pi//2 delay'
 * '<S70>'  : 'LCITopology6x6Minimal/PLL (3ph)1/Model/abc to dq0/Alpha-Beta-Zero to dq0/Subsystem1'
 * '<S71>'  : 'LCITopology6x6Minimal/Synchronous Machine SI Fundamental (custom)1/Electrical model1'
 * '<S72>'  : 'LCITopology6x6Minimal/Synchronous Machine SI Fundamental (custom)1/Measurements'
 * '<S73>'  : 'LCITopology6x6Minimal/Synchronous Machine SI Fundamental (custom)1/Mechanical model'
 * '<S74>'  : 'LCITopology6x6Minimal/Synchronous Machine SI Fundamental (custom)1/Electrical model1/Machine measurements'
 * '<S75>'  : 'LCITopology6x6Minimal/Synchronous Machine SI Fundamental (custom)1/Electrical model1/Synchronous Machine Discrete Model'
 * '<S76>'  : 'LCITopology6x6Minimal/Synchronous Machine SI Fundamental (custom)1/Electrical model1/abc to qd1'
 * '<S77>'  : 'LCITopology6x6Minimal/Synchronous Machine SI Fundamental (custom)1/Electrical model1/qd to abc1'
 * '<S78>'  : 'LCITopology6x6Minimal/Synchronous Machine SI Fundamental (custom)1/Electrical model1/Machine measurements/Delta angle'
 * '<S79>'  : 'LCITopology6x6Minimal/Synchronous Machine SI Fundamental (custom)1/Electrical model1/Machine measurements/PQ'
 * '<S80>'  : 'LCITopology6x6Minimal/Synchronous Machine SI Fundamental (custom)1/Electrical model1/Machine measurements/Delta angle/Cartesian to Polar'
 * '<S81>'  : 'LCITopology6x6Minimal/Synchronous Machine SI Fundamental (custom)1/Electrical model1/Machine measurements/Delta angle/Radians to Degrees'
 * '<S82>'  : 'LCITopology6x6Minimal/Synchronous Machine SI Fundamental (custom)1/Electrical model1/Synchronous Machine Discrete Model/Electromagnetic Torque'
 * '<S83>'  : 'LCITopology6x6Minimal/Synchronous Machine SI Fundamental (custom)1/Electrical model1/Synchronous Machine Discrete Model/Matrix W'
 * '<S84>'  : 'LCITopology6x6Minimal/Synchronous Machine SI Fundamental (custom)1/Electrical model1/Synchronous Machine Discrete Model/Phimqd'
 * '<S85>'  : 'LCITopology6x6Minimal/Synchronous Machine SI Fundamental (custom)1/Electrical model1/Synchronous Machine Discrete Model/Saturation'
 * '<S86>'  : 'LCITopology6x6Minimal/Synchronous Machine SI Fundamental (custom)1/Electrical model1/Synchronous Machine Discrete Model/phi'
 * '<S87>'  : 'LCITopology6x6Minimal/Synchronous Machine SI Fundamental (custom)1/Electrical model1/Synchronous Machine Discrete Model/Saturation/Lmd_sat'
 * '<S88>'  : 'LCITopology6x6Minimal/Synchronous Machine SI Fundamental (custom)1/Electrical model1/Synchronous Machine Discrete Model/Saturation/Lmq_sat'
 * '<S89>'  : 'LCITopology6x6Minimal/Synchronous Machine SI Fundamental (custom)1/Electrical model1/Synchronous Machine Discrete Model/Saturation/Update Matrix L'
 * '<S90>'  : 'LCITopology6x6Minimal/Synchronous Machine SI Fundamental (custom)1/Electrical model1/Synchronous Machine Discrete Model/Saturation/Lmd_sat/Lad'
 * '<S91>'  : 'LCITopology6x6Minimal/Synchronous Machine SI Fundamental (custom)1/Electrical model1/Synchronous Machine Discrete Model/Saturation/Lmd_sat/phimd'
 * '<S92>'  : 'LCITopology6x6Minimal/Synchronous Machine SI Fundamental (custom)1/Electrical model1/Synchronous Machine Discrete Model/Saturation/Lmq_sat/Laq'
 * '<S93>'  : 'LCITopology6x6Minimal/Synchronous Machine SI Fundamental (custom)1/Electrical model1/Synchronous Machine Discrete Model/Saturation/Lmq_sat/phimq'
 * '<S94>'  : 'LCITopology6x6Minimal/Synchronous Machine SI Fundamental (custom)1/Electrical model1/Synchronous Machine Discrete Model/phi/Subsystem'
 * '<S95>'  : 'LCITopology6x6Minimal/Synchronous Machine SI Fundamental (custom)1/Electrical model1/abc to qd1/abc to dq0'
 * '<S96>'  : 'LCITopology6x6Minimal/Synchronous Machine SI Fundamental (custom)1/Electrical model1/abc to qd1/abc to dq0/Alpha-Beta-Zero to dq0'
 * '<S97>'  : 'LCITopology6x6Minimal/Synchronous Machine SI Fundamental (custom)1/Electrical model1/abc to qd1/abc to dq0/abc to Alpha-Beta-Zero'
 * '<S98>'  : 'LCITopology6x6Minimal/Synchronous Machine SI Fundamental (custom)1/Electrical model1/abc to qd1/abc to dq0/Alpha-Beta-Zero to dq0/Compare To Constant'
 * '<S99>'  : 'LCITopology6x6Minimal/Synchronous Machine SI Fundamental (custom)1/Electrical model1/abc to qd1/abc to dq0/Alpha-Beta-Zero to dq0/Compare To Constant1'
 * '<S100>' : 'LCITopology6x6Minimal/Synchronous Machine SI Fundamental (custom)1/Electrical model1/abc to qd1/abc to dq0/Alpha-Beta-Zero to dq0/Subsystem - pi//2 delay'
 * '<S101>' : 'LCITopology6x6Minimal/Synchronous Machine SI Fundamental (custom)1/Electrical model1/abc to qd1/abc to dq0/Alpha-Beta-Zero to dq0/Subsystem1'
 * '<S102>' : 'LCITopology6x6Minimal/Synchronous Machine SI Fundamental (custom)1/Electrical model1/qd to abc1/dq0 to abc'
 * '<S103>' : 'LCITopology6x6Minimal/Synchronous Machine SI Fundamental (custom)1/Electrical model1/qd to abc1/dq0 to abc/Alpha-Beta-Zero to abc'
 * '<S104>' : 'LCITopology6x6Minimal/Synchronous Machine SI Fundamental (custom)1/Electrical model1/qd to abc1/dq0 to abc/dq0 to Alpha-Beta-Zero'
 * '<S105>' : 'LCITopology6x6Minimal/Synchronous Machine SI Fundamental (custom)1/Electrical model1/qd to abc1/dq0 to abc/dq0 to Alpha-Beta-Zero/Compare To Constant'
 * '<S106>' : 'LCITopology6x6Minimal/Synchronous Machine SI Fundamental (custom)1/Electrical model1/qd to abc1/dq0 to abc/dq0 to Alpha-Beta-Zero/Compare To Constant1'
 * '<S107>' : 'LCITopology6x6Minimal/Synchronous Machine SI Fundamental (custom)1/Electrical model1/qd to abc1/dq0 to abc/dq0 to Alpha-Beta-Zero/Subsystem - pi//2 delay'
 * '<S108>' : 'LCITopology6x6Minimal/Synchronous Machine SI Fundamental (custom)1/Electrical model1/qd to abc1/dq0 to abc/dq0 to Alpha-Beta-Zero/Subsystem1'
 * '<S109>' : 'LCITopology6x6Minimal/Synchronous Machine SI Fundamental (custom)1/Mechanical model/Subsystem'
 * '<S110>' : 'LCITopology6x6Minimal/Synchronous Machine SI Fundamental (custom)1/Mechanical model/Subsystem/Delay Prediction'
 * '<S111>' : 'LCITopology6x6Minimal/Three-Phase Transformer (Two Windings)/Model'
 * '<S112>' : 'LCITopology6x6Minimal/Three-Phase Transformer (Two Windings)/Model/Linear'
 * '<S113>' : 'LCITopology6x6Minimal/Three-Phase Transformer (Two Windings)/Model/Linear1'
 * '<S114>' : 'LCITopology6x6Minimal/Three-Phase Transformer (Two Windings)/Model/Linear2'
 * '<S115>' : 'LCITopology6x6Minimal/Three-Phase V-I Measurement/Mode I'
 * '<S116>' : 'LCITopology6x6Minimal/Three-Phase V-I Measurement/Mode V'
 * '<S117>' : 'LCITopology6x6Minimal/Three-Phase V-I Measurement/Model'
 * '<S118>' : 'LCITopology6x6Minimal/Three-Phase V-I Measurement/Model/I A:'
 * '<S119>' : 'LCITopology6x6Minimal/Three-Phase V-I Measurement/Model/I B:'
 * '<S120>' : 'LCITopology6x6Minimal/Three-Phase V-I Measurement/Model/I C:'
 * '<S121>' : 'LCITopology6x6Minimal/Three-Phase V-I Measurement/Model/U AB:'
 * '<S122>' : 'LCITopology6x6Minimal/Three-Phase V-I Measurement/Model/U BC:'
 * '<S123>' : 'LCITopology6x6Minimal/Three-Phase V-I Measurement/Model/U CA:'
 * '<S124>' : 'LCITopology6x6Minimal/Three-Phase V-I Measurement/Model/I A:/Model'
 * '<S125>' : 'LCITopology6x6Minimal/Three-Phase V-I Measurement/Model/I B:/Model'
 * '<S126>' : 'LCITopology6x6Minimal/Three-Phase V-I Measurement/Model/I C:/Model'
 * '<S127>' : 'LCITopology6x6Minimal/Three-Phase V-I Measurement/Model/U AB:/Model'
 * '<S128>' : 'LCITopology6x6Minimal/Three-Phase V-I Measurement/Model/U BC:/Model'
 * '<S129>' : 'LCITopology6x6Minimal/Three-Phase V-I Measurement/Model/U CA:/Model'
 * '<S130>' : 'LCITopology6x6Minimal/Three-Phase V-I Measurement1/Mode I'
 * '<S131>' : 'LCITopology6x6Minimal/Three-Phase V-I Measurement1/Mode V'
 * '<S132>' : 'LCITopology6x6Minimal/Three-Phase V-I Measurement1/Model'
 * '<S133>' : 'LCITopology6x6Minimal/Three-Phase V-I Measurement1/Model/I A:'
 * '<S134>' : 'LCITopology6x6Minimal/Three-Phase V-I Measurement1/Model/I B:'
 * '<S135>' : 'LCITopology6x6Minimal/Three-Phase V-I Measurement1/Model/I C:'
 * '<S136>' : 'LCITopology6x6Minimal/Three-Phase V-I Measurement1/Model/U AB:'
 * '<S137>' : 'LCITopology6x6Minimal/Three-Phase V-I Measurement1/Model/U BC:'
 * '<S138>' : 'LCITopology6x6Minimal/Three-Phase V-I Measurement1/Model/U CA:'
 * '<S139>' : 'LCITopology6x6Minimal/Three-Phase V-I Measurement1/Model/I A:/Model'
 * '<S140>' : 'LCITopology6x6Minimal/Three-Phase V-I Measurement1/Model/I B:/Model'
 * '<S141>' : 'LCITopology6x6Minimal/Three-Phase V-I Measurement1/Model/I C:/Model'
 * '<S142>' : 'LCITopology6x6Minimal/Three-Phase V-I Measurement1/Model/U AB:/Model'
 * '<S143>' : 'LCITopology6x6Minimal/Three-Phase V-I Measurement1/Model/U BC:/Model'
 * '<S144>' : 'LCITopology6x6Minimal/Three-Phase V-I Measurement1/Model/U CA:/Model'
 * '<S145>' : 'LCITopology6x6Minimal/Three-Phase V-I Measurement2/Mode I'
 * '<S146>' : 'LCITopology6x6Minimal/Three-Phase V-I Measurement2/Mode V'
 * '<S147>' : 'LCITopology6x6Minimal/Three-Phase V-I Measurement2/Model'
 * '<S148>' : 'LCITopology6x6Minimal/Three-Phase V-I Measurement2/Model/I A:'
 * '<S149>' : 'LCITopology6x6Minimal/Three-Phase V-I Measurement2/Model/I B:'
 * '<S150>' : 'LCITopology6x6Minimal/Three-Phase V-I Measurement2/Model/I C:'
 * '<S151>' : 'LCITopology6x6Minimal/Three-Phase V-I Measurement2/Model/U A:'
 * '<S152>' : 'LCITopology6x6Minimal/Three-Phase V-I Measurement2/Model/U B:'
 * '<S153>' : 'LCITopology6x6Minimal/Three-Phase V-I Measurement2/Model/U C:'
 * '<S154>' : 'LCITopology6x6Minimal/Three-Phase V-I Measurement2/Model/I A:/Model'
 * '<S155>' : 'LCITopology6x6Minimal/Three-Phase V-I Measurement2/Model/I B:/Model'
 * '<S156>' : 'LCITopology6x6Minimal/Three-Phase V-I Measurement2/Model/I C:/Model'
 * '<S157>' : 'LCITopology6x6Minimal/Three-Phase V-I Measurement2/Model/U A:/Model'
 * '<S158>' : 'LCITopology6x6Minimal/Three-Phase V-I Measurement2/Model/U B:/Model'
 * '<S159>' : 'LCITopology6x6Minimal/Three-Phase V-I Measurement2/Model/U C:/Model'
 * '<S160>' : 'LCITopology6x6Minimal/Universal Bridge1/Model'
 * '<S161>' : 'LCITopology6x6Minimal/Universal Bridge2/Model'
 * '<S162>' : 'LCITopology6x6Minimal/Voltage Measurement1/Model'
 * '<S163>' : 'LCITopology6x6Minimal/Voltage Measurement8/Model'
 * '<S164>' : 'LCITopology6x6Minimal/powergui1/EquivalentModel1'
 * '<S165>' : 'LCITopology6x6Minimal/powergui1/EquivalentModel1/Gates'
 * '<S166>' : 'LCITopology6x6Minimal/powergui1/EquivalentModel1/Sources'
 * '<S167>' : 'LCITopology6x6Minimal/powergui1/EquivalentModel1/Status'
 * '<S168>' : 'LCITopology6x6Minimal/powergui1/EquivalentModel1/Yout'
 */
#endif                                 /* RTW_HEADER_LCITopology6x6Minimal_h_ */
