//
// File: sfc_6p_6p_topology_machine.h
//
// Code generated for Simulink model 'sfc_6p_6p_topology_machine'.
//
// Model version                  : 18.6
// Simulink Coder version         : 23.2 (R2023b) 01-Aug-2023
// C/C++ source code generated on : Sat Nov  8 12:30:46 2025
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Windows64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_sfc_6p_6p_topology_machine_h_
#define RTW_HEADER_sfc_6p_6p_topology_machine_h_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include <cstring>
#include <stddef.h>
#include "zero_crossing_types.h"

// Macros for accessing real-time model data structure
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

#ifndef rtmGetTStart
#define rtmGetTStart(rtm)              ((rtm)->Timing.tStart)
#endif

#ifndef ODE1_INTG
#define ODE1_INTG

// ODE1 Integration Data
struct ODE1_IntgData {
  real_T *f[1];                        // derivatives
};

#endif

extern "C"
{
  static real_T rtGetInf(void);
  static real32_T rtGetInfF(void);
  static real_T rtGetMinusInf(void);
  static real32_T rtGetMinusInfF(void);
}                                      // extern "C"

extern "C"
{
  static real_T rtGetNaN(void);
  static real32_T rtGetNaNF(void);
}                                      // extern "C"

#define NOT_USING_NONFINITE_LITERALS   1

extern "C"
{
  extern real_T rtInf;
  extern real_T rtMinusInf;
  extern real_T rtNaN;
  extern real32_T rtInfF;
  extern real32_T rtMinusInfF;
  extern real32_T rtNaNF;
  static void rt_InitInfAndNaN(size_t realSize);
  static boolean_T rtIsInf(real_T value);
  static boolean_T rtIsInfF(real32_T value);
  static boolean_T rtIsNaN(real_T value);
  static boolean_T rtIsNaNF(real32_T value);
  struct BigEndianIEEEDouble {
    struct {
      uint32_T wordH;
      uint32_T wordL;
    } words;
  };

  struct LittleEndianIEEEDouble {
    struct {
      uint32_T wordL;
      uint32_T wordH;
    } words;
  };

  struct IEEESingle {
    union {
      real32_T wordLreal;
      uint32_T wordLuint;
    } wordL;
  };
}                                      // extern "C"

// Class declaration for model sfc_6p_6p_topology_machine
namespace generated
{
  namespace sfc
  {
    class sfc_6p_6p_topology_machine final
    {
      // public data and function members
     public:
      // Block signals and states (default storage) for system '<S48>/Subsystem - pi//2 delay' 
      struct DW_Subsystempi2delay_sfc_6p_6_T {
        boolean_T Subsystempi2delay_MODE;// '<S48>/Subsystem - pi//2 delay'
      };

      // Block signals and states (default storage) for system '<S48>/Subsystem1' 
      struct DW_Subsystem1_sfc_6p_6p_topol_T {
        boolean_T Subsystem1_MODE;     // '<S48>/Subsystem1'
      };

      // Block signals and states (default storage) for system '<Root>'
      struct DW_sfc_6p_6p_topology_machine_T {
        DW_Subsystem1_sfc_6p_6p_topol_T Subsystem1;// '<S59>/Subsystem1'
        DW_Subsystempi2delay_sfc_6p_6_T Subsystempi2delay;// '<S59>/Subsystem - pi//2 delay' 
        DW_Subsystem1_sfc_6p_6p_topol_T Subsystem1_ka;// '<S48>/Subsystem1'
        DW_Subsystempi2delay_sfc_6p_6_T Subsystempi2delay_k;// '<S48>/Subsystem - pi//2 delay' 
        creal_T RealImagtoComplex;     // '<S42>/Real-Imag to Complex'
        real_T xk1[6];                 // '<S80>/fluxes'
        real_T Linv[36];               // '<S69>/Switch'
        real_T Ik[6];                  // '<S69>/Product3'
        real_T changeIqIdcurrentsigns[6];// '<S69>/change Iq Id  current signs'
        real_T Switch[2];              // '<S98>/Switch'
        real_T TmpSignalConversionAtGain3Inpor[3];
        real_T Gain3[3];               // '<S97>/Gain3'
        real_T iabc[3];                // '<S71>/ib'
        real_T Gain2[3];               // '<S2>/Gain2'
        real_T Add1[3];                // '<S2>/Add1'
        real_T TrigonometricFunction1[3];// '<S2>/Trigonometric Function1'
        real_T Gain1[3];               // '<S2>/Gain1'
        real_T StateSpace_o1[35];      // '<S158>/State-Space'
        real_T StateSpace_o2[12];      // '<S158>/State-Space'
        real_T Kv[3];                  // '<S20>/Kv'
        real_T Kv1[3];                 // '<S20>/Kv1'
        real_T Kv1_j[3];               // '<S21>/Kv1'
        real_T Gain3_o[3];             // '<S60>/Gain3'
        real_T Gain1_f[3];             // '<S60>/Gain1'
        real_T Switch_a[2];            // '<S59>/Switch'
        real_T Kv_n[3];                // '<S21>/Kv'
        real_T Kv_n3[3];               // '<S19>/Kv'
        real_T Kv1_l[3];               // '<S19>/Kv1'
        real_T iqsids[2];              // '<S76>/1-1'
        real_T Mult1[2];               // '<S76>/Mult1'
        real_T uLl_q[3];               // '<S78>/1//Ll_q'
        real_T Switch3[2];             // '<S69>/Switch3'
        real_T Product_k[3];           // '<S78>/Product'
        real_T uLl_d[3];               // '<S78>/1//Ll_d'
        real_T Product1_h[3];          // '<S78>/Product1'
        real_T u_Vb[2];                // '<S70>/1_Vb'
        real_T TmpSignalConversionAtGain3Inp_a[3];
        real_T Gain3_d[3];             // '<S91>/Gain3'
        real_T Gain1_p[3];             // '<S91>/Gain1'
        real_T Switch_i[2];            // '<S90>/Switch'
        real_T Switch2[2];             // '<S69>/Switch2'
        real_T outputformatting[18];   // '<S68>/output formatting'
        real_T voltages[6];            // '<S80>/voltages'
        real_T IC[6];                  // '<S80>/IC'
        real_T ukuk1[6];               // '<S80>/sum'
        real_T Switch1[36];            // '<S69>/Switch1'
        real_T W12wr[36];              // '<S77>/W(1,2)=wr'
        real_T W21wr[36];              // '<S77>/W(2,1)=-wr'
        real_T A_m[36];                // '<S69>/Sum1'
        real_T wbaseTs2[36];           // '<S88>/wbase*Ts//2'
        real_T Sum1_b[36];             // '<S88>/Sum1'
        real_T inversion[36];          // '<S88>/inversion'
        real_T wbaseTs2_c[36];         // '<S88>/wbase*Ts//2 '
        real_T Product1_b[6];          // '<S80>/Product1'
        real_T Sum5[36];               // '<S88>/Sum5'
        real_T Product4_e[36];         // '<S88>/Product4'
        real_T Product2_a[6];          // '<S80>/Product2'
        real_T xk[6];                // '<S80>/Ad*x(k-1) + Bd*( u(k-1) + u(k))'
        real_T uLl_d_h[3];             // '<S85>/1//Ll_d'
        real_T UpdateLmqrows156col156[36];
                            // '<S83>/Update Lmq rows[1,5, (6)] col[1,5, (6)] '
        real_T UpdateLmdrows234col234[36];
                                   // '<S83>/Update Lmd rows[2,3,4] col[2,3,4]'
        real_T Sum2_h[36];             // '<S83>/Sum2'
        real_T Linv_d[36];             // '<S79>/inversion'
        real_T RLinv[36];              // '<S79>/Product1'
        real_T uLl_q_i[3];             // '<S87>/1//Ll_q'
        real_T Gain3_o4[3];            // '<S49>/Gain3'
        real_T Gain1_c[3];             // '<S49>/Gain1'
        real_T Switch_dq[2];           // '<S48>/Switch'
        real_T fluxes_DSTATE[6];       // '<S80>/fluxes'
        real_T StateSpace_DSTATE[21];  // '<S158>/State-Space'
        real_T voltages_DSTATE[6];     // '<S80>/voltages'
        real_T inversion_DWORK1[36];   // '<S88>/inversion'
        real_T inversion_DWORK3[36];   // '<S88>/inversion'
        real_T inversion_DWORK4[36];   // '<S88>/inversion'
        real_T inversion_DWORK4_b[36]; // '<S79>/inversion'
        struct {
          real_T modelTStart;
          real_T TUbufferArea[24576];
        } VariableTransportDelay_RWORK;// '<S58>/Variable Transport Delay'

        struct {
          real_T modelTStart;
          real_T TUbufferArea[24576];
        } VariableTransportDelay_RWORK_b;// '<S46>/Variable Transport Delay'

        struct {
          real_T modelTStart;
          real_T TUbufferArea[24576];
        } VariableTransportDelay_RWORK_p;// '<S47>/Variable Transport Delay'

        real_T d_thetarad;             // '<S67>/Rotor angle dthetae'
        real_T DigitalClock;           // '<S67>/Digital Clock'
        real_T web2;                   // '<S67>/web2'
        real_T Sum3;                   // '<S67>/Sum3'
        real_T Constant4;              // '<S2>/Constant4'
        real_T Integrator;             // '<S34>/Integrator'
        real_T Saturation;             // '<S34>/Saturation'
        real_T Sum;                    // '<S29>/Sum'
        real_T Mod;                    // '<S29>/Mod'
        real_T Product5;               // '<S2>/Product5'
        real_T Product6;               // '<S2>/Product6'
        real_T Product7;               // '<S2>/Product7'
        real_T donotdeletethisgain;    // '<S25>/do not delete this gain'
        real_T donotdeletethisgain_o;  // '<S3>/do not delete this gain'
        real_T donotdeletethisgain_c;  // '<S127>/do not delete this gain'
        real_T donotdeletethisgain_p;  // '<S128>/do not delete this gain'
        real_T donotdeletethisgain_cx; // '<S129>/do not delete this gain'
        real_T donotdeletethisgain_f;  // '<S130>/do not delete this gain'
        real_T donotdeletethisgain_k;  // '<S131>/do not delete this gain'
        real_T donotdeletethisgain_n;  // '<S132>/do not delete this gain'
        real_T Subtract;               // '<S36>/Subtract'
        real_T Initial;                // '<S36>/Initial'
        real_T Integrator_d;           // '<S36>/Integrator'
        real_T Memory;                 // '<S36>/Memory'
        real_T donotdeletethisgain_c2; // '<S145>/do not delete this gain'
        real_T donotdeletethisgain_g;  // '<S146>/do not delete this gain'
        real_T donotdeletethisgain_f1; // '<S147>/do not delete this gain'
        real_T Integrator_o;           // '<S38>/Integrator'
        real_T VariableTransportDelay; // '<S58>/Variable Transport Delay'
        real_T integrator;             // '<S58>/integrator'
        real_T Clock;                  // '<S58>/Clock'
        real_T Switch_g;               // '<S58>/Switch'
        real_T Divide_g;               // '<S36>/Divide'
        real_T Kp4;                    // '<S38>/Kp4'
        real_T Kp5;                    // '<S38>/Kp5'
        real_T Kp6;                    // '<S38>/Kp6'
        real_T TransferFcn;            // '<S38>/Transfer Fcn'
        real_T Sum6;                   // '<S38>/Sum6'
        real_T Saturation2;            // '<S38>/Saturation2'
        real_T Gain10;                 // '<S36>/Gain10'
        real_T RateLimiter;            // '<S36>/Rate Limiter'
        real_T x1;                     // '<S54>/Integrator_x1'
        real_T A11;                    // '<S55>/A11'
        real_T x2;                     // '<S54>/Integrator_x2'
        real_T A12;                    // '<S55>/A12'
        real_T A21;                    // '<S55>/A21'
        real_T A22;                    // '<S55>/A22'
        real_T sum2;                   // '<S55>/sum2'
        real_T sum3;                   // '<S55>/sum3'
        real_T B11;                    // '<S56>/B11'
        real_T x1_i;                   // '<S54>/A*x1+ B* u1'
        real_T B21;                    // '<S56>/B21'
        real_T x2_m;                   // '<S54>/A*x2+ B*u2'
        real_T C11;                    // '<S57>/C11'
        real_T C12;                    // '<S57>/C12'
        real_T sum2_a;                 // '<S57>/sum2'
        real_T Du;                     // '<S54>/D*u'
        real_T y;                      // '<S54>/C*x + D*u'
        real_T Freq;                   // '<S58>/To avoid division  by zero'
        real_T period;                 // '<S58>/period'
        real_T donotdeletethisgain_h;  // '<S142>/do not delete this gain'
        real_T donotdeletethisgain_d;  // '<S143>/do not delete this gain'
        real_T donotdeletethisgain_kk; // '<S144>/do not delete this gain'
        real_T donotdeletethisgain_m;  // '<S112>/do not delete this gain'
        real_T donotdeletethisgain_b;  // '<S113>/do not delete this gain'
        real_T donotdeletethisgain_n2; // '<S114>/do not delete this gain'
        real_T donotdeletethisgain_de; // '<S115>/do not delete this gain'
        real_T donotdeletethisgain_kv; // '<S116>/do not delete this gain'
        real_T donotdeletethisgain_g0; // '<S117>/do not delete this gain'
        real_T dw_delay;               // '<S104>/dw_delay'
        real_T F2;                     // '<S104>/F2'
        real_T dw_predict;             // '<S104>/dw_predict'
        real_T Sum1_h;                 // '<S104>/Sum1'
        real_T n;                      // '<S67>/Sum'
        real_T Switch_e;               // '<S67>/Switch'
        real_T Sum2;                   // '<S76>/Sum2'
        real_T theta;                  // '<S67>/theta'
        real_T Fcn;                    // '<S67>/Fcn'
        real_T Sign;                   // '<S5>/Sign'
        real_T Abs;                    // '<S5>/Abs'
        real_T Product;                // '<S5>/Product'
        real_T Sum_c;                  // '<S5>/Sum'
        real_T Product1;               // '<S5>/Product1'
        real_T Sum1_l;                 // '<S5>/Sum1'
        real_T Pe;                     // '<S67>/Product'
        real_T ikq2;                   // '<S68>/Switch'
        real_T phimq;                  // '<S78>/sum2'
        real_T phimd;                  // '<S78>/sum1'
        real_T Sum_h;                  // '<S70>/Sum'
        real_T B;                      // '<S70>/Gain'
        real_T A;                      // '<S70>/Sum1'
        real_T C;                      // '<S70>/Sum2'
        real_T xtheta;                 // '<S74>/x->theta'
        real_T Gain;                   // '<S75>/Gain'
        real_T Product2_e;             // '<S73>/Product2'
        real_T Product3;               // '<S73>/Product3'
        real_T Peo;                    // '<S73>/Sum'
        real_T Gain_d;                 // '<S73>/Gain'
        real_T Product4;               // '<S73>/Product4'
        real_T Product5_d;             // '<S73>/Product5'
        real_T Qeo;                    // '<S73>/Sum1'
        real_T Gain1_j;                // '<S73>/Gain1'
        real_T Vfd;                    // '<S65>/N'
        real_T DigitalClock_g;         // '<S80>/Digital Clock'
        real_T u_Pb;                   // '<S67>/1_Pb'
        real_T div;                    // '<S67>/div'
        real_T F;                      // '<S67>/F'
        real_T u_Pb1;                  // '<S67>/1_Pb1'
        real_T Sum2_o;                 // '<S67>/Sum2'
        real_T u2H;                    // '<S67>/1 ----- 2H'
        real_T Sum1_p;                 // '<S67>/Sum1'
        real_T dw;                     // '<S103>/Rotor spee<d deviation (dw)'
        real_T webase;                 // '<S67>/we base'
        real_T web3;                   // '<S67>/web3'
        real_T donotdeletethisgain_mj; // '<S24>/do not delete this gain'
        real_T Product4_m;             // '<S2>/Product4'
        real_T Fcn_m;                  // '<S102>/Fcn'
        real_T Fcn1;                   // '<S102>/Fcn1'
        real_T Fcn_p;                  // '<S101>/Fcn'
        real_T Fcn1_h;                 // '<S101>/Fcn1'
        real_T Fcn_b;                  // '<S95>/Fcn'
        real_T Fcn1_j;                 // '<S95>/Fcn1'
        real_T Fcn_bn;                 // '<S94>/Fcn'
        real_T Fcn1_p;                 // '<S94>/Fcn1'
        real_T Gain1_e;                // '<S77>/Gain1'
        real_T phimd_g;                // '<S85>/sum2'
        real_T Lmd_sat;                // '<S81>/Lmd_sat'
        real_T MathFunction2;          // '<S84>/Math Function2'
        real_T Sum1_bq;                // '<S84>/Sum1'
        real_T MathFunction3;          // '<S84>/Math Function3'
        real_T phimd_n;                // '<S85>/Product4'
        real_T Phisat;                 // '<S81>/Abs'
        real_T Ifdsat;                 // '<S81>/1-D Lookup Table'
        real_T Switch_d;               // '<S81>/Switch'
        real_T Lmsatd;                 // '<S81>/Lmd'
        real_T Switch1_j;              // '<S79>/Switch1'
        real_T Switch2_a;              // '<S79>/Switch2'
        real_T Lmq_sat;                // '<S82>/Lmq_sat'
        real_T MathFunction;           // '<S86>/Math Function'
        real_T Sum2_d;                 // '<S86>/Sum2'
        real_T MathFunction1;          // '<S86>/Math Function1'
        real_T phimq_p;                // '<S87>/sum1'
        real_T phimq_i;                // '<S87>/Product3'
        real_T Phisat_a;               // '<S82>/Abs'
        real_T Ifqsat;                 // '<S82>/1-D Lookup Table'
        real_T Switch_b;               // '<S82>/Switch'
        real_T Lmsatq;                 // '<S82>/Lmq'
        real_T Product_i;              // '<S82>/Product'
        real_T Product_n;              // '<S81>/Product'
        real_T Fcn_j;                  // '<S64>/Fcn'
        real_T Fcn1_k;                 // '<S64>/Fcn1'
        real_T Fcn_jq;                 // '<S63>/Fcn'
        real_T Fcn1_j3;                // '<S63>/Fcn1'
        real_T Sum7;                   // '<S58>/Sum7'
        real_T Meanvalue;              // '<S58>/Product'
        real_T VariableTransportDelay_p;// '<S46>/Variable Transport Delay'
        real_T integrator_d;           // '<S46>/integrator'
        real_T Clock_c;                // '<S46>/Clock'
        real_T Switch_n;               // '<S46>/Switch'
        real_T VariableTransportDelay_e;// '<S47>/Variable Transport Delay'
        real_T integrator_de;          // '<S47>/integrator'
        real_T Clock_b;                // '<S47>/Clock'
        real_T Switch_m;               // '<S47>/Switch'
        real_T ComplextoMagnitudeAngle_o1;// '<S42>/Complex to Magnitude-Angle'
        real_T ComplextoMagnitudeAngle_o2;// '<S42>/Complex to Magnitude-Angle'
        real_T Saturation_a;           // '<S37>/Saturation'
        real_T MathFunction_c;         // '<S37>/Math Function'
        real_T Freq_o;                 // '<S46>/To avoid division  by zero'
        real_T period_i;               // '<S46>/period'
        real_T Freq_l;                 // '<S47>/To avoid division  by zero'
        real_T period_a;               // '<S47>/period'
        real_T Fcn_k;                  // '<S53>/Fcn'
        real_T Fcn1_pv;                // '<S53>/Fcn1'
        real_T Fcn_d;                  // '<S52>/Fcn'
        real_T Fcn1_d;                 // '<S52>/Fcn1'
        real_T Sum7_i;                 // '<S47>/Sum7'
        real_T Meanvalue_j;            // '<S47>/Product'
        real_T Sum7_h;                 // '<S46>/Sum7'
        real_T Meanvalue_o;            // '<S46>/Product'
        real_T Rotorangledthetae_DSTATE;// '<S67>/Rotor angle dthetae'
        real_T Integrator_DSTATE;      // '<S34>/Integrator'
        real_T dw_delay_DSTATE;        // '<S104>/dw_delay'
        real_T dw_predict_DSTATE;      // '<S104>/dw_predict'
        real_T theta_DSTATE;           // '<S67>/theta'
        real_T Rotorspeeddeviationdw_DSTATE;// '<S103>/Rotor spee<d deviation (dw)' 
        real_T Lmd_sat_DSTATE;         // '<S81>/Lmd_sat'
        real_T Lmq_sat_DSTATE;         // '<S82>/Lmq_sat'
        real_T Initial_FirstOutputTime;// '<S36>/Initial'
        real_T Memory_PreviousInput;   // '<S36>/Memory'
        real_T PrevY;                  // '<S36>/Rate Limiter'
        real_T LastMajorTime;          // '<S36>/Rate Limiter'
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
        } StateSpace_PWORK;            // '<S158>/State-Space'

        struct {
          void *TUbufferPtrs[3];
        } VariableTransportDelay_PWORK;// '<S58>/Variable Transport Delay'

        struct {
          void *TUbufferPtrs[3];
        } VariableTransportDelay_PWORK_g;// '<S46>/Variable Transport Delay'

        struct {
          void *TUbufferPtrs[3];
        } VariableTransportDelay_PWORK_j;// '<S47>/Variable Transport Delay'

        int32_T inversion_DWORK2[6];   // '<S88>/inversion'
        int_T StateSpace_IWORK[11];    // '<S158>/State-Space'
        struct {
          int_T Tail;
          int_T Head;
          int_T Last;
          int_T CircularBufSize;
        } VariableTransportDelay_IWORK;// '<S58>/Variable Transport Delay'

        struct {
          int_T Tail;
          int_T Head;
          int_T Last;
          int_T CircularBufSize;
        } VariableTransportDelay_IWORK_g;// '<S46>/Variable Transport Delay'

        struct {
          int_T Tail;
          int_T Head;
          int_T Last;
          int_T CircularBufSize;
        } VariableTransportDelay_IWORK_a;// '<S47>/Variable Transport Delay'

        int_T Integrator_IWORK;        // '<S36>/Integrator'
        uint32_T m_bpIndex;            // '<S81>/1-D Lookup Table'
        uint32_T m_bpIndex_i;          // '<S82>/1-D Lookup Table'
        int8_T Integrator_PrevResetState;// '<S34>/Integrator'
        int8_T Rotorspeeddeviationdw_PrevReset;// '<S103>/Rotor spee<d deviation (dw)' 
        uint8_T Compare;               // '<S99>/Compare'
        uint8_T Compare_n;             // '<S100>/Compare'
        uint8_T Compare_e;             // '<S61>/Compare'
        uint8_T Compare_j;             // '<S62>/Compare'
        uint8_T Compare_l;             // '<S92>/Compare'
        uint8_T Compare_ld;            // '<S93>/Compare'
        uint8_T Compare_p;             // '<S51>/Compare'
        uint8_T Compare_j0;            // '<S50>/Compare'
        uint8_T Rotorspeeddeviationdw_IC_LOADIN;// '<S103>/Rotor spee<d deviation (dw)' 
        boolean_T RelationalOperator;  // '<S36>/Relational Operator'
        boolean_T RelationalOperator_g;// '<S58>/Relational Operator'
        boolean_T RelationalOperator_h;// '<S46>/Relational Operator'
        boolean_T RelationalOperator_g1;// '<S47>/Relational Operator'
        boolean_T PrevLimited;         // '<S36>/Rate Limiter'
        boolean_T AutomaticGainControl_MODE;// '<S36>/Automatic Gain Control'
      };

      // Continuous states (default storage)
      struct X_sfc_6p_6p_topology_machine_T {
        real_T Integrator_CSTATE;      // '<S36>/Integrator'
        real_T Integrator_CSTATE_i;    // '<S38>/Integrator'
        real_T VariableTransportDelay_CSTATE;// '<S58>/Variable Transport Delay' 
        real_T integrator_CSTATE;      // '<S58>/integrator'
        real_T TransferFcn_CSTATE;     // '<S38>/Transfer Fcn'
        real_T Integrator_x1_CSTATE;   // '<S54>/Integrator_x1'
        real_T Integrator_x2_CSTATE;   // '<S54>/Integrator_x2'
        real_T VariableTransportDelay_CSTATE_c;// '<S46>/Variable Transport Delay' 
        real_T integrator_CSTATE_g;    // '<S46>/integrator'
        real_T VariableTransportDelay_CSTATE_l;// '<S47>/Variable Transport Delay' 
        real_T integrator_CSTATE_b;    // '<S47>/integrator'
      };

      // State derivatives (default storage)
      struct XDot_sfc_6p_6p_topology_machi_T {
        real_T Integrator_CSTATE;      // '<S36>/Integrator'
        real_T Integrator_CSTATE_i;    // '<S38>/Integrator'
        real_T VariableTransportDelay_CSTATE;// '<S58>/Variable Transport Delay' 
        real_T integrator_CSTATE;      // '<S58>/integrator'
        real_T TransferFcn_CSTATE;     // '<S38>/Transfer Fcn'
        real_T Integrator_x1_CSTATE;   // '<S54>/Integrator_x1'
        real_T Integrator_x2_CSTATE;   // '<S54>/Integrator_x2'
        real_T VariableTransportDelay_CSTATE_c;// '<S46>/Variable Transport Delay' 
        real_T integrator_CSTATE_g;    // '<S46>/integrator'
        real_T VariableTransportDelay_CSTATE_l;// '<S47>/Variable Transport Delay' 
        real_T integrator_CSTATE_b;    // '<S47>/integrator'
      };

      // State disabled
      struct XDis_sfc_6p_6p_topology_machi_T {
        boolean_T Integrator_CSTATE;   // '<S36>/Integrator'
        boolean_T Integrator_CSTATE_i; // '<S38>/Integrator'
        boolean_T VariableTransportDelay_CSTATE;// '<S58>/Variable Transport Delay' 
        boolean_T integrator_CSTATE;   // '<S58>/integrator'
        boolean_T TransferFcn_CSTATE;  // '<S38>/Transfer Fcn'
        boolean_T Integrator_x1_CSTATE;// '<S54>/Integrator_x1'
        boolean_T Integrator_x2_CSTATE;// '<S54>/Integrator_x2'
        boolean_T VariableTransportDelay_CSTATE_c;// '<S46>/Variable Transport Delay' 
        boolean_T integrator_CSTATE_g; // '<S46>/integrator'
        boolean_T VariableTransportDelay_CSTATE_l;// '<S47>/Variable Transport Delay' 
        boolean_T integrator_CSTATE_b; // '<S47>/integrator'
      };

      // Zero-crossing (trigger) state
      struct PrevZCX_sfc_6p_6p_topology_ma_T {
        ZCSigState Integrator_Reset_ZCE;// '<S36>/Integrator'
      };

      // External inputs (root inport signals with default storage)
      struct ExtU_sfc_6p_6p_topology_machi_T {
        real_T pulses_in_grid[6];      // '<Root>/pulses_in_grid'
        real_T pulses_in_motor[6];     // '<Root>/pulses_in_motor'
        real_T voltage_magnitude_a;    // '<Root>/voltage_magnitude_a'
        real_T voltage_magnitude_b;    // '<Root>/voltage_magnitude_b'
        real_T voltage_magnitude_c;    // '<Root>/voltage_magnitude_c'
        real_T voltage_frequency;      // '<Root>/voltage_frequency'
        real_T voltage_excitation;     // '<Root>/voltage_excitation'
        real_T load_coefficients[3];   // '<Root>/load_coefficients'
        real_T grid_pt_ratio;          // '<Root>/grid_pt_ratio'
        real_T grid_ct_ratio;          // '<Root>/grid_ct_ratio'
        real_T forced_speed_pu;        // '<Root>/forced_speed_pu'
        real_T enable_forced_speed;    // '<Root>/enable_forced_speed'
        real_T close_dc_isolator;      // '<Root>/close_dc_isolator'
        real_T motor_pt_ratio;         // '<Root>/motor_pt_ratio'
        real_T motor_ct_ratio;         // '<Root>/motor_ct_ratio'
      };

      // External outputs (root outports fed by signals with default storage)
      struct ExtY_sfc_6p_6p_topology_machi_T {
        real_T Udc;                    // '<Root>/Udc'
        real_T Idc;                    // '<Root>/Idc'
        real_T Iph_grid[3];            // '<Root>/Iph_grid'
        real_T Uph_ph_grid[3];         // '<Root>/Uph_ph_grid'
        real_T wt;                     // '<Root>/wt'
        real_T Uph_n_source[3];        // '<Root>/Uph_n_source'
        real_T Iph_source[3];          // '<Root>/Iph_source'
        real_T Iph_motor[3];           // '<Root>/Iph_motor'
        real_T Uph_ph_motor[3];        // '<Root>/Uph_ph_motor'
        real_T machine_speed;          // '<Root>/machine_speed'
        real_T machine_electrical_torque;// '<Root>/machine_electrical_torque'
        real_T machine_mechanical_angle;// '<Root>/machine_mechanical_angle'
        real_T machine_load;           // '<Root>/machine_load'
        real_T machine_electrical_power;// '<Root>/machine_electrical_power'
        real_T Udc_machine;            // '<Root>/Udc_machine'
        real_T machine_flux_d;         // '<Root>/machine_flux_d'
        real_T machine_flux_q;         // '<Root>/machine_flux_q'
        real_T machine_stator_current_d;// '<Root>/machine_stator_current_d'
        real_T machine_stator_current_q;// '<Root>/machine_stator_current_q'
        real_T machine_field_current_d;// '<Root>/machine_field_current_d'
        real_T machine_damper_current_q1;// '<Root>/machine_damper_current_q1'
        real_T machine_damper_current_q2;// '<Root>/machine_damper_current_q2'
        real_T machine_damper_current_d;// '<Root>/machine_damper_current_d'
        real_T machine_stator_voltage_q;// '<Root>/machine_stator_voltage_q'
        real_T machine_stator_voltage_d;// '<Root>/machine_stator_voltage_d'
        real_T machine_phase_current_a;// '<Root>/machine_phase_current_a'
        real_T machine_phase_current_b;// '<Root>/machine_phase_current_b'
        real_T machine_phase_current_c;// '<Root>/machine_phase_current_c'
        real_T source_voltage_phase;   // '<Root>/source_voltage_phase'
      };

      // Parameters for system: '<S48>/Subsystem - pi//2 delay'
      struct P_Subsystempi2delay_sfc_6p_6p_T {
        real_T dq_Y0[2];               // Expression: [0,0]
                                          //  Referenced by: '<S52>/dq'

      };

      // Parameters for system: '<S48>/Subsystem1'
      struct P_Subsystem1_sfc_6p_6p_topolo_T {
        real_T dq_Y0[2];               // Expression: [0,0]
                                          //  Referenced by: '<S53>/dq'

      };

      // Parameters (default storage)
      struct P_sfc_6p_6p_topology_machine_T {
        real_T AlphaBetaZerotodq0_Alignment;
                                 // Mask Parameter: AlphaBetaZerotodq0_Alignment
                                    //  Referenced by: '<S48>/Constant'

        real_T dq0toAlphaBetaZero_Alignment;
                                 // Mask Parameter: dq0toAlphaBetaZero_Alignment
                                    //  Referenced by: '<S98>/Constant'

        real_T AlphaBetaZerotodq0_Alignment_n;
                               // Mask Parameter: AlphaBetaZerotodq0_Alignment_n
                                  //  Referenced by: '<S59>/Constant'

        real_T AlphaBetaZerotodq0_Alignment_b;
                               // Mask Parameter: AlphaBetaZerotodq0_Alignment_b
                                  //  Referenced by: '<S90>/Constant'

        real_T Continuous_Init;        // Mask Parameter: Continuous_Init
                                          //  Referenced by: '<S38>/Integrator'

        real_T Continuous_Kd;          // Mask Parameter: Continuous_Kd
                                          //  Referenced by: '<S38>/Kp6'

        real_T Continuous_Ki;          // Mask Parameter: Continuous_Ki
                                          //  Referenced by: '<S38>/Kp5'

        real_T Continuous_Kp;          // Mask Parameter: Continuous_Kp
                                          //  Referenced by: '<S38>/Kp4'

        real_T Continuous_Par_Limits[2];// Mask Parameter: Continuous_Par_Limits
                                           //  Referenced by:
                                           //    '<S38>/Integrator'
                                           //    '<S38>/Saturation2'

        real_T CompareToConstant1_const;
                                     // Mask Parameter: CompareToConstant1_const
                                        //  Referenced by: '<S51>/Constant'

        real_T CompareToConstant_const;
                                      // Mask Parameter: CompareToConstant_const
                                         //  Referenced by: '<S50>/Constant'

        real_T CompareToConstant_const_b;
                                    // Mask Parameter: CompareToConstant_const_b
                                       //  Referenced by: '<S99>/Constant'

        real_T CompareToConstant1_const_d;
                                   // Mask Parameter: CompareToConstant1_const_d
                                      //  Referenced by: '<S100>/Constant'

        real_T CompareToConstant_const_c;
                                    // Mask Parameter: CompareToConstant_const_c
                                       //  Referenced by: '<S61>/Constant'

        real_T CompareToConstant1_const_g;
                                   // Mask Parameter: CompareToConstant1_const_g
                                      //  Referenced by: '<S62>/Constant'

        real_T CompareToConstant_const_d;
                                    // Mask Parameter: CompareToConstant_const_d
                                       //  Referenced by: '<S92>/Constant'

        real_T CompareToConstant1_const_o;
                                   // Mask Parameter: CompareToConstant1_const_o
                                      //  Referenced by: '<S93>/Constant'

        real_T Gain_Y0;                // Expression: [1]
                                          //  Referenced by: '<S37>/Gain'

        real_T Gain3_Gain[9];
        // Expression: [ 1   -1/2   -1/2; 0   sqrt(3)/2   -sqrt(3)/2; 1/2  1/2  1/2 ]
           //  Referenced by: '<S49>/Gain3'

        real_T Gain1_Gain;             // Expression: 2/3
                                          //  Referenced by: '<S49>/Gain1'

        real_T VariableTransportDelay_MaxDelay;// Expression: 1/sps.Fmin+eps
                                                  //  Referenced by: '<S46>/Variable Transport Delay'

        real_T VariableTransportDelay_InitOutp;// Expression: 0
                                                  //  Referenced by: '<S46>/Variable Transport Delay'

        real_T integrator_IC;          // Expression: 0
                                          //  Referenced by: '<S46>/integrator'

        real_T Constant_Value;         // Expression: 1/sps.Finit
                                          //  Referenced by: '<S46>/Constant'

        real_T Constant1_Value;        // Expression: sps.Vinit
                                          //  Referenced by: '<S46>/Constant1'

        real_T VariableTransportDelay_MaxDel_i;// Expression: 1/sps.Fmin+eps
                                                  //  Referenced by: '<S47>/Variable Transport Delay'

        real_T VariableTransportDelay_InitOu_g;// Expression: 0
                                                  //  Referenced by: '<S47>/Variable Transport Delay'

        real_T integrator_IC_m;        // Expression: 0
                                          //  Referenced by: '<S47>/integrator'

        real_T Constant_Value_p;       // Expression: 1/sps.Finit
                                          //  Referenced by: '<S47>/Constant'

        real_T Constant1_Value_g;      // Expression: sps.Vinit
                                          //  Referenced by: '<S47>/Constant1'

        real_T Saturation_UpperSat;    // Expression: inf
                                          //  Referenced by: '<S37>/Saturation'

        real_T Saturation_LowerSat;    // Expression: eps
                                          //  Referenced by: '<S37>/Saturation'

        real_T Toavoiddivisionbyzero_UpperSat;// Expression: 1e6
                                                 //  Referenced by: '<S46>/To avoid division  by zero'

        real_T Toavoiddivisionbyzero_LowerSat;// Expression: eps
                                                 //  Referenced by: '<S46>/To avoid division  by zero'

        real_T Toavoiddivisionbyzero_UpperSa_h;// Expression: 1e6
                                                  //  Referenced by: '<S47>/To avoid division  by zero'

        real_T Toavoiddivisionbyzero_LowerSa_l;// Expression: eps
                                                  //  Referenced by: '<S47>/To avoid division  by zero'

        real_T Constant2_Value;        // Expression: 0
                                          //  Referenced by: '<S68>/Constant2'

        real_T Constant1_Value_c;      // Expression: 1
                                          //  Referenced by: '<S81>/Constant1'

        real_T Constant1_Value_i;      // Expression: 1
                                          //  Referenced by: '<S82>/Constant1'

        real_T uLl_q_Gain[3];          // Expression: SM.One_Llq
                                          //  Referenced by: '<S87>/1//Ll_q'

        real_T u2_Value[3];      // Expression: [ 1/SM.Ll 1/SM.Llkq1 1/SM.Llkq2]
                                    //  Referenced by: '<S86>/u2'

        real_T Lmq_sat_InitialCondition;// Expression: SM.Lmsatq(1)
                                           //  Referenced by: '<S82>/Lmq_sat'

        real_T uDLookupTable_tableData[2];
                  // Expression: [ 0 SM.Phisat(2:end)./SM.Lmsatq(2:end)*SM.Lmq ]
                     //  Referenced by: '<S82>/1-D Lookup Table'

        real_T uDLookupTable_bp01Data[2];// Expression: SM.Phisat
                                            //  Referenced by: '<S82>/1-D Lookup Table'

        real_T Lmq_Gain;               // Expression: SM.Lmq
                                          //  Referenced by: '<S82>/Lmq'

        real_T u3_Value;               // Expression: SM.Lmq
                                          //  Referenced by: '<S79>/u3'

        real_T u2_Value_n;             // Expression: SM.Laq
                                          //  Referenced by: '<S79>/u2'

        real_T u1_Value[36];           // Expression: SM.R
                                          //  Referenced by: '<S79>/u1'

        real_T uLl_d_Gain[3];
        // Expression: [ 1/SM.Ll   SM.Llkd/(SM.Llfd*SM.Llkd+(SM.Llfd+SM.Llkd)*SM.LC)   SM.Llfd/(SM.Llfd*SM.Llkd+(SM.Llfd+SM.Llkd)*SM.LC)]
           //  Referenced by: '<S85>/1//Ll_d'

        real_T u1_Value_e[2];
            // Expression: [1/SM.Ll 1/(SM.Llkd*SM.Llfd/(SM.Llkd+SM.Llfd)+SM.LC)]
               //  Referenced by: '<S84>/u1'

        real_T Lmd_sat_InitialCondition;// Expression: SM.Lmsatd(1)
                                           //  Referenced by: '<S81>/Lmd_sat'

        real_T uDLookupTable_tableData_a[2];
                  // Expression: [ 0 SM.Phisat(2:end)./SM.Lmsatd(2:end)*SM.Lmd ]
                     //  Referenced by: '<S81>/1-D Lookup Table'

        real_T uDLookupTable_bp01Data_i[2];// Expression: SM.Phisat
                                              //  Referenced by: '<S81>/1-D Lookup Table'

        real_T Lmd_Gain;               // Expression: SM.Lmd
                                          //  Referenced by: '<S81>/Lmd'

        real_T u1_Value_l[36];         // Expression: zeros(SM.nState,SM.nState)
                                          //  Referenced by: '<S83>/u1'

        real_T u5_Value[36];           // Expression: SM.Llqd
                                          //  Referenced by: '<S83>/u5'

        real_T Lmqd_nosat_Value[2];    // Expression: [SM.Lmq SM.Lmd]
                                          //  Referenced by: '<S69>/Lmqd_nosat'

        real_T Laqd_nosat_Value[2];    // Expression: [SM.Laq  SM.Lad]
                                          //  Referenced by: '<S69>/Laqd_nosat'

        real_T Constant4_Value[36];    // Expression: SM.Linv
                                          //  Referenced by: '<S69>/Constant4'

        real_T Constant6_Value[36];    // Expression: SM.RLinv
                                          //  Referenced by: '<S69>/Constant6'

        real_T Constant2_Value_i;      // Expression: SM.Sat
                                          //  Referenced by: '<S69>/Constant2'

        real_T Switch1_Threshold;      // Expression: 0.5
                                          //  Referenced by: '<S69>/Switch1'

        real_T u1_Value_p[36];         // Expression: zeros(SM.nState,SM.nState)
                                          //  Referenced by: '<S77>/u1'

        real_T Gain1_Gain_g;           // Expression: -1
                                          //  Referenced by: '<S77>/Gain1'

        real_T wbaseTs2_Gain;          // Expression: SM.web*Ts/2
                                          //  Referenced by: '<S88>/wbase*Ts//2'

        real_T u5_Value_o[36];         // Expression: eye(SM.nState,SM.nState)
                                          //  Referenced by: '<S88>/u5'

        real_T wbaseTs2_Gain_g;        // Expression: SM.web*Ts/2
                                          //  Referenced by: '<S88>/wbase*Ts//2 '

        real_T dq_Y0[2];               // Expression: [0,0]
                                          //  Referenced by: '<S94>/dq'

        real_T dq_Y0_m[2];             // Expression: [0,0]
                                          //  Referenced by: '<S95>/dq'

        real_T alpha_beta_Y0[2];       // Expression: [0,0]
                                          //  Referenced by: '<S101>/alpha_beta'

        real_T alpha_beta_Y0_l[2];     // Expression: [0,0]
                                          //  Referenced by: '<S102>/alpha_beta'

        real_T SwitchCurrents_Value[12];// Expression: zeros(12,1)
                                           //  Referenced by: '<S160>/SwitchCurrents'

        real_T fluxes_InitialCondition[6];// Expression: SM.phiqd0
                                             //  Referenced by: '<S80>/fluxes'

        real_T Constant1_Value_l;      // Expression: SM.Sat
                                          //  Referenced by: '<S69>/Constant1'

        real_T Constant3_Value;        // Expression: SM.Sat
                                          //  Referenced by: '<S69>/Constant3'

        real_T Switch_Threshold;       // Expression: 0.5
                                          //  Referenced by: '<S69>/Switch'

        real_T changeIqIdcurrentsigns_Gain[6];// Expression: SM.IqdSign
                                                 //  Referenced by: '<S69>/change Iq Id  current signs'

        real_T Rotorangledthetae_gainval;
                                // Computed Parameter: Rotorangledthetae_gainval
                                   //  Referenced by: '<S67>/Rotor angle dthetae'

        real_T Rotorangledthetae_IC;   // Expression: SM.tho
                                          //  Referenced by: '<S67>/Rotor angle dthetae'

        real_T web2_Gain;              // Expression: SM.web
                                          //  Referenced by: '<S67>/web2'

        real_T Constant_Value_k;       // Expression: 0
                                          //  Referenced by: '<S71>/Constant'

        real_T Gain3_Gain_l[9];
          // Expression: [ 1   0   1; -1/2  sqrt(3)/2   1; -1/2  -sqrt(3)/2  1 ]
             //  Referenced by: '<S97>/Gain3'

        real_T ib_Gain;                // Expression: SM.ib
                                          //  Referenced by: '<S71>/ib'

        real_T Constant4_Value_d;      // Expression: 0
                                          //  Referenced by: '<S2>/Constant4'

        real_T Integrator_gainval;     // Computed Parameter: Integrator_gainval
                                          //  Referenced by: '<S34>/Integrator'

        real_T Integrator_UpperSat;    // Expression: antiwindupUpperLimit
                                          //  Referenced by: '<S34>/Integrator'

        real_T Integrator_LowerSat;    // Expression: antiwindupLowerLimit
                                          //  Referenced by: '<S34>/Integrator'

        real_T Saturation_UpperSat_e;  // Expression: windupUpperLimit
                                          //  Referenced by: '<S34>/Saturation'

        real_T Saturation_LowerSat_f;  // Expression: windupLowerLimit
                                          //  Referenced by: '<S34>/Saturation'

        real_T Constant5_Value;        // Expression: pi
                                          //  Referenced by: '<S29>/Constant5'

        real_T Constant6_Value_k;      // Expression: 2*pi
                                          //  Referenced by: '<S29>/Constant6'

        real_T Constant1_Value_e;      // Expression: sim_source_phase_a_deg
                                          //  Referenced by: '<Root>/Constant1'

        real_T Constant2_Value_e;      // Expression: sim_source_phase_b_deg
                                          //  Referenced by: '<Root>/Constant2'

        real_T Constant3_Value_k;      // Expression: sim_source_phase_c_deg
                                          //  Referenced by: '<Root>/Constant3'

        real_T Gain2_Gain;             // Expression: pi/180
                                          //  Referenced by: '<S2>/Gain2'

        real_T Gain1_Gain_j;           // Expression: sqrt(2)
                                          //  Referenced by: '<S2>/Gain1'

        real_T StateSpace_AS_param[441];// Expression: S.A
                                           //  Referenced by: '<S158>/State-Space'

        real_T StateSpace_BS_param[357];// Expression: S.B
                                           //  Referenced by: '<S158>/State-Space'

        real_T StateSpace_CS_param[735];// Expression: S.C
                                           //  Referenced by: '<S158>/State-Space'

        real_T StateSpace_DS_param[595];// Expression: S.D
                                           //  Referenced by: '<S158>/State-Space'

        real_T StateSpace_X0_param[21];// Expression: S.x0
                                          //  Referenced by: '<S158>/State-Space'

        real_T donotdeletethisgain_Gain;// Expression: 1
                                           //  Referenced by: '<S25>/do not delete this gain'

        real_T donotdeletethisgain_Gain_d;// Expression: 1
                                             //  Referenced by: '<S3>/do not delete this gain'

        real_T donotdeletethisgain_Gain_i;// Expression: 1
                                             //  Referenced by: '<S127>/do not delete this gain'

        real_T donotdeletethisgain_Gain_o;// Expression: 1
                                             //  Referenced by: '<S128>/do not delete this gain'

        real_T donotdeletethisgain_Gain_m;// Expression: 1
                                             //  Referenced by: '<S129>/do not delete this gain'

        real_T Kv_Gain;                // Expression: Ki
                                          //  Referenced by: '<S20>/Kv'

        real_T donotdeletethisgain_Gain_k;// Expression: 1
                                             //  Referenced by: '<S130>/do not delete this gain'

        real_T donotdeletethisgain_Gain_h;// Expression: 1
                                             //  Referenced by: '<S131>/do not delete this gain'

        real_T donotdeletethisgain_Gain_n;// Expression: 1
                                             //  Referenced by: '<S132>/do not delete this gain'

        real_T Kv1_Gain;               // Expression: Kv
                                          //  Referenced by: '<S20>/Kv1'

        real_T Constant2_Value_ez;     // Expression: 2*pi
                                          //  Referenced by: '<S36>/Constant2'

        real_T Initial_Value;          // Expression: sps.Phase_Init*pi/180
                                          //  Referenced by: '<S36>/Initial'

        real_T Integrator_UpperSat_l;  // Expression: inf
                                          //  Referenced by: '<S36>/Integrator'

        real_T Integrator_LowerSat_n;  // Expression: -inf
                                          //  Referenced by: '<S36>/Integrator'

        real_T HitCrossing_Offset;     // Expression: pi
                                          //  Referenced by: '<S36>/Hit  Crossing'

        real_T Memory_InitialCondition;// Expression: sps.Finit
                                          //  Referenced by: '<S36>/Memory'

        real_T donotdeletethisgain_Gain_m0;// Expression: 1
                                              //  Referenced by: '<S145>/do not delete this gain'

        real_T donotdeletethisgain_Gain_ip;// Expression: 1
                                              //  Referenced by: '<S146>/do not delete this gain'

        real_T donotdeletethisgain_Gain_c;// Expression: 1
                                             //  Referenced by: '<S147>/do not delete this gain'

        real_T Kv1_Gain_a;             // Expression: Kv
                                          //  Referenced by: '<S21>/Kv1'

        real_T Constant1_Value_a;      // Expression: sps.AGC
                                          //  Referenced by: '<S36>/Constant1'

        real_T VariableTransportDelay_MaxDel_l;// Expression: 1/sps.Fmin+eps
                                                  //  Referenced by: '<S58>/Variable Transport Delay'

        real_T VariableTransportDelay_InitOu_b;// Expression: 0
                                                  //  Referenced by: '<S58>/Variable Transport Delay'

        real_T integrator_IC_n;        // Expression: 0
                                          //  Referenced by: '<S58>/integrator'

        real_T Constant_Value_f;       // Expression: 1/sps.Finit
                                          //  Referenced by: '<S58>/Constant'

        real_T Constant1_Value_ij;     // Expression: sps.Vinit
                                          //  Referenced by: '<S58>/Constant1'

        real_T TransferFcn_A;          // Computed Parameter: TransferFcn_A
                                          //  Referenced by: '<S38>/Transfer Fcn'

        real_T TransferFcn_C;          // Computed Parameter: TransferFcn_C
                                          //  Referenced by: '<S38>/Transfer Fcn'

        real_T TransferFcn_D;          // Computed Parameter: TransferFcn_D
                                          //  Referenced by: '<S38>/Transfer Fcn'

        real_T Gain10_Gain;            // Expression: 1/2/pi
                                          //  Referenced by: '<S36>/Gain10'

        real_T RateLimiter_RisingLim;  // Expression: sps.MaxRateChangeFreq
                                          //  Referenced by: '<S36>/Rate Limiter'

        real_T RateLimiter_FallingLim; // Expression: -sps.MaxRateChangeFreq
                                          //  Referenced by: '<S36>/Rate Limiter'

        real_T Integrator_x1_IC;       // Expression: sps.x0(1,:)
                                          //  Referenced by: '<S54>/Integrator_x1'

        real_T A11_Gain;               // Expression: sps.A11
                                          //  Referenced by: '<S55>/A11'

        real_T Integrator_x2_IC;       // Expression: sps.x0(2,:)
                                          //  Referenced by: '<S54>/Integrator_x2'

        real_T A12_Gain;               // Expression: sps.A12
                                          //  Referenced by: '<S55>/A12'

        real_T A21_Gain;               // Expression: sps.A21
                                          //  Referenced by: '<S55>/A21'

        real_T A22_Gain;               // Expression: sps.A22
                                          //  Referenced by: '<S55>/A22'

        real_T B11_Gain;               // Expression: sps.B11
                                          //  Referenced by: '<S56>/B11'

        real_T B21_Gain;               // Expression: sps.B21
                                          //  Referenced by: '<S56>/B21'

        real_T C11_Gain;               // Expression: sps.C11
                                          //  Referenced by: '<S57>/C11'

        real_T C12_Gain;               // Expression: sps.C12
                                          //  Referenced by: '<S57>/C12'

        real_T Du_Gain;                // Expression: sps.D
                                          //  Referenced by: '<S54>/D*u'

        real_T Toavoiddivisionbyzero_UpperSa_o;// Expression: 1e6
                                                  //  Referenced by: '<S58>/To avoid division  by zero'

        real_T Toavoiddivisionbyzero_LowerSa_n;// Expression: eps
                                                  //  Referenced by: '<S58>/To avoid division  by zero'

        real_T Gain3_Gain_j[9];
        // Expression: [ 1   -1/2   -1/2; 0   sqrt(3)/2   -sqrt(3)/2; 1/2  1/2  1/2 ]
           //  Referenced by: '<S60>/Gain3'

        real_T Gain1_Gain_o;           // Expression: 2/3
                                          //  Referenced by: '<S60>/Gain1'

        real_T donotdeletethisgain_Gain_b;// Expression: 1
                                             //  Referenced by: '<S142>/do not delete this gain'

        real_T donotdeletethisgain_Gain_km;// Expression: 1
                                              //  Referenced by: '<S143>/do not delete this gain'

        real_T donotdeletethisgain_Gain_p;// Expression: 1
                                             //  Referenced by: '<S144>/do not delete this gain'

        real_T Kv_Gain_e;              // Expression: Ki
                                          //  Referenced by: '<S21>/Kv'

        real_T donotdeletethisgain_Gain_f;// Expression: 1
                                             //  Referenced by: '<S112>/do not delete this gain'

        real_T donotdeletethisgain_Gain_hv;// Expression: 1
                                              //  Referenced by: '<S113>/do not delete this gain'

        real_T donotdeletethisgain_Gain_j;// Expression: 1
                                             //  Referenced by: '<S114>/do not delete this gain'

        real_T Kv_Gain_ej;             // Expression: Ki
                                          //  Referenced by: '<S19>/Kv'

        real_T donotdeletethisgain_Gain_fx;// Expression: 1
                                              //  Referenced by: '<S115>/do not delete this gain'

        real_T donotdeletethisgain_Gain_ka;// Expression: 1
                                              //  Referenced by: '<S116>/do not delete this gain'

        real_T donotdeletethisgain_Gain_fm;// Expression: 1
                                              //  Referenced by: '<S117>/do not delete this gain'

        real_T Kv1_Gain_k;             // Expression: Kv
                                          //  Referenced by: '<S19>/Kv1'

        real_T nominalspeed_Value;     // Expression: 1
                                          //  Referenced by: '<S67>/nominal speed'

        real_T dw_delay_InitialCondition;// Expression: SM.dwo
                                            //  Referenced by: '<S104>/dw_delay'

        real_T F2_Gain;                // Expression: 2
                                          //  Referenced by: '<S104>/F2'

        real_T dw_predict_InitialCondition;// Expression: SM.dwo
                                              //  Referenced by: '<S104>/dw_predict'

        real_T Switch_Threshold_k;     // Expression: 0
                                          //  Referenced by: '<S67>/Switch'

        real_T units_Gain;             // Expression: SM.Nb
                                          //  Referenced by: '<S67>/units'

        real_T u1_Gain[2];             // Expression: [1 -1]
                                          //  Referenced by: '<S76>/1-1'

        real_T units2_Gain;            // Expression: SM.Pb/SM.Nb
                                          //  Referenced by: '<S67>/units2'

        real_T theta_gainval;          // Computed Parameter: theta_gainval
                                          //  Referenced by: '<S67>/theta'

        real_T theta_IC;               // Expression: SM.tho/SM.p
                                          //  Referenced by: '<S67>/theta'

        real_T t_Gain;                 // Expression: 180/pi
                                          //  Referenced by: '<S67>/t'

        real_T units1_Gain;            // Expression: SM.Pb
                                          //  Referenced by: '<S67>/units1'

        real_T uLl_q_Gain_h[3];        // Expression: SM.One_Llq
                                          //  Referenced by: '<S78>/1//Ll_q'

        real_T Constant8_Value;        // Expression: SM.Sat
                                          //  Referenced by: '<S69>/Constant8'

        real_T Switch3_Threshold;      // Expression: 0.5
                                          //  Referenced by: '<S69>/Switch3'

        real_T uLl_d_Gain_p[3];
        // Expression: [ 1/SM.Ll   SM.Llkd/(SM.Llfd*SM.Llkd+(SM.Llfd+SM.Llkd)*SM.LC)   SM.Llfd/(SM.Llfd*SM.Llkd+(SM.Llfd+SM.Llkd)*SM.LC)]
           //  Referenced by: '<S78>/1//Ll_d'

        real_T u_Vb_Gain;              // Expression: 1/SM.Vb
                                          //  Referenced by: '<S70>/1_Vb'

        real_T Gain_Gain;              // Expression: 1/3
                                          //  Referenced by: '<S70>/Gain'

        real_T Gain3_Gain_o[9];
        // Expression: [ 1   -1/2   -1/2; 0   sqrt(3)/2   -sqrt(3)/2; 1/2  1/2  1/2 ]
           //  Referenced by: '<S91>/Gain3'

        real_T Gain1_Gain_gd;          // Expression: 2/3
                                          //  Referenced by: '<S91>/Gain1'

        real_T Constant5_Value_c;      // Expression: SM.Sat
                                          //  Referenced by: '<S69>/Constant5'

        real_T Switch2_Threshold;      // Expression: 0.5
                                          //  Referenced by: '<S69>/Switch2'

        real_T Gain_Gain_d;            // Expression: 180/pi
                                          //  Referenced by: '<S75>/Gain'

        real_T Gain_Gain_m;            // Expression: SM.Gain1
                                          //  Referenced by: '<S73>/Gain'

        real_T Gain1_Gain_f;           // Expression: SM.Gain1
                                          //  Referenced by: '<S73>/Gain1'

        real_T outputformatting_Gain[18];
        // Expression: [SM.Ib2*ones(1,5),SM.N2,SM.Ib2,SM.Ib2,SM.Ib2,SM.phib,SM.phib,SM.Vb2,SM.Vb2,(SM.phib/SM.Ib2)*ones(1,2),1,1,1]
           //  Referenced by: '<S68>/output formatting'

        real_T Constant6_Value_m;      // Expression: 0
                                          //  Referenced by: '<Root>/Constant6'

        real_T N_Gain;                 // Expression: SM.N
                                          //  Referenced by: '<S65>/N'

        real_T Vkd0Vkq10Vkq20_Value[3];// Expression: zeros(1, SM.nState-3)
                                          //  Referenced by: '<S65>/[ Vkd =0 Vkq1=0  Vkq2=0 ]'

        real_T voltages_InitialCondition;// Expression: 0
                                            //  Referenced by: '<S80>/voltages'

        real_T IC_Threshold;           // Expression: Ts
                                          //  Referenced by: '<S80>/IC'

        real_T u_Pb_Gain;              // Expression: 1/SM.Pb
                                          //  Referenced by: '<S67>/1_Pb'

        real_T F_Gain;                 // Expression: SM.F
                                          //  Referenced by: '<S67>/F'

        real_T u_Pb1_Gain;             // Expression: SM.Nb/SM.Pb
                                          //  Referenced by: '<S67>/1_Pb1'

        real_T u2H_Gain;               // Expression: 1/(2*SM.H)
                                          //  Referenced by: '<S67>/1 ----- 2H'

        real_T Rotorspeeddeviationdw_gainval;
                            // Computed Parameter: Rotorspeeddeviationdw_gainval
                               //  Referenced by: '<S103>/Rotor spee<d deviation (dw)'

        real_T webase_Gain;            // Expression: SM.web
                                          //  Referenced by: '<S67>/we base'

        real_T web3_Gain;              // Expression: SM.web/SM.p
                                          //  Referenced by: '<S67>/web3'

        real_T donotdeletethisgain_Gain_on;// Expression: 1
                                              //  Referenced by: '<S24>/do not delete this gain'

        real_T Constant9_Value;        // Expression: 2*pi
                                          //  Referenced by: '<S2>/Constant9'

        boolean_T Constant2_Value_io;  // Expression: SM.nState==6
                                          //  Referenced by: '<S79>/Constant2'

        boolean_T Constant1_Value_n;   // Expression: SM.nState==6
                                          //  Referenced by: '<S79>/Constant1'

        boolean_T Constant3_Value_p;   // Expression: SM.nState==6
                                          //  Referenced by: '<S79>/Constant3'

        boolean_T Constant1_Value_cr;  // Expression: SM.nState==6
                                          //  Referenced by: '<S68>/Constant1'

        P_Subsystem1_sfc_6p_6p_topolo_T Subsystem1;// '<S59>/Subsystem1'
        P_Subsystempi2delay_sfc_6p_6p_T Subsystempi2delay;// '<S59>/Subsystem - pi//2 delay' 
        P_Subsystem1_sfc_6p_6p_topolo_T Subsystem1_ka;// '<S48>/Subsystem1'
        P_Subsystempi2delay_sfc_6p_6p_T Subsystempi2delay_k;// '<S48>/Subsystem - pi//2 delay' 
      };

      // Real-time Model Data Structure
      struct RT_MODEL_sfc_6p_6p_topology_m_T {
        const char_T *errorStatus;
        RTWSolverInfo solverInfo;
        X_sfc_6p_6p_topology_machine_T *contStates;
        int_T *periodicContStateIndices;
        real_T *periodicContStateRanges;
        real_T *derivs;
        XDis_sfc_6p_6p_topology_machi_T *contStateDisabled;
        boolean_T zCCacheNeedsReset;
        boolean_T derivCacheNeedsReset;
        boolean_T CTOutputIncnstWithState;
        real_T odeF[1][11];
        ODE1_IntgData intgData;

        //
        //  Sizes:
        //  The following substructure contains sizes information
        //  for many of the model attributes such as inputs, outputs,
        //  dwork, sample times, etc.

        struct {
          int_T numContStates;
          int_T numPeriodicContStates;
          int_T numSampTimes;
        } Sizes;

        //
        //  Timing:
        //  The following substructure contains information regarding
        //  the timing information for the model.

        struct {
          uint32_T clockTick0;
          uint32_T clockTickH0;
          time_T stepSize0;
          uint32_T clockTick1;
          uint32_T clockTickH1;
          boolean_T firstInitCondFlag;
          time_T tStart;
          SimTimeStep simTimeStep;
          boolean_T stopRequestedFlag;
          time_T *t;
          time_T tArray[2];
        } Timing;
      };

      // Copy Constructor
      sfc_6p_6p_topology_machine(sfc_6p_6p_topology_machine const&) = delete;

      // Assignment Operator
      sfc_6p_6p_topology_machine& operator= (sfc_6p_6p_topology_machine const&)
        & = delete;

      // Move Constructor
      sfc_6p_6p_topology_machine(sfc_6p_6p_topology_machine &&) = delete;

      // Move Assignment Operator
      sfc_6p_6p_topology_machine& operator= (sfc_6p_6p_topology_machine &&) =
        delete;

      // Real-Time Model get method
      sfc_6p_6p_topology_machine::RT_MODEL_sfc_6p_6p_topology_m_T * getRTM();

      // Root inports set method
      void setExternalInputs(const ExtU_sfc_6p_6p_topology_machi_T
        *pExtU_sfc_6p_6p_topology_machi_T)
      {
        sfc_6p_6p_topology_machine_U = *pExtU_sfc_6p_6p_topology_machi_T;
      }

      // Root outports get method
      const ExtY_sfc_6p_6p_topology_machi_T &getExternalOutputs() const
      {
        return sfc_6p_6p_topology_machine_Y;
      }

      // model initialize function
      void initialize();

      // model step function
      void step();

      // model terminate function
      void terminate();

      // Constructor
      sfc_6p_6p_topology_machine();

      // Destructor
      ~sfc_6p_6p_topology_machine();

      // private data and function members
     private:
      // External inputs
      ExtU_sfc_6p_6p_topology_machi_T sfc_6p_6p_topology_machine_U;

      // External outputs
      ExtY_sfc_6p_6p_topology_machi_T sfc_6p_6p_topology_machine_Y;

      // Block states
      DW_sfc_6p_6p_topology_machine_T sfc_6p_6p_topology_machine_DW;

      // Tunable parameters
      static P_sfc_6p_6p_topology_machine_T sfc_6p_6p_topology_machine_P;

      // Block continuous states
      X_sfc_6p_6p_topology_machine_T sfc_6p_6p_topology_machine_X;

      // Block Continuous state disabled vector
      XDis_sfc_6p_6p_topology_machi_T sfc_6p_6p_topology_machine_XDis;

      // Triggered events
      PrevZCX_sfc_6p_6p_topology_ma_T sfc_6p_6p_topology_mach_PrevZCX;

      // private member function(s) for subsystem '<S48>/Subsystem - pi//2 delay'
      static void sfc_6p_6_Subsystempi2delay_Init(real_T *rty_dq, real_T
        *rty_dq_m, P_Subsystempi2delay_sfc_6p_6p_T *localP);
      static void sfc_6_Subsystempi2delay_Disable
        (DW_Subsystempi2delay_sfc_6p_6_T *localDW);
      void sfc_6p_6p_top_Subsystempi2delay(uint8_T rtu_Enable, const real_T
        rtu_alpha_beta[2], real_T rtu_wt, real_T *rty_dq, real_T *rty_dq_m,
        DW_Subsystempi2delay_sfc_6p_6_T *localDW);

      // private member function(s) for subsystem '<S48>/Subsystem1'
      static void sfc_6p_6p_topol_Subsystem1_Init(real_T *rty_dq, real_T
        *rty_dq_b, P_Subsystem1_sfc_6p_6p_topolo_T *localP);
      static void sfc_6p_6p_to_Subsystem1_Disable
        (DW_Subsystem1_sfc_6p_6p_topol_T *localDW);
      void sfc_6p_6p_topology_m_Subsystem1(uint8_T rtu_Enable, const real_T
        rtu_alpha_beta[2], real_T rtu_wt, real_T *rty_dq, real_T *rty_dq_b,
        DW_Subsystem1_sfc_6p_6p_topol_T *localDW);

      // Continuous states update member function
      void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si );

      // Derivatives member function
      void sfc_6p_6p_topology_machine_derivatives();

      // Real-Time Model
      RT_MODEL_sfc_6p_6p_topology_m_T sfc_6p_6p_topology_machine_M;
    };
  }
}

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<Root>/Constant' : Unused code path elimination
//  Block '<Root>/Constant4' : Unused code path elimination
//  Block '<Root>/Constant5' : Unused code path elimination
//  Block '<S42>/Rad->Deg.' : Unused code path elimination
//  Block '<S74>/x->r' : Unused code path elimination
//  Block '<S154>/Constant' : Unused code path elimination
//  Block '<S155>/Constant' : Unused code path elimination
//  Block '<S26>/Divide' : Unused code path elimination
//  Block '<S26>/Divide1' : Unused code path elimination
//  Block '<S26>/Gain' : Unused code path elimination
//  Block '<S65>/Rate Transition1' : Eliminated since input and output rates are identical
//  Block '<S65>/Rate Transition2' : Eliminated since input and output rates are identical
//  Block '<S65>/Rate Transition3' : Eliminated since input and output rates are identical
//  Block '<S67>/Rate Transition1' : Eliminated since input and output rates are identical
//  Block '<S67>/Rate Transition2' : Eliminated since input and output rates are identical
//  Block '<S154>/Data Type Conversion' : Eliminate redundant data type conversion
//  Block '<S155>/Data Type Conversion' : Eliminate redundant data type conversion


//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'sfc_6p_6p_topology_machine'
//  '<S1>'   : 'sfc_6p_6p_topology_machine/CBA to ABC'
//  '<S2>'   : 'sfc_6p_6p_topology_machine/Controlled_three_phase_source1'
//  '<S3>'   : 'sfc_6p_6p_topology_machine/Current Measurement2'
//  '<S4>'   : 'sfc_6p_6p_topology_machine/PLL (3ph)1'
//  '<S5>'   : 'sfc_6p_6p_topology_machine/Quadratic_load'
//  '<S6>'   : 'sfc_6p_6p_topology_machine/Reorder pulses from {positive, negative} to bridge'
//  '<S7>'   : 'sfc_6p_6p_topology_machine/Reorder pulses from {positive, negative} to bridge1'
//  '<S8>'   : 'sfc_6p_6p_topology_machine/Subsystem'
//  '<S9>'   : 'sfc_6p_6p_topology_machine/Subsystem1'
//  '<S10>'  : 'sfc_6p_6p_topology_machine/Subsystem2'
//  '<S11>'  : 'sfc_6p_6p_topology_machine/Subsystem3'
//  '<S12>'  : 'sfc_6p_6p_topology_machine/Subsystem4'
//  '<S13>'  : 'sfc_6p_6p_topology_machine/Subsystem5'
//  '<S14>'  : 'sfc_6p_6p_topology_machine/Subsystem6'
//  '<S15>'  : 'sfc_6p_6p_topology_machine/Subsystem7'
//  '<S16>'  : 'sfc_6p_6p_topology_machine/Subsystem8'
//  '<S17>'  : 'sfc_6p_6p_topology_machine/Synchronous Machine SI Fundamental (custom)1'
//  '<S18>'  : 'sfc_6p_6p_topology_machine/Three-Phase Transformer (Two Windings)'
//  '<S19>'  : 'sfc_6p_6p_topology_machine/Three-Phase V-I Measurement'
//  '<S20>'  : 'sfc_6p_6p_topology_machine/Three-Phase V-I Measurement1'
//  '<S21>'  : 'sfc_6p_6p_topology_machine/Three-Phase V-I Measurement2'
//  '<S22>'  : 'sfc_6p_6p_topology_machine/Universal Bridge1'
//  '<S23>'  : 'sfc_6p_6p_topology_machine/Universal Bridge2'
//  '<S24>'  : 'sfc_6p_6p_topology_machine/Voltage Measurement1'
//  '<S25>'  : 'sfc_6p_6p_topology_machine/Voltage Measurement8'
//  '<S26>'  : 'sfc_6p_6p_topology_machine/base_values_calculation'
//  '<S27>'  : 'sfc_6p_6p_topology_machine/convert_machine_outputs_to_si'
//  '<S28>'  : 'sfc_6p_6p_topology_machine/powergui1'
//  '<S29>'  : 'sfc_6p_6p_topology_machine/Controlled_three_phase_source1/-pi..pi_range'
//  '<S30>'  : 'sfc_6p_6p_topology_machine/Controlled_three_phase_source1/Controlled Voltage Source3'
//  '<S31>'  : 'sfc_6p_6p_topology_machine/Controlled_three_phase_source1/Controlled Voltage Source4'
//  '<S32>'  : 'sfc_6p_6p_topology_machine/Controlled_three_phase_source1/Controlled Voltage Source5'
//  '<S33>'  : 'sfc_6p_6p_topology_machine/Controlled_three_phase_source1/Integrator (Discrete or Continuous)'
//  '<S34>'  : 'sfc_6p_6p_topology_machine/Controlled_three_phase_source1/Integrator (Discrete or Continuous)/Discrete'
//  '<S35>'  : 'sfc_6p_6p_topology_machine/Current Measurement2/Model'
//  '<S36>'  : 'sfc_6p_6p_topology_machine/PLL (3ph)1/Model'
//  '<S37>'  : 'sfc_6p_6p_topology_machine/PLL (3ph)1/Model/Automatic Gain Control'
//  '<S38>'  : 'sfc_6p_6p_topology_machine/PLL (3ph)1/Model/Continuous'
//  '<S39>'  : 'sfc_6p_6p_topology_machine/PLL (3ph)1/Model/Second-Order Filter'
//  '<S40>'  : 'sfc_6p_6p_topology_machine/PLL (3ph)1/Model/Variable Frequency Mean value'
//  '<S41>'  : 'sfc_6p_6p_topology_machine/PLL (3ph)1/Model/abc to dq0'
//  '<S42>'  : 'sfc_6p_6p_topology_machine/PLL (3ph)1/Model/Automatic Gain Control/Positive-Sequence (PLL-Driven)'
//  '<S43>'  : 'sfc_6p_6p_topology_machine/PLL (3ph)1/Model/Automatic Gain Control/Positive-Sequence (PLL-Driven)/Mean (Variable Frequency)1'
//  '<S44>'  : 'sfc_6p_6p_topology_machine/PLL (3ph)1/Model/Automatic Gain Control/Positive-Sequence (PLL-Driven)/Mean (Variable Frequency)2'
//  '<S45>'  : 'sfc_6p_6p_topology_machine/PLL (3ph)1/Model/Automatic Gain Control/Positive-Sequence (PLL-Driven)/abc ->dq0'
//  '<S46>'  : 'sfc_6p_6p_topology_machine/PLL (3ph)1/Model/Automatic Gain Control/Positive-Sequence (PLL-Driven)/Mean (Variable Frequency)1/Model'
//  '<S47>'  : 'sfc_6p_6p_topology_machine/PLL (3ph)1/Model/Automatic Gain Control/Positive-Sequence (PLL-Driven)/Mean (Variable Frequency)2/Model'
//  '<S48>'  : 'sfc_6p_6p_topology_machine/PLL (3ph)1/Model/Automatic Gain Control/Positive-Sequence (PLL-Driven)/abc ->dq0/Alpha-Beta-Zero to dq0'
//  '<S49>'  : 'sfc_6p_6p_topology_machine/PLL (3ph)1/Model/Automatic Gain Control/Positive-Sequence (PLL-Driven)/abc ->dq0/abc to Alpha-Beta-Zero'
//  '<S50>'  : 'sfc_6p_6p_topology_machine/PLL (3ph)1/Model/Automatic Gain Control/Positive-Sequence (PLL-Driven)/abc ->dq0/Alpha-Beta-Zero to dq0/Compare To Constant'
//  '<S51>'  : 'sfc_6p_6p_topology_machine/PLL (3ph)1/Model/Automatic Gain Control/Positive-Sequence (PLL-Driven)/abc ->dq0/Alpha-Beta-Zero to dq0/Compare To Constant1'
//  '<S52>'  : 'sfc_6p_6p_topology_machine/PLL (3ph)1/Model/Automatic Gain Control/Positive-Sequence (PLL-Driven)/abc ->dq0/Alpha-Beta-Zero to dq0/Subsystem - pi//2 delay'
//  '<S53>'  : 'sfc_6p_6p_topology_machine/PLL (3ph)1/Model/Automatic Gain Control/Positive-Sequence (PLL-Driven)/abc ->dq0/Alpha-Beta-Zero to dq0/Subsystem1'
//  '<S54>'  : 'sfc_6p_6p_topology_machine/PLL (3ph)1/Model/Second-Order Filter/Model'
//  '<S55>'  : 'sfc_6p_6p_topology_machine/PLL (3ph)1/Model/Second-Order Filter/Model/A*x'
//  '<S56>'  : 'sfc_6p_6p_topology_machine/PLL (3ph)1/Model/Second-Order Filter/Model/B*u'
//  '<S57>'  : 'sfc_6p_6p_topology_machine/PLL (3ph)1/Model/Second-Order Filter/Model/C*x'
//  '<S58>'  : 'sfc_6p_6p_topology_machine/PLL (3ph)1/Model/Variable Frequency Mean value/Model'
//  '<S59>'  : 'sfc_6p_6p_topology_machine/PLL (3ph)1/Model/abc to dq0/Alpha-Beta-Zero to dq0'
//  '<S60>'  : 'sfc_6p_6p_topology_machine/PLL (3ph)1/Model/abc to dq0/abc to Alpha-Beta-Zero'
//  '<S61>'  : 'sfc_6p_6p_topology_machine/PLL (3ph)1/Model/abc to dq0/Alpha-Beta-Zero to dq0/Compare To Constant'
//  '<S62>'  : 'sfc_6p_6p_topology_machine/PLL (3ph)1/Model/abc to dq0/Alpha-Beta-Zero to dq0/Compare To Constant1'
//  '<S63>'  : 'sfc_6p_6p_topology_machine/PLL (3ph)1/Model/abc to dq0/Alpha-Beta-Zero to dq0/Subsystem - pi//2 delay'
//  '<S64>'  : 'sfc_6p_6p_topology_machine/PLL (3ph)1/Model/abc to dq0/Alpha-Beta-Zero to dq0/Subsystem1'
//  '<S65>'  : 'sfc_6p_6p_topology_machine/Synchronous Machine SI Fundamental (custom)1/Electrical model1'
//  '<S66>'  : 'sfc_6p_6p_topology_machine/Synchronous Machine SI Fundamental (custom)1/Measurements'
//  '<S67>'  : 'sfc_6p_6p_topology_machine/Synchronous Machine SI Fundamental (custom)1/Mechanical model'
//  '<S68>'  : 'sfc_6p_6p_topology_machine/Synchronous Machine SI Fundamental (custom)1/Electrical model1/Machine measurements'
//  '<S69>'  : 'sfc_6p_6p_topology_machine/Synchronous Machine SI Fundamental (custom)1/Electrical model1/Synchronous Machine Discrete Model'
//  '<S70>'  : 'sfc_6p_6p_topology_machine/Synchronous Machine SI Fundamental (custom)1/Electrical model1/abc to qd1'
//  '<S71>'  : 'sfc_6p_6p_topology_machine/Synchronous Machine SI Fundamental (custom)1/Electrical model1/qd to abc1'
//  '<S72>'  : 'sfc_6p_6p_topology_machine/Synchronous Machine SI Fundamental (custom)1/Electrical model1/Machine measurements/Delta angle'
//  '<S73>'  : 'sfc_6p_6p_topology_machine/Synchronous Machine SI Fundamental (custom)1/Electrical model1/Machine measurements/PQ'
//  '<S74>'  : 'sfc_6p_6p_topology_machine/Synchronous Machine SI Fundamental (custom)1/Electrical model1/Machine measurements/Delta angle/Cartesian to Polar'
//  '<S75>'  : 'sfc_6p_6p_topology_machine/Synchronous Machine SI Fundamental (custom)1/Electrical model1/Machine measurements/Delta angle/Radians to Degrees'
//  '<S76>'  : 'sfc_6p_6p_topology_machine/Synchronous Machine SI Fundamental (custom)1/Electrical model1/Synchronous Machine Discrete Model/Electromagnetic Torque'
//  '<S77>'  : 'sfc_6p_6p_topology_machine/Synchronous Machine SI Fundamental (custom)1/Electrical model1/Synchronous Machine Discrete Model/Matrix W'
//  '<S78>'  : 'sfc_6p_6p_topology_machine/Synchronous Machine SI Fundamental (custom)1/Electrical model1/Synchronous Machine Discrete Model/Phimqd'
//  '<S79>'  : 'sfc_6p_6p_topology_machine/Synchronous Machine SI Fundamental (custom)1/Electrical model1/Synchronous Machine Discrete Model/Saturation'
//  '<S80>'  : 'sfc_6p_6p_topology_machine/Synchronous Machine SI Fundamental (custom)1/Electrical model1/Synchronous Machine Discrete Model/phi'
//  '<S81>'  : 'sfc_6p_6p_topology_machine/Synchronous Machine SI Fundamental (custom)1/Electrical model1/Synchronous Machine Discrete Model/Saturation/Lmd_sat'
//  '<S82>'  : 'sfc_6p_6p_topology_machine/Synchronous Machine SI Fundamental (custom)1/Electrical model1/Synchronous Machine Discrete Model/Saturation/Lmq_sat'
//  '<S83>'  : 'sfc_6p_6p_topology_machine/Synchronous Machine SI Fundamental (custom)1/Electrical model1/Synchronous Machine Discrete Model/Saturation/Update Matrix L'
//  '<S84>'  : 'sfc_6p_6p_topology_machine/Synchronous Machine SI Fundamental (custom)1/Electrical model1/Synchronous Machine Discrete Model/Saturation/Lmd_sat/Lad'
//  '<S85>'  : 'sfc_6p_6p_topology_machine/Synchronous Machine SI Fundamental (custom)1/Electrical model1/Synchronous Machine Discrete Model/Saturation/Lmd_sat/phimd'
//  '<S86>'  : 'sfc_6p_6p_topology_machine/Synchronous Machine SI Fundamental (custom)1/Electrical model1/Synchronous Machine Discrete Model/Saturation/Lmq_sat/Laq'
//  '<S87>'  : 'sfc_6p_6p_topology_machine/Synchronous Machine SI Fundamental (custom)1/Electrical model1/Synchronous Machine Discrete Model/Saturation/Lmq_sat/phimq'
//  '<S88>'  : 'sfc_6p_6p_topology_machine/Synchronous Machine SI Fundamental (custom)1/Electrical model1/Synchronous Machine Discrete Model/phi/Subsystem'
//  '<S89>'  : 'sfc_6p_6p_topology_machine/Synchronous Machine SI Fundamental (custom)1/Electrical model1/abc to qd1/abc to dq0'
//  '<S90>'  : 'sfc_6p_6p_topology_machine/Synchronous Machine SI Fundamental (custom)1/Electrical model1/abc to qd1/abc to dq0/Alpha-Beta-Zero to dq0'
//  '<S91>'  : 'sfc_6p_6p_topology_machine/Synchronous Machine SI Fundamental (custom)1/Electrical model1/abc to qd1/abc to dq0/abc to Alpha-Beta-Zero'
//  '<S92>'  : 'sfc_6p_6p_topology_machine/Synchronous Machine SI Fundamental (custom)1/Electrical model1/abc to qd1/abc to dq0/Alpha-Beta-Zero to dq0/Compare To Constant'
//  '<S93>'  : 'sfc_6p_6p_topology_machine/Synchronous Machine SI Fundamental (custom)1/Electrical model1/abc to qd1/abc to dq0/Alpha-Beta-Zero to dq0/Compare To Constant1'
//  '<S94>'  : 'sfc_6p_6p_topology_machine/Synchronous Machine SI Fundamental (custom)1/Electrical model1/abc to qd1/abc to dq0/Alpha-Beta-Zero to dq0/Subsystem - pi//2 delay'
//  '<S95>'  : 'sfc_6p_6p_topology_machine/Synchronous Machine SI Fundamental (custom)1/Electrical model1/abc to qd1/abc to dq0/Alpha-Beta-Zero to dq0/Subsystem1'
//  '<S96>'  : 'sfc_6p_6p_topology_machine/Synchronous Machine SI Fundamental (custom)1/Electrical model1/qd to abc1/dq0 to abc'
//  '<S97>'  : 'sfc_6p_6p_topology_machine/Synchronous Machine SI Fundamental (custom)1/Electrical model1/qd to abc1/dq0 to abc/Alpha-Beta-Zero to abc'
//  '<S98>'  : 'sfc_6p_6p_topology_machine/Synchronous Machine SI Fundamental (custom)1/Electrical model1/qd to abc1/dq0 to abc/dq0 to Alpha-Beta-Zero'
//  '<S99>'  : 'sfc_6p_6p_topology_machine/Synchronous Machine SI Fundamental (custom)1/Electrical model1/qd to abc1/dq0 to abc/dq0 to Alpha-Beta-Zero/Compare To Constant'
//  '<S100>' : 'sfc_6p_6p_topology_machine/Synchronous Machine SI Fundamental (custom)1/Electrical model1/qd to abc1/dq0 to abc/dq0 to Alpha-Beta-Zero/Compare To Constant1'
//  '<S101>' : 'sfc_6p_6p_topology_machine/Synchronous Machine SI Fundamental (custom)1/Electrical model1/qd to abc1/dq0 to abc/dq0 to Alpha-Beta-Zero/Subsystem - pi//2 delay'
//  '<S102>' : 'sfc_6p_6p_topology_machine/Synchronous Machine SI Fundamental (custom)1/Electrical model1/qd to abc1/dq0 to abc/dq0 to Alpha-Beta-Zero/Subsystem1'
//  '<S103>' : 'sfc_6p_6p_topology_machine/Synchronous Machine SI Fundamental (custom)1/Mechanical model/Subsystem'
//  '<S104>' : 'sfc_6p_6p_topology_machine/Synchronous Machine SI Fundamental (custom)1/Mechanical model/Subsystem/Delay Prediction'
//  '<S105>' : 'sfc_6p_6p_topology_machine/Three-Phase Transformer (Two Windings)/Model'
//  '<S106>' : 'sfc_6p_6p_topology_machine/Three-Phase Transformer (Two Windings)/Model/Linear'
//  '<S107>' : 'sfc_6p_6p_topology_machine/Three-Phase Transformer (Two Windings)/Model/Linear1'
//  '<S108>' : 'sfc_6p_6p_topology_machine/Three-Phase Transformer (Two Windings)/Model/Linear2'
//  '<S109>' : 'sfc_6p_6p_topology_machine/Three-Phase V-I Measurement/Mode I'
//  '<S110>' : 'sfc_6p_6p_topology_machine/Three-Phase V-I Measurement/Mode V'
//  '<S111>' : 'sfc_6p_6p_topology_machine/Three-Phase V-I Measurement/Model'
//  '<S112>' : 'sfc_6p_6p_topology_machine/Three-Phase V-I Measurement/Model/I A:'
//  '<S113>' : 'sfc_6p_6p_topology_machine/Three-Phase V-I Measurement/Model/I B:'
//  '<S114>' : 'sfc_6p_6p_topology_machine/Three-Phase V-I Measurement/Model/I C:'
//  '<S115>' : 'sfc_6p_6p_topology_machine/Three-Phase V-I Measurement/Model/U AB:'
//  '<S116>' : 'sfc_6p_6p_topology_machine/Three-Phase V-I Measurement/Model/U BC:'
//  '<S117>' : 'sfc_6p_6p_topology_machine/Three-Phase V-I Measurement/Model/U CA:'
//  '<S118>' : 'sfc_6p_6p_topology_machine/Three-Phase V-I Measurement/Model/I A:/Model'
//  '<S119>' : 'sfc_6p_6p_topology_machine/Three-Phase V-I Measurement/Model/I B:/Model'
//  '<S120>' : 'sfc_6p_6p_topology_machine/Three-Phase V-I Measurement/Model/I C:/Model'
//  '<S121>' : 'sfc_6p_6p_topology_machine/Three-Phase V-I Measurement/Model/U AB:/Model'
//  '<S122>' : 'sfc_6p_6p_topology_machine/Three-Phase V-I Measurement/Model/U BC:/Model'
//  '<S123>' : 'sfc_6p_6p_topology_machine/Three-Phase V-I Measurement/Model/U CA:/Model'
//  '<S124>' : 'sfc_6p_6p_topology_machine/Three-Phase V-I Measurement1/Mode I'
//  '<S125>' : 'sfc_6p_6p_topology_machine/Three-Phase V-I Measurement1/Mode V'
//  '<S126>' : 'sfc_6p_6p_topology_machine/Three-Phase V-I Measurement1/Model'
//  '<S127>' : 'sfc_6p_6p_topology_machine/Three-Phase V-I Measurement1/Model/I A:'
//  '<S128>' : 'sfc_6p_6p_topology_machine/Three-Phase V-I Measurement1/Model/I B:'
//  '<S129>' : 'sfc_6p_6p_topology_machine/Three-Phase V-I Measurement1/Model/I C:'
//  '<S130>' : 'sfc_6p_6p_topology_machine/Three-Phase V-I Measurement1/Model/U AB:'
//  '<S131>' : 'sfc_6p_6p_topology_machine/Three-Phase V-I Measurement1/Model/U BC:'
//  '<S132>' : 'sfc_6p_6p_topology_machine/Three-Phase V-I Measurement1/Model/U CA:'
//  '<S133>' : 'sfc_6p_6p_topology_machine/Three-Phase V-I Measurement1/Model/I A:/Model'
//  '<S134>' : 'sfc_6p_6p_topology_machine/Three-Phase V-I Measurement1/Model/I B:/Model'
//  '<S135>' : 'sfc_6p_6p_topology_machine/Three-Phase V-I Measurement1/Model/I C:/Model'
//  '<S136>' : 'sfc_6p_6p_topology_machine/Three-Phase V-I Measurement1/Model/U AB:/Model'
//  '<S137>' : 'sfc_6p_6p_topology_machine/Three-Phase V-I Measurement1/Model/U BC:/Model'
//  '<S138>' : 'sfc_6p_6p_topology_machine/Three-Phase V-I Measurement1/Model/U CA:/Model'
//  '<S139>' : 'sfc_6p_6p_topology_machine/Three-Phase V-I Measurement2/Mode I'
//  '<S140>' : 'sfc_6p_6p_topology_machine/Three-Phase V-I Measurement2/Mode V'
//  '<S141>' : 'sfc_6p_6p_topology_machine/Three-Phase V-I Measurement2/Model'
//  '<S142>' : 'sfc_6p_6p_topology_machine/Three-Phase V-I Measurement2/Model/I A:'
//  '<S143>' : 'sfc_6p_6p_topology_machine/Three-Phase V-I Measurement2/Model/I B:'
//  '<S144>' : 'sfc_6p_6p_topology_machine/Three-Phase V-I Measurement2/Model/I C:'
//  '<S145>' : 'sfc_6p_6p_topology_machine/Three-Phase V-I Measurement2/Model/U A:'
//  '<S146>' : 'sfc_6p_6p_topology_machine/Three-Phase V-I Measurement2/Model/U B:'
//  '<S147>' : 'sfc_6p_6p_topology_machine/Three-Phase V-I Measurement2/Model/U C:'
//  '<S148>' : 'sfc_6p_6p_topology_machine/Three-Phase V-I Measurement2/Model/I A:/Model'
//  '<S149>' : 'sfc_6p_6p_topology_machine/Three-Phase V-I Measurement2/Model/I B:/Model'
//  '<S150>' : 'sfc_6p_6p_topology_machine/Three-Phase V-I Measurement2/Model/I C:/Model'
//  '<S151>' : 'sfc_6p_6p_topology_machine/Three-Phase V-I Measurement2/Model/U A:/Model'
//  '<S152>' : 'sfc_6p_6p_topology_machine/Three-Phase V-I Measurement2/Model/U B:/Model'
//  '<S153>' : 'sfc_6p_6p_topology_machine/Three-Phase V-I Measurement2/Model/U C:/Model'
//  '<S154>' : 'sfc_6p_6p_topology_machine/Universal Bridge1/Model'
//  '<S155>' : 'sfc_6p_6p_topology_machine/Universal Bridge2/Model'
//  '<S156>' : 'sfc_6p_6p_topology_machine/Voltage Measurement1/Model'
//  '<S157>' : 'sfc_6p_6p_topology_machine/Voltage Measurement8/Model'
//  '<S158>' : 'sfc_6p_6p_topology_machine/powergui1/EquivalentModel1'
//  '<S159>' : 'sfc_6p_6p_topology_machine/powergui1/EquivalentModel1/Gates'
//  '<S160>' : 'sfc_6p_6p_topology_machine/powergui1/EquivalentModel1/Sources'
//  '<S161>' : 'sfc_6p_6p_topology_machine/powergui1/EquivalentModel1/Status'
//  '<S162>' : 'sfc_6p_6p_topology_machine/powergui1/EquivalentModel1/Yout'

#endif                              // RTW_HEADER_sfc_6p_6p_topology_machine_h_

//
// File trailer for generated code.
//
// [EOF]
//
