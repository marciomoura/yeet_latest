//
// File: plant_schematic.h
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
#ifndef RTW_HEADER_plant_schematic_h_
#define RTW_HEADER_plant_schematic_h_
#include "rtwtypes.h"
#include "plant_schematic_types.h"
#include <array>

extern "C"
{

#include "rtGetNaN.h"

}

extern "C"
{

#include "rt_nonfinite.h"

}

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

// Class declaration for model plant_schematic
namespace mpc
{
  namespace model
  {
    class SynchronousMachineAttachedToInfiniteBusModel final
    {
      // public data and function members
     public:
      // Block signals (default storage)
      struct B_plant_schematic_T {
        std::array<real_T, 3> ib;      // '<S114>/ib'
        std::array<real_T, 3> Sum5;    // '<S68>/Sum5'
        std::array<real_T, 8> StateSpace;// '<S148>/State-Space'
        real_T LookUpTable;            // '<S75>/Look-Up Table'
        real_T Ramp;                   // '<S74>/Discrete-Time Integrator1'
        real_T Switch3;                // '<S70>/Switch3'
        real_T Switch2_e;              // '<S70>/Switch2'
        real_T DiscreteTimeIntegrator; // '<S70>/Discrete-Time Integrator'
        real_T OutportBufferForfrequency;// '<S70>/Discrete-Time Integrator'
        std::array<real_T, 3> Product1;// '<S71>/Product1'
        std::array<real_T, 3> Product1_f;// '<S72>/Product1'
        boolean_T LogicalOperator1;    // '<S70>/Logical Operator1'
      };

      // Block states (default storage) for system '<Root>'
      struct DW_plant_schematic_T {
        std::array<real_T, 5> fluxes_DSTATE;// '<S123>/fluxes'
        real_T dw_delay_DSTATE;        // '<S132>/dw_delay'
        real_T dw_predict_DSTATE;      // '<S132>/dw_predict'
        real_T Rotorangledthetae_DSTATE;// '<S110>/Rotor angle dthetae'
        std::array<real_T, 3> StateSpace_DSTATE;// '<S148>/State-Space'
        std::array<real_T, 5> voltages_DSTATE;// '<S123>/voltages'
        real_T Rotorspeeddeviationdw_DSTATE;// '<S110>/Rotor speed deviation (dw)' 
        real_T Integrator_DSTATE;      // '<S90>/Integrator'
        real_T Integrator_DSTATE_m;    // '<S91>/Integrator'
        real_T Integrator_DSTATE_a;    // '<S54>/Integrator'
        real_T Integrator_DSTATE_g;    // '<S55>/Integrator'
        real_T Integrator_DSTATE_p;    // '<S66>/Integrator'
        real_T Integrator_DSTATE_pj;   // '<S67>/Integrator'
        real_T Integrator_DSTATE_az;   // '<S102>/Integrator'
        real_T Integrator_DSTATE_f;    // '<S103>/Integrator'
        real_T Integrator_DSTATE_j;    // '<S46>/Integrator'
        real_T Integrator_DSTATE_h;    // '<S47>/Integrator'
        real_T Integrator_DSTATE_j1;   // '<S50>/Integrator'
        real_T Integrator_DSTATE_jz;   // '<S51>/Integrator'
        real_T Integrator_DSTATE_d;    // '<S94>/Integrator'
        real_T Integrator_DSTATE_k;    // '<S95>/Integrator'
        real_T Integrator_DSTATE_i;    // '<S38>/Integrator'
        real_T Integrator_DSTATE_mj;   // '<S39>/Integrator'
        real_T Integrator_DSTATE_mjg;  // '<S98>/Integrator'
        real_T Integrator_DSTATE_dl;   // '<S99>/Integrator'
        real_T Integrator_DSTATE_jo;   // '<S42>/Integrator'
        real_T Integrator_DSTATE_mw;   // '<S43>/Integrator'
        real_T Integrator_DSTATE_fa;   // '<S86>/Integrator'
        real_T Integrator_DSTATE_j3;   // '<S87>/Integrator'
        real_T Integrator_DSTATE_mjd;  // '<S58>/Integrator'
        real_T Integrator_DSTATE_ma;   // '<S59>/Integrator'
        real_T Integrator_DSTATE_c;    // '<S106>/Integrator'
        real_T Integrator_DSTATE_fj;   // '<S107>/Integrator'
        real_T Integrator_DSTATE_jp;   // '<S62>/Integrator'
        real_T Integrator_DSTATE_b;    // '<S63>/Integrator'
        real_T Integrator_DSTATE_l;    // '<S26>/Integrator'
        real_T Integrator_DSTATE_mjm;  // '<S27>/Integrator'
        real_T Integrator_DSTATE_c0;   // '<S30>/Integrator'
        real_T Integrator_DSTATE_l5;   // '<S31>/Integrator'
        real_T Integrator_DSTATE_df;   // '<S82>/Integrator'
        real_T Integrator_DSTATE_p4;   // '<S83>/Integrator'
        real_T Integrator_DSTATE_lc;   // '<S34>/Integrator'
        real_T Integrator_DSTATE_au;   // '<S35>/Integrator'
        real_T theta_DSTATE;           // '<S110>/theta'
        real_T Lmd_sat_DSTATE;         // '<S124>/Lmd_sat'
        real_T DiscreteTimeIntegrator1_DSTATE;// '<S74>/Discrete-Time Integrator1' 
        real_T UnitDelay_DSTATE;       // '<S74>/Unit Delay'
        real_T DiscreteTimeIntegrator_DSTATE;// '<S70>/Discrete-Time Integrator' 
        std::array<real_T, 25> inversion_DWORK4;// '<S131>/inversion'
        std::array<real_T, 25> inversion_DWORK4_d;// '<S122>/inversion'
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
        } StateSpace_PWORK;            // '<S148>/State-Space'

        int_T StateSpace_IWORK[11];    // '<S148>/State-Space'
        int8_T Integrator_PrevResetState;// '<S90>/Integrator'
        int8_T Integrator_PrevResetState_c;// '<S91>/Integrator'
        int8_T Integrator_PrevResetState_l;// '<S54>/Integrator'
        int8_T Integrator_PrevResetState_o;// '<S55>/Integrator'
        int8_T Integrator_PrevResetState_cb;// '<S66>/Integrator'
        int8_T Integrator_PrevResetState_k;// '<S67>/Integrator'
        int8_T Integrator_PrevResetState_l1;// '<S102>/Integrator'
        int8_T Integrator_PrevResetState_p;// '<S103>/Integrator'
        int8_T Integrator_PrevResetState_d;// '<S46>/Integrator'
        int8_T Integrator_PrevResetState_f;// '<S47>/Integrator'
        int8_T Integrator_PrevResetState_dr;// '<S50>/Integrator'
        int8_T Integrator_PrevResetState_j;// '<S51>/Integrator'
        int8_T Integrator_PrevResetState_c5;// '<S94>/Integrator'
        int8_T Integrator_PrevResetState_g;// '<S95>/Integrator'
        int8_T Integrator_PrevResetState_e;// '<S38>/Integrator'
        int8_T Integrator_PrevResetState_m;// '<S39>/Integrator'
        int8_T Integrator_PrevResetState_mu;// '<S98>/Integrator'
        int8_T Integrator_PrevResetState_ey;// '<S99>/Integrator'
        int8_T Integrator_PrevResetState_a;// '<S42>/Integrator'
        int8_T Integrator_PrevResetState_lg;// '<S43>/Integrator'
        int8_T Integrator_PrevResetState_dg;// '<S86>/Integrator'
        int8_T Integrator_PrevResetState_jy;// '<S87>/Integrator'
        int8_T Integrator_PrevResetState_op;// '<S58>/Integrator'
        int8_T Integrator_PrevResetState_n;// '<S59>/Integrator'
        int8_T Integrator_PrevResetState_pe;// '<S106>/Integrator'
        int8_T Integrator_PrevResetState_pi;// '<S107>/Integrator'
        int8_T Integrator_PrevResetState_h;// '<S62>/Integrator'
        int8_T Integrator_PrevResetState_b;// '<S63>/Integrator'
        int8_T Integrator_PrevResetState_ph;// '<S26>/Integrator'
        int8_T Integrator_PrevResetState_hz;// '<S27>/Integrator'
        int8_T Integrator_PrevResetState_da;// '<S30>/Integrator'
        int8_T Integrator_PrevResetState_o2;// '<S31>/Integrator'
        int8_T Integrator_PrevResetState_dx;// '<S82>/Integrator'
        int8_T Integrator_PrevResetState_lx;// '<S83>/Integrator'
        int8_T Integrator_PrevResetState_py;// '<S34>/Integrator'
        int8_T Integrator_PrevResetState_ob;// '<S35>/Integrator'
        uint8_T Rotorspeeddeviationdw_SYSTEM_EN;// '<S110>/Rotor speed deviation (dw)' 
        boolean_T Signalgenerator_MODE;// '<S68>/Signal generator'
        boolean_T HarmonicGenerator_MODE;// '<S68>/Harmonic Generator'
      };

      // Invariant block signals (default storage)
      struct ConstB_plant_schematic_T {
        real_T Gain;                   // '<S68>/Gain'
        real_T Gain3;                  // '<S68>/Gain3'
        real_T Gain1;                  // '<S74>/Gain1'
        real_T Gain3_h;                // '<S71>/Gain3'
        real_T Sum1;                   // '<S71>/Sum1'
        std::array<real_T, 3> MultiportSwitch;// '<S71>/Multiport Switch'
        real_T Gain3_a;                // '<S72>/Gain3'
        real_T Sum1_p;                 // '<S72>/Sum1'
        std::array<real_T, 3> MultiportSwitch_b;// '<S72>/Multiport Switch'
      };

      // Constant parameters (default storage)
      struct ConstP_plant_schematic_T {
        // Expression: sps.tv
        //  Referenced by: '<S75>/Look-Up Table'

        std::array<real_T, 8> LookUpTable_XData;

        // Expression: sps.opv
        //  Referenced by: '<S75>/Look-Up Table'

        std::array<real_T, 8> LookUpTable_YData;

        // Expression: SM.Linv
        //  Referenced by: '<S112>/Constant4'

        std::array<real_T, 25> Constant4_Value;

        // Expression: SM.RLinv
        //  Referenced by: '<S112>/Constant6'

        std::array<real_T, 25> Constant6_Value;

        // Expression: eye(SM.nState,SM.nState)
        //  Referenced by: '<S131>/u5'

        std::array<real_T, 25> u5_Value_d;

        // Expression: SM.phiqd0
        //  Referenced by: '<S123>/fluxes'

        std::array<real_T, 5> fluxes_InitialCondition;

        // Expression: SM.IqdSign
        //  Referenced by: '<S112>/change Iq Id  current signs'

        std::array<real_T, 5> changeIqIdcurrentsigns_Gain;

        // Expression: S.A
        //  Referenced by: '<S148>/State-Space'

        std::array<real_T, 9> StateSpace_AS_param;

        // Expression: S.B
        //  Referenced by: '<S148>/State-Space'

        std::array<real_T, 15> StateSpace_BS_param;

        // Expression: S.C
        //  Referenced by: '<S148>/State-Space'

        std::array<real_T, 24> StateSpace_CS_param;

        // Expression: S.D
        //  Referenced by: '<S148>/State-Space'

        std::array<real_T, 40> StateSpace_DS_param;

        // Expression: S.x0
        //  Referenced by: '<S148>/State-Space'

        std::array<real_T, 3> StateSpace_X0_param;
      };

      // External inputs (root inport signals with default storage)
      struct ExtU_plant_schematic_T {
        real_T mechanical_power;       // '<Root>/mechanical_power'
        real_T field_voltage;          // '<Root>/field_voltage'
      };

      // External outputs (root outports fed by signals with default storage)
      struct ExtY_plant_schematic_T {
        real_T vt1;                    // '<Root>/vt1'
        real_T vt2;                    // '<Root>/vt2'
        real_T vt_filtered;            // '<Root>/vt_filtered'
        real_T it_filtered;            // '<Root>/it_filtered'
        real_T ifd_filtered;           // '<Root>/ifd_filtered'
        real_T pt_filtered;            // '<Root>/pt_filtered'
        real_T qt_filtered;            // '<Root>/qt_filtered'
        real_T st_filtered;            // '<Root>/st_filtered'
        real_T vd_filtered;            // '<Root>/vd_filtered'
        real_T vq_filtered;            // '<Root>/vq_filtered'
        real_T id_filtered;            // '<Root>/id_filtered'
        real_T iq_filtered;            // '<Root>/iq_filtered'
        real_T load_angle_deg_filtered;// '<Root>/load_angle_deg_filtered'
        real_T vt;                     // '<Root>/vt'
        real_T it;                     // '<Root>/it'
        real_T ifd;                    // '<Root>/ifd'
        real_T pt;                     // '<Root>/pt'
        real_T qt;                     // '<Root>/qt'
        real_T st;                     // '<Root>/st'
        real_T vd;                     // '<Root>/vd'
        real_T vq;                     // '<Root>/vq'
        real_T id;                     // '<Root>/id'
        real_T iq;                     // '<Root>/iq'
        real_T wm;                     // '<Root>/wm'
        real_T load_angle_deg;         // '<Root>/load_angle_deg'
        real_T wt;                     // '<Root>/wt'
      };

      // Real-time Model Data Structure
      struct RT_MODEL_plant_schematic_T {
        const char_T * volatile errorStatus;

        //
        //  Timing:
        //  The following substructure contains information regarding
        //  the timing information for the model.

        struct {
          uint32_T clockTick0;
        } Timing;
      };

      // Copy Constructor
      SynchronousMachineAttachedToInfiniteBusModel
        (SynchronousMachineAttachedToInfiniteBusModel const&) = delete;

      // Assignment Operator
      SynchronousMachineAttachedToInfiniteBusModel& operator=
        (SynchronousMachineAttachedToInfiniteBusModel const&) & = delete;

      // Move Constructor
      SynchronousMachineAttachedToInfiniteBusModel
        (SynchronousMachineAttachedToInfiniteBusModel &&) = delete;

      // Move Assignment Operator
      SynchronousMachineAttachedToInfiniteBusModel& operator=
        (SynchronousMachineAttachedToInfiniteBusModel &&) = delete;

      // Real-Time Model get method
      SynchronousMachineAttachedToInfiniteBusModel::RT_MODEL_plant_schematic_T *
        getRTM();

      // Root inports set method
      void setExternalInputs(const ExtU_plant_schematic_T
        *pExtU_plant_schematic_T)
      {
        plant_schematic_U = *pExtU_plant_schematic_T;
      }

      // Root outports get method
      const ExtY_plant_schematic_T &getExternalOutputs() const
      {
        return plant_schematic_Y;
      }

      // model initialize function
      void initialize();

      // model step function
      void step();

      // model terminate function
      void terminate();

      // Constructor
      SynchronousMachineAttachedToInfiniteBusModel();

      // Destructor
      ~SynchronousMachineAttachedToInfiniteBusModel();

      // private data and function members
     private:
      // External inputs
      ExtU_plant_schematic_T plant_schematic_U;

      // External outputs
      ExtY_plant_schematic_T plant_schematic_Y;

      // Block signals
      B_plant_schematic_T plant_schematic_B;

      // Block states
      DW_plant_schematic_T plant_schematic_DW;

      // Real-Time Model
      RT_MODEL_plant_schematic_T plant_schematic_M;
    };
  }
}

extern const mpc::model::SynchronousMachineAttachedToInfiniteBusModel::
  ConstB_plant_schematic_T plant_schematic_ConstB;// constant block i/o

// Constant parameters (default storage)
extern const mpc::model::SynchronousMachineAttachedToInfiniteBusModel::
  ConstP_plant_schematic_T plant_schematic_ConstP;

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S117>/x->r' : Unused code path elimination
//  Block '<S22>/Kv' : Unused code path elimination
//  Block '<S22>/Kv1' : Unused code path elimination
//  Block '<S136>/do not delete this gain' : Unused code path elimination
//  Block '<S137>/do not delete this gain' : Unused code path elimination
//  Block '<S138>/do not delete this gain' : Unused code path elimination
//  Block '<S139>/do not delete this gain' : Unused code path elimination
//  Block '<S140>/do not delete this gain' : Unused code path elimination
//  Block '<S141>/do not delete this gain' : Unused code path elimination
//  Block '<S26>/Saturation' : Eliminated Saturate block
//  Block '<S27>/Saturation' : Eliminated Saturate block
//  Block '<S30>/Saturation' : Eliminated Saturate block
//  Block '<S31>/Saturation' : Eliminated Saturate block
//  Block '<S34>/Saturation' : Eliminated Saturate block
//  Block '<S35>/Saturation' : Eliminated Saturate block
//  Block '<S38>/Saturation' : Eliminated Saturate block
//  Block '<S39>/Saturation' : Eliminated Saturate block
//  Block '<S42>/Saturation' : Eliminated Saturate block
//  Block '<S43>/Saturation' : Eliminated Saturate block
//  Block '<S46>/Saturation' : Eliminated Saturate block
//  Block '<S47>/Saturation' : Eliminated Saturate block
//  Block '<S50>/Saturation' : Eliminated Saturate block
//  Block '<S51>/Saturation' : Eliminated Saturate block
//  Block '<S54>/Saturation' : Eliminated Saturate block
//  Block '<S55>/Saturation' : Eliminated Saturate block
//  Block '<S58>/Saturation' : Eliminated Saturate block
//  Block '<S59>/Saturation' : Eliminated Saturate block
//  Block '<S62>/Saturation' : Eliminated Saturate block
//  Block '<S63>/Saturation' : Eliminated Saturate block
//  Block '<S66>/Saturation' : Eliminated Saturate block
//  Block '<S67>/Saturation' : Eliminated Saturate block
//  Block '<S74>/Data Type  Conversion1' : Eliminate redundant data type conversion
//  Block '<S82>/Saturation' : Eliminated Saturate block
//  Block '<S83>/Saturation' : Eliminated Saturate block
//  Block '<S86>/Saturation' : Eliminated Saturate block
//  Block '<S87>/Saturation' : Eliminated Saturate block
//  Block '<S90>/Saturation' : Eliminated Saturate block
//  Block '<S91>/Saturation' : Eliminated Saturate block
//  Block '<S94>/Saturation' : Eliminated Saturate block
//  Block '<S95>/Saturation' : Eliminated Saturate block
//  Block '<S98>/Saturation' : Eliminated Saturate block
//  Block '<S99>/Saturation' : Eliminated Saturate block
//  Block '<S102>/Saturation' : Eliminated Saturate block
//  Block '<S103>/Saturation' : Eliminated Saturate block
//  Block '<S106>/Saturation' : Eliminated Saturate block
//  Block '<S107>/Saturation' : Eliminated Saturate block
//  Block '<S116>/Gain' : Eliminated nontunable gain of 1
//  Block '<S116>/Gain1' : Eliminated nontunable gain of 1
//  Block '<S108>/Rate Transition1' : Eliminated since input and output rates are identical
//  Block '<S108>/Rate Transition2' : Eliminated since input and output rates are identical
//  Block '<S108>/Rate Transition3' : Eliminated since input and output rates are identical
//  Block '<S110>/1_Pb' : Eliminated nontunable gain of 1
//  Block '<S110>/Rate Transition1' : Eliminated since input and output rates are identical
//  Block '<S110>/Rate Transition2' : Eliminated since input and output rates are identical
//  Block '<S110>/units' : Eliminated nontunable gain of 1
//  Block '<S110>/units1' : Eliminated nontunable gain of 1
//  Block '<S110>/units2' : Eliminated nontunable gain of 1
//  Block '<S68>/SinglePhase' : Unused code path elimination
//  Block '<S111>/Constant1' : Unused code path elimination
//  Block '<S110>/Product' : Unused code path elimination


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
//  '<Root>' : 'plant_schematic'
//  '<S1>'   : 'plant_schematic/Bandpass 10'
//  '<S2>'   : 'plant_schematic/Bandpass 11'
//  '<S3>'   : 'plant_schematic/Bandpass 12'
//  '<S4>'   : 'plant_schematic/Bandpass 2'
//  '<S5>'   : 'plant_schematic/Bandpass 3'
//  '<S6>'   : 'plant_schematic/Bandpass 4'
//  '<S7>'   : 'plant_schematic/Bandpass 5'
//  '<S8>'   : 'plant_schematic/Bandpass 6'
//  '<S9>'   : 'plant_schematic/Bandpass 7'
//  '<S10>'  : 'plant_schematic/Bandpass 8'
//  '<S11>'  : 'plant_schematic/Bandpass 9'
//  '<S12>'  : 'plant_schematic/Grid Voltage Source'
//  '<S13>'  : 'plant_schematic/Idea machine measurements in dq1'
//  '<S14>'  : 'plant_schematic/LPF 1'
//  '<S15>'  : 'plant_schematic/LPF 2'
//  '<S16>'  : 'plant_schematic/LPF 4'
//  '<S17>'  : 'plant_schematic/LPF 5'
//  '<S18>'  : 'plant_schematic/LPF 6'
//  '<S19>'  : 'plant_schematic/LPF 7'
//  '<S20>'  : 'plant_schematic/LPF 8'
//  '<S21>'  : 'plant_schematic/Synchronous Machine'
//  '<S22>'  : 'plant_schematic/Terminal  Measurements'
//  '<S23>'  : 'plant_schematic/powergui'
//  '<S24>'  : 'plant_schematic/Bandpass 10/Integrator (Discrete or Continuous)'
//  '<S25>'  : 'plant_schematic/Bandpass 10/Integrator (Discrete or Continuous)1'
//  '<S26>'  : 'plant_schematic/Bandpass 10/Integrator (Discrete or Continuous)/Discrete'
//  '<S27>'  : 'plant_schematic/Bandpass 10/Integrator (Discrete or Continuous)1/Discrete'
//  '<S28>'  : 'plant_schematic/Bandpass 11/Integrator (Discrete or Continuous)'
//  '<S29>'  : 'plant_schematic/Bandpass 11/Integrator (Discrete or Continuous)1'
//  '<S30>'  : 'plant_schematic/Bandpass 11/Integrator (Discrete or Continuous)/Discrete'
//  '<S31>'  : 'plant_schematic/Bandpass 11/Integrator (Discrete or Continuous)1/Discrete'
//  '<S32>'  : 'plant_schematic/Bandpass 12/Integrator (Discrete or Continuous)'
//  '<S33>'  : 'plant_schematic/Bandpass 12/Integrator (Discrete or Continuous)1'
//  '<S34>'  : 'plant_schematic/Bandpass 12/Integrator (Discrete or Continuous)/Discrete'
//  '<S35>'  : 'plant_schematic/Bandpass 12/Integrator (Discrete or Continuous)1/Discrete'
//  '<S36>'  : 'plant_schematic/Bandpass 2/Integrator (Discrete or Continuous)'
//  '<S37>'  : 'plant_schematic/Bandpass 2/Integrator (Discrete or Continuous)1'
//  '<S38>'  : 'plant_schematic/Bandpass 2/Integrator (Discrete or Continuous)/Discrete'
//  '<S39>'  : 'plant_schematic/Bandpass 2/Integrator (Discrete or Continuous)1/Discrete'
//  '<S40>'  : 'plant_schematic/Bandpass 3/Integrator (Discrete or Continuous)'
//  '<S41>'  : 'plant_schematic/Bandpass 3/Integrator (Discrete or Continuous)1'
//  '<S42>'  : 'plant_schematic/Bandpass 3/Integrator (Discrete or Continuous)/Discrete'
//  '<S43>'  : 'plant_schematic/Bandpass 3/Integrator (Discrete or Continuous)1/Discrete'
//  '<S44>'  : 'plant_schematic/Bandpass 4/Integrator (Discrete or Continuous)'
//  '<S45>'  : 'plant_schematic/Bandpass 4/Integrator (Discrete or Continuous)1'
//  '<S46>'  : 'plant_schematic/Bandpass 4/Integrator (Discrete or Continuous)/Discrete'
//  '<S47>'  : 'plant_schematic/Bandpass 4/Integrator (Discrete or Continuous)1/Discrete'
//  '<S48>'  : 'plant_schematic/Bandpass 5/Integrator (Discrete or Continuous)'
//  '<S49>'  : 'plant_schematic/Bandpass 5/Integrator (Discrete or Continuous)1'
//  '<S50>'  : 'plant_schematic/Bandpass 5/Integrator (Discrete or Continuous)/Discrete'
//  '<S51>'  : 'plant_schematic/Bandpass 5/Integrator (Discrete or Continuous)1/Discrete'
//  '<S52>'  : 'plant_schematic/Bandpass 6/Integrator (Discrete or Continuous)'
//  '<S53>'  : 'plant_schematic/Bandpass 6/Integrator (Discrete or Continuous)1'
//  '<S54>'  : 'plant_schematic/Bandpass 6/Integrator (Discrete or Continuous)/Discrete'
//  '<S55>'  : 'plant_schematic/Bandpass 6/Integrator (Discrete or Continuous)1/Discrete'
//  '<S56>'  : 'plant_schematic/Bandpass 7/Integrator (Discrete or Continuous)'
//  '<S57>'  : 'plant_schematic/Bandpass 7/Integrator (Discrete or Continuous)1'
//  '<S58>'  : 'plant_schematic/Bandpass 7/Integrator (Discrete or Continuous)/Discrete'
//  '<S59>'  : 'plant_schematic/Bandpass 7/Integrator (Discrete or Continuous)1/Discrete'
//  '<S60>'  : 'plant_schematic/Bandpass 8/Integrator (Discrete or Continuous)'
//  '<S61>'  : 'plant_schematic/Bandpass 8/Integrator (Discrete or Continuous)1'
//  '<S62>'  : 'plant_schematic/Bandpass 8/Integrator (Discrete or Continuous)/Discrete'
//  '<S63>'  : 'plant_schematic/Bandpass 8/Integrator (Discrete or Continuous)1/Discrete'
//  '<S64>'  : 'plant_schematic/Bandpass 9/Integrator (Discrete or Continuous)'
//  '<S65>'  : 'plant_schematic/Bandpass 9/Integrator (Discrete or Continuous)1'
//  '<S66>'  : 'plant_schematic/Bandpass 9/Integrator (Discrete or Continuous)/Discrete'
//  '<S67>'  : 'plant_schematic/Bandpass 9/Integrator (Discrete or Continuous)1/Discrete'
//  '<S68>'  : 'plant_schematic/Grid Voltage Source/Model'
//  '<S69>'  : 'plant_schematic/Grid Voltage Source/Model/Harmonic Generator'
//  '<S70>'  : 'plant_schematic/Grid Voltage Source/Model/Signal generator'
//  '<S71>'  : 'plant_schematic/Grid Voltage Source/Model/Harmonic Generator/Harmonic A generation'
//  '<S72>'  : 'plant_schematic/Grid Voltage Source/Model/Harmonic Generator/Harmonic B generation'
//  '<S73>'  : 'plant_schematic/Grid Voltage Source/Model/Signal generator/Stair Generator'
//  '<S74>'  : 'plant_schematic/Grid Voltage Source/Model/Signal generator/Variation SubSystem'
//  '<S75>'  : 'plant_schematic/Grid Voltage Source/Model/Signal generator/Stair Generator/Model'
//  '<S76>'  : 'plant_schematic/Idea machine measurements in dq1/Magnitude 1'
//  '<S77>'  : 'plant_schematic/Idea machine measurements in dq1/Magnitude 2'
//  '<S78>'  : 'plant_schematic/Idea machine measurements in dq1/Magnitude 3'
//  '<S79>'  : 'plant_schematic/Idea machine measurements in dq1/ranging pi -pi9'
//  '<S80>'  : 'plant_schematic/LPF 1/Integrator (Discrete or Continuous)'
//  '<S81>'  : 'plant_schematic/LPF 1/Integrator (Discrete or Continuous)1'
//  '<S82>'  : 'plant_schematic/LPF 1/Integrator (Discrete or Continuous)/Discrete'
//  '<S83>'  : 'plant_schematic/LPF 1/Integrator (Discrete or Continuous)1/Discrete'
//  '<S84>'  : 'plant_schematic/LPF 2/Integrator (Discrete or Continuous)'
//  '<S85>'  : 'plant_schematic/LPF 2/Integrator (Discrete or Continuous)1'
//  '<S86>'  : 'plant_schematic/LPF 2/Integrator (Discrete or Continuous)/Discrete'
//  '<S87>'  : 'plant_schematic/LPF 2/Integrator (Discrete or Continuous)1/Discrete'
//  '<S88>'  : 'plant_schematic/LPF 4/Integrator (Discrete or Continuous)'
//  '<S89>'  : 'plant_schematic/LPF 4/Integrator (Discrete or Continuous)1'
//  '<S90>'  : 'plant_schematic/LPF 4/Integrator (Discrete or Continuous)/Discrete'
//  '<S91>'  : 'plant_schematic/LPF 4/Integrator (Discrete or Continuous)1/Discrete'
//  '<S92>'  : 'plant_schematic/LPF 5/Integrator (Discrete or Continuous)'
//  '<S93>'  : 'plant_schematic/LPF 5/Integrator (Discrete or Continuous)1'
//  '<S94>'  : 'plant_schematic/LPF 5/Integrator (Discrete or Continuous)/Discrete'
//  '<S95>'  : 'plant_schematic/LPF 5/Integrator (Discrete or Continuous)1/Discrete'
//  '<S96>'  : 'plant_schematic/LPF 6/Integrator (Discrete or Continuous)'
//  '<S97>'  : 'plant_schematic/LPF 6/Integrator (Discrete or Continuous)1'
//  '<S98>'  : 'plant_schematic/LPF 6/Integrator (Discrete or Continuous)/Discrete'
//  '<S99>'  : 'plant_schematic/LPF 6/Integrator (Discrete or Continuous)1/Discrete'
//  '<S100>' : 'plant_schematic/LPF 7/Integrator (Discrete or Continuous)'
//  '<S101>' : 'plant_schematic/LPF 7/Integrator (Discrete or Continuous)1'
//  '<S102>' : 'plant_schematic/LPF 7/Integrator (Discrete or Continuous)/Discrete'
//  '<S103>' : 'plant_schematic/LPF 7/Integrator (Discrete or Continuous)1/Discrete'
//  '<S104>' : 'plant_schematic/LPF 8/Integrator (Discrete or Continuous)'
//  '<S105>' : 'plant_schematic/LPF 8/Integrator (Discrete or Continuous)1'
//  '<S106>' : 'plant_schematic/LPF 8/Integrator (Discrete or Continuous)/Discrete'
//  '<S107>' : 'plant_schematic/LPF 8/Integrator (Discrete or Continuous)1/Discrete'
//  '<S108>' : 'plant_schematic/Synchronous Machine/Electrical model'
//  '<S109>' : 'plant_schematic/Synchronous Machine/Measurements'
//  '<S110>' : 'plant_schematic/Synchronous Machine/Mechanical model'
//  '<S111>' : 'plant_schematic/Synchronous Machine/Electrical model/Machine measurements'
//  '<S112>' : 'plant_schematic/Synchronous Machine/Electrical model/Synchronous Machine Discrete Model'
//  '<S113>' : 'plant_schematic/Synchronous Machine/Electrical model/abc to qd transformation'
//  '<S114>' : 'plant_schematic/Synchronous Machine/Electrical model/qd to abc transformation'
//  '<S115>' : 'plant_schematic/Synchronous Machine/Electrical model/Machine measurements/Delta angle'
//  '<S116>' : 'plant_schematic/Synchronous Machine/Electrical model/Machine measurements/PQ'
//  '<S117>' : 'plant_schematic/Synchronous Machine/Electrical model/Machine measurements/Delta angle/Cartesian to Polar'
//  '<S118>' : 'plant_schematic/Synchronous Machine/Electrical model/Machine measurements/Delta angle/Radians to Degrees'
//  '<S119>' : 'plant_schematic/Synchronous Machine/Electrical model/Synchronous Machine Discrete Model/Electromagnetic Torque'
//  '<S120>' : 'plant_schematic/Synchronous Machine/Electrical model/Synchronous Machine Discrete Model/Matrix W'
//  '<S121>' : 'plant_schematic/Synchronous Machine/Electrical model/Synchronous Machine Discrete Model/Phimqd'
//  '<S122>' : 'plant_schematic/Synchronous Machine/Electrical model/Synchronous Machine Discrete Model/Saturation'
//  '<S123>' : 'plant_schematic/Synchronous Machine/Electrical model/Synchronous Machine Discrete Model/phi'
//  '<S124>' : 'plant_schematic/Synchronous Machine/Electrical model/Synchronous Machine Discrete Model/Saturation/Lmd_sat'
//  '<S125>' : 'plant_schematic/Synchronous Machine/Electrical model/Synchronous Machine Discrete Model/Saturation/Lmq_sat'
//  '<S126>' : 'plant_schematic/Synchronous Machine/Electrical model/Synchronous Machine Discrete Model/Saturation/Update Matrix L'
//  '<S127>' : 'plant_schematic/Synchronous Machine/Electrical model/Synchronous Machine Discrete Model/Saturation/Lmd_sat/Lad'
//  '<S128>' : 'plant_schematic/Synchronous Machine/Electrical model/Synchronous Machine Discrete Model/Saturation/Lmd_sat/phimd'
//  '<S129>' : 'plant_schematic/Synchronous Machine/Electrical model/Synchronous Machine Discrete Model/Saturation/Lmq_sat/Laq'
//  '<S130>' : 'plant_schematic/Synchronous Machine/Electrical model/Synchronous Machine Discrete Model/Saturation/Lmq_sat/phimq'
//  '<S131>' : 'plant_schematic/Synchronous Machine/Electrical model/Synchronous Machine Discrete Model/phi/Subsystem'
//  '<S132>' : 'plant_schematic/Synchronous Machine/Mechanical model/Delay Prediction'
//  '<S133>' : 'plant_schematic/Terminal  Measurements/Mode I'
//  '<S134>' : 'plant_schematic/Terminal  Measurements/Mode V'
//  '<S135>' : 'plant_schematic/Terminal  Measurements/Model'
//  '<S136>' : 'plant_schematic/Terminal  Measurements/Model/I A:'
//  '<S137>' : 'plant_schematic/Terminal  Measurements/Model/I B:'
//  '<S138>' : 'plant_schematic/Terminal  Measurements/Model/I C:'
//  '<S139>' : 'plant_schematic/Terminal  Measurements/Model/U A:'
//  '<S140>' : 'plant_schematic/Terminal  Measurements/Model/U B:'
//  '<S141>' : 'plant_schematic/Terminal  Measurements/Model/U C:'
//  '<S142>' : 'plant_schematic/Terminal  Measurements/Model/I A:/Model'
//  '<S143>' : 'plant_schematic/Terminal  Measurements/Model/I B:/Model'
//  '<S144>' : 'plant_schematic/Terminal  Measurements/Model/I C:/Model'
//  '<S145>' : 'plant_schematic/Terminal  Measurements/Model/U A:/Model'
//  '<S146>' : 'plant_schematic/Terminal  Measurements/Model/U B:/Model'
//  '<S147>' : 'plant_schematic/Terminal  Measurements/Model/U C:/Model'
//  '<S148>' : 'plant_schematic/powergui/EquivalentModel1'
//  '<S149>' : 'plant_schematic/powergui/EquivalentModel1/Sources'
//  '<S150>' : 'plant_schematic/powergui/EquivalentModel1/Yout'

#endif                                 // RTW_HEADER_plant_schematic_h_

//
// File trailer for generated code.
//
// [EOF]
//
