//
// File: indirect_excitation_2mva_plant_model.h
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
#ifndef RTW_HEADER_indirect_excitation_2mva_plant_model_h_
#define RTW_HEADER_indirect_excitation_2mva_plant_model_h_
#include "rtwtypes.h"
#include "indirect_excitation_2mva_plant_model_types.h"

extern "C"
{

#include "rt_nonfinite.h"

}

extern "C"
{

#include "rtGetNaN.h"

}

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

// Class declaration for model indirect_excitation_2mva_plant_model
namespace plant
{
  class indirect_excitation_2mva final
  {
    // public data and function members
   public:
    // Block signals (default storage)
    struct B_indirect_excitation_2mva_pl_T {
      real_T ib[3];                    // '<S89>/ib'
      real_T Sum5[3];                  // '<S70>/Sum5'
      real_T StateSpace_o1[11];        // '<S124>/State-Space'
      real_T StateSpace_o2[3];         // '<S124>/State-Space'
      real_T ib_g[3];                  // '<S48>/ib'
      real_T UnitDelay2;               // '<S4>/Unit Delay2'
      real_T StateSpace_o1_l[15];      // '<S123>/State-Space'
      real_T StateSpace_o2_p[6];       // '<S123>/State-Space'
      real_T Switch3;                  // '<S116>/Switch3'
      real_T Switch3_b;                // '<S118>/Switch3'
      real_T Switch3_k;                // '<S120>/Switch3'
      real_T LookUpTable;              // '<S77>/Look-Up Table'
      real_T Switch3_g;                // '<S72>/Switch3'
      real_T Switch2_e;                // '<S72>/Switch2'
      real_T DiscreteTimeIntegrator;   // '<S72>/Discrete-Time Integrator'
      real_T Product1[3];              // '<S73>/Product1'
      real_T Product1_l[3];            // '<S74>/Product1'
      boolean_T LogicalOperator1;      // '<S72>/Logical Operator1'
    };

    // Block states (default storage) for system '<Root>'
    struct DW_indirect_excitation_2mva_p_T {
      real_T fluxes_DSTATE[5];         // '<S57>/fluxes'
      real_T dw_delay_DSTATE;          // '<S107>/dw_delay'
      real_T dw_predict_DSTATE;        // '<S107>/dw_predict'
      real_T fluxes_DSTATE_m[5];       // '<S98>/fluxes'
      real_T Rotorangledthetae_DSTATE; // '<S85>/Rotor angle dthetae'
      real_T StateSpace_DSTATE[6];     // '<S124>/State-Space'
      real_T Rotorangledtheta_DSTATE;  // '<S44>/Rotor angle dtheta'
      real_T UnitDelay2_DSTATE;        // '<S4>/Unit Delay2'
      real_T StateSpace_DSTATE_h[6];   // '<S123>/State-Space'
      real_T UnitDelay4_DSTATE;        // '<S4>/Unit Delay4'
      real_T voltages_DSTATE[5];       // '<S98>/voltages'
      real_T voltages_DSTATE_b[5];     // '<S57>/voltages'
      real_T Rotorspeeddeviationdw_DSTATE;// '<S85>/Rotor speed deviation (dw)'
      real_T theta_DSTATE;             // '<S85>/theta'
      real_T Lmd_sat_DSTATE;           // '<S99>/Lmd_sat'
      real_T DiscreteTimeIntegrator1_DSTATE;// '<S76>/Discrete-Time Integrator1' 
      real_T UnitDelay_DSTATE;         // '<S76>/Unit Delay'
      real_T DiscreteTimeIntegrator_DSTATE;// '<S72>/Discrete-Time Integrator'
      real_T Lmd_sat_DSTATE_h;         // '<S58>/Lmd_sat'
      real_T inversion_DWORK4[25];     // '<S106>/inversion'
      real_T inversion_DWORK4_i[25];   // '<S97>/inversion'
      real_T inversion_DWORK4_k[25];   // '<S65>/inversion'
      real_T inversion_DWORK4_c[25];   // '<S56>/inversion'
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
      } StateSpace_PWORK;              // '<S124>/State-Space'

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
      } StateSpace_PWORK_n;            // '<S123>/State-Space'

      int_T StateSpace_IWORK[11];      // '<S124>/State-Space'
      int_T StateSpace_IWORK_f[11];    // '<S123>/State-Space'
      uint8_T Rotorspeeddeviationdw_SYSTEM_EN;// '<S85>/Rotor speed deviation (dw)' 
      boolean_T Signalgenerator_MODE;  // '<S70>/Signal generator'
      boolean_T HarmonicGenerator_MODE;// '<S70>/Harmonic Generator'
    };

    // Invariant block signals (default storage)
    struct ConstB_indirect_excitation_2m_T {
      real_T Gain;                     // '<S70>/Gain'
      real_T Gain3;                    // '<S70>/Gain3'
      real_T Gain1;                    // '<S76>/Gain1'
      real_T Gain3_i;                  // '<S73>/Gain3'
      real_T Sum1;                     // '<S73>/Sum1'
      real_T MultiportSwitch[3];       // '<S73>/Multiport Switch'
      real_T Gain3_m;                  // '<S74>/Gain3'
      real_T Sum1_o;                   // '<S74>/Sum1'
      real_T MultiportSwitch_d[3];     // '<S74>/Multiport Switch'
      boolean_T DataTypeConversion1;   // '<S10>/Data Type  Conversion1'
      boolean_T DataTypeConversion2;   // '<S10>/Data Type  Conversion2'
      boolean_T DataTypeConversion3;   // '<S10>/Data Type  Conversion3'
    };

    // Constant parameters (default storage)
    struct ConstP_indirect_excitation_2m_T {
      // Expression: sps.tv
      //  Referenced by: '<S77>/Look-Up Table'

      real_T LookUpTable_XData[8];

      // Expression: sps.opv
      //  Referenced by: '<S77>/Look-Up Table'

      real_T LookUpTable_YData[8];

      // Expression: SM.Linv
      //  Referenced by: '<S46>/Constant4'

      real_T Constant4_Value[25];

      // Expression: SM.RLinv
      //  Referenced by: '<S46>/Constant6'

      real_T Constant6_Value[25];

      // Pooled Parameter (Expression: eye(SM.nState,SM.nState))
      //  Referenced by:
      //    '<S65>/u5'
      //    '<S106>/u5'

      real_T pooled23[25];

      // Expression: SM.Linv
      //  Referenced by: '<S87>/Constant4'

      real_T Constant4_Value_m[25];

      // Expression: SM.RLinv
      //  Referenced by: '<S87>/Constant6'

      real_T Constant6_Value_o[25];

      // Pooled Parameter (Mixed Expressions)
      //  Referenced by:
      //    '<S7>/g'
      //    '<S126>/SwitchCurrents'

      real_T pooled25[6];

      // Expression: SM.phiqd0
      //  Referenced by: '<S57>/fluxes'

      real_T fluxes_InitialCondition[5];

      // Expression: SM.phiqd0
      //  Referenced by: '<S98>/fluxes'

      real_T fluxes_InitialCondition_d[5];

      // Pooled Parameter (Expression: SM.IqdSign)
      //  Referenced by:
      //    '<S46>/change Iq Id  current signs'
      //    '<S87>/change Iq Id  current signs'

      real_T pooled28[5];

      // Expression: S.A
      //  Referenced by: '<S124>/State-Space'

      real_T StateSpace_AS_param[36];

      // Expression: S.B
      //  Referenced by: '<S124>/State-Space'

      real_T StateSpace_BS_param[48];

      // Expression: S.C
      //  Referenced by: '<S124>/State-Space'

      real_T StateSpace_CS_param[66];

      // Expression: S.D
      //  Referenced by: '<S124>/State-Space'

      real_T StateSpace_DS_param[88];

      // Expression: S.x0
      //  Referenced by: '<S124>/State-Space'

      real_T StateSpace_X0_param[6];

      // Expression: S.A
      //  Referenced by: '<S123>/State-Space'

      real_T StateSpace_AS_param_k[36];

      // Expression: S.B
      //  Referenced by: '<S123>/State-Space'

      real_T StateSpace_BS_param_m[54];

      // Expression: S.C
      //  Referenced by: '<S123>/State-Space'

      real_T StateSpace_CS_param_m[90];

      // Expression: S.D
      //  Referenced by: '<S123>/State-Space'

      real_T StateSpace_DS_param_e[135];

      // Expression: S.x0
      //  Referenced by: '<S123>/State-Space'

      real_T StateSpace_X0_param_k[6];
    };

    // External inputs (root inport signals with default storage)
    struct ExtU_indirect_excitation_2mva_T {
      real_T mechanical_power;         // '<Root>/mechanical_power'
      real_T field_voltage;            // '<Root>/field_voltage'
      real_T three_phase_fault_en;     // '<Root>/three_phase_fault_en'
    };

    // External outputs (root outports fed by signals with default storage)
    struct ExtY_indirect_excitation_2mva_T {
      real_T v_abc_sm_pu[3];           // '<Root>/v_abc_sm_pu'
      real_T i_abc_sm_pu[3];           // '<Root>/i_abc_sm_pu'
      real_T v_abc_exc_pu[3];          // '<Root>/v_abc_exc_pu'
      real_T i_abc_exc_pu[3];          // '<Root>/i_abc_exc_pu'
      real_T vt;                       // '<Root>/vt'
      real_T it;                       // '<Root>/it'
      real_T ifd;                      // '<Root>/ifd'
      real_T pt;                       // '<Root>/pt'
      real_T qt;                       // '<Root>/qt'
      real_T st;                       // '<Root>/st'
      real_T vd;                       // '<Root>/vd'
      real_T vq;                       // '<Root>/vq'
      real_T id;                       // '<Root>/id'
      real_T iq;                       // '<Root>/iq'
      real_T wm;                       // '<Root>/wm'
      real_T load_angle_deg;           // '<Root>/load_angle_deg'
      real_T wt;                       // '<Root>/wt'
    };

    // Real-time Model Data Structure
    struct RT_MODEL_indirect_excitation__T {
      const char_T * volatile errorStatus;

      //
      //  Timing:
      //  The following substructure contains information regarding
      //  the timing information for the model.

      struct {
        uint32_T clockTick1;
      } Timing;
    };

    // Copy Constructor
    indirect_excitation_2mva(indirect_excitation_2mva const&) = delete;

    // Assignment Operator
    indirect_excitation_2mva& operator= (indirect_excitation_2mva const&) & =
      delete;

    // Move Constructor
    indirect_excitation_2mva(indirect_excitation_2mva &&) = delete;

    // Move Assignment Operator
    indirect_excitation_2mva& operator= (indirect_excitation_2mva &&) = delete;

    // Real-Time Model get method
    indirect_excitation_2mva::RT_MODEL_indirect_excitation__T * getRTM();

    // Root inports set method
    void setExternalInputs(const ExtU_indirect_excitation_2mva_T
      *pExtU_indirect_excitation_2mva_T)
    {
      indirect_excitation_2mva_plan_U = *pExtU_indirect_excitation_2mva_T;
    }

    // Root outports get method
    const ExtY_indirect_excitation_2mva_T &getExternalOutputs() const
    {
      return indirect_excitation_2mva_plan_Y;
    }

    // model initialize function
    void initialize();

    // model step function
    void step0();

    // model step function
    void step1();

    // model terminate function
    void terminate();

    // Constructor
    indirect_excitation_2mva();

    // Destructor
    ~indirect_excitation_2mva();

    // private data and function members
   private:
    // External inputs
    ExtU_indirect_excitation_2mva_T indirect_excitation_2mva_plan_U;

    // External outputs
    ExtY_indirect_excitation_2mva_T indirect_excitation_2mva_plan_Y;

    // Block signals
    B_indirect_excitation_2mva_pl_T indirect_excitation_2mva_plan_B;

    // Block states
    DW_indirect_excitation_2mva_p_T indirect_excitation_2mva_pla_DW;

    // Real-Time Model
    RT_MODEL_indirect_excitation__T indirect_excitation_2mva_pla_M;
  };
}

extern const plant::indirect_excitation_2mva::ConstB_indirect_excitation_2m_T
  indirect_excitation_2mva_ConstB;     // constant block i/o

// Constant parameters (default storage)
extern const plant::indirect_excitation_2mva::ConstP_indirect_excitation_2m_T
  indirect_excitation_2mva_ConstP;

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S45>/Constant1' : Unused code path elimination
//  Block '<S45>/Constant2' : Unused code path elimination
//  Block '<S51>/x->r' : Unused code path elimination
//  Block '<S51>/x->theta' : Unused code path elimination
//  Block '<S52>/Gain' : Unused code path elimination
//  Block '<S50>/Gain' : Unused code path elimination
//  Block '<S50>/Gain1' : Unused code path elimination
//  Block '<S50>/Product2' : Unused code path elimination
//  Block '<S50>/Product3' : Unused code path elimination
//  Block '<S50>/Product4' : Unused code path elimination
//  Block '<S50>/Product5' : Unused code path elimination
//  Block '<S50>/Sum' : Unused code path elimination
//  Block '<S50>/Sum1' : Unused code path elimination
//  Block '<S45>/Switch' : Unused code path elimination
//  Block '<S45>/output formatting' : Unused code path elimination
//  Block '<S42>/Rate Transition3' : Unused code path elimination
//  Block '<S46>/Constant5' : Unused code path elimination
//  Block '<S46>/Constant8' : Unused code path elimination
//  Block '<S53>/1-1' : Unused code path elimination
//  Block '<S53>/Mult1' : Unused code path elimination
//  Block '<S53>/Sum2' : Unused code path elimination
//  Block '<S46>/Laqd_nosat' : Unused code path elimination
//  Block '<S46>/Lmqd_nosat' : Unused code path elimination
//  Block '<S55>/1//Ll_d' : Unused code path elimination
//  Block '<S55>/1//Ll_q' : Unused code path elimination
//  Block '<S55>/Product' : Unused code path elimination
//  Block '<S55>/Product1' : Unused code path elimination
//  Block '<S55>/sum1' : Unused code path elimination
//  Block '<S55>/sum2' : Unused code path elimination
//  Block '<S56>/Constant3' : Unused code path elimination
//  Block '<S56>/Switch2' : Unused code path elimination
//  Block '<S56>/u2' : Unused code path elimination
//  Block '<S46>/Switch2' : Unused code path elimination
//  Block '<S46>/Switch3' : Unused code path elimination
//  Block '<S44>/Rate Transition2' : Unused code path elimination
//  Block '<S66>/Fcn' : Unused code path elimination
//  Block '<S66>/Product' : Unused code path elimination
//  Block '<S66>/t' : Unused code path elimination
//  Block '<S66>/theta' : Unused code path elimination
//  Block '<S66>/units' : Unused code path elimination
//  Block '<S66>/units1' : Unused code path elimination
//  Block '<S66>/units2' : Unused code path elimination
//  Block '<S66>/web3' : Unused code path elimination
//  Block '<S82>/Constant' : Unused code path elimination
//  Block '<S92>/x->r' : Unused code path elimination
//  Block '<S1>/Kv' : Eliminated nontunable gain of 1
//  Block '<S15>/do not delete this gain' : Eliminated nontunable gain of 1
//  Block '<S16>/do not delete this gain' : Eliminated nontunable gain of 1
//  Block '<S17>/do not delete this gain' : Eliminated nontunable gain of 1
//  Block '<S18>/do not delete this gain' : Eliminated nontunable gain of 1
//  Block '<S19>/do not delete this gain' : Eliminated nontunable gain of 1
//  Block '<S20>/do not delete this gain' : Eliminated nontunable gain of 1
//  Block '<S30>/do not delete this gain' : Eliminated nontunable gain of 1
//  Block '<S31>/do not delete this gain' : Eliminated nontunable gain of 1
//  Block '<S32>/do not delete this gain' : Eliminated nontunable gain of 1
//  Block '<S33>/do not delete this gain' : Eliminated nontunable gain of 1
//  Block '<S34>/do not delete this gain' : Eliminated nontunable gain of 1
//  Block '<S35>/do not delete this gain' : Eliminated nontunable gain of 1
//  Block '<S42>/Rate Transition1' : Eliminated since input and output rates are identical
//  Block '<S42>/Rate Transition2' : Eliminated since input and output rates are identical
//  Block '<S44>/Rate Transition1' : Eliminated since input and output rates are identical
//  Block '<S68>/do not delete this gain' : Eliminated nontunable gain of 1
//  Block '<S76>/Data Type  Conversion1' : Eliminate redundant data type conversion
//  Block '<S82>/Data Type Conversion' : Eliminate redundant data type conversion
//  Block '<S83>/Rate Transition1' : Eliminated since input and output rates are identical
//  Block '<S83>/Rate Transition2' : Eliminated since input and output rates are identical
//  Block '<S83>/Rate Transition3' : Eliminated since input and output rates are identical
//  Block '<S85>/Rate Transition1' : Eliminated since input and output rates are identical
//  Block '<S85>/Rate Transition2' : Eliminated since input and output rates are identical
//  Block '<S116>/Data Type  Conversion' : Eliminate redundant data type conversion
//  Block '<S118>/Data Type  Conversion' : Eliminate redundant data type conversion
//  Block '<S120>/Data Type  Conversion' : Eliminate redundant data type conversion
//  Block '<S70>/SinglePhase' : Unused code path elimination
//  Block '<S86>/Constant1' : Unused code path elimination
//  Block '<S85>/Product' : Unused code path elimination
//  Block '<S85>/units1' : Unused code path elimination
//  Block '<S85>/units2' : Unused code path elimination
//  Block '<S10>/C4' : Unused code path elimination
//  Block '<S116>/C4' : Unused code path elimination
//  Block '<S117>/Digital Clock' : Unused code path elimination
//  Block '<S117>/Look-Up Table' : Unused code path elimination
//  Block '<S118>/C4' : Unused code path elimination
//  Block '<S119>/Digital Clock' : Unused code path elimination
//  Block '<S119>/Look-Up Table' : Unused code path elimination
//  Block '<S120>/C4' : Unused code path elimination
//  Block '<S121>/Digital Clock' : Unused code path elimination
//  Block '<S121>/Look-Up Table' : Unused code path elimination
//  Block '<S122>/Digital Clock' : Unused code path elimination
//  Block '<S122>/Look-Up Table' : Unused code path elimination


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
//  '<Root>' : 'indirect_excitation_2mva_plant_model'
//  '<S1>'   : 'indirect_excitation_2mva_plant_model/BExc'
//  '<S2>'   : 'indirect_excitation_2mva_plant_model/BSM'
//  '<S3>'   : 'indirect_excitation_2mva_plant_model/Exciter System 8.1 kVA 400V'
//  '<S4>'   : 'indirect_excitation_2mva_plant_model/Field Connection'
//  '<S5>'   : 'indirect_excitation_2mva_plant_model/Grid Voltage Source'
//  '<S6>'   : 'indirect_excitation_2mva_plant_model/Idea machine measurements in dq1'
//  '<S7>'   : 'indirect_excitation_2mva_plant_model/Rectifier'
//  '<S8>'   : 'indirect_excitation_2mva_plant_model/Synchronous  Generator 2 MVA 400V'
//  '<S9>'   : 'indirect_excitation_2mva_plant_model/Transformer 10 kVA 400V// 12V'
//  '<S10>'  : 'indirect_excitation_2mva_plant_model/Two-Phase Fault'
//  '<S11>'  : 'indirect_excitation_2mva_plant_model/powergui1'
//  '<S12>'  : 'indirect_excitation_2mva_plant_model/BExc/Mode I'
//  '<S13>'  : 'indirect_excitation_2mva_plant_model/BExc/Mode V'
//  '<S14>'  : 'indirect_excitation_2mva_plant_model/BExc/Model'
//  '<S15>'  : 'indirect_excitation_2mva_plant_model/BExc/Model/I A:'
//  '<S16>'  : 'indirect_excitation_2mva_plant_model/BExc/Model/I B:'
//  '<S17>'  : 'indirect_excitation_2mva_plant_model/BExc/Model/I C:'
//  '<S18>'  : 'indirect_excitation_2mva_plant_model/BExc/Model/U A:'
//  '<S19>'  : 'indirect_excitation_2mva_plant_model/BExc/Model/U B:'
//  '<S20>'  : 'indirect_excitation_2mva_plant_model/BExc/Model/U C:'
//  '<S21>'  : 'indirect_excitation_2mva_plant_model/BExc/Model/I A:/Model'
//  '<S22>'  : 'indirect_excitation_2mva_plant_model/BExc/Model/I B:/Model'
//  '<S23>'  : 'indirect_excitation_2mva_plant_model/BExc/Model/I C:/Model'
//  '<S24>'  : 'indirect_excitation_2mva_plant_model/BExc/Model/U A:/Model'
//  '<S25>'  : 'indirect_excitation_2mva_plant_model/BExc/Model/U B:/Model'
//  '<S26>'  : 'indirect_excitation_2mva_plant_model/BExc/Model/U C:/Model'
//  '<S27>'  : 'indirect_excitation_2mva_plant_model/BSM/Mode I'
//  '<S28>'  : 'indirect_excitation_2mva_plant_model/BSM/Mode V'
//  '<S29>'  : 'indirect_excitation_2mva_plant_model/BSM/Model'
//  '<S30>'  : 'indirect_excitation_2mva_plant_model/BSM/Model/I A:'
//  '<S31>'  : 'indirect_excitation_2mva_plant_model/BSM/Model/I B:'
//  '<S32>'  : 'indirect_excitation_2mva_plant_model/BSM/Model/I C:'
//  '<S33>'  : 'indirect_excitation_2mva_plant_model/BSM/Model/U A:'
//  '<S34>'  : 'indirect_excitation_2mva_plant_model/BSM/Model/U B:'
//  '<S35>'  : 'indirect_excitation_2mva_plant_model/BSM/Model/U C:'
//  '<S36>'  : 'indirect_excitation_2mva_plant_model/BSM/Model/I A:/Model'
//  '<S37>'  : 'indirect_excitation_2mva_plant_model/BSM/Model/I B:/Model'
//  '<S38>'  : 'indirect_excitation_2mva_plant_model/BSM/Model/I C:/Model'
//  '<S39>'  : 'indirect_excitation_2mva_plant_model/BSM/Model/U A:/Model'
//  '<S40>'  : 'indirect_excitation_2mva_plant_model/BSM/Model/U B:/Model'
//  '<S41>'  : 'indirect_excitation_2mva_plant_model/BSM/Model/U C:/Model'
//  '<S42>'  : 'indirect_excitation_2mva_plant_model/Exciter System 8.1 kVA 400V/Electrical model'
//  '<S43>'  : 'indirect_excitation_2mva_plant_model/Exciter System 8.1 kVA 400V/Measurements'
//  '<S44>'  : 'indirect_excitation_2mva_plant_model/Exciter System 8.1 kVA 400V/Mechanical model'
//  '<S45>'  : 'indirect_excitation_2mva_plant_model/Exciter System 8.1 kVA 400V/Electrical model/Machine measurements'
//  '<S46>'  : 'indirect_excitation_2mva_plant_model/Exciter System 8.1 kVA 400V/Electrical model/Synchronous Machine Discrete Model'
//  '<S47>'  : 'indirect_excitation_2mva_plant_model/Exciter System 8.1 kVA 400V/Electrical model/abc to qd transformation'
//  '<S48>'  : 'indirect_excitation_2mva_plant_model/Exciter System 8.1 kVA 400V/Electrical model/qd to abc transformation'
//  '<S49>'  : 'indirect_excitation_2mva_plant_model/Exciter System 8.1 kVA 400V/Electrical model/Machine measurements/Delta angle'
//  '<S50>'  : 'indirect_excitation_2mva_plant_model/Exciter System 8.1 kVA 400V/Electrical model/Machine measurements/PQ'
//  '<S51>'  : 'indirect_excitation_2mva_plant_model/Exciter System 8.1 kVA 400V/Electrical model/Machine measurements/Delta angle/Cartesian to Polar'
//  '<S52>'  : 'indirect_excitation_2mva_plant_model/Exciter System 8.1 kVA 400V/Electrical model/Machine measurements/Delta angle/Radians to Degrees'
//  '<S53>'  : 'indirect_excitation_2mva_plant_model/Exciter System 8.1 kVA 400V/Electrical model/Synchronous Machine Discrete Model/Electromagnetic Torque'
//  '<S54>'  : 'indirect_excitation_2mva_plant_model/Exciter System 8.1 kVA 400V/Electrical model/Synchronous Machine Discrete Model/Matrix W'
//  '<S55>'  : 'indirect_excitation_2mva_plant_model/Exciter System 8.1 kVA 400V/Electrical model/Synchronous Machine Discrete Model/Phimqd'
//  '<S56>'  : 'indirect_excitation_2mva_plant_model/Exciter System 8.1 kVA 400V/Electrical model/Synchronous Machine Discrete Model/Saturation'
//  '<S57>'  : 'indirect_excitation_2mva_plant_model/Exciter System 8.1 kVA 400V/Electrical model/Synchronous Machine Discrete Model/phi'
//  '<S58>'  : 'indirect_excitation_2mva_plant_model/Exciter System 8.1 kVA 400V/Electrical model/Synchronous Machine Discrete Model/Saturation/Lmd_sat'
//  '<S59>'  : 'indirect_excitation_2mva_plant_model/Exciter System 8.1 kVA 400V/Electrical model/Synchronous Machine Discrete Model/Saturation/Lmq_sat'
//  '<S60>'  : 'indirect_excitation_2mva_plant_model/Exciter System 8.1 kVA 400V/Electrical model/Synchronous Machine Discrete Model/Saturation/Update Matrix L'
//  '<S61>'  : 'indirect_excitation_2mva_plant_model/Exciter System 8.1 kVA 400V/Electrical model/Synchronous Machine Discrete Model/Saturation/Lmd_sat/Lad'
//  '<S62>'  : 'indirect_excitation_2mva_plant_model/Exciter System 8.1 kVA 400V/Electrical model/Synchronous Machine Discrete Model/Saturation/Lmd_sat/phimd'
//  '<S63>'  : 'indirect_excitation_2mva_plant_model/Exciter System 8.1 kVA 400V/Electrical model/Synchronous Machine Discrete Model/Saturation/Lmq_sat/Laq'
//  '<S64>'  : 'indirect_excitation_2mva_plant_model/Exciter System 8.1 kVA 400V/Electrical model/Synchronous Machine Discrete Model/Saturation/Lmq_sat/phimq'
//  '<S65>'  : 'indirect_excitation_2mva_plant_model/Exciter System 8.1 kVA 400V/Electrical model/Synchronous Machine Discrete Model/phi/Subsystem'
//  '<S66>'  : 'indirect_excitation_2mva_plant_model/Exciter System 8.1 kVA 400V/Mechanical model/measurements'
//  '<S67>'  : 'indirect_excitation_2mva_plant_model/Field Connection/Ifd'
//  '<S68>'  : 'indirect_excitation_2mva_plant_model/Field Connection/Vf '
//  '<S69>'  : 'indirect_excitation_2mva_plant_model/Field Connection/Vf /Model'
//  '<S70>'  : 'indirect_excitation_2mva_plant_model/Grid Voltage Source/Model'
//  '<S71>'  : 'indirect_excitation_2mva_plant_model/Grid Voltage Source/Model/Harmonic Generator'
//  '<S72>'  : 'indirect_excitation_2mva_plant_model/Grid Voltage Source/Model/Signal generator'
//  '<S73>'  : 'indirect_excitation_2mva_plant_model/Grid Voltage Source/Model/Harmonic Generator/Harmonic A generation'
//  '<S74>'  : 'indirect_excitation_2mva_plant_model/Grid Voltage Source/Model/Harmonic Generator/Harmonic B generation'
//  '<S75>'  : 'indirect_excitation_2mva_plant_model/Grid Voltage Source/Model/Signal generator/Stair Generator'
//  '<S76>'  : 'indirect_excitation_2mva_plant_model/Grid Voltage Source/Model/Signal generator/Variation SubSystem'
//  '<S77>'  : 'indirect_excitation_2mva_plant_model/Grid Voltage Source/Model/Signal generator/Stair Generator/Model'
//  '<S78>'  : 'indirect_excitation_2mva_plant_model/Idea machine measurements in dq1/Magnitude 1'
//  '<S79>'  : 'indirect_excitation_2mva_plant_model/Idea machine measurements in dq1/Magnitude 2'
//  '<S80>'  : 'indirect_excitation_2mva_plant_model/Idea machine measurements in dq1/Magnitude 3'
//  '<S81>'  : 'indirect_excitation_2mva_plant_model/Idea machine measurements in dq1/ranging pi -pi9'
//  '<S82>'  : 'indirect_excitation_2mva_plant_model/Rectifier/Model'
//  '<S83>'  : 'indirect_excitation_2mva_plant_model/Synchronous  Generator 2 MVA 400V/Electrical model'
//  '<S84>'  : 'indirect_excitation_2mva_plant_model/Synchronous  Generator 2 MVA 400V/Measurements'
//  '<S85>'  : 'indirect_excitation_2mva_plant_model/Synchronous  Generator 2 MVA 400V/Mechanical model'
//  '<S86>'  : 'indirect_excitation_2mva_plant_model/Synchronous  Generator 2 MVA 400V/Electrical model/Machine measurements'
//  '<S87>'  : 'indirect_excitation_2mva_plant_model/Synchronous  Generator 2 MVA 400V/Electrical model/Synchronous Machine Discrete Model'
//  '<S88>'  : 'indirect_excitation_2mva_plant_model/Synchronous  Generator 2 MVA 400V/Electrical model/abc to qd transformation'
//  '<S89>'  : 'indirect_excitation_2mva_plant_model/Synchronous  Generator 2 MVA 400V/Electrical model/qd to abc transformation'
//  '<S90>'  : 'indirect_excitation_2mva_plant_model/Synchronous  Generator 2 MVA 400V/Electrical model/Machine measurements/Delta angle'
//  '<S91>'  : 'indirect_excitation_2mva_plant_model/Synchronous  Generator 2 MVA 400V/Electrical model/Machine measurements/PQ'
//  '<S92>'  : 'indirect_excitation_2mva_plant_model/Synchronous  Generator 2 MVA 400V/Electrical model/Machine measurements/Delta angle/Cartesian to Polar'
//  '<S93>'  : 'indirect_excitation_2mva_plant_model/Synchronous  Generator 2 MVA 400V/Electrical model/Machine measurements/Delta angle/Radians to Degrees'
//  '<S94>'  : 'indirect_excitation_2mva_plant_model/Synchronous  Generator 2 MVA 400V/Electrical model/Synchronous Machine Discrete Model/Electromagnetic Torque'
//  '<S95>'  : 'indirect_excitation_2mva_plant_model/Synchronous  Generator 2 MVA 400V/Electrical model/Synchronous Machine Discrete Model/Matrix W'
//  '<S96>'  : 'indirect_excitation_2mva_plant_model/Synchronous  Generator 2 MVA 400V/Electrical model/Synchronous Machine Discrete Model/Phimqd'
//  '<S97>'  : 'indirect_excitation_2mva_plant_model/Synchronous  Generator 2 MVA 400V/Electrical model/Synchronous Machine Discrete Model/Saturation'
//  '<S98>'  : 'indirect_excitation_2mva_plant_model/Synchronous  Generator 2 MVA 400V/Electrical model/Synchronous Machine Discrete Model/phi'
//  '<S99>'  : 'indirect_excitation_2mva_plant_model/Synchronous  Generator 2 MVA 400V/Electrical model/Synchronous Machine Discrete Model/Saturation/Lmd_sat'
//  '<S100>' : 'indirect_excitation_2mva_plant_model/Synchronous  Generator 2 MVA 400V/Electrical model/Synchronous Machine Discrete Model/Saturation/Lmq_sat'
//  '<S101>' : 'indirect_excitation_2mva_plant_model/Synchronous  Generator 2 MVA 400V/Electrical model/Synchronous Machine Discrete Model/Saturation/Update Matrix L'
//  '<S102>' : 'indirect_excitation_2mva_plant_model/Synchronous  Generator 2 MVA 400V/Electrical model/Synchronous Machine Discrete Model/Saturation/Lmd_sat/Lad'
//  '<S103>' : 'indirect_excitation_2mva_plant_model/Synchronous  Generator 2 MVA 400V/Electrical model/Synchronous Machine Discrete Model/Saturation/Lmd_sat/phimd'
//  '<S104>' : 'indirect_excitation_2mva_plant_model/Synchronous  Generator 2 MVA 400V/Electrical model/Synchronous Machine Discrete Model/Saturation/Lmq_sat/Laq'
//  '<S105>' : 'indirect_excitation_2mva_plant_model/Synchronous  Generator 2 MVA 400V/Electrical model/Synchronous Machine Discrete Model/Saturation/Lmq_sat/phimq'
//  '<S106>' : 'indirect_excitation_2mva_plant_model/Synchronous  Generator 2 MVA 400V/Electrical model/Synchronous Machine Discrete Model/phi/Subsystem'
//  '<S107>' : 'indirect_excitation_2mva_plant_model/Synchronous  Generator 2 MVA 400V/Mechanical model/Delay Prediction'
//  '<S108>' : 'indirect_excitation_2mva_plant_model/Transformer 10 kVA 400V// 12V/Model'
//  '<S109>' : 'indirect_excitation_2mva_plant_model/Transformer 10 kVA 400V// 12V/Model/Linear'
//  '<S110>' : 'indirect_excitation_2mva_plant_model/Transformer 10 kVA 400V// 12V/Model/Linear1'
//  '<S111>' : 'indirect_excitation_2mva_plant_model/Transformer 10 kVA 400V// 12V/Model/Linear2'
//  '<S112>' : 'indirect_excitation_2mva_plant_model/Two-Phase Fault/Fault A'
//  '<S113>' : 'indirect_excitation_2mva_plant_model/Two-Phase Fault/Fault B'
//  '<S114>' : 'indirect_excitation_2mva_plant_model/Two-Phase Fault/Fault C'
//  '<S115>' : 'indirect_excitation_2mva_plant_model/Two-Phase Fault/Stair Generator'
//  '<S116>' : 'indirect_excitation_2mva_plant_model/Two-Phase Fault/Fault A/Model'
//  '<S117>' : 'indirect_excitation_2mva_plant_model/Two-Phase Fault/Fault A/Model/Timer'
//  '<S118>' : 'indirect_excitation_2mva_plant_model/Two-Phase Fault/Fault B/Model'
//  '<S119>' : 'indirect_excitation_2mva_plant_model/Two-Phase Fault/Fault B/Model/Timer'
//  '<S120>' : 'indirect_excitation_2mva_plant_model/Two-Phase Fault/Fault C/Model'
//  '<S121>' : 'indirect_excitation_2mva_plant_model/Two-Phase Fault/Fault C/Model/Timer'
//  '<S122>' : 'indirect_excitation_2mva_plant_model/Two-Phase Fault/Stair Generator/Model'
//  '<S123>' : 'indirect_excitation_2mva_plant_model/powergui1/EquivalentModel1'
//  '<S124>' : 'indirect_excitation_2mva_plant_model/powergui1/EquivalentModel2'
//  '<S125>' : 'indirect_excitation_2mva_plant_model/powergui1/EquivalentModel1/Gates'
//  '<S126>' : 'indirect_excitation_2mva_plant_model/powergui1/EquivalentModel1/Sources'
//  '<S127>' : 'indirect_excitation_2mva_plant_model/powergui1/EquivalentModel1/Status'
//  '<S128>' : 'indirect_excitation_2mva_plant_model/powergui1/EquivalentModel1/Yout'
//  '<S129>' : 'indirect_excitation_2mva_plant_model/powergui1/EquivalentModel2/Gates'
//  '<S130>' : 'indirect_excitation_2mva_plant_model/powergui1/EquivalentModel2/Sources'
//  '<S131>' : 'indirect_excitation_2mva_plant_model/powergui1/EquivalentModel2/Status'
//  '<S132>' : 'indirect_excitation_2mva_plant_model/powergui1/EquivalentModel2/Yout'

#endif                    // RTW_HEADER_indirect_excitation_2mva_plant_model_h_

//
// File trailer for generated code.
//
// [EOF]
//
