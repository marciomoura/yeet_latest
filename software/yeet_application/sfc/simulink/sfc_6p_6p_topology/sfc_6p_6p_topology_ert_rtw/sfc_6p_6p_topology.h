//
// File: sfc_6p_6p_topology.h
//
// Code generated for Simulink model 'sfc_6p_6p_topology'.
//
// Model version                  : 12.60
// Simulink Coder version         : 23.2 (R2023b) 01-Aug-2023
// C/C++ source code generated on : Tue Jul 15 16:36:05 2025
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Windows64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_sfc_6p_6p_topology_h_
#define RTW_HEADER_sfc_6p_6p_topology_h_
#include <stdbool.h>
#include <stdint.h>
#include <array>

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

// user code (top of header file)
#include <cstring>

// Class declaration for model sfc_6p_6p_topology
namespace generated
{
  namespace sfc
  {
    class sfc_6p_6p_topology final
    {
      // public data and function members
     public:
      // Block signals and states (default storage) for system '<Root>'
      struct DW_sfc_6p_6p_topology_T {
        std::array<double, 3> sqrt_2;  // '<S1>/sqrt_2'
        std::array<double, 3> sqrt_2_j;// '<S2>/sqrt_2'
        std::array<double, 25> StateSpace_o1;// '<S59>/State-Space'
        std::array<double, 12> StateSpace_o2;// '<S59>/State-Space'
        std::array<double, 18> StateSpace_DSTATE;// '<S59>/State-Space'
        double Integrator_DSTATE;      // '<S14>/Integrator'
        double Integrator_DSTATE_f;    // '<S22>/Integrator'
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
        } StateSpace_PWORK;            // '<S59>/State-Space'

        int StateSpace_IWORK[11];      // '<S59>/State-Space'
        bool Initial_FirstOutputTime;  // '<S14>/Initial'
        bool Initial_FirstOutputTime_b;// '<S22>/Initial'
      };

      // Constant parameters (default storage)
      struct ConstP_sfc_6p_6p_topology_T {
        // Expression: zeros(12,1)
        //  Referenced by: '<S61>/SwitchCurrents'

        std::array<double, 12> SwitchCurrents_Value;

        // Expression: S.A
        //  Referenced by: '<S59>/State-Space'

        std::array<double, 324> StateSpace_AS_param;

        // Expression: S.B
        //  Referenced by: '<S59>/State-Space'

        std::array<double, 324> StateSpace_BS_param;

        // Expression: S.C
        //  Referenced by: '<S59>/State-Space'

        std::array<double, 450> StateSpace_CS_param;

        // Expression: S.D
        //  Referenced by: '<S59>/State-Space'

        std::array<double, 450> StateSpace_DS_param;

        // Expression: S.x0
        //  Referenced by: '<S59>/State-Space'

        std::array<double, 18> StateSpace_X0_param;
      };

      // External inputs (root inport signals with default storage)
      struct ExtU_sfc_6p_6p_topology_T {
        std::array<double, 6> grid_side_converter_pulses;// '<Root>/grid_side_converter_pulses'
        std::array<double, 6> machine_side_converter_pulses;// '<Root>/machine_side_converter_pulses'
        double grid_voltage_source_frequency_hz;// '<Root>/grid_voltage_source_frequency_hz'
        double grid_voltage_source_line_voltage_rms;// '<Root>/grid_voltage_source_line_voltage_rms'
        double machine_voltage_source_frequency_hz;// '<Root>/machine_voltage_source_frequency_hz'
        double machine_voltage_source_line_voltage_rms;// '<Root>/machine_voltage_source_line_voltage_rms'
      };

      // External outputs (root outports fed by signals with default storage)
      struct ExtY_sfc_6p_6p_topology_T {
        double dc_link_current_si;     // '<Root>/dc_link_current_si'
        double grid_voltage_source_angle_wt;// '<Root>/grid_voltage_source_angle_wt'
        double machine_voltage_source_angle_wt;// '<Root>/machine_voltage_source_angle_wt'
        std::array<double, 3> grid_side_converter_voltage_phase_si;// '<Root>/grid_side_converter_voltage_phase_si'
        std::array<double, 3> grid_side_converter_current_si;// '<Root>/grid_side_converter_current_si'
        std::array<double, 3> machine_side_converter_voltage_phase_si;// '<Root>/machine_side_converter_voltage_phase_si'
        std::array<double, 3> machine_side_converter_current_si;// '<Root>/machine_side_converter_current_si'
      };

      // Real-time Model Data Structure
      struct RT_MODEL_sfc_6p_6p_topology_T {
        const char * volatile errorStatus;
      };

      // Copy Constructor
      sfc_6p_6p_topology(sfc_6p_6p_topology const&) = delete;

      // Assignment Operator
      sfc_6p_6p_topology& operator= (sfc_6p_6p_topology const&) & = delete;

      // Move Constructor
      sfc_6p_6p_topology(sfc_6p_6p_topology &&) = delete;

      // Move Assignment Operator
      sfc_6p_6p_topology& operator= (sfc_6p_6p_topology &&) = delete;

      // Real-Time Model get method
      sfc_6p_6p_topology::RT_MODEL_sfc_6p_6p_topology_T * getRTM()
      {
        return (&sfc_6p_6p_topology_M);
      }

      // Real-Time Model set method
      void setRTM(const RT_MODEL_sfc_6p_6p_topology_T *psfc_6p_6p_topology_M)
      {
        sfc_6p_6p_topology_M = *psfc_6p_6p_topology_M;
      }

      // External inputs
      ExtU_sfc_6p_6p_topology_T sfc_6p_6p_topology_U;

      // External outputs
      ExtY_sfc_6p_6p_topology_T sfc_6p_6p_topology_Y;

      // Root inports set method
      void setExternalInputs(const ExtU_sfc_6p_6p_topology_T *pExtU_sfc_6p_6p_topology_T)
      {
        sfc_6p_6p_topology_U = *pExtU_sfc_6p_6p_topology_T;
      }

      // Root outports get method
      const ExtY_sfc_6p_6p_topology_T &getExternalOutputs() const
      {
        return sfc_6p_6p_topology_Y;
      }

      // Block states get method
      const DW_sfc_6p_6p_topology_T &getDWork() const
      {
        return sfc_6p_6p_topology_DW;
      }

      // Block states set method
      void setDWork(const DW_sfc_6p_6p_topology_T *pDW_sfc_6p_6p_topology_T)
      {
        sfc_6p_6p_topology_DW = *pDW_sfc_6p_6p_topology_T;
      }

      // model initialize function
      void initialize();

      // model step function
      void step();

      // model terminate function
      void terminate();

      // Constructor
      sfc_6p_6p_topology();

      // Destructor
      ~sfc_6p_6p_topology();

      // private data and function members
     private:
      // Block states
      DW_sfc_6p_6p_topology_T sfc_6p_6p_topology_DW;

      // Real-Time Model
      RT_MODEL_sfc_6p_6p_topology_T sfc_6p_6p_topology_M;
    };
  }
}

// Constant parameters (default storage)
extern const generated::sfc::sfc_6p_6p_topology::ConstP_sfc_6p_6p_topology_T sfc_6p_6p_topology_ConstP;

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S26>/Constant' : Unused code path elimination
//  Block '<S43>/Constant' : Unused code path elimination
//  Block '<S26>/Data Type Conversion' : Eliminate redundant data type conversion
//  Block '<S4>/Kv' : Eliminated nontunable gain of 1
//  Block '<S4>/Kv1' : Eliminated nontunable gain of 1
//  Block '<S30>/do not delete this gain' : Eliminated nontunable gain of 1
//  Block '<S31>/do not delete this gain' : Eliminated nontunable gain of 1
//  Block '<S32>/do not delete this gain' : Eliminated nontunable gain of 1
//  Block '<S33>/do not delete this gain' : Eliminated nontunable gain of 1
//  Block '<S34>/do not delete this gain' : Eliminated nontunable gain of 1
//  Block '<S35>/do not delete this gain' : Eliminated nontunable gain of 1
//  Block '<S5>/do not delete this gain' : Eliminated nontunable gain of 1
//  Block '<S43>/Data Type Conversion' : Eliminate redundant data type conversion
//  Block '<S7>/Kv' : Eliminated nontunable gain of 1
//  Block '<S7>/Kv1' : Eliminated nontunable gain of 1
//  Block '<S47>/do not delete this gain' : Eliminated nontunable gain of 1
//  Block '<S48>/do not delete this gain' : Eliminated nontunable gain of 1
//  Block '<S49>/do not delete this gain' : Eliminated nontunable gain of 1
//  Block '<S50>/do not delete this gain' : Eliminated nontunable gain of 1
//  Block '<S51>/do not delete this gain' : Eliminated nontunable gain of 1
//  Block '<S52>/do not delete this gain' : Eliminated nontunable gain of 1


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
//  '<Root>' : 'sfc_6p_6p_topology'
//  '<S1>'   : 'sfc_6p_6p_topology/ABC Grid Voltage Source'
//  '<S2>'   : 'sfc_6p_6p_topology/ABC Machine Voltage Source'
//  '<S3>'   : 'sfc_6p_6p_topology/Grid Side Converter'
//  '<S4>'   : 'sfc_6p_6p_topology/Grid Side Measurements'
//  '<S5>'   : 'sfc_6p_6p_topology/Id1'
//  '<S6>'   : 'sfc_6p_6p_topology/Machine Side Converter'
//  '<S7>'   : 'sfc_6p_6p_topology/Machine Side Measurements'
//  '<S8>'   : 'sfc_6p_6p_topology/Subsystem1'
//  '<S9>'   : 'sfc_6p_6p_topology/powergui1'
//  '<S10>'  : 'sfc_6p_6p_topology/ABC Grid Voltage Source/Controlled Voltage Source'
//  '<S11>'  : 'sfc_6p_6p_topology/ABC Grid Voltage Source/Controlled Voltage Source1'
//  '<S12>'  : 'sfc_6p_6p_topology/ABC Grid Voltage Source/Controlled Voltage Source2'
//  '<S13>'  : 'sfc_6p_6p_topology/ABC Grid Voltage Source/Integrator with Wrapped State (Discrete or Continuous)'
//  '<S14>'  : 'sfc_6p_6p_topology/ABC Grid Voltage Source/Integrator with Wrapped State (Discrete or Continuous)/Discrete'
//  '<S15>'  : 'sfc_6p_6p_topology/ABC Grid Voltage Source/Integrator with Wrapped State (Discrete or Continuous)/Discrete/Compare To Constant'
//  '<S16>'  : 'sfc_6p_6p_topology/ABC Grid Voltage Source/Integrator with Wrapped State (Discrete or Continuous)/Discrete/Compare To Constant1'
//  '<S17>'  : 'sfc_6p_6p_topology/ABC Grid Voltage Source/Integrator with Wrapped State (Discrete or Continuous)/Discrete/Reinitialization'
//  '<S18>'  : 'sfc_6p_6p_topology/ABC Machine Voltage Source/Controlled Voltage Source'
//  '<S19>'  : 'sfc_6p_6p_topology/ABC Machine Voltage Source/Controlled Voltage Source1'
//  '<S20>'  : 'sfc_6p_6p_topology/ABC Machine Voltage Source/Controlled Voltage Source2'
//  '<S21>'  : 'sfc_6p_6p_topology/ABC Machine Voltage Source/Integrator with Wrapped State (Discrete or Continuous)'
//  '<S22>'  : 'sfc_6p_6p_topology/ABC Machine Voltage Source/Integrator with Wrapped State (Discrete or Continuous)/Discrete'
//  '<S23>'  : 'sfc_6p_6p_topology/ABC Machine Voltage Source/Integrator with Wrapped State (Discrete or Continuous)/Discrete/Compare To Constant'
//  '<S24>'  : 'sfc_6p_6p_topology/ABC Machine Voltage Source/Integrator with Wrapped State (Discrete or Continuous)/Discrete/Compare To Constant1'
//  '<S25>'  : 'sfc_6p_6p_topology/ABC Machine Voltage Source/Integrator with Wrapped State (Discrete or Continuous)/Discrete/Reinitialization'
//  '<S26>'  : 'sfc_6p_6p_topology/Grid Side Converter/Model'
//  '<S27>'  : 'sfc_6p_6p_topology/Grid Side Measurements/Mode I'
//  '<S28>'  : 'sfc_6p_6p_topology/Grid Side Measurements/Mode V'
//  '<S29>'  : 'sfc_6p_6p_topology/Grid Side Measurements/Model'
//  '<S30>'  : 'sfc_6p_6p_topology/Grid Side Measurements/Model/I A:'
//  '<S31>'  : 'sfc_6p_6p_topology/Grid Side Measurements/Model/I B:'
//  '<S32>'  : 'sfc_6p_6p_topology/Grid Side Measurements/Model/I C:'
//  '<S33>'  : 'sfc_6p_6p_topology/Grid Side Measurements/Model/U A:'
//  '<S34>'  : 'sfc_6p_6p_topology/Grid Side Measurements/Model/U B:'
//  '<S35>'  : 'sfc_6p_6p_topology/Grid Side Measurements/Model/U C:'
//  '<S36>'  : 'sfc_6p_6p_topology/Grid Side Measurements/Model/I A:/Model'
//  '<S37>'  : 'sfc_6p_6p_topology/Grid Side Measurements/Model/I B:/Model'
//  '<S38>'  : 'sfc_6p_6p_topology/Grid Side Measurements/Model/I C:/Model'
//  '<S39>'  : 'sfc_6p_6p_topology/Grid Side Measurements/Model/U A:/Model'
//  '<S40>'  : 'sfc_6p_6p_topology/Grid Side Measurements/Model/U B:/Model'
//  '<S41>'  : 'sfc_6p_6p_topology/Grid Side Measurements/Model/U C:/Model'
//  '<S42>'  : 'sfc_6p_6p_topology/Id1/Model'
//  '<S43>'  : 'sfc_6p_6p_topology/Machine Side Converter/Model'
//  '<S44>'  : 'sfc_6p_6p_topology/Machine Side Measurements/Mode I'
//  '<S45>'  : 'sfc_6p_6p_topology/Machine Side Measurements/Mode V'
//  '<S46>'  : 'sfc_6p_6p_topology/Machine Side Measurements/Model'
//  '<S47>'  : 'sfc_6p_6p_topology/Machine Side Measurements/Model/I A:'
//  '<S48>'  : 'sfc_6p_6p_topology/Machine Side Measurements/Model/I B:'
//  '<S49>'  : 'sfc_6p_6p_topology/Machine Side Measurements/Model/I C:'
//  '<S50>'  : 'sfc_6p_6p_topology/Machine Side Measurements/Model/U A:'
//  '<S51>'  : 'sfc_6p_6p_topology/Machine Side Measurements/Model/U B:'
//  '<S52>'  : 'sfc_6p_6p_topology/Machine Side Measurements/Model/U C:'
//  '<S53>'  : 'sfc_6p_6p_topology/Machine Side Measurements/Model/I A:/Model'
//  '<S54>'  : 'sfc_6p_6p_topology/Machine Side Measurements/Model/I B:/Model'
//  '<S55>'  : 'sfc_6p_6p_topology/Machine Side Measurements/Model/I C:/Model'
//  '<S56>'  : 'sfc_6p_6p_topology/Machine Side Measurements/Model/U A:/Model'
//  '<S57>'  : 'sfc_6p_6p_topology/Machine Side Measurements/Model/U B:/Model'
//  '<S58>'  : 'sfc_6p_6p_topology/Machine Side Measurements/Model/U C:/Model'
//  '<S59>'  : 'sfc_6p_6p_topology/powergui1/EquivalentModel1'
//  '<S60>'  : 'sfc_6p_6p_topology/powergui1/EquivalentModel1/Gates'
//  '<S61>'  : 'sfc_6p_6p_topology/powergui1/EquivalentModel1/Sources'
//  '<S62>'  : 'sfc_6p_6p_topology/powergui1/EquivalentModel1/Status'
//  '<S63>'  : 'sfc_6p_6p_topology/powergui1/EquivalentModel1/Yout'

#endif                                 // RTW_HEADER_sfc_6p_6p_topology_h_

//
// File trailer for generated code.
//
// [EOF]
//
