//
// File: plant_schematic.cpp
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
#include "plant_schematic.h"
#include "rtwtypes.h"
#include <cstring>
#include <cmath>
#include "rt_look.h"
#include "rt_look1d.h"
#include "plant_schematic_private.h"
#include <emmintrin.h>
#include <array>
#include "rt_defines.h"
#include <cfloat>

extern "C"
{

#include "rt_nonfinite.h"

}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  if (std::isnan(u0) || std::isnan(u1)) {
    y = (rtNaN);
  } else if (std::isinf(u0) && std::isinf(u1)) {
    int32_T tmp;
    int32_T tmp_0;
    if (u0 > 0.0) {
      tmp = 1;
    } else {
      tmp = -1;
    }

    if (u1 > 0.0) {
      tmp_0 = 1;
    } else {
      tmp_0 = -1;
    }

    y = std::atan2(static_cast<real_T>(tmp), static_cast<real_T>(tmp_0));
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = std::atan2(u0, u1);
  }

  return y;
}

void rt_invd5x5_snf(const real_T u[25], real_T y[25])
{
  std::array<int8_T, 5> ipiv;
  std::array<int8_T, 5> p;
  std::array<real_T, 25> x;
  real_T smax;
  int32_T c;
  int32_T c_0;
  int32_T idxmax;
  int32_T jA;
  int32_T jj;
  int32_T kAcol;
  int32_T pipk;
  int32_T temp_tmp;
  for (idxmax = 0; idxmax < 25; idxmax++) {
    y[idxmax] = 0.0;
    x[idxmax] = u[idxmax];
  }

  for (idxmax = 0; idxmax < 5; idxmax++) {
    ipiv[idxmax] = static_cast<int8_T>(idxmax + 1);
  }

  for (pipk = 0; pipk < 4; pipk++) {
    c = pipk * 6 + 2;
    jj = pipk * 6;
    c_0 = 5 - pipk;
    idxmax = 1;
    smax = std::abs(x[jj]);
    for (jA = 2; jA <= c_0; jA++) {
      real_T s;
      s = std::abs(x[(c + jA) - 3]);
      if (s > smax) {
        idxmax = jA;
        smax = s;
      }
    }

    if (x[(c + idxmax) - 3] != 0.0) {
      if (idxmax - 1 != 0) {
        idxmax += pipk;
        ipiv[pipk] = static_cast<int8_T>(idxmax);
        for (jA = 0; jA < 5; jA++) {
          temp_tmp = jA * 5 + pipk;
          smax = x[temp_tmp];
          c_0 = (jA * 5 + idxmax) - 1;
          x[temp_tmp] = x[c_0];
          x[c_0] = smax;
        }
      }

      idxmax = c - pipk;
      for (c_0 = c; c_0 <= idxmax + 3; c_0++) {
        x[c_0 - 1] /= x[jj];
      }
    }

    c_0 = 3 - pipk;
    jA = jj;
    for (kAcol = 0; kAcol <= c_0; kAcol++) {
      smax = x[(kAcol * 5 + jj) + 5];
      if (smax != 0.0) {
        idxmax = jA + 7;
        temp_tmp = (jA - pipk) + 10;
        for (int32_T ijA{idxmax}; ijA <= temp_tmp; ijA++) {
          x[ijA - 1] += x[((c + ijA) - jA) - 8] * -smax;
        }
      }

      jA += 5;
    }
  }

  for (idxmax = 0; idxmax < 5; idxmax++) {
    p[idxmax] = static_cast<int8_T>(idxmax + 1);
  }

  if (ipiv[0] > 1) {
    pipk = p[ipiv[0] - 1];
    p[ipiv[0] - 1] = p[0];
    p[0] = static_cast<int8_T>(pipk);
  }

  if (ipiv[1] > 2) {
    pipk = p[ipiv[1] - 1];
    p[ipiv[1] - 1] = p[1];
    p[1] = static_cast<int8_T>(pipk);
  }

  if (ipiv[2] > 3) {
    pipk = p[ipiv[2] - 1];
    p[ipiv[2] - 1] = p[2];
    p[2] = static_cast<int8_T>(pipk);
  }

  if (ipiv[3] > 4) {
    pipk = p[ipiv[3] - 1];
    p[ipiv[3] - 1] = p[3];
    p[3] = static_cast<int8_T>(pipk);
  }

  for (jA = 0; jA < 5; jA++) {
    jj = (p[jA] - 1) * 5;
    y[jA + jj] = 1.0;
    for (pipk = jA + 1; pipk < 6; pipk++) {
      idxmax = (jj + pipk) - 1;
      if (y[idxmax] != 0.0) {
        for (c_0 = pipk + 1; c_0 < 6; c_0++) {
          c = (jj + c_0) - 1;
          y[c] -= x[((pipk - 1) * 5 + c_0) - 1] * y[idxmax];
        }
      }
    }
  }

  for (pipk = 0; pipk < 5; pipk++) {
    jj = 5 * pipk;
    for (jA = 4; jA >= 0; jA--) {
      kAcol = 5 * jA;
      idxmax = jA + jj;
      smax = y[idxmax];
      if (smax != 0.0) {
        y[idxmax] = smax / x[jA + kAcol];
        temp_tmp = jA - 1;
        for (c_0 = 0; c_0 <= temp_tmp; c_0++) {
          c = c_0 + jj;
          y[c] -= x[c_0 + kAcol] * y[idxmax];
        }
      }
    }
  }
}

real_T rt_remd_snf(real_T u0, real_T u1)
{
  real_T y;
  if (std::isnan(u0) || std::isnan(u1) || std::isinf(u0)) {
    y = (rtNaN);
  } else if (std::isinf(u1)) {
    y = u0;
  } else if ((u1 != 0.0) && (u1 != std::trunc(u1))) {
    real_T q;
    q = std::abs(u0 / u1);
    if (!(std::abs(q - std::floor(q + 0.5)) > DBL_EPSILON * q)) {
      y = 0.0 * u0;
    } else {
      y = std::fmod(u0, u1);
    }
  } else {
    y = std::fmod(u0, u1);
  }

  return y;
}

real_T rt_modd_snf(real_T u0, real_T u1)
{
  real_T y;
  y = u0;
  if (u1 == 0.0) {
    if (u0 == 0.0) {
      y = u1;
    }
  } else if (std::isnan(u0) || std::isnan(u1) || std::isinf(u0)) {
    y = (rtNaN);
  } else if (u0 == 0.0) {
    y = 0.0 / u1;
  } else if (std::isinf(u1)) {
    if ((u1 < 0.0) != (u0 < 0.0)) {
      y = u1;
    }
  } else {
    boolean_T yEq;
    y = std::fmod(u0, u1);
    yEq = (y == 0.0);
    if ((!yEq) && (u1 > std::floor(u1))) {
      real_T q;
      q = std::abs(u0 / u1);
      yEq = !(std::abs(q - std::floor(q + 0.5)) > DBL_EPSILON * q);
    }

    if (yEq) {
      y = u1 * 0.0;
    } else if ((u0 < 0.0) != (u1 < 0.0)) {
      y += u1;
    }
  }

  return y;
}

namespace mpc
{
  namespace model
  {
    // Model step function
    void SynchronousMachineAttachedToInfiniteBusModel::step()
    {
      __m128d tmp;
      __m128d tmp_0;
      std::array<real_T, 25> rtb_Product2;
      std::array<real_T, 25> rtb_UpdateLmdrows234col234;
      std::array<real_T, 5> rtb_changeIqIdcurrentsigns;
      std::array<real_T, 25> rtb_ukuk1;
      std::array<real_T, 5> rtb_xk1;
      real_T rtb_Gain3_a;
      real_T rtb_Gain3_f;
      real_T rtb_Gain3_gg;
      real_T rtb_Gain3_hh;
      real_T rtb_Gain3_i;
      real_T rtb_Gain3_j;
      real_T rtb_Gain3_je;
      real_T rtb_Gain3_o;
      real_T rtb_Gain3_o0;
      real_T rtb_Gain3_py;
      real_T rtb_Integrator_er;
      real_T rtb_Integrator_hz;
      real_T rtb_Integrator_ln;
      real_T rtb_Sum1_ay;
      real_T rtb_Sum3;
      real_T rtb_Sum_a;
      real_T rtb_Sum_i;
      real_T rtb_Sum_jx;
      real_T rtb_Sum_l;
      real_T rtb_Sum_ob;
      real_T rtb_Sum_py;
      real_T rtb_dw_delay;
      real_T rtb_u2H;
      int32_T i;
      int32_T i_0;
      int32_T i_1;

      // UnitDelay: '<S123>/fluxes'
      for (i = 0; i < 5; i++) {
        rtb_xk1[i] = plant_schematic_DW.fluxes_DSTATE[i];
      }

      // UnitDelay: '<S132>/dw_delay'
      rtb_dw_delay = plant_schematic_DW.dw_delay_DSTATE;

      // Sum: '<S132>/Sum1' incorporates:
      //   Gain: '<S132>/F2'
      //   UnitDelay: '<S132>/dw_predict'

      rtb_Sum1_ay = 2.0 * rtb_dw_delay - plant_schematic_DW.dw_predict_DSTATE;

      // Switch: '<S112>/Switch' incorporates:
      //   Constant: '<S112>/Constant4'
      //   Product: '<S122>/inversion'

      std::memcpy(&rtb_ukuk1[0], &plant_schematic_ConstP.Constant4_Value[0], 25U
                  * sizeof(real_T));
      for (i = 0; i < 5; i++) {
        // Product: '<S112>/Product3' incorporates:
        //   UnitDelay: '<S123>/fluxes'

        rtb_Integrator_er = 0.0;
        for (i_0 = 0; i_0 < 5; i_0++) {
          rtb_Integrator_er += rtb_ukuk1[5 * i_0 + i] *
            plant_schematic_DW.fluxes_DSTATE[i_0];
        }

        // Gain: '<S112>/change Iq Id  current signs' incorporates:
        //   Product: '<S112>/Product3'

        rtb_changeIqIdcurrentsigns[i] =
          plant_schematic_ConstP.changeIqIdcurrentsigns_Gain[i] *
          rtb_Integrator_er;
      }

      // Outputs for Enabled SubSystem: '<S68>/Signal generator' incorporates:
      //   EnablePort: '<S70>/Enable'

      // DigitalClock: '<S110>/Digital Clock' incorporates:
      //   Step: '<S74>/Step'
      //   Step: '<S74>/Step1'

      rtb_u2H = (((&plant_schematic_M)->Timing.clockTick0) * 0.0001);

      // End of Outputs for SubSystem: '<S68>/Signal generator'

      // Sum: '<S110>/Sum3' incorporates:
      //   DigitalClock: '<S110>/Digital Clock'
      //   DiscreteIntegrator: '<S110>/Rotor angle dthetae'
      //   Gain: '<S110>/web2'

      rtb_Sum3 = 376.99111843077515 * rtb_u2H +
        plant_schematic_DW.Rotorangledthetae_DSTATE;

      // Trigonometry: '<S113>/Elementary Math'
      rtb_Integrator_er = std::sin(rtb_Sum3);
      rtb_Sum3 = std::cos(rtb_Sum3);

      // Fcn: '<S114>/Fcn'
      rtb_Integrator_hz = rtb_changeIqIdcurrentsigns[0] * rtb_Sum3 +
        rtb_changeIqIdcurrentsigns[1] * rtb_Integrator_er;

      // Fcn: '<S114>/Fcn1'
      rtb_Integrator_ln = ((-rtb_changeIqIdcurrentsigns[0] - 1.7320508075688772 *
                            rtb_changeIqIdcurrentsigns[1]) * rtb_Sum3 +
                           (1.7320508075688772 * rtb_changeIqIdcurrentsigns[0] -
                            rtb_changeIqIdcurrentsigns[1]) * rtb_Integrator_er) *
        0.5;

      // Gain: '<S114>/ib' incorporates:
      //   Sum: '<S114>/Sum'

      plant_schematic_B.ib[0] = 18881.483433953665 * rtb_Integrator_hz;
      plant_schematic_B.ib[1] = 18881.483433953665 * rtb_Integrator_ln;
      plant_schematic_B.ib[2] = ((0.0 - rtb_Integrator_hz) - rtb_Integrator_ln) *
        18881.483433953665;

      // Outputs for Enabled SubSystem: '<S68>/Signal generator' incorporates:
      //   EnablePort: '<S70>/Enable'

      plant_schematic_DW.Signalgenerator_MODE = true;

      // Lookup: '<S75>/Look-Up Table' incorporates:
      //   DigitalClock: '<S110>/Digital Clock'

      plant_schematic_B.LookUpTable = rt_Lookup
        (&plant_schematic_ConstP.LookUpTable_XData[0], 8, rtb_u2H,
         &plant_schematic_ConstP.LookUpTable_YData[0]);

      // Logic: '<S70>/Logical Operator1' incorporates:
      //   Constant: '<S70>/Constant6'
      //   Constant: '<S70>/valp_nom5'
      //   RelationalOperator: '<S70>/Relational Operator3'

      plant_schematic_B.LogicalOperator1 = false;

      // Step: '<S74>/Step1'
      i = (rtb_u2H < 0.0);

      // DiscreteIntegrator: '<S74>/Discrete-Time Integrator1'
      plant_schematic_B.Ramp = plant_schematic_DW.DiscreteTimeIntegrator1_DSTATE;

      // Switch: '<S74>/Switch2'
      if (i >= 0.5) {
        // UnitDelay: '<S74>/Unit Delay' incorporates:
        //   MultiPortSwitch: '<S74>/Multiport Switch1'
        //   Product: '<S74>/Product2'

        plant_schematic_DW.UnitDelay_DSTATE = 0.0;
      }

      // End of Switch: '<S74>/Switch2'

      // Switch: '<S70>/Switch3' incorporates:
      //   Constant: '<S70>/Constant4'

      plant_schematic_B.Switch3 = 0.0;

      // Switch: '<S74>/Switch3' incorporates:
      //   DataTypeConversion: '<S74>/Data Type  Conversion2'
      //   Logic: '<S74>/Logical Operator1'

      if (i == 0) {
        // Switch: '<S70>/Switch2' incorporates:
        //   Constant: '<S74>/Constant1'

        plant_schematic_B.Switch2_e = 0.0;
      } else {
        // Switch: '<S70>/Switch2' incorporates:
        //   UnitDelay: '<S74>/Unit Delay'

        plant_schematic_B.Switch2_e = plant_schematic_DW.UnitDelay_DSTATE;
      }

      // End of Switch: '<S74>/Switch3'

      // DiscreteIntegrator: '<S70>/Discrete-Time Integrator'
      plant_schematic_B.DiscreteTimeIntegrator =
        plant_schematic_DW.DiscreteTimeIntegrator_DSTATE;

      // SignalConversion generated from: '<S70>/frequency'
      plant_schematic_B.OutportBufferForfrequency =
        plant_schematic_B.DiscreteTimeIntegrator;

      // Switch: '<S74>/Switch' incorporates:
      //   Constant: '<S74>/Constant2'
      //   Constant: '<S74>/Constant4'
      //   Step: '<S74>/Step'
      //   Switch: '<S74>/Switch1'

      if ((!(rtb_u2H < 0.0)) >= 0.5) {
        i = 1;
      } else {
        i = 0;
      }

      // Update for DiscreteIntegrator: '<S74>/Discrete-Time Integrator1' incorporates:
      //   Switch: '<S74>/Switch'

      plant_schematic_DW.DiscreteTimeIntegrator1_DSTATE += 0.0001 * static_cast<
        real_T>(i);

      // End of Outputs for SubSystem: '<S68>/Signal generator'

      // Sum: '<S68>/Sum' incorporates:
      //   DigitalClock: '<S110>/Digital Clock'
      //   Product: '<S68>/Product'

      rtb_Integrator_hz = rtb_u2H * plant_schematic_ConstB.Gain +
        plant_schematic_B.OutportBufferForfrequency;

      // Outputs for Enabled SubSystem: '<S68>/Harmonic Generator' incorporates:
      //   EnablePort: '<S69>/Enable'

      if (plant_schematic_DW.HarmonicGenerator_MODE) {
        // Disable for Product: '<S71>/Product1' incorporates:
        //   Outport: '<S69>/Out1'

        plant_schematic_B.Product1[0] = 0.0;

        // Disable for Product: '<S72>/Product1' incorporates:
        //   Outport: '<S69>/Out2'

        plant_schematic_B.Product1_f[0] = 0.0;

        // Disable for Product: '<S71>/Product1' incorporates:
        //   Outport: '<S69>/Out1'

        plant_schematic_B.Product1[1] = 0.0;

        // Disable for Product: '<S72>/Product1' incorporates:
        //   Outport: '<S69>/Out2'

        plant_schematic_B.Product1_f[1] = 0.0;

        // Disable for Product: '<S71>/Product1' incorporates:
        //   Outport: '<S69>/Out1'

        plant_schematic_B.Product1[2] = 0.0;

        // Disable for Product: '<S72>/Product1' incorporates:
        //   Outport: '<S69>/Out2'

        plant_schematic_B.Product1_f[2] = 0.0;
        plant_schematic_DW.HarmonicGenerator_MODE = false;
      }

      // End of Outputs for SubSystem: '<S68>/Harmonic Generator'

      // Sum: '<S68>/Sum5' incorporates:
      //   Constant: '<S68>/P1'
      //   Constant: '<S68>/valp_nom2'
      //   Product: '<S68>/Product1'
      //   Sum: '<S68>/Sum1'
      //   Sum: '<S68>/Sum2'
      //   Sum: '<S68>/Sum3'
      //   Trigonometry: '<S68>/Trigonometric Function'

      plant_schematic_B.Sum5[0] = ((plant_schematic_B.Switch2_e +
        19497.938352554094) * std::sin(rtb_Integrator_hz +
        plant_schematic_ConstB.Gain3) + plant_schematic_B.Product1[0]) +
        plant_schematic_B.Product1_f[0];
      plant_schematic_B.Sum5[1] = (std::sin((plant_schematic_ConstB.Gain3 -
        2.0943951023931953) + rtb_Integrator_hz) * (plant_schematic_B.Switch2_e
        + 19497.938352554094) + plant_schematic_B.Product1[1]) +
        plant_schematic_B.Product1_f[1];
      plant_schematic_B.Sum5[2] = (std::sin((plant_schematic_ConstB.Gain3 +
        2.0943951023931953) + rtb_Integrator_hz) * (plant_schematic_B.Switch2_e
        + 19497.938352554094) + plant_schematic_B.Product1[2]) +
        plant_schematic_B.Product1_f[2];

      // S-Function (sfun_spssw_discc): '<S148>/State-Space'

      // S-Function block: <S148>/State-Space
      {
        real_T accum;

        //
        //  Compute outputs:
        //  ---------------

        real_T *Cs = (real_T*)plant_schematic_DW.StateSpace_PWORK.CS;
        real_T *Ds = (real_T*)plant_schematic_DW.StateSpace_PWORK.DS;

        {
          int_T i1;
          real_T *y0{ &plant_schematic_B.StateSpace[0] };

          for (i1=0; i1 < 8; i1++) {
            accum = 0.0;
            accum += *(Cs++) * plant_schematic_DW.StateSpace_DSTATE[0];
            accum += *(Cs++) * plant_schematic_DW.StateSpace_DSTATE[1];
            accum += *(Cs++) * plant_schematic_DW.StateSpace_DSTATE[2];
            accum += *(Ds++) * plant_schematic_B.ib[0];
            accum += *(Ds++) * plant_schematic_B.ib[1];
            accum += *(Ds++) * plant_schematic_B.Sum5[0];
            accum += *(Ds++) * plant_schematic_B.Sum5[1];
            accum += *(Ds++) * plant_schematic_B.Sum5[2];
            y0[i1] = accum;
          }
        }
      }

      // Gain: '<S113>/1_Vb'
      rtb_Integrator_hz = 5.1031036307982879E-5 * plant_schematic_B.StateSpace[1];

      // Fcn: '<S113>/Fcn2' incorporates:
      //   Fcn: '<S113>/Fcn3'
      //   Gain: '<S113>/1_Vb'

      rtb_Integrator_ln = 5.1031036307982879E-5 * plant_schematic_B.StateSpace[0]
        * 2.0 + rtb_Integrator_hz;

      // Fcn: '<S113>/Fcn2'
      plant_schematic_Y.vq = (1.7320508075688772 * rtb_Integrator_hz *
        rtb_Integrator_er + rtb_Integrator_ln * rtb_Sum3) * 0.33333333333333331;

      // Fcn: '<S113>/Fcn3'
      plant_schematic_Y.vd = (-1.7320508075688772 * rtb_Integrator_hz * rtb_Sum3
        + rtb_Integrator_ln * rtb_Integrator_er) * 0.33333333333333331;

      // DiscreteIntegrator: '<S50>/Integrator' incorporates:
      //   Fcn: '<S117>/x->theta'
      //   Gain: '<S118>/Gain'

      plant_schematic_Y.load_angle_deg = 57.295779513082323 * rt_atan2d_snf
        (plant_schematic_Y.vd, plant_schematic_Y.vq);

      // DiscreteIntegrator: '<S90>/Integrator' incorporates:
      //   Product: '<S116>/Product2'
      //   Product: '<S116>/Product3'
      //   Sum: '<S116>/Sum'

      plant_schematic_Y.pt = plant_schematic_Y.vq * rtb_changeIqIdcurrentsigns[0]
        + plant_schematic_Y.vd * rtb_changeIqIdcurrentsigns[1];

      // DiscreteIntegrator: '<S38>/Integrator' incorporates:
      //   Product: '<S116>/Product4'
      //   Product: '<S116>/Product5'
      //   Sum: '<S116>/Sum1'

      plant_schematic_Y.qt = plant_schematic_Y.vq * rtb_changeIqIdcurrentsigns[1]
        - plant_schematic_Y.vd * rtb_changeIqIdcurrentsigns[0];

      // Gain: '<S111>/output formatting'
      plant_schematic_Y.ifd = 1.35 * rtb_changeIqIdcurrentsigns[2];

      // Outport: '<Root>/id' incorporates:
      //   Gain: '<S111>/output formatting'

      plant_schematic_Y.id = rtb_changeIqIdcurrentsigns[1];

      // Outport: '<Root>/iq' incorporates:
      //   Gain: '<S111>/output formatting'

      plant_schematic_Y.iq = rtb_changeIqIdcurrentsigns[0];

      // Gain: '<S108>/N' incorporates:
      //   Inport: '<Root>/field_voltage'

      rtb_Integrator_er = 0.00039788735772973839 *
        plant_schematic_U.field_voltage;

      // Switch: '<S123>/IC' incorporates:
      //   DigitalClock: '<S110>/Digital Clock'

      if (rtb_u2H >= 0.0001) {
        // Assignment: '<S120>/W(2,1)=-wr' incorporates:
        //   SignalConversion generated from: '<S120>/W(1,2)=wr'

        std::memset(&rtb_UpdateLmdrows234col234[0], 0, 25U * sizeof(real_T));

        // Assignment: '<S120>/W(1,2)=wr' incorporates:
        //   Assignment: '<S120>/W(2,1)=-wr'
        //   Constant: '<S110>/nominal speed'
        //   Sum: '<S110>/Sum'

        rtb_UpdateLmdrows234col234[5] = rtb_Sum1_ay + 1.0;

        // Assignment: '<S120>/W(2,1)=-wr' incorporates:
        //   Constant: '<S110>/nominal speed'
        //   Gain: '<S120>/Gain1'
        //   Sum: '<S110>/Sum'

        rtb_UpdateLmdrows234col234[1] = -(rtb_Sum1_ay + 1.0);
        for (i = 0; i <= 22; i += 2) {
          tmp = _mm_loadu_pd(&rtb_UpdateLmdrows234col234[i]);
          tmp = _mm_mul_pd(_mm_sub_pd(_mm_sub_pd(_mm_set1_pd(0.0), tmp),
            _mm_loadu_pd(&plant_schematic_ConstP.Constant6_Value[i])),
                           _mm_set1_pd(0.018849555921538759));
          _mm_storeu_pd(&rtb_UpdateLmdrows234col234[i], tmp);
          tmp_0 = _mm_loadu_pd(&plant_schematic_ConstP.u5_Value_d[i]);
          _mm_storeu_pd(&rtb_Product2[i], _mm_add_pd(tmp_0, tmp));
          _mm_storeu_pd(&rtb_ukuk1[i], _mm_sub_pd(tmp_0, tmp));
        }

        for (i = 24; i < 25; i++) {
          // Gain: '<S131>/wbase*Ts//2' incorporates:
          //   Assignment: '<S120>/W(2,1)=-wr'
          //   Constant: '<S112>/Constant6'
          //   Gain: '<S131>/wbase*Ts//2 '
          //   Sum: '<S112>/Sum1'
          //   Switch: '<S112>/Switch1'

          rtb_u2H = ((0.0 - rtb_UpdateLmdrows234col234[i]) -
                     plant_schematic_ConstP.Constant6_Value[i]) *
            0.018849555921538759;
          rtb_UpdateLmdrows234col234[i] = rtb_u2H;

          // Sum: '<S131>/Sum5' incorporates:
          //   Constant: '<S131>/u5'
          //   Gain: '<S131>/wbase*Ts//2 '
          //   Product: '<S123>/Product2'

          rtb_Sum3 = plant_schematic_ConstP.u5_Value_d[i];
          rtb_Product2[i] = rtb_Sum3 + rtb_u2H;

          // Sum: '<S131>/Sum1' incorporates:
          //   Constant: '<S131>/u5'
          //   Gain: '<S131>/wbase*Ts//2 '

          rtb_ukuk1[i] = rtb_Sum3 - rtb_u2H;
        }

        // Product: '<S131>/inversion' incorporates:
        //   Assignment: '<S120>/W(2,1)=-wr'
        //   Constant: '<S112>/Constant6'
        //   Constant: '<S131>/u5'
        //   Gain: '<S131>/wbase*Ts//2'
        //   Gain: '<S131>/wbase*Ts//2 '
        //   Product: '<S123>/Product2'
        //   Sum: '<S112>/Sum1'
        //   Sum: '<S131>/Sum1'
        //   Sum: '<S131>/Sum5'
        //   Switch: '<S112>/Switch1'

        rt_invd5x5_snf(&rtb_ukuk1[0], &rtb_UpdateLmdrows234col234[0]);

        // Product: '<S131>/Product4' incorporates:
        //   Gain: '<S131>/wbase*Ts//2 '
        //   Product: '<S123>/Product2'
        //   Sum: '<S123>/sum'

        for (i = 0; i < 5; i++) {
          for (i_0 = 0; i_0 < 5; i_0++) {
            rtb_u2H = 0.0;
            for (i_1 = 0; i_1 < 5; i_1++) {
              rtb_u2H += rtb_UpdateLmdrows234col234[5 * i_1 + i_0] *
                rtb_Product2[5 * i + i_1];
            }

            rtb_ukuk1[i_0 + 5 * i] = rtb_u2H;
          }
        }

        // End of Product: '<S131>/Product4'

        // Product: '<S123>/Product2' incorporates:
        //   Sum: '<S123>/sum'
        //   UnitDelay: '<S123>/fluxes'

        for (i = 0; i < 5; i++) {
          rtb_u2H = 0.0;
          for (i_0 = 0; i_0 < 5; i_0++) {
            rtb_u2H += rtb_ukuk1[5 * i_0 + i] *
              plant_schematic_DW.fluxes_DSTATE[i_0];
          }

          rtb_Product2[i] = rtb_u2H;
        }

        // End of Product: '<S123>/Product2'

        // Sum: '<S123>/sum' incorporates:
        //   UnitDelay: '<S123>/voltages'

        rtb_ukuk1[0] = plant_schematic_Y.vq +
          plant_schematic_DW.voltages_DSTATE[0];
        rtb_ukuk1[1] = plant_schematic_Y.vd +
          plant_schematic_DW.voltages_DSTATE[1];
        rtb_ukuk1[2] = rtb_Integrator_er + plant_schematic_DW.voltages_DSTATE[2];
        rtb_ukuk1[3] = plant_schematic_DW.voltages_DSTATE[3];
        rtb_ukuk1[4] = plant_schematic_DW.voltages_DSTATE[4];

        // Product: '<S123>/Product1' incorporates:
        //   Gain: '<S131>/wbase*Ts//2 '

        for (i = 0; i < 5; i++) {
          rtb_u2H = 0.0;
          for (i_0 = 0; i_0 < 5; i_0++) {
            rtb_u2H += rtb_UpdateLmdrows234col234[5 * i_0 + i] *
              0.018849555921538759 * rtb_ukuk1[i_0];
          }

          // UnitDelay: '<S123>/fluxes' incorporates:
          //   Gain: '<S131>/wbase*Ts//2 '
          //   Product: '<S123>/Product1'
          //   Sum: '<S123>/Ad*x(k-1) + Bd*( u(k-1) + u(k))'

          plant_schematic_DW.fluxes_DSTATE[i] = rtb_Product2[i] + rtb_u2H;
        }

        // End of Product: '<S123>/Product1'
      }

      // End of Switch: '<S123>/IC'

      // Gain: '<S110>/1 ----- 2H' incorporates:
      //   Constant: '<S110>/nominal speed'
      //   Fcn: '<S110>/div'
      //   Gain: '<S110>/F'
      //   Gain: '<S119>/1-1'
      //   Inport: '<Root>/mechanical_power'
      //   Product: '<S119>/Mult1'
      //   Sum: '<S110>/Sum'
      //   Sum: '<S110>/Sum2'
      //   Sum: '<S119>/Sum2'

      rtb_u2H = ((plant_schematic_U.mechanical_power / (rtb_Sum1_ay + 1.0) -
                  (rtb_changeIqIdcurrentsigns[0] * rtb_xk1[1] + rtb_xk1[0] *
                   -rtb_changeIqIdcurrentsigns[1])) - (rtb_Sum1_ay + 1.0) * 0.5)
        * 0.14285714285714285;

      // DiscreteIntegrator: '<S110>/Rotor speed deviation (dw)'
      if (plant_schematic_DW.Rotorspeeddeviationdw_SYSTEM_EN == 0) {
        // DiscreteIntegrator: '<S110>/Rotor speed deviation (dw)'
        plant_schematic_DW.Rotorspeeddeviationdw_DSTATE += 5.0E-5 * rtb_u2H;
      }

      // End of DiscreteIntegrator: '<S110>/Rotor speed deviation (dw)'

      // DiscreteIntegrator: '<S50>/Integrator' incorporates:
      //   Gain: '<S111>/output formatting'
      //   Math: '<S78>/Square'
      //   Math: '<S78>/Square1'
      //   Sqrt: '<S78>/Sqrt'
      //   Sum: '<S78>/Sum'

      plant_schematic_Y.st = std::sqrt(plant_schematic_Y.vq *
        plant_schematic_Y.vq + plant_schematic_Y.vd * plant_schematic_Y.vd);

      // Outport: '<Root>/vt'
      plant_schematic_Y.vt = plant_schematic_Y.st;

      // DiscreteIntegrator: '<S90>/Integrator' incorporates:
      //   Constant: '<S16>/Constant3'

      if (plant_schematic_DW.Integrator_PrevResetState != 0) {
        plant_schematic_DW.Integrator_DSTATE = 0.0;
      }

      // Gain: '<S16>/Gain3' incorporates:
      //   DiscreteIntegrator: '<S90>/Integrator'

      rtb_Integrator_hz = 568489.21350274689 *
        plant_schematic_DW.Integrator_DSTATE;

      // DiscreteIntegrator: '<S91>/Integrator' incorporates:
      //   Constant: '<S16>/Constant2'

      if (plant_schematic_DW.Integrator_PrevResetState_c != 0) {
        plant_schematic_DW.Integrator_DSTATE_m = 0.0;
      }

      // Sum: '<S16>/Sum' incorporates:
      //   DiscreteIntegrator: '<S91>/Integrator'
      //   Gain: '<S16>/Gain'

      rtb_Sum3 = (plant_schematic_Y.st - rtb_Integrator_hz) - 1066.1308829222321
        * plant_schematic_DW.Integrator_DSTATE_m;

      // DiscreteIntegrator: '<S54>/Integrator' incorporates:
      //   Constant: '<S8>/Constant3'

      if (plant_schematic_DW.Integrator_PrevResetState_l != 0) {
        plant_schematic_DW.Integrator_DSTATE_a = 0.0;
      }

      // Gain: '<S8>/Gain3' incorporates:
      //   DiscreteIntegrator: '<S54>/Integrator'

      rtb_Integrator_ln = 142122.30337568672 *
        plant_schematic_DW.Integrator_DSTATE_a;

      // DiscreteIntegrator: '<S55>/Integrator' incorporates:
      //   Constant: '<S8>/Constant2'

      if (plant_schematic_DW.Integrator_PrevResetState_o != 0) {
        plant_schematic_DW.Integrator_DSTATE_g = 0.0;
      }

      // Sum: '<S8>/Sum' incorporates:
      //   DiscreteIntegrator: '<S55>/Integrator'
      //   Gain: '<S8>/Gain'
      //   MultiPortSwitch: '<S16>/Multiport Switch'

      rtb_Integrator_hz = (rtb_Integrator_hz - rtb_Integrator_ln) -
        533.065441461116 * plant_schematic_DW.Integrator_DSTATE_g;

      // Outport: '<Root>/vt_filtered' incorporates:
      //   MultiPortSwitch: '<S8>/Multiport Switch'
      //   Sum: '<S8>/Sum1'

      plant_schematic_Y.vt_filtered = rtb_Integrator_hz + rtb_Integrator_ln;

      // DiscreteIntegrator: '<S50>/Integrator' incorporates:
      //   Gain: '<S111>/output formatting'
      //   Math: '<S77>/Square'
      //   Math: '<S77>/Square1'
      //   Sqrt: '<S77>/Sqrt'
      //   Sum: '<S77>/Sum'

      plant_schematic_Y.st = std::sqrt(rtb_changeIqIdcurrentsigns[0] *
        rtb_changeIqIdcurrentsigns[0] + rtb_changeIqIdcurrentsigns[1] *
        rtb_changeIqIdcurrentsigns[1]);

      // Outport: '<Root>/it'
      plant_schematic_Y.it = plant_schematic_Y.st;

      // DiscreteIntegrator: '<S66>/Integrator' incorporates:
      //   Constant: '<S11>/Constant3'

      if (plant_schematic_DW.Integrator_PrevResetState_cb != 0) {
        plant_schematic_DW.Integrator_DSTATE_p = 0.0;
      }

      // Gain: '<S11>/Gain3' incorporates:
      //   DiscreteIntegrator: '<S66>/Integrator'

      rtb_Gain3_je = 142122.30337568672 * plant_schematic_DW.Integrator_DSTATE_p;

      // DiscreteIntegrator: '<S67>/Integrator' incorporates:
      //   Constant: '<S11>/Constant2'

      if (plant_schematic_DW.Integrator_PrevResetState_k != 0) {
        plant_schematic_DW.Integrator_DSTATE_pj = 0.0;
      }

      // Sum: '<S11>/Sum' incorporates:
      //   DiscreteIntegrator: '<S67>/Integrator'
      //   Gain: '<S11>/Gain'

      rtb_Integrator_ln = (plant_schematic_Y.st - rtb_Gain3_je) -
        533.065441461116 * plant_schematic_DW.Integrator_DSTATE_pj;

      // Outport: '<Root>/it_filtered' incorporates:
      //   MultiPortSwitch: '<S11>/Multiport Switch'
      //   Sum: '<S11>/Sum1'

      plant_schematic_Y.it_filtered = rtb_Integrator_ln + rtb_Gain3_je;

      // DiscreteIntegrator: '<S50>/Integrator' incorporates:
      //   Gain: '<S111>/output formatting'
      //   Math: '<S76>/Square'
      //   Math: '<S76>/Square1'
      //   Sqrt: '<S76>/Sqrt'
      //   Sum: '<S76>/Sum'

      plant_schematic_Y.st = std::sqrt(plant_schematic_Y.pt *
        plant_schematic_Y.pt + plant_schematic_Y.qt * plant_schematic_Y.qt);

      // DiscreteIntegrator: '<S102>/Integrator' incorporates:
      //   Constant: '<S19>/Constant3'

      if (plant_schematic_DW.Integrator_PrevResetState_l1 != 0) {
        plant_schematic_DW.Integrator_DSTATE_az = 0.0;
      }

      // Gain: '<S19>/Gain3' incorporates:
      //   DiscreteIntegrator: '<S102>/Integrator'

      rtb_Gain3_je = 568489.21350274689 *
        plant_schematic_DW.Integrator_DSTATE_az;

      // DiscreteIntegrator: '<S103>/Integrator' incorporates:
      //   Constant: '<S19>/Constant2'

      if (plant_schematic_DW.Integrator_PrevResetState_p != 0) {
        plant_schematic_DW.Integrator_DSTATE_f = 0.0;
      }

      // DiscreteIntegrator: '<S46>/Integrator' incorporates:
      //   Constant: '<S6>/Constant3'

      if (plant_schematic_DW.Integrator_PrevResetState_d != 0) {
        plant_schematic_DW.Integrator_DSTATE_j = 0.0;
      }

      // Gain: '<S6>/Gain3' incorporates:
      //   DiscreteIntegrator: '<S46>/Integrator'

      rtb_Gain3_a = 142122.30337568672 * plant_schematic_DW.Integrator_DSTATE_j;

      // DiscreteIntegrator: '<S47>/Integrator' incorporates:
      //   Constant: '<S6>/Constant2'

      if (plant_schematic_DW.Integrator_PrevResetState_f != 0) {
        plant_schematic_DW.Integrator_DSTATE_h = 0.0;
      }

      // Sum: '<S6>/Sum' incorporates:
      //   DiscreteIntegrator: '<S47>/Integrator'
      //   Gain: '<S6>/Gain'
      //   MultiPortSwitch: '<S19>/Multiport Switch'

      rtb_Sum_i = (rtb_Gain3_je - rtb_Gain3_a) - 533.065441461116 *
        plant_schematic_DW.Integrator_DSTATE_h;

      // Outport: '<Root>/st_filtered' incorporates:
      //   MultiPortSwitch: '<S6>/Multiport Switch'
      //   Sum: '<S6>/Sum1'

      plant_schematic_Y.st_filtered = rtb_Sum_i + rtb_Gain3_a;

      // DiscreteIntegrator: '<S50>/Integrator' incorporates:
      //   Constant: '<S7>/Constant3'

      if (plant_schematic_DW.Integrator_PrevResetState_dr != 0) {
        plant_schematic_DW.Integrator_DSTATE_j1 = 0.0;
      }

      // Gain: '<S7>/Gain3' incorporates:
      //   DiscreteIntegrator: '<S50>/Integrator'

      rtb_Gain3_i = 142122.30337568672 * plant_schematic_DW.Integrator_DSTATE_j1;

      // DiscreteIntegrator: '<S51>/Integrator' incorporates:
      //   Constant: '<S7>/Constant2'

      if (plant_schematic_DW.Integrator_PrevResetState_j != 0) {
        plant_schematic_DW.Integrator_DSTATE_jz = 0.0;
      }

      // Sum: '<S7>/Sum' incorporates:
      //   DiscreteIntegrator: '<S51>/Integrator'
      //   Gain: '<S7>/Gain'

      rtb_Gain3_a = (plant_schematic_Y.ifd - rtb_Gain3_i) - 533.065441461116 *
        plant_schematic_DW.Integrator_DSTATE_jz;

      // Outport: '<Root>/ifd_filtered' incorporates:
      //   MultiPortSwitch: '<S7>/Multiport Switch'
      //   Sum: '<S7>/Sum1'

      plant_schematic_Y.ifd_filtered = rtb_Gain3_a + rtb_Gain3_i;

      // DiscreteIntegrator: '<S94>/Integrator' incorporates:
      //   Constant: '<S17>/Constant3'

      if (plant_schematic_DW.Integrator_PrevResetState_c5 != 0) {
        plant_schematic_DW.Integrator_DSTATE_d = 0.0;
      }

      // Gain: '<S17>/Gain3' incorporates:
      //   DiscreteIntegrator: '<S94>/Integrator'

      rtb_Gain3_i = 568489.21350274689 * plant_schematic_DW.Integrator_DSTATE_d;

      // DiscreteIntegrator: '<S95>/Integrator' incorporates:
      //   Constant: '<S17>/Constant2'

      if (plant_schematic_DW.Integrator_PrevResetState_g != 0) {
        plant_schematic_DW.Integrator_DSTATE_k = 0.0;
      }

      // DiscreteIntegrator: '<S38>/Integrator' incorporates:
      //   Constant: '<S4>/Constant3'

      if (plant_schematic_DW.Integrator_PrevResetState_e != 0) {
        plant_schematic_DW.Integrator_DSTATE_i = 0.0;
      }

      // Gain: '<S4>/Gain3' incorporates:
      //   DiscreteIntegrator: '<S38>/Integrator'

      rtb_Gain3_f = 142122.30337568672 * plant_schematic_DW.Integrator_DSTATE_i;

      // DiscreteIntegrator: '<S39>/Integrator' incorporates:
      //   Constant: '<S4>/Constant2'

      if (plant_schematic_DW.Integrator_PrevResetState_m != 0) {
        plant_schematic_DW.Integrator_DSTATE_mj = 0.0;
      }

      // Sum: '<S4>/Sum' incorporates:
      //   DiscreteIntegrator: '<S39>/Integrator'
      //   Gain: '<S4>/Gain'
      //   MultiPortSwitch: '<S17>/Multiport Switch'

      rtb_Sum_jx = (rtb_Gain3_i - rtb_Gain3_f) - 533.065441461116 *
        plant_schematic_DW.Integrator_DSTATE_mj;

      // Outport: '<Root>/pt_filtered' incorporates:
      //   MultiPortSwitch: '<S4>/Multiport Switch'
      //   Sum: '<S4>/Sum1'

      plant_schematic_Y.pt_filtered = rtb_Sum_jx + rtb_Gain3_f;

      // DiscreteIntegrator: '<S98>/Integrator' incorporates:
      //   Constant: '<S18>/Constant3'

      if (plant_schematic_DW.Integrator_PrevResetState_mu != 0) {
        plant_schematic_DW.Integrator_DSTATE_mjg = 0.0;
      }

      // Gain: '<S18>/Gain3' incorporates:
      //   DiscreteIntegrator: '<S98>/Integrator'

      rtb_Gain3_f = 568489.21350274689 *
        plant_schematic_DW.Integrator_DSTATE_mjg;

      // DiscreteIntegrator: '<S99>/Integrator' incorporates:
      //   Constant: '<S18>/Constant2'

      if (plant_schematic_DW.Integrator_PrevResetState_ey != 0) {
        plant_schematic_DW.Integrator_DSTATE_dl = 0.0;
      }

      // DiscreteIntegrator: '<S42>/Integrator' incorporates:
      //   Constant: '<S5>/Constant3'

      if (plant_schematic_DW.Integrator_PrevResetState_a != 0) {
        plant_schematic_DW.Integrator_DSTATE_jo = 0.0;
      }

      // Gain: '<S5>/Gain3' incorporates:
      //   DiscreteIntegrator: '<S42>/Integrator'

      rtb_Gain3_hh = 142122.30337568672 *
        plant_schematic_DW.Integrator_DSTATE_jo;

      // DiscreteIntegrator: '<S43>/Integrator' incorporates:
      //   Constant: '<S5>/Constant2'

      if (plant_schematic_DW.Integrator_PrevResetState_lg != 0) {
        plant_schematic_DW.Integrator_DSTATE_mw = 0.0;
      }

      // Sum: '<S5>/Sum' incorporates:
      //   DiscreteIntegrator: '<S43>/Integrator'
      //   Gain: '<S5>/Gain'
      //   MultiPortSwitch: '<S18>/Multiport Switch'

      rtb_Sum_l = (rtb_Gain3_f - rtb_Gain3_hh) - 533.065441461116 *
        plant_schematic_DW.Integrator_DSTATE_mw;

      // Outport: '<Root>/qt_filtered' incorporates:
      //   MultiPortSwitch: '<S5>/Multiport Switch'
      //   Sum: '<S5>/Sum1'

      plant_schematic_Y.qt_filtered = rtb_Sum_l + rtb_Gain3_hh;

      // DiscreteIntegrator: '<S86>/Integrator' incorporates:
      //   Constant: '<S15>/Constant3'

      if (plant_schematic_DW.Integrator_PrevResetState_dg != 0) {
        plant_schematic_DW.Integrator_DSTATE_fa = 0.0;
      }

      // Gain: '<S15>/Gain3' incorporates:
      //   DiscreteIntegrator: '<S86>/Integrator'

      rtb_Gain3_hh = 568489.21350274689 *
        plant_schematic_DW.Integrator_DSTATE_fa;

      // DiscreteIntegrator: '<S87>/Integrator' incorporates:
      //   Constant: '<S15>/Constant2'

      if (plant_schematic_DW.Integrator_PrevResetState_jy != 0) {
        plant_schematic_DW.Integrator_DSTATE_j3 = 0.0;
      }

      // DiscreteIntegrator: '<S58>/Integrator' incorporates:
      //   Constant: '<S9>/Constant3'

      if (plant_schematic_DW.Integrator_PrevResetState_op != 0) {
        plant_schematic_DW.Integrator_DSTATE_mjd = 0.0;
      }

      // Gain: '<S9>/Gain3' incorporates:
      //   DiscreteIntegrator: '<S58>/Integrator'

      rtb_Gain3_o = 142122.30337568672 *
        plant_schematic_DW.Integrator_DSTATE_mjd;

      // DiscreteIntegrator: '<S59>/Integrator' incorporates:
      //   Constant: '<S9>/Constant2'

      if (plant_schematic_DW.Integrator_PrevResetState_n != 0) {
        plant_schematic_DW.Integrator_DSTATE_ma = 0.0;
      }

      // Sum: '<S9>/Sum' incorporates:
      //   DiscreteIntegrator: '<S59>/Integrator'
      //   Gain: '<S9>/Gain'
      //   MultiPortSwitch: '<S15>/Multiport Switch'

      rtb_Sum_py = (rtb_Gain3_hh - rtb_Gain3_o) - 533.065441461116 *
        plant_schematic_DW.Integrator_DSTATE_ma;

      // Outport: '<Root>/vd_filtered' incorporates:
      //   MultiPortSwitch: '<S9>/Multiport Switch'
      //   Sum: '<S9>/Sum1'

      plant_schematic_Y.vd_filtered = rtb_Sum_py + rtb_Gain3_o;

      // DiscreteIntegrator: '<S106>/Integrator' incorporates:
      //   Constant: '<S20>/Constant3'

      if (plant_schematic_DW.Integrator_PrevResetState_pe != 0) {
        plant_schematic_DW.Integrator_DSTATE_c = 0.0;
      }

      // Gain: '<S20>/Gain3' incorporates:
      //   DiscreteIntegrator: '<S106>/Integrator'

      rtb_Gain3_o = 568489.21350274689 * plant_schematic_DW.Integrator_DSTATE_c;

      // DiscreteIntegrator: '<S107>/Integrator' incorporates:
      //   Constant: '<S20>/Constant2'

      if (plant_schematic_DW.Integrator_PrevResetState_pi != 0) {
        plant_schematic_DW.Integrator_DSTATE_fj = 0.0;
      }

      // DiscreteIntegrator: '<S62>/Integrator' incorporates:
      //   Constant: '<S10>/Constant3'

      if (plant_schematic_DW.Integrator_PrevResetState_h != 0) {
        plant_schematic_DW.Integrator_DSTATE_jp = 0.0;
      }

      // Gain: '<S10>/Gain3' incorporates:
      //   DiscreteIntegrator: '<S62>/Integrator'

      rtb_Gain3_o0 = 142122.30337568672 *
        plant_schematic_DW.Integrator_DSTATE_jp;

      // DiscreteIntegrator: '<S63>/Integrator' incorporates:
      //   Constant: '<S10>/Constant2'

      if (plant_schematic_DW.Integrator_PrevResetState_b != 0) {
        plant_schematic_DW.Integrator_DSTATE_b = 0.0;
      }

      // Sum: '<S10>/Sum' incorporates:
      //   DiscreteIntegrator: '<S63>/Integrator'
      //   Gain: '<S10>/Gain'
      //   MultiPortSwitch: '<S20>/Multiport Switch'

      rtb_Sum_a = (rtb_Gain3_o - rtb_Gain3_o0) - 533.065441461116 *
        plant_schematic_DW.Integrator_DSTATE_b;

      // Outport: '<Root>/vq_filtered' incorporates:
      //   MultiPortSwitch: '<S10>/Multiport Switch'
      //   Sum: '<S10>/Sum1'

      plant_schematic_Y.vq_filtered = rtb_Sum_a + rtb_Gain3_o0;

      // DiscreteIntegrator: '<S26>/Integrator' incorporates:
      //   Constant: '<S1>/Constant3'

      if (plant_schematic_DW.Integrator_PrevResetState_ph != 0) {
        plant_schematic_DW.Integrator_DSTATE_l = 0.0;
      }

      // Gain: '<S1>/Gain3' incorporates:
      //   DiscreteIntegrator: '<S26>/Integrator'

      rtb_Gain3_py = 142122.30337568672 * plant_schematic_DW.Integrator_DSTATE_l;

      // DiscreteIntegrator: '<S27>/Integrator' incorporates:
      //   Constant: '<S1>/Constant2'

      if (plant_schematic_DW.Integrator_PrevResetState_hz != 0) {
        plant_schematic_DW.Integrator_DSTATE_mjm = 0.0;
      }

      // Sum: '<S1>/Sum' incorporates:
      //   DiscreteIntegrator: '<S27>/Integrator'
      //   Gain: '<S111>/output formatting'
      //   Gain: '<S1>/Gain'

      rtb_Gain3_o0 = (rtb_changeIqIdcurrentsigns[1] - rtb_Gain3_py) -
        533.065441461116 * plant_schematic_DW.Integrator_DSTATE_mjm;

      // Outport: '<Root>/id_filtered' incorporates:
      //   MultiPortSwitch: '<S1>/Multiport Switch'
      //   Sum: '<S1>/Sum1'

      plant_schematic_Y.id_filtered = rtb_Gain3_o0 + rtb_Gain3_py;

      // DiscreteIntegrator: '<S30>/Integrator' incorporates:
      //   Constant: '<S2>/Constant3'

      if (plant_schematic_DW.Integrator_PrevResetState_da != 0) {
        plant_schematic_DW.Integrator_DSTATE_c0 = 0.0;
      }

      // Gain: '<S2>/Gain3' incorporates:
      //   DiscreteIntegrator: '<S30>/Integrator'

      rtb_Gain3_j = 142122.30337568672 * plant_schematic_DW.Integrator_DSTATE_c0;

      // DiscreteIntegrator: '<S31>/Integrator' incorporates:
      //   Constant: '<S2>/Constant2'

      if (plant_schematic_DW.Integrator_PrevResetState_o2 != 0) {
        plant_schematic_DW.Integrator_DSTATE_l5 = 0.0;
      }

      // Sum: '<S2>/Sum' incorporates:
      //   DiscreteIntegrator: '<S31>/Integrator'
      //   Gain: '<S111>/output formatting'
      //   Gain: '<S2>/Gain'

      rtb_Gain3_py = (rtb_changeIqIdcurrentsigns[0] - rtb_Gain3_j) -
        533.065441461116 * plant_schematic_DW.Integrator_DSTATE_l5;

      // Outport: '<Root>/iq_filtered' incorporates:
      //   MultiPortSwitch: '<S2>/Multiport Switch'
      //   Sum: '<S2>/Sum1'

      plant_schematic_Y.iq_filtered = rtb_Gain3_py + rtb_Gain3_j;

      // DiscreteIntegrator: '<S82>/Integrator' incorporates:
      //   Constant: '<S14>/Constant3'

      if (plant_schematic_DW.Integrator_PrevResetState_dx != 0) {
        plant_schematic_DW.Integrator_DSTATE_df = 0.0;
      }

      // Gain: '<S14>/Gain3' incorporates:
      //   DiscreteIntegrator: '<S82>/Integrator'

      rtb_Gain3_j = 568489.21350274689 * plant_schematic_DW.Integrator_DSTATE_df;

      // DiscreteIntegrator: '<S83>/Integrator' incorporates:
      //   Constant: '<S14>/Constant2'

      if (plant_schematic_DW.Integrator_PrevResetState_lx != 0) {
        plant_schematic_DW.Integrator_DSTATE_p4 = 0.0;
      }

      // DiscreteIntegrator: '<S34>/Integrator' incorporates:
      //   Constant: '<S3>/Constant3'

      if (plant_schematic_DW.Integrator_PrevResetState_py != 0) {
        plant_schematic_DW.Integrator_DSTATE_lc = 0.0;
      }

      // Gain: '<S3>/Gain3' incorporates:
      //   DiscreteIntegrator: '<S34>/Integrator'

      rtb_Gain3_gg = 142122.30337568672 *
        plant_schematic_DW.Integrator_DSTATE_lc;

      // DiscreteIntegrator: '<S35>/Integrator' incorporates:
      //   Constant: '<S3>/Constant2'

      if (plant_schematic_DW.Integrator_PrevResetState_ob != 0) {
        plant_schematic_DW.Integrator_DSTATE_au = 0.0;
      }

      // Sum: '<S3>/Sum' incorporates:
      //   DiscreteIntegrator: '<S35>/Integrator'
      //   Gain: '<S3>/Gain'
      //   MultiPortSwitch: '<S14>/Multiport Switch'

      rtb_Sum_ob = (rtb_Gain3_j - rtb_Gain3_gg) - 533.065441461116 *
        plant_schematic_DW.Integrator_DSTATE_au;

      // Outport: '<Root>/load_angle_deg_filtered' incorporates:
      //   MultiPortSwitch: '<S3>/Multiport Switch'
      //   Sum: '<S3>/Sum1'

      plant_schematic_Y.load_angle_deg_filtered = rtb_Sum_ob + rtb_Gain3_gg;

      // Outport: '<Root>/wm' incorporates:
      //   Constant: '<S110>/nominal speed'
      //   Sum: '<S110>/Sum'

      plant_schematic_Y.wm = rtb_Sum1_ay + 1.0;

      // Outport: '<Root>/wt' incorporates:
      //   Constant: '<S79>/Constant2'
      //   Constant: '<S79>/Constant3'
      //   DiscreteIntegrator: '<S110>/theta'
      //   Fcn: '<S110>/Fcn'
      //   Gain: '<S110>/t'
      //   Gain: '<S13>/Gain'
      //   Math: '<S79>/Mod'
      //   Sum: '<S79>/Sum3'
      //   Sum: '<S79>/Sum4'

      plant_schematic_Y.wt = rt_modd_snf(57.295779513082323 * rt_remd_snf
        (plant_schematic_DW.theta_DSTATE, 6.2831853071795862) *
        0.017453292519943295 + 3.1415926535897931, 6.2831853071795862) -
        3.1415926535897931;

      // Update for UnitDelay: '<S132>/dw_delay'
      plant_schematic_DW.dw_delay_DSTATE =
        plant_schematic_DW.Rotorspeeddeviationdw_DSTATE;

      // Update for UnitDelay: '<S132>/dw_predict'
      plant_schematic_DW.dw_predict_DSTATE = rtb_dw_delay;

      // Update for DiscreteIntegrator: '<S110>/Rotor angle dthetae' incorporates:
      //   Gain: '<S110>/we base'

      plant_schematic_DW.Rotorangledthetae_DSTATE += 376.99111843077515 *
        rtb_Sum1_ay * 0.0001;

      // Update for S-Function (sfun_spssw_discc): '<S148>/State-Space'

      // S-Function block: <S148>/State-Space
      {
        const real_T *As = (real_T*)plant_schematic_DW.StateSpace_PWORK.AS;
        const real_T *Bs = (real_T*)plant_schematic_DW.StateSpace_PWORK.BS;
        real_T *xtmp = (real_T*)plant_schematic_DW.StateSpace_PWORK.XTMP;
        real_T accum;

        // Calculate new states...
        accum = 0.0;
        accum += *(As++) * plant_schematic_DW.StateSpace_DSTATE[0];
        accum += *(As++) * plant_schematic_DW.StateSpace_DSTATE[1];
        accum += *(As++) * plant_schematic_DW.StateSpace_DSTATE[2];
        accum += *(Bs++) * plant_schematic_B.ib[0];
        accum += *(Bs++) * plant_schematic_B.ib[1];
        accum += *(Bs++) * plant_schematic_B.Sum5[0];
        accum += *(Bs++) * plant_schematic_B.Sum5[1];
        accum += *(Bs++) * plant_schematic_B.Sum5[2];
        xtmp[0] = accum;
        accum = 0.0;
        accum += *(As++) * plant_schematic_DW.StateSpace_DSTATE[0];
        accum += *(As++) * plant_schematic_DW.StateSpace_DSTATE[1];
        accum += *(As++) * plant_schematic_DW.StateSpace_DSTATE[2];
        accum += *(Bs++) * plant_schematic_B.ib[0];
        accum += *(Bs++) * plant_schematic_B.ib[1];
        accum += *(Bs++) * plant_schematic_B.Sum5[0];
        accum += *(Bs++) * plant_schematic_B.Sum5[1];
        accum += *(Bs++) * plant_schematic_B.Sum5[2];
        xtmp[1] = accum;
        accum = 0.0;
        accum += *(As++) * plant_schematic_DW.StateSpace_DSTATE[0];
        accum += *(As++) * plant_schematic_DW.StateSpace_DSTATE[1];
        accum += *(As++) * plant_schematic_DW.StateSpace_DSTATE[2];
        accum += *(Bs++) * plant_schematic_B.ib[0];
        accum += *(Bs++) * plant_schematic_B.ib[1];
        accum += *(Bs++) * plant_schematic_B.Sum5[0];
        accum += *(Bs++) * plant_schematic_B.Sum5[1];
        accum += *(Bs++) * plant_schematic_B.Sum5[2];
        xtmp[2] = accum;
        plant_schematic_DW.StateSpace_DSTATE[0] = xtmp[0];
        plant_schematic_DW.StateSpace_DSTATE[1] = xtmp[1];
        plant_schematic_DW.StateSpace_DSTATE[2] = xtmp[2];
      }

      // Update for UnitDelay: '<S123>/voltages' incorporates:
      //   Constant: '<S108>/[ Vkd =0 Vkq1=0  Vkq2=0 ]'

      plant_schematic_DW.voltages_DSTATE[0] = plant_schematic_Y.vq;
      plant_schematic_DW.voltages_DSTATE[1] = plant_schematic_Y.vd;
      plant_schematic_DW.voltages_DSTATE[2] = rtb_Integrator_er;
      plant_schematic_DW.voltages_DSTATE[3] = 0.0;
      plant_schematic_DW.voltages_DSTATE[4] = 0.0;

      // Update for DiscreteIntegrator: '<S110>/Rotor speed deviation (dw)'
      plant_schematic_DW.Rotorspeeddeviationdw_SYSTEM_EN = 0U;
      plant_schematic_DW.Rotorspeeddeviationdw_DSTATE += 5.0E-5 * rtb_u2H;

      // Update for DiscreteIntegrator: '<S90>/Integrator' incorporates:
      //   DiscreteIntegrator: '<S91>/Integrator'

      plant_schematic_DW.Integrator_DSTATE += 0.0001 *
        plant_schematic_DW.Integrator_DSTATE_m;
      plant_schematic_DW.Integrator_PrevResetState = 0;

      // Update for DiscreteIntegrator: '<S91>/Integrator'
      plant_schematic_DW.Integrator_DSTATE_m += 0.0001 * rtb_Sum3;
      plant_schematic_DW.Integrator_PrevResetState_c = 0;

      // Update for DiscreteIntegrator: '<S54>/Integrator' incorporates:
      //   DiscreteIntegrator: '<S55>/Integrator'

      plant_schematic_DW.Integrator_DSTATE_a += 0.0001 *
        plant_schematic_DW.Integrator_DSTATE_g;
      plant_schematic_DW.Integrator_PrevResetState_l = 0;

      // Update for DiscreteIntegrator: '<S55>/Integrator'
      plant_schematic_DW.Integrator_DSTATE_g += 0.0001 * rtb_Integrator_hz;
      plant_schematic_DW.Integrator_PrevResetState_o = 0;

      // Update for DiscreteIntegrator: '<S66>/Integrator' incorporates:
      //   DiscreteIntegrator: '<S67>/Integrator'

      plant_schematic_DW.Integrator_DSTATE_p += 0.0001 *
        plant_schematic_DW.Integrator_DSTATE_pj;
      plant_schematic_DW.Integrator_PrevResetState_cb = 0;

      // Update for DiscreteIntegrator: '<S67>/Integrator'
      plant_schematic_DW.Integrator_DSTATE_pj += 0.0001 * rtb_Integrator_ln;
      plant_schematic_DW.Integrator_PrevResetState_k = 0;

      // Update for DiscreteIntegrator: '<S102>/Integrator' incorporates:
      //   DiscreteIntegrator: '<S103>/Integrator'

      plant_schematic_DW.Integrator_DSTATE_az += 0.0001 *
        plant_schematic_DW.Integrator_DSTATE_f;
      plant_schematic_DW.Integrator_PrevResetState_l1 = 0;

      // Update for DiscreteIntegrator: '<S103>/Integrator' incorporates:
      //   Gain: '<S19>/Gain'
      //   Sum: '<S19>/Sum'

      plant_schematic_DW.Integrator_DSTATE_f += ((plant_schematic_Y.st -
        rtb_Gain3_je) - 1066.1308829222321 *
        plant_schematic_DW.Integrator_DSTATE_f) * 0.0001;
      plant_schematic_DW.Integrator_PrevResetState_p = 0;

      // Update for DiscreteIntegrator: '<S46>/Integrator' incorporates:
      //   DiscreteIntegrator: '<S47>/Integrator'

      plant_schematic_DW.Integrator_DSTATE_j += 0.0001 *
        plant_schematic_DW.Integrator_DSTATE_h;
      plant_schematic_DW.Integrator_PrevResetState_d = 0;

      // Update for DiscreteIntegrator: '<S47>/Integrator'
      plant_schematic_DW.Integrator_DSTATE_h += 0.0001 * rtb_Sum_i;
      plant_schematic_DW.Integrator_PrevResetState_f = 0;

      // Update for DiscreteIntegrator: '<S50>/Integrator' incorporates:
      //   DiscreteIntegrator: '<S51>/Integrator'

      plant_schematic_DW.Integrator_DSTATE_j1 += 0.0001 *
        plant_schematic_DW.Integrator_DSTATE_jz;
      plant_schematic_DW.Integrator_PrevResetState_dr = 0;

      // Update for DiscreteIntegrator: '<S51>/Integrator'
      plant_schematic_DW.Integrator_DSTATE_jz += 0.0001 * rtb_Gain3_a;
      plant_schematic_DW.Integrator_PrevResetState_j = 0;

      // Update for DiscreteIntegrator: '<S94>/Integrator' incorporates:
      //   DiscreteIntegrator: '<S95>/Integrator'

      plant_schematic_DW.Integrator_DSTATE_d += 0.0001 *
        plant_schematic_DW.Integrator_DSTATE_k;
      plant_schematic_DW.Integrator_PrevResetState_c5 = 0;

      // Update for DiscreteIntegrator: '<S95>/Integrator' incorporates:
      //   Gain: '<S111>/output formatting'
      //   Gain: '<S17>/Gain'
      //   Sum: '<S17>/Sum'

      plant_schematic_DW.Integrator_DSTATE_k += ((plant_schematic_Y.pt -
        rtb_Gain3_i) - 1066.1308829222321 *
        plant_schematic_DW.Integrator_DSTATE_k) * 0.0001;
      plant_schematic_DW.Integrator_PrevResetState_g = 0;

      // Update for DiscreteIntegrator: '<S38>/Integrator' incorporates:
      //   DiscreteIntegrator: '<S39>/Integrator'

      plant_schematic_DW.Integrator_DSTATE_i += 0.0001 *
        plant_schematic_DW.Integrator_DSTATE_mj;
      plant_schematic_DW.Integrator_PrevResetState_e = 0;

      // Update for DiscreteIntegrator: '<S39>/Integrator'
      plant_schematic_DW.Integrator_DSTATE_mj += 0.0001 * rtb_Sum_jx;
      plant_schematic_DW.Integrator_PrevResetState_m = 0;

      // Update for DiscreteIntegrator: '<S98>/Integrator' incorporates:
      //   DiscreteIntegrator: '<S99>/Integrator'

      plant_schematic_DW.Integrator_DSTATE_mjg += 0.0001 *
        plant_schematic_DW.Integrator_DSTATE_dl;
      plant_schematic_DW.Integrator_PrevResetState_mu = 0;

      // Update for DiscreteIntegrator: '<S99>/Integrator' incorporates:
      //   Gain: '<S111>/output formatting'
      //   Gain: '<S18>/Gain'
      //   Sum: '<S18>/Sum'

      plant_schematic_DW.Integrator_DSTATE_dl += ((plant_schematic_Y.qt -
        rtb_Gain3_f) - 1066.1308829222321 *
        plant_schematic_DW.Integrator_DSTATE_dl) * 0.0001;
      plant_schematic_DW.Integrator_PrevResetState_ey = 0;

      // Update for DiscreteIntegrator: '<S42>/Integrator' incorporates:
      //   DiscreteIntegrator: '<S43>/Integrator'

      plant_schematic_DW.Integrator_DSTATE_jo += 0.0001 *
        plant_schematic_DW.Integrator_DSTATE_mw;
      plant_schematic_DW.Integrator_PrevResetState_a = 0;

      // Update for DiscreteIntegrator: '<S43>/Integrator'
      plant_schematic_DW.Integrator_DSTATE_mw += 0.0001 * rtb_Sum_l;
      plant_schematic_DW.Integrator_PrevResetState_lg = 0;

      // Update for DiscreteIntegrator: '<S86>/Integrator' incorporates:
      //   DiscreteIntegrator: '<S87>/Integrator'

      plant_schematic_DW.Integrator_DSTATE_fa += 0.0001 *
        plant_schematic_DW.Integrator_DSTATE_j3;
      plant_schematic_DW.Integrator_PrevResetState_dg = 0;

      // Update for DiscreteIntegrator: '<S87>/Integrator' incorporates:
      //   Gain: '<S111>/output formatting'
      //   Gain: '<S15>/Gain'
      //   Sum: '<S15>/Sum'

      plant_schematic_DW.Integrator_DSTATE_j3 += ((plant_schematic_Y.vd -
        rtb_Gain3_hh) - 1066.1308829222321 *
        plant_schematic_DW.Integrator_DSTATE_j3) * 0.0001;
      plant_schematic_DW.Integrator_PrevResetState_jy = 0;

      // Update for DiscreteIntegrator: '<S58>/Integrator' incorporates:
      //   DiscreteIntegrator: '<S59>/Integrator'

      plant_schematic_DW.Integrator_DSTATE_mjd += 0.0001 *
        plant_schematic_DW.Integrator_DSTATE_ma;
      plant_schematic_DW.Integrator_PrevResetState_op = 0;

      // Update for DiscreteIntegrator: '<S59>/Integrator'
      plant_schematic_DW.Integrator_DSTATE_ma += 0.0001 * rtb_Sum_py;
      plant_schematic_DW.Integrator_PrevResetState_n = 0;

      // Update for DiscreteIntegrator: '<S106>/Integrator' incorporates:
      //   DiscreteIntegrator: '<S107>/Integrator'

      plant_schematic_DW.Integrator_DSTATE_c += 0.0001 *
        plant_schematic_DW.Integrator_DSTATE_fj;
      plant_schematic_DW.Integrator_PrevResetState_pe = 0;

      // Update for DiscreteIntegrator: '<S107>/Integrator' incorporates:
      //   Gain: '<S111>/output formatting'
      //   Gain: '<S20>/Gain'
      //   Sum: '<S20>/Sum'

      plant_schematic_DW.Integrator_DSTATE_fj += ((plant_schematic_Y.vq -
        rtb_Gain3_o) - 1066.1308829222321 *
        plant_schematic_DW.Integrator_DSTATE_fj) * 0.0001;
      plant_schematic_DW.Integrator_PrevResetState_pi = 0;

      // Update for DiscreteIntegrator: '<S62>/Integrator' incorporates:
      //   DiscreteIntegrator: '<S63>/Integrator'

      plant_schematic_DW.Integrator_DSTATE_jp += 0.0001 *
        plant_schematic_DW.Integrator_DSTATE_b;
      plant_schematic_DW.Integrator_PrevResetState_h = 0;

      // Update for DiscreteIntegrator: '<S63>/Integrator'
      plant_schematic_DW.Integrator_DSTATE_b += 0.0001 * rtb_Sum_a;
      plant_schematic_DW.Integrator_PrevResetState_b = 0;

      // Update for DiscreteIntegrator: '<S26>/Integrator' incorporates:
      //   DiscreteIntegrator: '<S27>/Integrator'

      plant_schematic_DW.Integrator_DSTATE_l += 0.0001 *
        plant_schematic_DW.Integrator_DSTATE_mjm;
      plant_schematic_DW.Integrator_PrevResetState_ph = 0;

      // Update for DiscreteIntegrator: '<S27>/Integrator'
      plant_schematic_DW.Integrator_DSTATE_mjm += 0.0001 * rtb_Gain3_o0;
      plant_schematic_DW.Integrator_PrevResetState_hz = 0;

      // Update for DiscreteIntegrator: '<S30>/Integrator' incorporates:
      //   DiscreteIntegrator: '<S31>/Integrator'

      plant_schematic_DW.Integrator_DSTATE_c0 += 0.0001 *
        plant_schematic_DW.Integrator_DSTATE_l5;
      plant_schematic_DW.Integrator_PrevResetState_da = 0;

      // Update for DiscreteIntegrator: '<S31>/Integrator'
      plant_schematic_DW.Integrator_DSTATE_l5 += 0.0001 * rtb_Gain3_py;
      plant_schematic_DW.Integrator_PrevResetState_o2 = 0;

      // Update for DiscreteIntegrator: '<S82>/Integrator' incorporates:
      //   DiscreteIntegrator: '<S83>/Integrator'

      plant_schematic_DW.Integrator_DSTATE_df += 0.0001 *
        plant_schematic_DW.Integrator_DSTATE_p4;
      plant_schematic_DW.Integrator_PrevResetState_dx = 0;

      // Update for DiscreteIntegrator: '<S83>/Integrator' incorporates:
      //   Gain: '<S111>/output formatting'
      //   Gain: '<S14>/Gain'
      //   Sum: '<S14>/Sum'

      plant_schematic_DW.Integrator_DSTATE_p4 +=
        ((plant_schematic_Y.load_angle_deg - rtb_Gain3_j) - 1066.1308829222321 *
         plant_schematic_DW.Integrator_DSTATE_p4) * 0.0001;
      plant_schematic_DW.Integrator_PrevResetState_lx = 0;

      // Update for DiscreteIntegrator: '<S34>/Integrator' incorporates:
      //   DiscreteIntegrator: '<S35>/Integrator'

      plant_schematic_DW.Integrator_DSTATE_lc += 0.0001 *
        plant_schematic_DW.Integrator_DSTATE_au;
      plant_schematic_DW.Integrator_PrevResetState_py = 0;

      // Update for DiscreteIntegrator: '<S35>/Integrator'
      plant_schematic_DW.Integrator_DSTATE_au += 0.0001 * rtb_Sum_ob;
      plant_schematic_DW.Integrator_PrevResetState_ob = 0;

      // Update for DiscreteIntegrator: '<S110>/theta' incorporates:
      //   Constant: '<S110>/nominal speed'
      //   Gain: '<S110>/web3'
      //   Sum: '<S110>/Sum'

      plant_schematic_DW.theta_DSTATE += (rtb_Sum1_ay + 1.0) *
        376.99111843077515 * 0.0001;

      // Update absolute time for base rate
      // The "clockTick0" counts the number of times the code of this task has
      //  been executed. The resolution of this integer timer is 0.0001, which is the step size
      //  of the task. Size of "clockTick0" ensures timer will not overflow during the
      //  application lifespan selected.

      (&plant_schematic_M)->Timing.clockTick0++;
    }

    // Model initialize function
    void SynchronousMachineAttachedToInfiniteBusModel::initialize()
    {
      // Registration code

      // initialize non-finites
      rt_InitInfAndNaN(sizeof(real_T));

      {
        int32_T i;

        // Start for S-Function (sfun_spssw_discc): '<S148>/State-Space'

        // S-Function block: <S148>/State-Space
        {
          plant_schematic_DW.StateSpace_PWORK.AS = (real_T*)calloc(3 * 3, sizeof
            (real_T));
          plant_schematic_DW.StateSpace_PWORK.BS = (real_T*)calloc(3 * 5, sizeof
            (real_T));
          plant_schematic_DW.StateSpace_PWORK.CS = (real_T*)calloc(8 * 3, sizeof
            (real_T));
          plant_schematic_DW.StateSpace_PWORK.DS = (real_T*)calloc(8 * 5, sizeof
            (real_T));
          plant_schematic_DW.StateSpace_PWORK.DX_COL = (real_T*)calloc(8, sizeof
            (real_T));
          plant_schematic_DW.StateSpace_PWORK.TMP2 = (real_T*)calloc(5, sizeof
            (real_T));
          plant_schematic_DW.StateSpace_PWORK.BD_COL = (real_T*)calloc(3, sizeof
            (real_T));
          plant_schematic_DW.StateSpace_PWORK.TMP1 = (real_T*)calloc(3, sizeof
            (real_T));
          plant_schematic_DW.StateSpace_PWORK.XTMP = (real_T*)calloc(3, sizeof
            (real_T));
        }

        // InitializeConditions for UnitDelay: '<S123>/fluxes' incorporates:
        //   Product: '<S123>/Product1'

        for (i = 0; i < 5; i++) {
          plant_schematic_DW.fluxes_DSTATE[i] =
            plant_schematic_ConstP.fluxes_InitialCondition[i];
        }

        // End of InitializeConditions for UnitDelay: '<S123>/fluxes'

        // InitializeConditions for DiscreteIntegrator: '<S110>/Rotor angle dthetae' 
        plant_schematic_DW.Rotorangledthetae_DSTATE = -0.84411278476378948;

        // InitializeConditions for S-Function (sfun_spssw_discc): '<S148>/State-Space' 
        {
          int32_T i, j;
          real_T *As = (real_T*)plant_schematic_DW.StateSpace_PWORK.AS;
          real_T *Bs = (real_T*)plant_schematic_DW.StateSpace_PWORK.BS;
          real_T *Cs = (real_T*)plant_schematic_DW.StateSpace_PWORK.CS;
          real_T *Ds = (real_T*)plant_schematic_DW.StateSpace_PWORK.DS;
          real_T *X0 = (real_T*)&plant_schematic_DW.StateSpace_DSTATE[0];
          for (i = 0; i < 3; i++) {
            X0[i] = (plant_schematic_ConstP.StateSpace_X0_param[i]);
          }

          // Copy and transpose A and B
          for (i=0; i<3; i++) {
            for (j=0; j<3; j++)
              As[i*3 + j] = (plant_schematic_ConstP.StateSpace_AS_param[i + j*3]);
            for (j=0; j<5; j++)
              Bs[i*5 + j] = (plant_schematic_ConstP.StateSpace_BS_param[i + j*3]);
          }

          // Copy and transpose C
          for (i=0; i<8; i++) {
            for (j=0; j<3; j++)
              Cs[i*3 + j] = (plant_schematic_ConstP.StateSpace_CS_param[i + j*8]);
          }

          // Copy and transpose D
          for (i=0; i<8; i++) {
            for (j=0; j<5; j++)
              Ds[i*5 + j] = (plant_schematic_ConstP.StateSpace_DS_param[i + j*8]);
          }
        }

        // InitializeConditions for DiscreteIntegrator: '<S110>/theta'
        plant_schematic_DW.theta_DSTATE = -0.84411278476378948;

        // SystemInitialize for Enabled SubSystem: '<S112>/Saturation'
        // InitializeConditions for UnitDelay: '<S124>/Lmd_sat'
        plant_schematic_DW.Lmd_sat_DSTATE = 1.35;

        // End of SystemInitialize for SubSystem: '<S112>/Saturation'

        // Enable for DiscreteIntegrator: '<S110>/Rotor speed deviation (dw)'
        plant_schematic_DW.Rotorspeeddeviationdw_SYSTEM_EN = 1U;
      }
    }

    // Model terminate function
    void SynchronousMachineAttachedToInfiniteBusModel::terminate()
    {
      // Terminate for S-Function (sfun_spssw_discc): '<S148>/State-Space'

      // S-Function block: <S148>/State-Space
      {
        // Free memory
        free(plant_schematic_DW.StateSpace_PWORK.AS);
        free(plant_schematic_DW.StateSpace_PWORK.BS);
        free(plant_schematic_DW.StateSpace_PWORK.CS);
        free(plant_schematic_DW.StateSpace_PWORK.DS);
        free(plant_schematic_DW.StateSpace_PWORK.DX_COL);
        free(plant_schematic_DW.StateSpace_PWORK.TMP2);
        free(plant_schematic_DW.StateSpace_PWORK.BD_COL);
        free(plant_schematic_DW.StateSpace_PWORK.TMP1);
        free(plant_schematic_DW.StateSpace_PWORK.XTMP);
      }
    }

    // Constructor
    SynchronousMachineAttachedToInfiniteBusModel::
      SynchronousMachineAttachedToInfiniteBusModel() :
      plant_schematic_U(),
      plant_schematic_Y(),
      plant_schematic_B(),
      plant_schematic_DW(),
      plant_schematic_M()
    {
      // Currently there is no constructor body generated.
    }

    // Destructor
    // Currently there is no destructor body generated.
    SynchronousMachineAttachedToInfiniteBusModel::
      ~SynchronousMachineAttachedToInfiniteBusModel() = default;

    // Real-Time Model get method
    SynchronousMachineAttachedToInfiniteBusModel::RT_MODEL_plant_schematic_T
      * SynchronousMachineAttachedToInfiniteBusModel::getRTM()
    {
      return (&plant_schematic_M);
    }
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
