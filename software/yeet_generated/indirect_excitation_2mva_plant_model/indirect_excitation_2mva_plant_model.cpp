//
// File: indirect_excitation_2mva_plant_model.cpp
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
#include "indirect_excitation_2mva_plant_model.h"

#include <cmath>
#include <cstring>

#include "indirect_excitation_2mva_plant_model_private.h"
#include "rt_look.h"
#include "rt_look1d.h"
#include "rtwtypes.h"


extern "C" {

#include "rt_nonfinite.h"
}

#include <cfloat>

#include "rt_defines.h"


namespace plant {

real_T rt_atan2d_snf(real_T u0, real_T u1) {
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

void rt_invd5x5_snf(const real_T u[25], real_T y[25]) {
    real_T x[25];
    real_T smax;
    int32_T c;
    int32_T c_0;
    int32_T idxmax;
    int32_T jA;
    int32_T jj;
    int32_T kAcol;
    int32_T pipk;
    int32_T temp_tmp;
    int8_T ipiv[5];
    int8_T p[5];
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

real_T rt_remd_snf(real_T u0, real_T u1) {
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

real_T rt_modd_snf(real_T u0, real_T u1) {
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

// Model step function for TID0
void indirect_excitation_2mva::step0()  // Sample time: [5.0E-6s, 0.0s]
{
    // (no output/update code required)
}

// Model step function for TID1
void indirect_excitation_2mva::step1()  // Sample time: [0.0001s, 0.0s]
{
    real_T rtb_Sum5[25];
    real_T rtb_UpdateLmdrows234col234_p[25];
    real_T rtb_ukuk1_p[25];
    real_T rtb_UpdateLmdrows234col234_a[5];
    real_T rtb_changeIqIdcurrentsigns[5];
    real_T rtb_changeIqIdcurrentsigns_b[5];
    real_T rtb_xk1_i[5];
    real_T tmp[5];
    real_T rtb_DataTypeConversion6;
    real_T rtb_Fcn1;
    real_T rtb_Fcn1_d;
    real_T rtb_Fcn2_b;
    real_T rtb_Gain1_i;
    real_T rtb_Mult1_idx_1;
    real_T rtb_Product5;
    real_T rtb_Sum1_b;
    real_T rtb_Sum3;
    real_T rtb_UpdateLmdrows234col234_a_0;
    real_T rtb_dw_delay;
    real_T rtb_outputformatting_idx_11;
    real_T rtb_outputformatting_idx_12;
    real_T rtb_outputformatting_idx_3;
    real_T rtb_outputformatting_idx_4;
    real_T rtb_theta_f;
    real_T rtb_u2H;
    real_T rtb_units_tmp;
    int32_T i;
    int32_T i_0;
    int32_T rtb_UpdateLmdrows234col234_a_tm;

    // UnitDelay: '<S107>/dw_delay'
    rtb_dw_delay = indirect_excitation_2mva_pla_DW.dw_delay_DSTATE;

    // Sum: '<S107>/Sum1' incorporates:
    //   Gain: '<S107>/F2'
    //   UnitDelay: '<S107>/dw_delay'
    //   UnitDelay: '<S107>/dw_predict'

    rtb_Sum1_b =
        2.0 * indirect_excitation_2mva_pla_DW.dw_delay_DSTATE - indirect_excitation_2mva_pla_DW.dw_predict_DSTATE;

    // Gain: '<S85>/units' incorporates:
    //   Constant: '<S85>/nominal speed'
    //   Gain: '<S85>/web3'
    //   Sum: '<S85>/Sum'

    rtb_units_tmp = (rtb_Sum1_b + 1.0) * 157.07963267948966;

    // Gain: '<S44>/1_Pb' incorporates:
    //   Gain: '<Root>/Gain8'
    //   Gain: '<S85>/units'

    indirect_excitation_2mva_plan_Y.wm = 0.0063661977236758134 * rtb_units_tmp;

    // UnitDelay: '<S98>/fluxes'
    for (i = 0; i < 5; i++) {
        rtb_xk1_i[i] = indirect_excitation_2mva_pla_DW.fluxes_DSTATE_m[i];
    }

    // Switch: '<S87>/Switch' incorporates:
    //   Constant: '<S87>/Constant4'
    //   Product: '<S97>/inversion'

    std::memcpy(&rtb_ukuk1_p[0], &indirect_excitation_2mva_ConstP.Constant4_Value_m[0], 25U * sizeof(real_T));
    for (i = 0; i < 5; i++) {
        // Product: '<S87>/Product3' incorporates:
        //   UnitDelay: '<S98>/fluxes'

        rtb_Mult1_idx_1 = 0.0;
        for (i_0 = 0; i_0 < 5; i_0++) {
            rtb_Mult1_idx_1 += rtb_ukuk1_p[5 * i_0 + i] * indirect_excitation_2mva_pla_DW.fluxes_DSTATE_m[i_0];
        }

        // Gain: '<S87>/change Iq Id  current signs' incorporates:
        //   Product: '<S87>/Product3'

        rtb_changeIqIdcurrentsigns[i] = indirect_excitation_2mva_ConstP.pooled28[i] * rtb_Mult1_idx_1;
    }

    // Outputs for Enabled SubSystem: '<S70>/Signal generator' incorporates:
    //   EnablePort: '<S72>/Enable'

    // DigitalClock: '<S85>/Digital Clock' incorporates:
    //   Step: '<S76>/Step'
    //   Step: '<S76>/Step1'

    rtb_u2H = (((&indirect_excitation_2mva_pla_M)->Timing.clockTick1) * 0.0001);

    // End of Outputs for SubSystem: '<S70>/Signal generator'

    // Sum: '<S85>/Sum3' incorporates:
    //   DigitalClock: '<S85>/Digital Clock'
    //   DiscreteIntegrator: '<S85>/Rotor angle dthetae'
    //   Gain: '<S85>/web2'

    rtb_Sum3 = 314.15926535897933 * rtb_u2H + indirect_excitation_2mva_pla_DW.Rotorangledthetae_DSTATE;

    // Trigonometry: '<S88>/Elementary Math'
    rtb_Product5 = std::sin(rtb_Sum3);
    rtb_Gain1_i = std::cos(rtb_Sum3);

    // Fcn: '<S89>/Fcn'
    rtb_Sum3 = rtb_changeIqIdcurrentsigns[0] * rtb_Gain1_i + rtb_changeIqIdcurrentsigns[1] * rtb_Product5;

    // Fcn: '<S89>/Fcn1'
    rtb_Fcn1 = ((-rtb_changeIqIdcurrentsigns[0] - 1.7320508075688772 * rtb_changeIqIdcurrentsigns[1]) * rtb_Gain1_i +
                (1.7320508075688772 * rtb_changeIqIdcurrentsigns[0] - rtb_changeIqIdcurrentsigns[1]) * rtb_Product5) *
               0.5;

    // Gain: '<S89>/ib' incorporates:
    //   Sum: '<S89>/Sum'

    indirect_excitation_2mva_plan_B.ib[0] = 4082.48290463863 * rtb_Sum3;
    indirect_excitation_2mva_plan_B.ib[1] = 4082.48290463863 * rtb_Fcn1;
    indirect_excitation_2mva_plan_B.ib[2] = ((0.0 - rtb_Sum3) - rtb_Fcn1) * 4082.48290463863;

    // Outputs for Enabled SubSystem: '<S70>/Signal generator' incorporates:
    //   EnablePort: '<S72>/Enable'

    indirect_excitation_2mva_pla_DW.Signalgenerator_MODE = true;

    // Lookup: '<S77>/Look-Up Table' incorporates:
    //   DigitalClock: '<S85>/Digital Clock'

    indirect_excitation_2mva_plan_B.LookUpTable =
        rt_Lookup(&indirect_excitation_2mva_ConstP.LookUpTable_XData[0], 8, rtb_u2H,
                  &indirect_excitation_2mva_ConstP.LookUpTable_YData[0]);

    // Logic: '<S72>/Logical Operator1' incorporates:
    //   Constant: '<S72>/Constant6'
    //   Constant: '<S72>/valp_nom5'
    //   RelationalOperator: '<S72>/Relational Operator3'

    indirect_excitation_2mva_plan_B.LogicalOperator1 = false;

    // Step: '<S76>/Step1'
    i = (rtb_u2H < 0.0);

    // Switch: '<S76>/Switch2'
    if (i >= 0.5) {
        // UnitDelay: '<S76>/Unit Delay' incorporates:
        //   MultiPortSwitch: '<S76>/Multiport Switch1'
        //   Product: '<S76>/Product2'

        indirect_excitation_2mva_pla_DW.UnitDelay_DSTATE = 0.0;
    }

    // End of Switch: '<S76>/Switch2'

    // Switch: '<S72>/Switch3' incorporates:
    //   Constant: '<S72>/Constant4'

    indirect_excitation_2mva_plan_B.Switch3_g = 0.0;

    // Switch: '<S76>/Switch3' incorporates:
    //   DataTypeConversion: '<S76>/Data Type  Conversion2'
    //   Logic: '<S76>/Logical Operator1'

    if (i == 0) {
        // Switch: '<S72>/Switch2' incorporates:
        //   Constant: '<S76>/Constant1'

        indirect_excitation_2mva_plan_B.Switch2_e = 0.0;
    } else {
        // Switch: '<S72>/Switch2' incorporates:
        //   UnitDelay: '<S76>/Unit Delay'

        indirect_excitation_2mva_plan_B.Switch2_e = indirect_excitation_2mva_pla_DW.UnitDelay_DSTATE;
    }

    // End of Switch: '<S76>/Switch3'

    // DiscreteIntegrator: '<S72>/Discrete-Time Integrator'
    indirect_excitation_2mva_plan_B.DiscreteTimeIntegrator =
        indirect_excitation_2mva_pla_DW.DiscreteTimeIntegrator_DSTATE;

    // Switch: '<S76>/Switch' incorporates:
    //   Constant: '<S76>/Constant2'
    //   Constant: '<S76>/Constant4'
    //   Step: '<S76>/Step'
    //   Switch: '<S76>/Switch1'

    if ((!(rtb_u2H < 0.0)) >= 0.5) {
        i = 1;
    } else {
        i = 0;
    }

    // Update for DiscreteIntegrator: '<S76>/Discrete-Time Integrator1' incorporates:
    //   Switch: '<S76>/Switch'

    indirect_excitation_2mva_pla_DW.DiscreteTimeIntegrator1_DSTATE += 0.0001 * static_cast<real_T>(i);

    // End of Outputs for SubSystem: '<S70>/Signal generator'

    // Switch: '<S70>/Switch5' incorporates:
    //   Constant: '<S70>/valp_nom2'
    //   Gain: '<S2>/Kv'
    //   Sum: '<S70>/Sum3'

    indirect_excitation_2mva_plan_B.Sum5[0] = indirect_excitation_2mva_plan_B.Switch2_e + 324.96563920923495;
    indirect_excitation_2mva_plan_B.Sum5[1] = indirect_excitation_2mva_plan_B.Switch2_e + 324.96563920923495;
    indirect_excitation_2mva_plan_B.Sum5[2] = indirect_excitation_2mva_plan_B.Switch2_e + 324.96563920923495;

    // Sum: '<S70>/Sum' incorporates:
    //   DigitalClock: '<S85>/Digital Clock'
    //   Product: '<S70>/Product'

    rtb_Sum3 = rtb_u2H * indirect_excitation_2mva_ConstB.Gain + indirect_excitation_2mva_plan_B.DiscreteTimeIntegrator;

    // Outputs for Enabled SubSystem: '<S70>/Harmonic Generator' incorporates:
    //   EnablePort: '<S71>/Enable'

    if (indirect_excitation_2mva_pla_DW.HarmonicGenerator_MODE) {
        // Disable for Product: '<S73>/Product1' incorporates:
        //   Outport: '<S71>/Out1'

        indirect_excitation_2mva_plan_B.Product1[0] = 0.0;

        // Disable for Product: '<S74>/Product1' incorporates:
        //   Outport: '<S71>/Out2'

        indirect_excitation_2mva_plan_B.Product1_l[0] = 0.0;

        // Disable for Product: '<S73>/Product1' incorporates:
        //   Outport: '<S71>/Out1'

        indirect_excitation_2mva_plan_B.Product1[1] = 0.0;

        // Disable for Product: '<S74>/Product1' incorporates:
        //   Outport: '<S71>/Out2'

        indirect_excitation_2mva_plan_B.Product1_l[1] = 0.0;

        // Disable for Product: '<S73>/Product1' incorporates:
        //   Outport: '<S71>/Out1'

        indirect_excitation_2mva_plan_B.Product1[2] = 0.0;

        // Disable for Product: '<S74>/Product1' incorporates:
        //   Outport: '<S71>/Out2'

        indirect_excitation_2mva_plan_B.Product1_l[2] = 0.0;
        indirect_excitation_2mva_pla_DW.HarmonicGenerator_MODE = false;
    }

    // End of Outputs for SubSystem: '<S70>/Harmonic Generator'

    // Sum: '<S70>/Sum5' incorporates:
    //   Constant: '<S70>/P1'
    //   Gain: '<S2>/Kv'
    //   Product: '<S70>/Product1'
    //   Sum: '<S70>/Sum1'
    //   Sum: '<S70>/Sum2'
    //   Trigonometry: '<S70>/Trigonometric Function'

    indirect_excitation_2mva_plan_B.Sum5[0] =
        (std::sin(rtb_Sum3 + indirect_excitation_2mva_ConstB.Gain3) * indirect_excitation_2mva_plan_B.Sum5[0] +
         indirect_excitation_2mva_plan_B.Product1[0]) +
        indirect_excitation_2mva_plan_B.Product1_l[0];
    indirect_excitation_2mva_plan_B.Sum5[1] =
        (std::sin((indirect_excitation_2mva_ConstB.Gain3 - 2.0943951023931953) + rtb_Sum3) *
             indirect_excitation_2mva_plan_B.Sum5[1] +
         indirect_excitation_2mva_plan_B.Product1[1]) +
        indirect_excitation_2mva_plan_B.Product1_l[1];
    indirect_excitation_2mva_plan_B.Sum5[2] =
        (std::sin((indirect_excitation_2mva_ConstB.Gain3 + 2.0943951023931953) + rtb_Sum3) *
             indirect_excitation_2mva_plan_B.Sum5[2] +
         indirect_excitation_2mva_plan_B.Product1[2]) +
        indirect_excitation_2mva_plan_B.Product1_l[2];

    // S-Function (sfun_spssw_discc): '<S124>/State-Space' incorporates:
    //   Constant: '<S130>/SwitchCurrents'

    // S-Function block: <S124>/State-Space
    {
        real_T accum;

        // Circuit has switches
        int_T *switch_status = (int_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK.SWITCH_STATUS;
        int_T *switch_status_init = (int_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK.SWITCH_STATUS_INIT;
        int_T *SwitchChange = (int_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK.SW_CHG;
        int_T *gState = (int_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK.G_STATE;
        real_T *yswitch = (real_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK.Y_SWITCH;
        int_T *switchTypes = (int_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK.SWITCH_TYPES;
        int_T *idxOutSw = (int_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK.IDX_OUT_SW;
        real_T *DxCol = (real_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK.DX_COL;
        real_T *tmp2 = (real_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK.TMP2;
        real_T *BDcol = (real_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK.BD_COL;
        real_T *tmp1 = (real_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK.TMP1;
        real_T *uswlast = (real_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK.USWLAST;
        int_T newState;
        int_T swChanged = 0;
        int loopsToDo = 20;
        real_T temp;

        // keep an initial copy of switch_status
        memcpy(switch_status_init, switch_status, 3 * sizeof(int_T));
        memcpy(uswlast, &indirect_excitation_2mva_plan_B.StateSpace_o1[0], 3 * sizeof(real_T));
        do {
            if (loopsToDo == 1) {  // Need to reset some variables:
                swChanged = 0;

                // return to the original switch status
                {
                    int_T i1;
                    for (i1 = 0; i1 < 3; i1++) {
                        swChanged =
                            ((SwitchChange[i1] = switch_status_init[i1] - switch_status[i1]) != 0) ? 1 : swChanged;
                        switch_status[i1] = switch_status_init[i1];
                    }
                }
            } else {
                //
                //  Compute outputs:
                //  ---------------

                real_T *Cs = (real_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK.CS;
                real_T *Ds = (real_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK.DS;

                {
                    int_T i1;
                    real_T *y0{&indirect_excitation_2mva_plan_B.StateSpace_o1[0]};

                    for (i1 = 0; i1 < 11; i1++) {
                        accum = 0.0;

                        {
                            int_T i2;
                            real_T *xd{&indirect_excitation_2mva_pla_DW.StateSpace_DSTATE[0]};

                            for (i2 = 0; i2 < 6; i2++) {
                                accum += *(Cs++) * xd[i2];
                            }
                        }

                        accum += *(Ds++) * 0.0;
                        accum += *(Ds++) * 0.0;
                        accum += *(Ds++) * 0.0;
                        accum += *(Ds++) * indirect_excitation_2mva_plan_B.ib[0];
                        accum += *(Ds++) * indirect_excitation_2mva_plan_B.ib[1];
                        accum += *(Ds++) * indirect_excitation_2mva_plan_B.Sum5[0];
                        accum += *(Ds++) * indirect_excitation_2mva_plan_B.Sum5[1];
                        accum += *(Ds++) * indirect_excitation_2mva_plan_B.Sum5[2];
                        y0[i1] = accum;
                    }
                }

                swChanged = 0;

                // Breaker
                newState = (gState[0] > 0) ? 1
                                           : ((indirect_excitation_2mva_plan_B.StateSpace_o1[0] * uswlast[0] < 0.0)
                                                  ? 0
                                                  : switch_status[0]);
                swChanged = ((SwitchChange[0] = newState - switch_status[0]) != 0) ? 1 : swChanged;
                switch_status[0] = newState;  // Keep new state

                // Breaker
                newState = (gState[1] > 0) ? 1
                                           : ((indirect_excitation_2mva_plan_B.StateSpace_o1[1] * uswlast[1] < 0.0)
                                                  ? 0
                                                  : switch_status[1]);
                swChanged = ((SwitchChange[1] = newState - switch_status[1]) != 0) ? 1 : swChanged;
                switch_status[1] = newState;  // Keep new state

                // Breaker
                newState = (gState[2] > 0) ? 1
                                           : ((indirect_excitation_2mva_plan_B.StateSpace_o1[2] * uswlast[2] < 0.0)
                                                  ? 0
                                                  : switch_status[2]);
                swChanged = ((SwitchChange[2] = newState - switch_status[2]) != 0) ? 1 : swChanged;
                switch_status[2] = newState;  // Keep new state
            }

            //
            //  Compute new As, Bs, Cs and Ds matrixes:
            //  --------------------------------------

            if (swChanged) {
                real_T *As = (real_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK.AS;
                real_T *Cs = (real_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK.CS;
                real_T *Bs = (real_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK.BS;
                real_T *Ds = (real_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK.DS;
                real_T a1;

                {
                    int_T i1;
                    for (i1 = 0; i1 < 3; i1++) {
                        if (SwitchChange[i1] != 0) {
                            a1 = 200.0 * SwitchChange[i1];
                            temp = 1 / (1 - Ds[i1 * 9] * a1);

                            {
                                int_T i2;
                                for (i2 = 0; i2 < 11; i2++) {
                                    DxCol[i2] = Ds[i2 * 8 + i1] * temp * a1;
                                }
                            }

                            DxCol[i1] = temp;

                            {
                                int_T i2;
                                for (i2 = 0; i2 < 6; i2++) {
                                    BDcol[i2] = Bs[i2 * 8 + i1] * a1;
                                }
                            }

                            // Copy row nSw of Cs into tmp1 and zero it out in Cs
                            memcpy(tmp1, &Cs[i1 * 6], 6 * sizeof(real_T));
                            memset(&Cs[i1 * 6], '\0', 6 * sizeof(real_T));

                            // Copy row nSw of Ds into tmp2 and zero it out in Ds
                            memcpy(tmp2, &Ds[i1 * 8], 8 * sizeof(real_T));
                            memset(&Ds[i1 * 8], '\0', 8 * sizeof(real_T));

                            // Cs = Cs + DxCol * tmp1, Ds = Ds + DxCol * tmp2 ******************
                            {
                                int_T i2;
                                for (i2 = 0; i2 < 11; i2++) {
                                    a1 = DxCol[i2];

                                    {
                                        int_T i3;
                                        for (i3 = 0; i3 < 6; i3++) {
                                            Cs[i2 * 6 + i3] += a1 * tmp1[i3];
                                        }
                                    }

                                    {
                                        int_T i3;
                                        for (i3 = 0; i3 < 8; i3++) {
                                            Ds[i2 * 8 + i3] += a1 * tmp2[i3];
                                        }
                                    }
                                }
                            }

                            // As = As + BdCol*Cs(nSw,:), Bs = Bs + BdCol*Ds(nSw,:) ************
                            {
                                int_T i2;
                                for (i2 = 0; i2 < 6; i2++) {
                                    a1 = BDcol[i2];

                                    {
                                        int_T i3;
                                        for (i3 = 0; i3 < 6; i3++) {
                                            As[i2 * 6 + i3] += a1 * Cs[i1 * 6 + i3];
                                        }
                                    }

                                    {
                                        int_T i3;
                                        for (i3 = 0; i3 < 8; i3++) {
                                            Bs[i2 * 8 + i3] += a1 * Ds[i1 * 8 + i3];
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }  // if (swChanged)
        } while (swChanged > 0 && --loopsToDo > 0);

        if (loopsToDo == 0) {
            real_T *Cs = (real_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK.CS;
            real_T *Ds = (real_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK.DS;

            {
                int_T i1;
                real_T *y0{&indirect_excitation_2mva_plan_B.StateSpace_o1[0]};

                for (i1 = 0; i1 < 11; i1++) {
                    accum = 0.0;

                    {
                        int_T i2;
                        real_T *xd{&indirect_excitation_2mva_pla_DW.StateSpace_DSTATE[0]};

                        for (i2 = 0; i2 < 6; i2++) {
                            accum += *(Cs++) * xd[i2];
                        }
                    }

                    accum += *(Ds++) * 0.0;
                    accum += *(Ds++) * 0.0;
                    accum += *(Ds++) * 0.0;
                    accum += *(Ds++) * indirect_excitation_2mva_plan_B.ib[0];
                    accum += *(Ds++) * indirect_excitation_2mva_plan_B.ib[1];
                    accum += *(Ds++) * indirect_excitation_2mva_plan_B.Sum5[0];
                    accum += *(Ds++) * indirect_excitation_2mva_plan_B.Sum5[1];
                    accum += *(Ds++) * indirect_excitation_2mva_plan_B.Sum5[2];
                    y0[i1] = accum;
                }
            }
        }

        // Output new switches states
        indirect_excitation_2mva_plan_B.StateSpace_o2[0] = (real_T)switch_status[0];
        indirect_excitation_2mva_plan_B.StateSpace_o2[1] = (real_T)switch_status[1];
        indirect_excitation_2mva_plan_B.StateSpace_o2[2] = (real_T)switch_status[2];
    }

    // Gain: '<S88>/1_Vb'
    rtb_Mult1_idx_1 = 0.0030618621784789728 * indirect_excitation_2mva_plan_B.StateSpace_o1[4];

    // Fcn: '<S88>/Fcn2' incorporates:
    //   Fcn: '<S88>/Fcn3'
    //   Gain: '<S88>/1_Vb'

    rtb_Fcn1 = 0.0030618621784789728 * indirect_excitation_2mva_plan_B.StateSpace_o1[3] * 2.0 + rtb_Mult1_idx_1;
    rtb_Sum3 = (1.7320508075688772 * rtb_Mult1_idx_1 * rtb_Product5 + rtb_Fcn1 * rtb_Gain1_i) * 0.33333333333333331;

    // Fcn: '<S88>/Fcn3'
    rtb_Product5 =
        (-1.7320508075688772 * rtb_Mult1_idx_1 * rtb_Gain1_i + rtb_Fcn1 * rtb_Product5) * 0.33333333333333331;

    // Gain: '<S86>/output formatting' incorporates:
    //   Gain: '<S91>/Gain'
    //   Gain: '<S91>/Gain1'
    //   Product: '<S91>/Product2'
    //   Product: '<S91>/Product3'
    //   Product: '<S91>/Product4'
    //   Product: '<S91>/Product5'
    //   Sum: '<S91>/Sum'
    //   Sum: '<S91>/Sum1'

    rtb_outputformatting_idx_3 = 4082.48290463863 * rtb_changeIqIdcurrentsigns[0];
    rtb_outputformatting_idx_4 = 4082.48290463863 * rtb_changeIqIdcurrentsigns[1];
    rtb_Gain1_i = 206.00993825915066 * rtb_changeIqIdcurrentsigns[2];
    rtb_outputformatting_idx_11 = 326.59863237109039 * rtb_Sum3;
    rtb_outputformatting_idx_12 = 326.59863237109039 * rtb_Product5;
    indirect_excitation_2mva_plan_Y.pt =
        (rtb_Sum3 * rtb_changeIqIdcurrentsigns[0] + rtb_Product5 * rtb_changeIqIdcurrentsigns[1]) * 2.0E+6;
    indirect_excitation_2mva_plan_Y.qt =
        (rtb_Sum3 * rtb_changeIqIdcurrentsigns[1] - rtb_Product5 * rtb_changeIqIdcurrentsigns[0]) * 2.0E+6;

    // Outport: '<Root>/load_angle_deg' incorporates:
    //   Fcn: '<S92>/x->theta'
    //   Gain: '<S93>/Gain'

    indirect_excitation_2mva_plan_Y.load_angle_deg = 57.295779513082323 * rt_atan2d_snf(rtb_Product5, rtb_Sum3);

    // Switch: '<S46>/Switch' incorporates:
    //   Constant: '<S46>/Constant4'
    //   Product: '<S56>/inversion'

    std::memcpy(&rtb_ukuk1_p[0], &indirect_excitation_2mva_ConstP.Constant4_Value[0], 25U * sizeof(real_T));
    for (i = 0; i < 5; i++) {
        // Product: '<S46>/Product3' incorporates:
        //   UnitDelay: '<S57>/fluxes'

        rtb_Mult1_idx_1 = 0.0;
        for (i_0 = 0; i_0 < 5; i_0++) {
            rtb_Mult1_idx_1 += rtb_ukuk1_p[5 * i_0 + i] * indirect_excitation_2mva_pla_DW.fluxes_DSTATE[i_0];
        }

        // Gain: '<S46>/change Iq Id  current signs' incorporates:
        //   Product: '<S46>/Product3'

        rtb_changeIqIdcurrentsigns_b[i] = indirect_excitation_2mva_ConstP.pooled28[i] * rtb_Mult1_idx_1;
    }

    // Sum: '<S44>/Sum3' incorporates:
    //   DigitalClock: '<S44>/Digital Clock'
    //   DiscreteIntegrator: '<S44>/Rotor angle dtheta'
    //   Gain: '<S44>/web2'

    rtb_Fcn1 = 314.15926535897933 * (((&indirect_excitation_2mva_pla_M)->Timing.clockTick1) * 0.0001) +
               indirect_excitation_2mva_pla_DW.Rotorangledtheta_DSTATE;

    // Trigonometry: '<S47>/Elementary Math'
    rtb_theta_f = std::sin(rtb_Fcn1);
    rtb_DataTypeConversion6 = std::cos(rtb_Fcn1);

    // Fcn: '<S48>/Fcn'
    rtb_Fcn1 =
        rtb_changeIqIdcurrentsigns_b[0] * rtb_DataTypeConversion6 + rtb_changeIqIdcurrentsigns_b[1] * rtb_theta_f;

    // Fcn: '<S48>/Fcn1'
    rtb_Fcn1_d =
        ((-rtb_changeIqIdcurrentsigns_b[0] - 1.7320508075688772 * rtb_changeIqIdcurrentsigns_b[1]) *
             rtb_DataTypeConversion6 +
         (1.7320508075688772 * rtb_changeIqIdcurrentsigns_b[0] - rtb_changeIqIdcurrentsigns_b[1]) * rtb_theta_f) *
        0.5;

    // Gain: '<S48>/ib' incorporates:
    //   Sum: '<S48>/Sum'

    indirect_excitation_2mva_plan_B.ib_g[0] = 16.534055763786451 * rtb_Fcn1;
    indirect_excitation_2mva_plan_B.ib_g[1] = 16.534055763786451 * rtb_Fcn1_d;
    indirect_excitation_2mva_plan_B.ib_g[2] = ((0.0 - rtb_Fcn1) - rtb_Fcn1_d) * 16.534055763786451;

    // UnitDelay: '<S4>/Unit Delay2'
    indirect_excitation_2mva_plan_B.UnitDelay2 = indirect_excitation_2mva_pla_DW.UnitDelay2_DSTATE;

    // S-Function (sfun_spssw_discc): '<S123>/State-Space' incorporates:
    //   Constant: '<S126>/SwitchCurrents'
    //   Constant: '<S7>/g'

    // S-Function block: <S123>/State-Space
    {
        real_T accum;

        // Circuit has switches
        int_T *switch_status = (int_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.SWITCH_STATUS;
        int_T *switch_status_init = (int_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.SWITCH_STATUS_INIT;
        int_T *SwitchChange = (int_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.SW_CHG;
        int_T *gState = (int_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.G_STATE;
        real_T *yswitch = (real_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.Y_SWITCH;
        int_T *switchTypes = (int_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.SWITCH_TYPES;
        int_T *idxOutSw = (int_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.IDX_OUT_SW;
        real_T *DxCol = (real_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.DX_COL;
        real_T *tmp2 = (real_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.TMP2;
        real_T *BDcol = (real_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.BD_COL;
        real_T *tmp1 = (real_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.TMP1;
        real_T *uswlast = (real_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.USWLAST;
        int_T newState;
        int_T swChanged = 0;
        int loopsToDo = 20;
        real_T temp;

        // keep an initial copy of switch_status
        memcpy(switch_status_init, switch_status, 6 * sizeof(int_T));
        memcpy(uswlast, &indirect_excitation_2mva_plan_B.StateSpace_o1_l[0], 6 * sizeof(real_T));
        do {
            if (loopsToDo == 1) {  // Need to reset some variables:
                swChanged = 0;

                // return to the original switch status
                {
                    int_T i1;
                    for (i1 = 0; i1 < 6; i1++) {
                        swChanged =
                            ((SwitchChange[i1] = switch_status_init[i1] - switch_status[i1]) != 0) ? 1 : swChanged;
                        switch_status[i1] = switch_status_init[i1];
                    }
                }
            } else {
                //
                //  Compute outputs:
                //  ---------------

                real_T *Cs = (real_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.CS;
                real_T *Ds = (real_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.DS;

                {
                    int_T i1;
                    real_T *y0{&indirect_excitation_2mva_plan_B.StateSpace_o1_l[0]};

                    for (i1 = 0; i1 < 15; i1++) {
                        accum = 0.0;

                        {
                            int_T i2;
                            real_T *xd{&indirect_excitation_2mva_pla_DW.StateSpace_DSTATE_h[0]};

                            for (i2 = 0; i2 < 6; i2++) {
                                accum += *(Cs++) * xd[i2];
                            }
                        }

                        {
                            int_T i2;
                            const real_T *u0{indirect_excitation_2mva_ConstP.pooled25};

                            for (i2 = 0; i2 < 6; i2++) {
                                accum += *(Ds++) * u0[i2];
                            }

                            accum += *(Ds++) * indirect_excitation_2mva_plan_B.ib_g[0];
                            accum += *(Ds++) * indirect_excitation_2mva_plan_B.ib_g[1];
                            accum += *(Ds++) * indirect_excitation_2mva_plan_B.UnitDelay2;
                        }

                        y0[i1] = accum;
                    }
                }

                swChanged = 0;

                {
                    int_T i1;
                    real_T *y0{&indirect_excitation_2mva_plan_B.StateSpace_o1_l[0]};

                    for (i1 = 0; i1 < 6; i1++) {
                        newState = y0[i1] > 0.0 ? 1 : ((y0[i1] < 0.0) ? 0 : switch_status[i1]);
                        swChanged = ((SwitchChange[i1] = newState - switch_status[i1]) != 0) ? 1 : swChanged;
                        switch_status[i1] = newState;  // Keep new state
                    }
                }
            }

            //
            //  Compute new As, Bs, Cs and Ds matrixes:
            //  --------------------------------------

            if (swChanged) {
                real_T *As = (real_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.AS;
                real_T *Cs = (real_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.CS;
                real_T *Bs = (real_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.BS;
                real_T *Ds = (real_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.DS;
                real_T a1;

                {
                    int_T i1;
                    for (i1 = 0; i1 < 6; i1++) {
                        if (SwitchChange[i1] != 0) {
                            a1 = 99999.999999999985 * SwitchChange[i1];
                            temp = 1 / (1 - Ds[i1 * 10] * a1);

                            {
                                int_T i2;
                                for (i2 = 0; i2 < 15; i2++) {
                                    DxCol[i2] = Ds[i2 * 9 + i1] * temp * a1;
                                }
                            }

                            DxCol[i1] = temp;

                            {
                                int_T i2;
                                for (i2 = 0; i2 < 6; i2++) {
                                    BDcol[i2] = Bs[i2 * 9 + i1] * a1;
                                }
                            }

                            // Copy row nSw of Cs into tmp1 and zero it out in Cs
                            memcpy(tmp1, &Cs[i1 * 6], 6 * sizeof(real_T));
                            memset(&Cs[i1 * 6], '\0', 6 * sizeof(real_T));

                            // Copy row nSw of Ds into tmp2 and zero it out in Ds
                            memcpy(tmp2, &Ds[i1 * 9], 9 * sizeof(real_T));
                            memset(&Ds[i1 * 9], '\0', 9 * sizeof(real_T));

                            // Cs = Cs + DxCol * tmp1, Ds = Ds + DxCol * tmp2 ******************
                            {
                                int_T i2;
                                for (i2 = 0; i2 < 15; i2++) {
                                    a1 = DxCol[i2];

                                    {
                                        int_T i3;
                                        for (i3 = 0; i3 < 6; i3++) {
                                            Cs[i2 * 6 + i3] += a1 * tmp1[i3];
                                        }
                                    }

                                    {
                                        int_T i3;
                                        for (i3 = 0; i3 < 9; i3++) {
                                            Ds[i2 * 9 + i3] += a1 * tmp2[i3];
                                        }
                                    }
                                }
                            }

                            // As = As + BdCol*Cs(nSw,:), Bs = Bs + BdCol*Ds(nSw,:) ************
                            {
                                int_T i2;
                                for (i2 = 0; i2 < 6; i2++) {
                                    a1 = BDcol[i2];

                                    {
                                        int_T i3;
                                        for (i3 = 0; i3 < 6; i3++) {
                                            As[i2 * 6 + i3] += a1 * Cs[i1 * 6 + i3];
                                        }
                                    }

                                    {
                                        int_T i3;
                                        for (i3 = 0; i3 < 9; i3++) {
                                            Bs[i2 * 9 + i3] += a1 * Ds[i1 * 9 + i3];
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }  // if (swChanged)
        } while (swChanged > 0 && --loopsToDo > 0);

        if (loopsToDo == 0) {
            real_T *Cs = (real_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.CS;
            real_T *Ds = (real_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.DS;

            {
                int_T i1;
                real_T *y0{&indirect_excitation_2mva_plan_B.StateSpace_o1_l[0]};

                for (i1 = 0; i1 < 15; i1++) {
                    accum = 0.0;

                    {
                        int_T i2;
                        real_T *xd{&indirect_excitation_2mva_pla_DW.StateSpace_DSTATE_h[0]};

                        for (i2 = 0; i2 < 6; i2++) {
                            accum += *(Cs++) * xd[i2];
                        }
                    }

                    {
                        int_T i2;
                        const real_T *u0{indirect_excitation_2mva_ConstP.pooled25};

                        for (i2 = 0; i2 < 6; i2++) {
                            accum += *(Ds++) * u0[i2];
                        }

                        accum += *(Ds++) * indirect_excitation_2mva_plan_B.ib_g[0];
                        accum += *(Ds++) * indirect_excitation_2mva_plan_B.ib_g[1];
                        accum += *(Ds++) * indirect_excitation_2mva_plan_B.UnitDelay2;
                    }

                    y0[i1] = accum;
                }
            }
        }

        // Output new switches states
        {
            int_T i1;
            real_T *y1{&indirect_excitation_2mva_plan_B.StateSpace_o2_p[0]};

            for (i1 = 0; i1 < 6; i1++) {
                y1[i1] = (real_T)switch_status[i1];
            }
        }
    }

    // Outport: '<Root>/i_abc_exc_pu'
    indirect_excitation_2mva_plan_Y.i_abc_exc_pu[0] = indirect_excitation_2mva_plan_B.StateSpace_o1_l[12];
    indirect_excitation_2mva_plan_Y.i_abc_exc_pu[1] = indirect_excitation_2mva_plan_B.StateSpace_o1_l[13];
    indirect_excitation_2mva_plan_Y.i_abc_exc_pu[2] = indirect_excitation_2mva_plan_B.StateSpace_o1_l[14];

    // Gain: '<S83>/N' incorporates:
    //   UnitDelay: '<S4>/Unit Delay4'

    rtb_Fcn1 = 0.00010300496912957533 * indirect_excitation_2mva_pla_DW.UnitDelay4_DSTATE;

    // Switch: '<S98>/IC' incorporates:
    //   DigitalClock: '<S85>/Digital Clock'
    //   Sum: '<S98>/Ad*x(k-1) + Bd*( u(k-1) + u(k))'
    //   UnitDelay: '<S98>/fluxes'

    if (rtb_u2H >= 0.0001) {
        // Assignment: '<S95>/W(2,1)=-wr' incorporates:
        //   SignalConversion generated from: '<S95>/W(1,2)=wr'

        std::memset(&rtb_UpdateLmdrows234col234_p[0], 0, 25U * sizeof(real_T));

        // Assignment: '<S95>/W(1,2)=wr' incorporates:
        //   Assignment: '<S95>/W(2,1)=-wr'
        //   Constant: '<S85>/nominal speed'
        //   Sum: '<S85>/Sum'

        rtb_UpdateLmdrows234col234_p[5] = rtb_Sum1_b + 1.0;

        // Assignment: '<S95>/W(2,1)=-wr' incorporates:
        //   Constant: '<S85>/nominal speed'
        //   Gain: '<S95>/Gain1'
        //   Sum: '<S85>/Sum'

        rtb_UpdateLmdrows234col234_p[1] = -(rtb_Sum1_b + 1.0);
        for (i = 0; i < 25; i++) {
            // Gain: '<S106>/wbase*Ts//2' incorporates:
            //   Assignment: '<S95>/W(2,1)=-wr'
            //   Constant: '<S87>/Constant6'
            //   Gain: '<S106>/wbase*Ts//2 '
            //   Sum: '<S87>/Sum1'
            //   Switch: '<S87>/Switch1'

            rtb_UpdateLmdrows234col234_a_0 =
                ((0.0 - rtb_UpdateLmdrows234col234_p[i]) - indirect_excitation_2mva_ConstP.Constant6_Value_o[i]) *
                0.015707963267948967;
            rtb_UpdateLmdrows234col234_p[i] = rtb_UpdateLmdrows234col234_a_0;

            // Sum: '<S106>/Sum5' incorporates:
            //   Constant: '<S106>/u5'
            //   Gain: '<S106>/wbase*Ts//2 '

            rtb_Mult1_idx_1 = indirect_excitation_2mva_ConstP.pooled23[i];
            rtb_Sum5[i] = rtb_Mult1_idx_1 + rtb_UpdateLmdrows234col234_a_0;

            // Sum: '<S106>/Sum1' incorporates:
            //   Constant: '<S106>/u5'
            //   Gain: '<S106>/wbase*Ts//2 '

            rtb_ukuk1_p[i] = rtb_Mult1_idx_1 - rtb_UpdateLmdrows234col234_a_0;
        }

        // Product: '<S106>/inversion' incorporates:
        //   Gain: '<S106>/wbase*Ts//2 '

        rt_invd5x5_snf(rtb_ukuk1_p, rtb_UpdateLmdrows234col234_p);

        // Product: '<S106>/Product4' incorporates:
        //   Gain: '<S106>/wbase*Ts//2 '
        //   Sum: '<S106>/Sum5'

        for (i = 0; i < 5; i++) {
            for (i_0 = 0; i_0 < 5; i_0++) {
                rtb_UpdateLmdrows234col234_a_0 = 0.0;
                for (rtb_UpdateLmdrows234col234_a_tm = 0; rtb_UpdateLmdrows234col234_a_tm < 5;
                     rtb_UpdateLmdrows234col234_a_tm++) {
                    rtb_UpdateLmdrows234col234_a_0 +=
                        rtb_UpdateLmdrows234col234_p[5 * rtb_UpdateLmdrows234col234_a_tm + i_0] *
                        rtb_Sum5[5 * i + rtb_UpdateLmdrows234col234_a_tm];
                }

                rtb_ukuk1_p[i_0 + 5 * i] = rtb_UpdateLmdrows234col234_a_0;
            }
        }

        // End of Product: '<S106>/Product4'

        // Sum: '<S98>/sum' incorporates:
        //   Constant: '<S83>/[ Vkd =0 Vkq1=0  Vkq2=0 ]'
        //   UnitDelay: '<S98>/voltages'

        rtb_changeIqIdcurrentsigns_b[0] = rtb_Sum3 + indirect_excitation_2mva_pla_DW.voltages_DSTATE[0];
        rtb_changeIqIdcurrentsigns_b[1] = rtb_Product5 + indirect_excitation_2mva_pla_DW.voltages_DSTATE[1];
        rtb_changeIqIdcurrentsigns_b[2] = rtb_Fcn1 + indirect_excitation_2mva_pla_DW.voltages_DSTATE[2];
        rtb_changeIqIdcurrentsigns_b[3] = indirect_excitation_2mva_pla_DW.voltages_DSTATE[3];
        rtb_changeIqIdcurrentsigns_b[4] = indirect_excitation_2mva_pla_DW.voltages_DSTATE[4];
        for (i = 0; i < 5; i++) {
            // Product: '<S98>/Product2'
            rtb_UpdateLmdrows234col234_a_0 = 0.0;

            // Product: '<S98>/Product1' incorporates:
            //   Gain: '<S106>/wbase*Ts//2 '

            rtb_Mult1_idx_1 = 0.0;
            for (i_0 = 0; i_0 < 5; i_0++) {
                // Product: '<S98>/Product2' incorporates:
                //   Gain: '<S106>/wbase*Ts//2 '
                //   Product: '<S98>/Product1'
                //   UnitDelay: '<S98>/fluxes'

                rtb_UpdateLmdrows234col234_a_tm = 5 * i_0 + i;
                rtb_UpdateLmdrows234col234_a_0 +=
                    rtb_ukuk1_p[rtb_UpdateLmdrows234col234_a_tm] * indirect_excitation_2mva_pla_DW.fluxes_DSTATE_m[i_0];

                // Product: '<S98>/Product1' incorporates:
                //   Gain: '<S106>/wbase*Ts//2 '

                rtb_Mult1_idx_1 += rtb_UpdateLmdrows234col234_p[rtb_UpdateLmdrows234col234_a_tm] *
                                   0.015707963267948967 * rtb_changeIqIdcurrentsigns_b[i_0];
            }

            // Product: '<S98>/Product1' incorporates:
            //   Gain: '<S106>/wbase*Ts//2 '

            tmp[i] = rtb_Mult1_idx_1;

            // Product: '<S98>/Product2'
            rtb_UpdateLmdrows234col234_a[i] = rtb_UpdateLmdrows234col234_a_0;
        }

        for (i = 0; i < 5; i++) {
            indirect_excitation_2mva_pla_DW.fluxes_DSTATE_m[i] = rtb_UpdateLmdrows234col234_a[i] + tmp[i];
        }
    }

    // End of Switch: '<S98>/IC'

    // Gain: '<S42>/N' incorporates:
    //   Inport: '<Root>/field_voltage'

    rtb_Fcn1_d = 0.0030618621784789728 * indirect_excitation_2mva_plan_U.field_voltage;

    // Gain: '<S47>/1_Vb'
    rtb_Mult1_idx_1 = 0.0030618621784789728 * indirect_excitation_2mva_plan_B.StateSpace_o1_l[7];

    // Fcn: '<S47>/Fcn2' incorporates:
    //   Fcn: '<S47>/Fcn3'
    //   Gain: '<S47>/1_Vb'

    rtb_UpdateLmdrows234col234_a_0 =
        0.0030618621784789728 * indirect_excitation_2mva_plan_B.StateSpace_o1_l[6] * 2.0 + rtb_Mult1_idx_1;
    rtb_Fcn2_b = (1.7320508075688772 * rtb_Mult1_idx_1 * rtb_theta_f +
                  rtb_UpdateLmdrows234col234_a_0 * rtb_DataTypeConversion6) *
                 0.33333333333333331;

    // Fcn: '<S47>/Fcn3'
    rtb_theta_f = (-1.7320508075688772 * rtb_Mult1_idx_1 * rtb_DataTypeConversion6 +
                   rtb_UpdateLmdrows234col234_a_0 * rtb_theta_f) *
                  0.33333333333333331;

    // Switch: '<S57>/IC' incorporates:
    //   DigitalClock: '<S85>/Digital Clock'
    //   Gain: '<S65>/wbase*Ts//2 '
    //   Product: '<S57>/Product1'
    //   Sum: '<S57>/Ad*x(k-1) + Bd*( u(k-1) + u(k))'
    //   UnitDelay: '<S57>/fluxes'

    if (rtb_u2H >= 0.0001) {
        // Assignment: '<S54>/W(2,1)=-wr' incorporates:
        //   SignalConversion generated from: '<S54>/W(1,2)=wr'

        std::memset(&rtb_UpdateLmdrows234col234_p[0], 0, 25U * sizeof(real_T));

        // Assignment: '<S54>/W(1,2)=wr' incorporates:
        //   Assignment: '<S54>/W(2,1)=-wr'
        //   Gain: '<S44>/1_Pb'

        rtb_UpdateLmdrows234col234_p[5] = indirect_excitation_2mva_plan_Y.wm;

        // Assignment: '<S54>/W(2,1)=-wr' incorporates:
        //   Gain: '<S44>/1_Pb'
        //   Gain: '<S54>/Gain1'

        rtb_UpdateLmdrows234col234_p[1] = -indirect_excitation_2mva_plan_Y.wm;
        for (i = 0; i < 25; i++) {
            // Gain: '<S65>/wbase*Ts//2' incorporates:
            //   Assignment: '<S54>/W(2,1)=-wr'
            //   Constant: '<S46>/Constant6'
            //   Gain: '<S65>/wbase*Ts//2 '
            //   Sum: '<S46>/Sum1'
            //   Switch: '<S46>/Switch1'

            rtb_UpdateLmdrows234col234_a_0 =
                ((0.0 - rtb_UpdateLmdrows234col234_p[i]) - indirect_excitation_2mva_ConstP.Constant6_Value[i]) *
                0.015707963267948967;
            rtb_UpdateLmdrows234col234_p[i] = rtb_UpdateLmdrows234col234_a_0;

            // Sum: '<S65>/Sum5' incorporates:
            //   Constant: '<S65>/u5'
            //   Gain: '<S65>/wbase*Ts//2 '
            //   Product: '<S57>/Product2'

            rtb_Mult1_idx_1 = indirect_excitation_2mva_ConstP.pooled23[i];
            rtb_Sum5[i] = rtb_Mult1_idx_1 + rtb_UpdateLmdrows234col234_a_0;

            // Sum: '<S65>/Sum1' incorporates:
            //   Constant: '<S65>/u5'
            //   Gain: '<S65>/wbase*Ts//2 '

            rtb_ukuk1_p[i] = rtb_Mult1_idx_1 - rtb_UpdateLmdrows234col234_a_0;
        }

        // Product: '<S65>/inversion' incorporates:
        //   Gain: '<S65>/wbase*Ts//2 '

        rt_invd5x5_snf(rtb_ukuk1_p, rtb_UpdateLmdrows234col234_p);

        // Product: '<S65>/Product4' incorporates:
        //   Gain: '<S65>/wbase*Ts//2 '
        //   Product: '<S57>/Product2'
        //   Sum: '<S57>/sum'

        for (i = 0; i < 5; i++) {
            for (i_0 = 0; i_0 < 5; i_0++) {
                rtb_u2H = 0.0;
                for (rtb_UpdateLmdrows234col234_a_tm = 0; rtb_UpdateLmdrows234col234_a_tm < 5;
                     rtb_UpdateLmdrows234col234_a_tm++) {
                    rtb_u2H += rtb_UpdateLmdrows234col234_p[5 * rtb_UpdateLmdrows234col234_a_tm + i_0] *
                               rtb_Sum5[5 * i + rtb_UpdateLmdrows234col234_a_tm];
                }

                rtb_ukuk1_p[i_0 + 5 * i] = rtb_u2H;
            }
        }

        // End of Product: '<S65>/Product4'

        // Product: '<S57>/Product2' incorporates:
        //   Sum: '<S57>/sum'
        //   UnitDelay: '<S57>/fluxes'

        for (i = 0; i < 5; i++) {
            rtb_u2H = 0.0;
            for (i_0 = 0; i_0 < 5; i_0++) {
                rtb_u2H += rtb_ukuk1_p[5 * i_0 + i] * indirect_excitation_2mva_pla_DW.fluxes_DSTATE[i_0];
            }

            rtb_Sum5[i] = rtb_u2H;
        }

        // End of Product: '<S57>/Product2'

        // Sum: '<S57>/sum' incorporates:
        //   Constant: '<S42>/[ Vkd =0 Vkq1=0  Vkq2=0 ]'
        //   UnitDelay: '<S57>/voltages'

        rtb_changeIqIdcurrentsigns_b[0] = rtb_Fcn2_b + indirect_excitation_2mva_pla_DW.voltages_DSTATE_b[0];
        rtb_changeIqIdcurrentsigns_b[1] = rtb_theta_f + indirect_excitation_2mva_pla_DW.voltages_DSTATE_b[1];
        rtb_changeIqIdcurrentsigns_b[2] = rtb_Fcn1_d + indirect_excitation_2mva_pla_DW.voltages_DSTATE_b[2];
        rtb_changeIqIdcurrentsigns_b[3] = indirect_excitation_2mva_pla_DW.voltages_DSTATE_b[3];
        rtb_changeIqIdcurrentsigns_b[4] = indirect_excitation_2mva_pla_DW.voltages_DSTATE_b[4];
        for (i = 0; i < 5; i++) {
            rtb_Mult1_idx_1 = 0.0;
            for (i_0 = 0; i_0 < 5; i_0++) {
                rtb_Mult1_idx_1 += rtb_UpdateLmdrows234col234_p[5 * i_0 + i] * 0.015707963267948967 *
                                   rtb_changeIqIdcurrentsigns_b[i_0];
            }

            indirect_excitation_2mva_pla_DW.fluxes_DSTATE[i] = rtb_Sum5[i] + rtb_Mult1_idx_1;
        }
    }

    // End of Switch: '<S57>/IC'

    // Outport: '<Root>/v_abc_exc_pu' incorporates:
    //   Gain: '<S1>/Kv1'

    indirect_excitation_2mva_plan_Y.v_abc_exc_pu[0] =
        0.0030618621784789723 * indirect_excitation_2mva_plan_B.StateSpace_o1_l[8];

    // Outport: '<Root>/v_abc_sm_pu' incorporates:
    //   Gain: '<S2>/Kv1'

    indirect_excitation_2mva_plan_Y.v_abc_sm_pu[0] =
        0.0030618621784789723 * indirect_excitation_2mva_plan_B.StateSpace_o1[5];

    // Outport: '<Root>/i_abc_sm_pu' incorporates:
    //   Gain: '<S2>/Kv'

    indirect_excitation_2mva_plan_Y.i_abc_sm_pu[0] =
        0.00024494897427831779 * indirect_excitation_2mva_plan_B.StateSpace_o1[8];

    // Outport: '<Root>/v_abc_exc_pu' incorporates:
    //   Gain: '<S1>/Kv1'

    indirect_excitation_2mva_plan_Y.v_abc_exc_pu[1] =
        0.0030618621784789723 * indirect_excitation_2mva_plan_B.StateSpace_o1_l[9];

    // Outport: '<Root>/v_abc_sm_pu' incorporates:
    //   Gain: '<S2>/Kv1'

    indirect_excitation_2mva_plan_Y.v_abc_sm_pu[1] =
        0.0030618621784789723 * indirect_excitation_2mva_plan_B.StateSpace_o1[6];

    // Outport: '<Root>/i_abc_sm_pu' incorporates:
    //   Gain: '<S2>/Kv'

    indirect_excitation_2mva_plan_Y.i_abc_sm_pu[1] =
        0.00024494897427831779 * indirect_excitation_2mva_plan_B.StateSpace_o1[9];

    // Outport: '<Root>/v_abc_exc_pu' incorporates:
    //   Gain: '<S1>/Kv1'

    indirect_excitation_2mva_plan_Y.v_abc_exc_pu[2] =
        0.0030618621784789723 * indirect_excitation_2mva_plan_B.StateSpace_o1_l[10];

    // Outport: '<Root>/v_abc_sm_pu' incorporates:
    //   Gain: '<S2>/Kv1'

    indirect_excitation_2mva_plan_Y.v_abc_sm_pu[2] =
        0.0030618621784789723 * indirect_excitation_2mva_plan_B.StateSpace_o1[7];

    // Outport: '<Root>/i_abc_sm_pu' incorporates:
    //   Gain: '<S2>/Kv'

    indirect_excitation_2mva_plan_Y.i_abc_sm_pu[2] =
        0.00024494897427831779 * indirect_excitation_2mva_plan_B.StateSpace_o1[10];

    // Gain: '<S85>/1 ----- 2H' incorporates:
    //   Constant: '<S85>/nominal speed'
    //   Fcn: '<S85>/div'
    //   Gain: '<Root>/Gain7'
    //   Gain: '<S85>/1_Pb'
    //   Gain: '<S85>/F'
    //   Gain: '<S94>/1-1'
    //   Inport: '<Root>/mechanical_power'
    //   Product: '<S94>/Mult1'
    //   Sum: '<S85>/Sum'
    //   Sum: '<S85>/Sum2'
    //   Sum: '<S94>/Sum2'

    rtb_u2H = ((2.0E+6 * indirect_excitation_2mva_plan_U.mechanical_power * 5.0E-7 / (rtb_Sum1_b + 1.0) -
                (rtb_changeIqIdcurrentsigns[0] * rtb_xk1_i[1] + rtb_xk1_i[0] * -rtb_changeIqIdcurrentsigns[1])) -
               (rtb_Sum1_b + 1.0) * 0.01973920880217872) *
              0.71855682994995107;

    // DiscreteIntegrator: '<S85>/Rotor speed deviation (dw)' incorporates:
    //   UnitDelay: '<S107>/dw_delay'

    if (indirect_excitation_2mva_pla_DW.Rotorspeeddeviationdw_SYSTEM_EN != 0) {
        indirect_excitation_2mva_pla_DW.dw_delay_DSTATE = indirect_excitation_2mva_pla_DW.Rotorspeeddeviationdw_DSTATE;
    } else {
        indirect_excitation_2mva_pla_DW.dw_delay_DSTATE =
            5.0E-5 * rtb_u2H + indirect_excitation_2mva_pla_DW.Rotorspeeddeviationdw_DSTATE;
    }

    // End of DiscreteIntegrator: '<S85>/Rotor speed deviation (dw)'

    // Outport: '<Root>/vt' incorporates:
    //   Gain: '<Root>/Gain3'
    //   Math: '<S80>/Square'
    //   Math: '<S80>/Square1'
    //   Sqrt: '<S80>/Sqrt'
    //   Sum: '<S80>/Sum'

    indirect_excitation_2mva_plan_Y.vt = std::sqrt(rtb_outputformatting_idx_11 * rtb_outputformatting_idx_11 +
                                                   rtb_outputformatting_idx_12 * rtb_outputformatting_idx_12) *
                                         0.0030618621784789728;

    // Outport: '<Root>/it' incorporates:
    //   Gain: '<Root>/Gain6'
    //   Math: '<S79>/Square'
    //   Math: '<S79>/Square1'
    //   Sqrt: '<S79>/Sqrt'
    //   Sum: '<S79>/Sum'

    indirect_excitation_2mva_plan_Y.it = std::sqrt(rtb_outputformatting_idx_3 * rtb_outputformatting_idx_3 +
                                                   rtb_outputformatting_idx_4 * rtb_outputformatting_idx_4) *
                                         0.00024494897427831779;

    // Outport: '<Root>/ifd' incorporates:
    //   Gain: '<Root>/Gain'

    indirect_excitation_2mva_plan_Y.ifd = 0.01 * rtb_Gain1_i;

    // Outport: '<Root>/st' incorporates:
    //   Math: '<S78>/Square'
    //   Math: '<S78>/Square1'
    //   Sqrt: '<S78>/Sqrt'
    //   Sum: '<S78>/Sum'

    indirect_excitation_2mva_plan_Y.st =
        std::sqrt(indirect_excitation_2mva_plan_Y.pt * indirect_excitation_2mva_plan_Y.pt +
                  indirect_excitation_2mva_plan_Y.qt * indirect_excitation_2mva_plan_Y.qt);

    // Outport: '<Root>/vd' incorporates:
    //   Gain: '<Root>/Gain1'

    indirect_excitation_2mva_plan_Y.vd = 0.0030618621784789728 * rtb_outputformatting_idx_12;

    // Outport: '<Root>/vq' incorporates:
    //   Gain: '<Root>/Gain2'

    indirect_excitation_2mva_plan_Y.vq = 0.0030618621784789728 * rtb_outputformatting_idx_11;

    // Outport: '<Root>/id' incorporates:
    //   Gain: '<Root>/Gain4'

    indirect_excitation_2mva_plan_Y.id = 0.00024494897427831779 * rtb_outputformatting_idx_4;

    // Outport: '<Root>/iq' incorporates:
    //   Gain: '<Root>/Gain5'

    indirect_excitation_2mva_plan_Y.iq = 0.00024494897427831779 * rtb_outputformatting_idx_3;

    // Outport: '<Root>/wt' incorporates:
    //   Constant: '<S81>/Constant2'
    //   Constant: '<S81>/Constant3'
    //   DiscreteIntegrator: '<S85>/theta'
    //   Fcn: '<S85>/Fcn'
    //   Gain: '<S6>/Gain'
    //   Gain: '<S85>/t'
    //   Math: '<S81>/Mod'
    //   Sum: '<S81>/Sum3'
    //   Sum: '<S81>/Sum4'

    indirect_excitation_2mva_plan_Y.wt =
        rt_modd_snf(57.295779513082323 * rt_remd_snf(indirect_excitation_2mva_pla_DW.theta_DSTATE, 6.2831853071795862) *
                            0.017453292519943295 +
                        3.1415926535897931,
                    6.2831853071795862) -
        3.1415926535897931;

    // Switch: '<S116>/Switch3' incorporates:
    //   DataTypeConversion: '<S10>/Data Type  Conversion'
    //   DataTypeConversion: '<S10>/Data Type  Conversion4'
    //   Inport: '<Root>/three_phase_fault_en'
    //   Logic: '<S10>/Logical Operator1'

    indirect_excitation_2mva_plan_B.Switch3 = ((indirect_excitation_2mva_plan_U.three_phase_fault_en != 0.0) &&
                                               indirect_excitation_2mva_ConstB.DataTypeConversion3);

    // Switch: '<S118>/Switch3' incorporates:
    //   DataTypeConversion: '<S10>/Data Type  Conversion'
    //   DataTypeConversion: '<S10>/Data Type  Conversion5'
    //   Inport: '<Root>/three_phase_fault_en'
    //   Logic: '<S10>/Logical Operator2'

    indirect_excitation_2mva_plan_B.Switch3_b = ((indirect_excitation_2mva_plan_U.three_phase_fault_en != 0.0) &&
                                                 indirect_excitation_2mva_ConstB.DataTypeConversion2);

    // Switch: '<S120>/Switch3' incorporates:
    //   DataTypeConversion: '<S10>/Data Type  Conversion'
    //   DataTypeConversion: '<S10>/Data Type  Conversion6'
    //   Inport: '<Root>/three_phase_fault_en'
    //   Logic: '<S10>/Logical Operator3'

    indirect_excitation_2mva_plan_B.Switch3_k = ((indirect_excitation_2mva_plan_U.three_phase_fault_en != 0.0) &&
                                                 indirect_excitation_2mva_ConstB.DataTypeConversion1);

    // Update for UnitDelay: '<S107>/dw_predict'
    indirect_excitation_2mva_pla_DW.dw_predict_DSTATE = rtb_dw_delay;

    // Update for DiscreteIntegrator: '<S85>/Rotor angle dthetae' incorporates:
    //   Gain: '<S85>/we base'

    indirect_excitation_2mva_pla_DW.Rotorangledthetae_DSTATE += 314.15926535897933 * rtb_Sum1_b * 0.0001;

    // Update for S-Function (sfun_spssw_discc): '<S124>/State-Space' incorporates:
    //   Constant: '<S130>/SwitchCurrents'

    // S-Function block: <S124>/State-Space
    {
        const real_T *As = (real_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK.AS;
        const real_T *Bs = (real_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK.BS;
        real_T *xtmp = (real_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK.XTMP;
        real_T accum;

        // Calculate new states...
        {
            int_T i1;
            real_T *xd{&indirect_excitation_2mva_pla_DW.StateSpace_DSTATE[0]};

            for (i1 = 0; i1 < 6; i1++) {
                accum = 0.0;

                {
                    int_T i2;
                    real_T *xd{&indirect_excitation_2mva_pla_DW.StateSpace_DSTATE[0]};

                    for (i2 = 0; i2 < 6; i2++) {
                        accum += *(As++) * xd[i2];
                    }
                }

                accum += *(Bs++) * 0.0;
                accum += *(Bs++) * 0.0;
                accum += *(Bs++) * 0.0;
                accum += *(Bs++) * indirect_excitation_2mva_plan_B.ib[0];
                accum += *(Bs++) * indirect_excitation_2mva_plan_B.ib[1];
                accum += *(Bs++) * indirect_excitation_2mva_plan_B.Sum5[0];
                accum += *(Bs++) * indirect_excitation_2mva_plan_B.Sum5[1];
                accum += *(Bs++) * indirect_excitation_2mva_plan_B.Sum5[2];
                xtmp[i1] = accum;
            }
        }

        {
            int_T i1;
            real_T *xd{&indirect_excitation_2mva_pla_DW.StateSpace_DSTATE[0]};

            for (i1 = 0; i1 < 6; i1++) {
                xd[i1] = xtmp[i1];
            }
        }

        {
            int_T *gState = (int_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK.G_STATE;

            // Store switch gates values for next step
            *(gState++) = (int_T)indirect_excitation_2mva_plan_B.Switch3;
            *(gState++) = (int_T)indirect_excitation_2mva_plan_B.Switch3_b;
            *(gState++) = (int_T)indirect_excitation_2mva_plan_B.Switch3_k;
        }
    }

    // Update for DiscreteIntegrator: '<S44>/Rotor angle dtheta' incorporates:
    //   Constant: '<S44>/Nominal speed'
    //   Gain: '<S44>/1_Pb'
    //   Gain: '<S44>/web1'
    //   Sum: '<S44>/Add'

    indirect_excitation_2mva_pla_DW.Rotorangledtheta_DSTATE +=
        (indirect_excitation_2mva_plan_Y.wm - 1.0) * 314.15926535897933 * 0.0001;

    // Update for UnitDelay: '<S4>/Unit Delay2'
    indirect_excitation_2mva_pla_DW.UnitDelay2_DSTATE = rtb_Gain1_i;

    // Update for S-Function (sfun_spssw_discc): '<S123>/State-Space' incorporates:
    //   Constant: '<S126>/SwitchCurrents'
    //   Constant: '<S7>/g'

    // S-Function block: <S123>/State-Space
    {
        const real_T *As = (real_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.AS;
        const real_T *Bs = (real_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.BS;
        real_T *xtmp = (real_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.XTMP;
        real_T accum;

        // Calculate new states...
        {
            int_T i1;
            real_T *xd{&indirect_excitation_2mva_pla_DW.StateSpace_DSTATE_h[0]};

            for (i1 = 0; i1 < 6; i1++) {
                accum = 0.0;

                {
                    int_T i2;
                    real_T *xd{&indirect_excitation_2mva_pla_DW.StateSpace_DSTATE_h[0]};

                    for (i2 = 0; i2 < 6; i2++) {
                        accum += *(As++) * xd[i2];
                    }
                }

                {
                    int_T i2;
                    const real_T *u0{indirect_excitation_2mva_ConstP.pooled25};

                    for (i2 = 0; i2 < 6; i2++) {
                        accum += *(Bs++) * u0[i2];
                    }

                    accum += *(Bs++) * indirect_excitation_2mva_plan_B.ib_g[0];
                    accum += *(Bs++) * indirect_excitation_2mva_plan_B.ib_g[1];
                    accum += *(Bs++) * indirect_excitation_2mva_plan_B.UnitDelay2;
                }

                xtmp[i1] = accum;
            }
        }

        {
            int_T i1;
            real_T *xd{&indirect_excitation_2mva_pla_DW.StateSpace_DSTATE_h[0]};

            for (i1 = 0; i1 < 6; i1++) {
                xd[i1] = xtmp[i1];
            }
        }

        {
            int_T *gState = (int_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.G_STATE;

            // Store switch gates values for next step
            {
                int_T i1;
                const real_T *u1{indirect_excitation_2mva_ConstP.pooled25};

                for (i1 = 0; i1 < 6; i1++) {
                    *(gState++) = (int_T)u1[i1];
                }
            }
        }
    }

    // Update for UnitDelay: '<S4>/Unit Delay4'
    indirect_excitation_2mva_pla_DW.UnitDelay4_DSTATE = indirect_excitation_2mva_plan_B.StateSpace_o1_l[11];

    // Update for UnitDelay: '<S98>/voltages'
    indirect_excitation_2mva_pla_DW.voltages_DSTATE[0] = rtb_Sum3;
    indirect_excitation_2mva_pla_DW.voltages_DSTATE[1] = rtb_Product5;
    indirect_excitation_2mva_pla_DW.voltages_DSTATE[2] = rtb_Fcn1;

    // Update for UnitDelay: '<S57>/voltages'
    indirect_excitation_2mva_pla_DW.voltages_DSTATE_b[0] = rtb_Fcn2_b;
    indirect_excitation_2mva_pla_DW.voltages_DSTATE_b[1] = rtb_theta_f;
    indirect_excitation_2mva_pla_DW.voltages_DSTATE_b[2] = rtb_Fcn1_d;

    // Update for UnitDelay: '<S98>/voltages' incorporates:
    //   Constant: '<S83>/[ Vkd =0 Vkq1=0  Vkq2=0 ]'

    indirect_excitation_2mva_pla_DW.voltages_DSTATE[3] = 0.0;

    // Update for UnitDelay: '<S57>/voltages' incorporates:
    //   Constant: '<S42>/[ Vkd =0 Vkq1=0  Vkq2=0 ]'

    indirect_excitation_2mva_pla_DW.voltages_DSTATE_b[3] = 0.0;

    // Update for UnitDelay: '<S98>/voltages' incorporates:
    //   Constant: '<S83>/[ Vkd =0 Vkq1=0  Vkq2=0 ]'

    indirect_excitation_2mva_pla_DW.voltages_DSTATE[4] = 0.0;

    // Update for UnitDelay: '<S57>/voltages' incorporates:
    //   Constant: '<S42>/[ Vkd =0 Vkq1=0  Vkq2=0 ]'

    indirect_excitation_2mva_pla_DW.voltages_DSTATE_b[4] = 0.0;

    // Update for DiscreteIntegrator: '<S85>/Rotor speed deviation (dw)' incorporates:
    //   UnitDelay: '<S107>/dw_delay'

    indirect_excitation_2mva_pla_DW.Rotorspeeddeviationdw_SYSTEM_EN = 0U;
    indirect_excitation_2mva_pla_DW.Rotorspeeddeviationdw_DSTATE =
        5.0E-5 * rtb_u2H + indirect_excitation_2mva_pla_DW.dw_delay_DSTATE;

    // Update for DiscreteIntegrator: '<S85>/theta'
    indirect_excitation_2mva_pla_DW.theta_DSTATE += rtb_units_tmp * 0.0001;

    // Update absolute time
    // The "clockTick1" counts the number of times the code of this task has
    //  been executed. The resolution of this integer timer is 0.0001, which is the step size
    //  of the task. Size of "clockTick1" ensures timer will not overflow during the
    //  application lifespan selected.

    (&indirect_excitation_2mva_pla_M)->Timing.clockTick1++;
}

// Model initialize function
void indirect_excitation_2mva::initialize() {
    // Registration code

    // initialize non-finites
    rt_InitInfAndNaN(sizeof(real_T));

    {
        int32_T i;

        // Start for S-Function (sfun_spssw_discc): '<S124>/State-Space' incorporates:
        //   Constant: '<S130>/SwitchCurrents'

        // S-Function block: <S124>/State-Space
        {
            indirect_excitation_2mva_pla_DW.StateSpace_PWORK.AS = (real_T *)calloc(6 * 6, sizeof(real_T));
            indirect_excitation_2mva_pla_DW.StateSpace_PWORK.BS = (real_T *)calloc(6 * 8, sizeof(real_T));
            indirect_excitation_2mva_pla_DW.StateSpace_PWORK.CS = (real_T *)calloc(11 * 6, sizeof(real_T));
            indirect_excitation_2mva_pla_DW.StateSpace_PWORK.DS = (real_T *)calloc(11 * 8, sizeof(real_T));
            indirect_excitation_2mva_pla_DW.StateSpace_PWORK.DX_COL = (real_T *)calloc(11, sizeof(real_T));
            indirect_excitation_2mva_pla_DW.StateSpace_PWORK.TMP2 = (real_T *)calloc(8, sizeof(real_T));
            indirect_excitation_2mva_pla_DW.StateSpace_PWORK.BD_COL = (real_T *)calloc(6, sizeof(real_T));
            indirect_excitation_2mva_pla_DW.StateSpace_PWORK.TMP1 = (real_T *)calloc(6, sizeof(real_T));
            indirect_excitation_2mva_pla_DW.StateSpace_PWORK.XTMP = (real_T *)calloc(6, sizeof(real_T));
            indirect_excitation_2mva_pla_DW.StateSpace_PWORK.SWITCH_STATUS = (int_T *)calloc(3, sizeof(int_T));
            indirect_excitation_2mva_pla_DW.StateSpace_PWORK.SW_CHG = (int_T *)calloc(3, sizeof(int_T));
            indirect_excitation_2mva_pla_DW.StateSpace_PWORK.G_STATE = (int_T *)calloc(3, sizeof(int_T));
            indirect_excitation_2mva_pla_DW.StateSpace_PWORK.Y_SWITCH = (real_T *)calloc(3, sizeof(real_T));
            indirect_excitation_2mva_pla_DW.StateSpace_PWORK.SWITCH_TYPES = (int_T *)calloc(3, sizeof(int_T));
            indirect_excitation_2mva_pla_DW.StateSpace_PWORK.IDX_OUT_SW = (int_T *)calloc(3, sizeof(int_T));
            indirect_excitation_2mva_pla_DW.StateSpace_PWORK.SWITCH_STATUS_INIT = (int_T *)calloc(3, sizeof(int_T));
            indirect_excitation_2mva_pla_DW.StateSpace_PWORK.USWLAST = (real_T *)calloc(3, sizeof(real_T));
        }

        // Start for S-Function (sfun_spssw_discc): '<S123>/State-Space' incorporates:
        //   Constant: '<S126>/SwitchCurrents'
        //   Constant: '<S7>/g'

        // S-Function block: <S123>/State-Space
        {
            indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.AS = (real_T *)calloc(6 * 6, sizeof(real_T));
            indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.BS = (real_T *)calloc(6 * 9, sizeof(real_T));
            indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.CS = (real_T *)calloc(15 * 6, sizeof(real_T));
            indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.DS = (real_T *)calloc(15 * 9, sizeof(real_T));
            indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.DX_COL = (real_T *)calloc(15, sizeof(real_T));
            indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.TMP2 = (real_T *)calloc(9, sizeof(real_T));
            indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.BD_COL = (real_T *)calloc(6, sizeof(real_T));
            indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.TMP1 = (real_T *)calloc(6, sizeof(real_T));
            indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.XTMP = (real_T *)calloc(6, sizeof(real_T));
            indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.SWITCH_STATUS = (int_T *)calloc(6, sizeof(int_T));
            indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.SW_CHG = (int_T *)calloc(6, sizeof(int_T));
            indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.G_STATE = (int_T *)calloc(6, sizeof(int_T));
            indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.Y_SWITCH = (real_T *)calloc(6, sizeof(real_T));
            indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.SWITCH_TYPES = (int_T *)calloc(6, sizeof(int_T));
            indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.IDX_OUT_SW = (int_T *)calloc(6, sizeof(int_T));
            indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.SWITCH_STATUS_INIT = (int_T *)calloc(6, sizeof(int_T));
            indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.USWLAST = (real_T *)calloc(6, sizeof(real_T));
        }

        for (i = 0; i < 5; i++) {
            // InitializeConditions for UnitDelay: '<S57>/fluxes'
            indirect_excitation_2mva_pla_DW.fluxes_DSTATE[i] =
                indirect_excitation_2mva_ConstP.fluxes_InitialCondition[i];

            // InitializeConditions for UnitDelay: '<S98>/fluxes'
            indirect_excitation_2mva_pla_DW.fluxes_DSTATE_m[i] =
                indirect_excitation_2mva_ConstP.fluxes_InitialCondition_d[i];
        }

        // InitializeConditions for DiscreteIntegrator: '<S85>/Rotor angle dthetae'
        indirect_excitation_2mva_pla_DW.Rotorangledthetae_DSTATE = -1.1736728354423667;

        // InitializeConditions for S-Function (sfun_spssw_discc): '<S124>/State-Space' incorporates:
        //   Constant: '<S130>/SwitchCurrents'

        {
            int32_T i, j;
            real_T *As = (real_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK.AS;
            real_T *Bs = (real_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK.BS;
            real_T *Cs = (real_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK.CS;
            real_T *Ds = (real_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK.DS;
            real_T *X0 = (real_T *)&indirect_excitation_2mva_pla_DW.StateSpace_DSTATE[0];
            for (i = 0; i < 6; i++) {
                X0[i] = (indirect_excitation_2mva_ConstP.StateSpace_X0_param[i]);
            }

            // Copy and transpose A and B
            for (i = 0; i < 6; i++) {
                for (j = 0; j < 6; j++)
                    As[i * 6 + j] = (indirect_excitation_2mva_ConstP.StateSpace_AS_param[i + j * 6]);
                for (j = 0; j < 8; j++)
                    Bs[i * 8 + j] = (indirect_excitation_2mva_ConstP.StateSpace_BS_param[i + j * 6]);
            }

            // Copy and transpose C
            for (i = 0; i < 11; i++) {
                for (j = 0; j < 6; j++)
                    Cs[i * 6 + j] = (indirect_excitation_2mva_ConstP.StateSpace_CS_param[i + j * 11]);
            }

            // Copy and transpose D
            for (i = 0; i < 11; i++) {
                for (j = 0; j < 8; j++)
                    Ds[i * 8 + j] = (indirect_excitation_2mva_ConstP.StateSpace_DS_param[i + j * 11]);
            }

            {
                // Switches work vectors
                int_T *switch_status = (int_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK.SWITCH_STATUS;
                int_T *gState = (int_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK.G_STATE;
                real_T *yswitch = (real_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK.Y_SWITCH;
                int_T *switchTypes = (int_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK.SWITCH_TYPES;
                int_T *idxOutSw = (int_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK.IDX_OUT_SW;
                int_T *switch_status_init =
                    (int_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK.SWITCH_STATUS_INIT;

                // Initialize work vectors
                switch_status[0] = 0;
                switch_status_init[0] = 0;
                gState[0] = (int_T)0.0;
                yswitch[0] = 1 / 0.005;
                switchTypes[0] = (int_T)2.0;
                idxOutSw[0] = ((int_T)0.0) - 1;
                switch_status[1] = 0;
                switch_status_init[1] = 0;
                gState[1] = (int_T)0.0;
                yswitch[1] = 1 / 0.005;
                switchTypes[1] = (int_T)2.0;
                idxOutSw[1] = ((int_T)0.0) - 1;
                switch_status[2] = 0;
                switch_status_init[2] = 0;
                gState[2] = (int_T)0.0;
                yswitch[2] = 1 / 0.005;
                switchTypes[2] = (int_T)2.0;
                idxOutSw[2] = ((int_T)0.0) - 1;
            }
        }

        // InitializeConditions for DiscreteIntegrator: '<S44>/Rotor angle dtheta'
        indirect_excitation_2mva_pla_DW.Rotorangledtheta_DSTATE = -1.5157748221257752;

        // InitializeConditions for UnitDelay: '<S4>/Unit Delay2'
        indirect_excitation_2mva_pla_DW.UnitDelay2_DSTATE = 112.53999999999999;

        // InitializeConditions for S-Function (sfun_spssw_discc): '<S123>/State-Space' incorporates:
        //   Constant: '<S126>/SwitchCurrents'
        //   Constant: '<S7>/g'

        {
            int32_T i, j;
            real_T *As = (real_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.AS;
            real_T *Bs = (real_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.BS;
            real_T *Cs = (real_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.CS;
            real_T *Ds = (real_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.DS;
            real_T *X0 = (real_T *)&indirect_excitation_2mva_pla_DW.StateSpace_DSTATE_h[0];
            for (i = 0; i < 6; i++) {
                X0[i] = (indirect_excitation_2mva_ConstP.StateSpace_X0_param_k[i]);
            }

            // Copy and transpose A and B
            for (i = 0; i < 6; i++) {
                for (j = 0; j < 6; j++)
                    As[i * 6 + j] = (indirect_excitation_2mva_ConstP.StateSpace_AS_param_k[i + j * 6]);
                for (j = 0; j < 9; j++)
                    Bs[i * 9 + j] = (indirect_excitation_2mva_ConstP.StateSpace_BS_param_m[i + j * 6]);
            }

            // Copy and transpose C
            for (i = 0; i < 15; i++) {
                for (j = 0; j < 6; j++)
                    Cs[i * 6 + j] = (indirect_excitation_2mva_ConstP.StateSpace_CS_param_m[i + j * 15]);
            }

            // Copy and transpose D
            for (i = 0; i < 15; i++) {
                for (j = 0; j < 9; j++)
                    Ds[i * 9 + j] = (indirect_excitation_2mva_ConstP.StateSpace_DS_param_e[i + j * 15]);
            }

            {
                // Switches work vectors
                int_T *switch_status = (int_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.SWITCH_STATUS;
                int_T *gState = (int_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.G_STATE;
                real_T *yswitch = (real_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.Y_SWITCH;
                int_T *switchTypes = (int_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.SWITCH_TYPES;
                int_T *idxOutSw = (int_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.IDX_OUT_SW;
                int_T *switch_status_init =
                    (int_T *)indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.SWITCH_STATUS_INIT;

                // Initialize work vectors
                switch_status[0] = 0;
                switch_status_init[0] = 0;
                gState[0] = (int_T)0.0;
                yswitch[0] = 1 / 1.0E-5;
                switchTypes[0] = (int_T)3.0;
                idxOutSw[0] = ((int_T)0.0) - 1;
                switch_status[1] = 0;
                switch_status_init[1] = 0;
                gState[1] = (int_T)0.0;
                yswitch[1] = 1 / 1.0E-5;
                switchTypes[1] = (int_T)3.0;
                idxOutSw[1] = ((int_T)0.0) - 1;
                switch_status[2] = 0;
                switch_status_init[2] = 0;
                gState[2] = (int_T)0.0;
                yswitch[2] = 1 / 1.0E-5;
                switchTypes[2] = (int_T)3.0;
                idxOutSw[2] = ((int_T)0.0) - 1;
                switch_status[3] = 0;
                switch_status_init[3] = 0;
                gState[3] = (int_T)0.0;
                yswitch[3] = 1 / 1.0E-5;
                switchTypes[3] = (int_T)3.0;
                idxOutSw[3] = ((int_T)0.0) - 1;
                switch_status[4] = 0;
                switch_status_init[4] = 0;
                gState[4] = (int_T)0.0;
                yswitch[4] = 1 / 1.0E-5;
                switchTypes[4] = (int_T)3.0;
                idxOutSw[4] = ((int_T)0.0) - 1;
                switch_status[5] = 0;
                switch_status_init[5] = 0;
                gState[5] = (int_T)0.0;
                yswitch[5] = 1 / 1.0E-5;
                switchTypes[5] = (int_T)3.0;
                idxOutSw[5] = ((int_T)0.0) - 1;
            }
        }

        // InitializeConditions for UnitDelay: '<S4>/Unit Delay4'
        indirect_excitation_2mva_pla_DW.UnitDelay4_DSTATE = 10.4482;

        // InitializeConditions for DiscreteIntegrator: '<S85>/theta'
        indirect_excitation_2mva_pla_DW.theta_DSTATE = -0.58683641772118333;

        // SystemInitialize for Enabled SubSystem: '<S46>/Saturation'
        // InitializeConditions for UnitDelay: '<S58>/Lmd_sat'
        indirect_excitation_2mva_pla_DW.Lmd_sat_DSTATE_h = 1.7272083710354986;

        // End of SystemInitialize for SubSystem: '<S46>/Saturation'

        // SystemInitialize for Enabled SubSystem: '<S87>/Saturation'
        // InitializeConditions for UnitDelay: '<S99>/Lmd_sat'
        indirect_excitation_2mva_pla_DW.Lmd_sat_DSTATE = 2.0600993825915066;

        // End of SystemInitialize for SubSystem: '<S87>/Saturation'

        // Enable for DiscreteIntegrator: '<S85>/Rotor speed deviation (dw)'
        indirect_excitation_2mva_pla_DW.Rotorspeeddeviationdw_SYSTEM_EN = 1U;
    }
}

// Model terminate function
void indirect_excitation_2mva::terminate() {
    // Terminate for S-Function (sfun_spssw_discc): '<S124>/State-Space' incorporates:
    //   Constant: '<S130>/SwitchCurrents'

    // S-Function block: <S124>/State-Space
    {
        // Free memory
        free(indirect_excitation_2mva_pla_DW.StateSpace_PWORK.AS);
        free(indirect_excitation_2mva_pla_DW.StateSpace_PWORK.BS);
        free(indirect_excitation_2mva_pla_DW.StateSpace_PWORK.CS);
        free(indirect_excitation_2mva_pla_DW.StateSpace_PWORK.DS);
        free(indirect_excitation_2mva_pla_DW.StateSpace_PWORK.DX_COL);
        free(indirect_excitation_2mva_pla_DW.StateSpace_PWORK.TMP2);
        free(indirect_excitation_2mva_pla_DW.StateSpace_PWORK.BD_COL);
        free(indirect_excitation_2mva_pla_DW.StateSpace_PWORK.TMP1);
        free(indirect_excitation_2mva_pla_DW.StateSpace_PWORK.XTMP);

        //
        //  Circuit has switches
        free(indirect_excitation_2mva_pla_DW.StateSpace_PWORK.G_STATE);
        free(indirect_excitation_2mva_pla_DW.StateSpace_PWORK.SWITCH_STATUS);
        free(indirect_excitation_2mva_pla_DW.StateSpace_PWORK.SW_CHG);
        free(indirect_excitation_2mva_pla_DW.StateSpace_PWORK.SWITCH_STATUS_INIT);
    }

    // Terminate for S-Function (sfun_spssw_discc): '<S123>/State-Space' incorporates:
    //   Constant: '<S126>/SwitchCurrents'
    //   Constant: '<S7>/g'

    // S-Function block: <S123>/State-Space
    {
        // Free memory
        free(indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.AS);
        free(indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.BS);
        free(indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.CS);
        free(indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.DS);
        free(indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.DX_COL);
        free(indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.TMP2);
        free(indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.BD_COL);
        free(indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.TMP1);
        free(indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.XTMP);

        //
        //  Circuit has switches
        free(indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.G_STATE);
        free(indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.SWITCH_STATUS);
        free(indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.SW_CHG);
        free(indirect_excitation_2mva_pla_DW.StateSpace_PWORK_n.SWITCH_STATUS_INIT);
    }
}

// Constructor
indirect_excitation_2mva::indirect_excitation_2mva()
    : indirect_excitation_2mva_plan_U(),
      indirect_excitation_2mva_plan_Y(),
      indirect_excitation_2mva_plan_B(),
      indirect_excitation_2mva_pla_DW(),
      indirect_excitation_2mva_pla_M() {
    // Currently there is no constructor body generated.
}

// Destructor
// Currently there is no destructor body generated.
indirect_excitation_2mva::~indirect_excitation_2mva() = default;

// Real-Time Model get method
indirect_excitation_2mva::RT_MODEL_indirect_excitation__T *indirect_excitation_2mva::getRTM() {
    return (&indirect_excitation_2mva_pla_M);
}
}  // namespace plant

//
// File trailer for generated code.
//
// [EOF]
//
