/*
 * sfc_6p_6p_topology_machine.cpp
 *
 * Code generation for model "sfc_6p_6p_topology_machine".
 *
 * Model version              : 18.6
 * Simulink Coder version : 23.2 (R2023b) 01-Aug-2023
 * C++ source code generated on : Sat Nov  8 11:51:06 2025
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: 32-bit Generic
 * Code generation objective: Execution efficiency
 * Validation result: Not run
 */

#include "sfc_6p_6p_topology_machine.h"
#include "rtwtypes.h"
#include <cmath>
#include "sfc_6p_6p_topology_machine_private.h"
#include <cstring>

extern "C"
{

#include "rt_nonfinite.h"

}

#include "zero_crossing_types.h"
#include <cfloat>
#include "rt_defines.h"

/* user code (top of source file) */
#include <string.h>

/* For variable transport delay block, find the real delay time */
real_T rt_VTDelayfindtDInterpolate(
  real_T x,real_T* uBuf,int_T bufSz,int_T head,int_T tail,int_T* pLast,real_T t,
  real_T tStart,boolean_T discrete,boolean_T minorStepAndTAtLastMajorOutput,
  real_T initOutput,real_T* appliedDelay)
{
  int_T n, k;
  real_T f;
  int_T kp1;
  real_T tminustD, tL, tR, uD, uL, uR, fU;
  real_T* tBuf = uBuf + bufSz;
  real_T* xBuf = uBuf + 2* bufSz;
  if (minorStepAndTAtLastMajorOutput) {
    /* pretend that the entry at head has not been added */
    if (*pLast == head) {
      *pLast = (*pLast == 0) ? bufSz-1 : *pLast-1;
    }

    head = (head == 0) ? bufSz-1 : head-1;
  }

  /*
   * The loop below finds k such that:
   *      x(t)-x(tminustD) =1 or
   *      x - xBuf[k+1] <= 1.0 < x - xBuf[k]
   *
   * Note that we have:
   *
   * tStart = tBuf[0] < tBuf[1] < ... < tBuf[tail] < ... tBuf[head] <= t
   *      0 = xBuf[0] < xBuf[1] < ... < xBuf[tail] < ... xBuf[head] <  x
   *
   * This is true if we assume the direction of transport is always positive
   * such as a flow goes through a pipe from one end to another. However, for
   * model such as convey belt, the transportation can change direction. For
   * this case, there will be more than one solution to x(t)-x(tminustD) = 1,
   * should found the minimum tminustD and tminustD > 0. The search will not
   * be as efficient as the following code.
   */

  /*
   * when x<=1, physically it means the flow didn't reach the output yet,
   * t-tD will be less then zero, so force output to be the initial output
   */
  if (x <= 1) {
    return initOutput;
  }

  /*
   * if the x is monoton increase, only one solution. use k=pLast for now
   */
  k= *pLast;
  n = 0;
  for (;;) {
    n++;
    if (n>bufSz)
      break;
    if (x - xBuf[k] > 1.0) {
      /* move k forward, unless k = head */
      if (k == head) {
        /* xxx this situation means tD= appliedDelay = 0
         *
         * linearly interpolate using (tBuf[head], xBuf[head])
         * and (t,x) to find (tD,xD) such that: x - xD = 1.0
         */
        int_T km1;
        f = (x - 1.0 - xBuf[k]) / (x - xBuf[k]);
        tminustD = (1.0-f)*tBuf[k] + f*t;
        km1 = k-1;
        if (km1 < 0)
          km1 = bufSz-1;
        tL = tBuf[km1];
        tR = tBuf[k];
        uL = uBuf[km1];
        uR = uBuf[k];
        break;
      }

      kp1 = k+1;
      if (kp1 == bufSz)
        kp1 = 0;
      if (x - xBuf[kp1] <= 1.0) {
        /*
         * linearly interpolate using (tBuf[k], xBuf[k])
         * and  (tBuf[k+1], xBuf[k+1]) to find (tminustD,xD)
         * such that: x - xD = 1.0
         */
        f = (x - 1.0 - xBuf[k]) / (xBuf[kp1] - xBuf[k]);
        tL = tBuf[k];
        tR = tBuf[kp1];
        uL = uBuf[k];
        uR = uBuf[kp1];
        tminustD = (1.0-f)*tL + f*tR;
        break;
      }

      k = kp1;
    } else {
      /* moved k backward, unless k = tail */
      if (k == tail) {
        /* This situation means tminustD <= Ttail*/
        f = (x - 1.0)/xBuf[k];
        if (discrete) {
          return(uBuf[tail]);
        }

        kp1 = k+1;
        if (kp1 == bufSz)
          kp1 = 0;

        /* * linearly interpolate using (tStart, 0)
         * and  (tBuf[tail], xBuf[tail]) to find (tminustD,xD)
         * such that: x - xD = 1.0
         */

        /* Here it is better to use Tstart because since x>1, tminustD
         * must > 0. Since x is monotone increase, its linearity is
         * better.
         */
        tminustD = (1-f)*tStart + f*tBuf[k];

        /* linearly interpolate using (t[tail], x[tail])
         * and  (tBuf[tail+1], xBuf[tail+1]) to find (tminustD,xD)
         * such that: x - xD = 1.0.
         * For time delay block, use t[tail] and t[tail+1], not good
         * for transport delay block since it may give tminstD < 0
         */

        /*  f = (tBuf[kp1]-tBuf[k])/(xBuf[kp1]-xBuf[k]);
         *  tminustD = tBuf[kp1]-f*(1+xBuf[kp1]-x);
         */
        tL = tBuf[k];
        tR = tBuf[kp1];
        uL = uBuf[k];
        uR = uBuf[kp1];
        break;
      }

      k = k - 1;
      if (k < 0)
        k = bufSz-1;
    }
  }

  *pLast = k;
  if (tR == tL) {
    fU = 1.0;
  } else {
    fU = (tminustD-tL)/(tR-tL);
  }

  /* for discrete signal, no interpolation, use either uL or uR
   * depend on wehre tminustD is.
   */
  if (discrete) {
    uD= (fU > (1.0-fU))? uR: uL;
  } else {
    uD = (1.0-fU)*uL + fU*uR;
  }

  /* we want return tD= t-(t-tD);*/
  *appliedDelay = t-tminustD;
  return uD;
}

real_T look1_pbinlxpw(real_T u0, const real_T bp0[], const real_T table[],
                      uint32_T prevIndex[], uint32_T maxIndex)
{
  real_T frac;
  real_T yL_0d0;
  uint32_T bpIdx;

  /* Column-major Lookup 1-D
     Search method: 'binary'
     Use previous index: 'on'
     Interpolation method: 'Linear point-slope'
     Extrapolation method: 'Linear'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Linear'
     Use previous index: 'on'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u0 <= bp0[0U]) {
    bpIdx = 0U;
    frac = (u0 - bp0[0U]) / (bp0[1U] - bp0[0U]);
  } else if (u0 < bp0[maxIndex]) {
    uint32_T found;
    uint32_T iLeft;
    uint32_T iRght;

    /* Binary Search using Previous Index */
    bpIdx = prevIndex[0U];
    iLeft = 0U;
    iRght = maxIndex;
    found = 0U;
    while (found == 0U) {
      if (u0 < bp0[bpIdx]) {
        iRght = bpIdx - 1U;
        bpIdx = ((bpIdx + iLeft) - 1U) >> 1U;
      } else if (u0 < bp0[bpIdx + 1U]) {
        found = 1U;
      } else {
        iLeft = bpIdx + 1U;
        bpIdx = ((bpIdx + iRght) + 1U) >> 1U;
      }
    }

    frac = (u0 - bp0[bpIdx]) / (bp0[bpIdx + 1U] - bp0[bpIdx]);
  } else {
    bpIdx = maxIndex - 1U;
    frac = (u0 - bp0[maxIndex - 1U]) / (bp0[maxIndex] - bp0[maxIndex - 1U]);
  }

  prevIndex[0U] = bpIdx;

  /* Column-major Interpolation 1-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'off'
     Overflow mode: 'portable wrapping'
   */
  yL_0d0 = table[bpIdx];
  return (table[bpIdx + 1U] - yL_0d0) * frac + yL_0d0;
}

/*
 * This function updates continuous states using the ODE1 fixed-step
 * solver algorithm
 */
void sfc_6p_6p_topology_machine::rt_ertODEUpdateContinuousStates(RTWSolverInfo
  *si )
{
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE1_IntgData *id = static_cast<ODE1_IntgData *>(rtsiGetSolverData(si));
  real_T *f0 = id->f[0];
  int_T i;
  int_T nXc = 11;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);
  rtsiSetdX(si, f0);
  sfc_6p_6p_topology_machine_derivatives();
  rtsiSetT(si, tnew);
  for (i = 0; i < nXc; ++i) {
    x[i] += h * f0[i];
  }

  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

/*
 * System initialize for enable system:
 *    '<S48>/Subsystem - pi//2 delay'
 *    '<S59>/Subsystem - pi//2 delay'
 */
void sfc_6p_6p_topology_machine::sfc_6p_6_Subsystempi2delay_Init
  (B_Subsystempi2delay_sfc_6p_6p_T *localB, P_Subsystempi2delay_sfc_6p_6p_T
   *localP)
{
  /* SystemInitialize for Fcn: '<S52>/Fcn' incorporates:
   *  Outport: '<S52>/dq'
   */
  localB->Fcn = localP->dq_Y0[0];

  /* SystemInitialize for Fcn: '<S52>/Fcn1' incorporates:
   *  Outport: '<S52>/dq'
   */
  localB->Fcn1 = localP->dq_Y0[1];
}

/*
 * Disable for enable system:
 *    '<S48>/Subsystem - pi//2 delay'
 *    '<S59>/Subsystem - pi//2 delay'
 */
void sfc_6p_6p_topology_machine::sfc_6_Subsystempi2delay_Disable
  (DW_Subsystempi2delay_sfc_6p_6_T *localDW)
{
  localDW->Subsystempi2delay_MODE = false;
}

/*
 * Output and update for enable system:
 *    '<S48>/Subsystem - pi//2 delay'
 *    '<S59>/Subsystem - pi//2 delay'
 */
void sfc_6p_6p_topology_machine::sfc_6p_6p_top_Subsystempi2delay(uint8_T
  rtu_Enable, const real_T rtu_alpha_beta[2], real_T rtu_wt,
  B_Subsystempi2delay_sfc_6p_6p_T *localB, DW_Subsystempi2delay_sfc_6p_6_T
  *localDW)
{
  real_T Fcn_tmp;
  real_T Fcn_tmp_0;

  /* Outputs for Enabled SubSystem: '<S48>/Subsystem - pi//2 delay' incorporates:
   *  EnablePort: '<S52>/Enable'
   */
  if (rtmIsMajorTimeStep((&sfc_6p_6p_topology_machine_M)) &&
      rtsiIsModeUpdateTimeStep(&(&sfc_6p_6p_topology_machine_M)->solverInfo)) {
    if (rtu_Enable > 0) {
      localDW->Subsystempi2delay_MODE = true;
    } else if (localDW->Subsystempi2delay_MODE) {
      sfc_6_Subsystempi2delay_Disable(localDW);
    }
  }

  if (localDW->Subsystempi2delay_MODE) {
    /* Fcn: '<S52>/Fcn' incorporates:
     *  Fcn: '<S52>/Fcn1'
     */
    Fcn_tmp = std::cos(rtu_wt);
    Fcn_tmp_0 = std::sin(rtu_wt);

    /* Fcn: '<S52>/Fcn' */
    localB->Fcn = rtu_alpha_beta[0] * Fcn_tmp_0 - rtu_alpha_beta[1] * Fcn_tmp;

    /* Fcn: '<S52>/Fcn1' */
    localB->Fcn1 = rtu_alpha_beta[0] * Fcn_tmp + rtu_alpha_beta[1] * Fcn_tmp_0;
  }

  /* End of Outputs for SubSystem: '<S48>/Subsystem - pi//2 delay' */
}

/*
 * System initialize for enable system:
 *    '<S48>/Subsystem1'
 *    '<S59>/Subsystem1'
 */
void sfc_6p_6p_topology_machine::sfc_6p_6p_topol_Subsystem1_Init
  (B_Subsystem1_sfc_6p_6p_topolo_T *localB, P_Subsystem1_sfc_6p_6p_topolo_T
   *localP)
{
  /* SystemInitialize for Fcn: '<S53>/Fcn' incorporates:
   *  Outport: '<S53>/dq'
   */
  localB->Fcn = localP->dq_Y0[0];

  /* SystemInitialize for Fcn: '<S53>/Fcn1' incorporates:
   *  Outport: '<S53>/dq'
   */
  localB->Fcn1 = localP->dq_Y0[1];
}

/*
 * Disable for enable system:
 *    '<S48>/Subsystem1'
 *    '<S59>/Subsystem1'
 */
void sfc_6p_6p_topology_machine::sfc_6p_6p_to_Subsystem1_Disable
  (DW_Subsystem1_sfc_6p_6p_topol_T *localDW)
{
  localDW->Subsystem1_MODE = false;
}

/*
 * Output and update for enable system:
 *    '<S48>/Subsystem1'
 *    '<S59>/Subsystem1'
 */
void sfc_6p_6p_topology_machine::sfc_6p_6p_topology_m_Subsystem1(uint8_T
  rtu_Enable, const real_T rtu_alpha_beta[2], real_T rtu_wt,
  B_Subsystem1_sfc_6p_6p_topolo_T *localB, DW_Subsystem1_sfc_6p_6p_topol_T
  *localDW)
{
  real_T Fcn_tmp;
  real_T Fcn_tmp_0;

  /* Outputs for Enabled SubSystem: '<S48>/Subsystem1' incorporates:
   *  EnablePort: '<S53>/Enable'
   */
  if (rtmIsMajorTimeStep((&sfc_6p_6p_topology_machine_M)) &&
      rtsiIsModeUpdateTimeStep(&(&sfc_6p_6p_topology_machine_M)->solverInfo)) {
    if (rtu_Enable > 0) {
      localDW->Subsystem1_MODE = true;
    } else if (localDW->Subsystem1_MODE) {
      sfc_6p_6p_to_Subsystem1_Disable(localDW);
    }
  }

  if (localDW->Subsystem1_MODE) {
    /* Fcn: '<S53>/Fcn' incorporates:
     *  Fcn: '<S53>/Fcn1'
     */
    Fcn_tmp = std::sin(rtu_wt);
    Fcn_tmp_0 = std::cos(rtu_wt);

    /* Fcn: '<S53>/Fcn' */
    localB->Fcn = rtu_alpha_beta[0] * Fcn_tmp_0 + rtu_alpha_beta[1] * Fcn_tmp;

    /* Fcn: '<S53>/Fcn1' */
    localB->Fcn1 = -rtu_alpha_beta[0] * Fcn_tmp + rtu_alpha_beta[1] * Fcn_tmp_0;
  }

  /* End of Outputs for SubSystem: '<S48>/Subsystem1' */
}

void rt_invd6x6_snf(const real_T u[36], real_T y[36])
{
  real_T A[36];
  real_T smax;
  real_T x;
  int32_T Aoffset;
  int32_T ONE;
  int32_T ipk;
  int32_T ix;
  int32_T jm1;
  int32_T jpiv_offset;
  int32_T mmj;
  int32_T pipk;
  int8_T ipiv[6];
  int8_T p[6];
  for (ONE = 0; ONE < 36; ONE++) {
    A[ONE] = u[ONE];
    y[ONE] = 0.0;
  }

  for (ONE = 0; ONE < 6; ONE++) {
    ipiv[ONE] = static_cast<int8_T>(ONE + 1);
  }

  for (ipk = 0; ipk < 5; ipk++) {
    real_T s;
    int32_T jj;
    int32_T jp1j;
    pipk = ipk + 1;
    jm1 = pipk - 1;
    mmj = 6 - pipk;
    Aoffset = jm1 * 7;
    ONE = 1;
    jj = Aoffset + 1;
    jp1j = jj + 1;
    Aoffset = mmj + 1;
    ix = jj - 1;
    x = A[jj - 1];
    s = std::abs(x);
    smax = s;
    for (jpiv_offset = 2; jpiv_offset <= Aoffset; jpiv_offset++) {
      ix++;
      x = A[ix];
      s = std::abs(x);
      if (s > smax) {
        ONE = jpiv_offset;
        smax = s;
      }
    }

    jpiv_offset = ONE - 1;
    ONE = (jj + jpiv_offset) - 1;
    if (A[ONE] != 0.0) {
      if (jpiv_offset != 0) {
        Aoffset = pipk + jpiv_offset;
        ipiv[pipk - 1] = static_cast<int8_T>(Aoffset);
        ONE = jm1 + jpiv_offset;
        for (jpiv_offset = 0; jpiv_offset < 6; jpiv_offset++) {
          s = A[jm1];
          A[jm1] = A[ONE];
          A[ONE] = s;
          jm1 += 6;
          ONE += 6;
        }
      }

      Aoffset = mmj - 1;
      jm1 = jp1j + Aoffset;
      for (int32_T b = jp1j; b <= jm1; b++) {
        x = A[b - 1];
        s = A[jj - 1];
        smax = x / s;
        A[b - 1] = smax;
      }
    }

    Aoffset = 5 - pipk;
    jpiv_offset = jj + 5;
    pipk = jj + 7;
    ONE = pipk - 1;
    for (pipk = 0; pipk <= Aoffset; pipk++) {
      s = A[jpiv_offset];
      if (s != 0.0) {
        s = -s;
        ix = jp1j - 1;
        jm1 = ONE + 1;
        jj = mmj + ONE;
        for (int32_T b = jm1; b <= jj; b++) {
          A[b - 1] += A[ix] * s;
          ix++;
        }
      }

      jpiv_offset += 6;
      ONE += 6;
    }
  }

  for (ONE = 0; ONE < 6; ONE++) {
    p[ONE] = static_cast<int8_T>(ONE + 1);
  }

  for (jpiv_offset = 0; jpiv_offset < 5; jpiv_offset++) {
    smax = static_cast<real_T>(jpiv_offset) + 1.0;
    pipk = static_cast<int32_T>(smax);
    ONE = pipk - 1;
    ipk = ipiv[ONE] - 1;
    if (ipk + 1 > pipk) {
      pipk = p[ipk];
      p[ipk] = p[ONE];
      p[ONE] = static_cast<int8_T>(pipk);
    }
  }

  for (jpiv_offset = 0; jpiv_offset < 6; jpiv_offset++) {
    pipk = jpiv_offset;
    Aoffset = p[pipk] - 1;
    y[pipk + 6 * Aoffset] = 1.0;
    for (ipk = pipk + 1; ipk < 7; ipk++) {
      ONE = (6 * Aoffset + ipk) - 1;
      if (y[ONE] != 0.0) {
        jm1 = ipk + 1;
        for (int32_T b = jm1; b < 7; b++) {
          mmj = (6 * Aoffset + b) - 1;
          y[mmj] -= A[((ipk - 1) * 6 + b) - 1] * y[ONE];
        }
      }
    }
  }

  for (ipk = 0; ipk < 6; ipk++) {
    pipk = ipk;
    pipk *= 6;
    for (jpiv_offset = 5; jpiv_offset >= 0; jpiv_offset--) {
      ix = 6 * jpiv_offset;
      ONE = jpiv_offset + pipk;
      x = y[ONE];
      if (x != 0.0) {
        y[ONE] = x / A[jpiv_offset + ix];
        jm1 = jpiv_offset - 1;
        for (int32_T b = 0; b <= jm1; b++) {
          mmj = b + pipk;
          y[mmj] -= A[b + ix] * y[ONE];
        }
      }
    }
  }
}

real_T rt_modd_snf(real_T u0, real_T u1)
{
  real_T y;
  y = u0;
  if (u1 == 0.0) {
    if (u0 == 0.0) {
      y = u1;
    }
  } else if (rtIsNaN(u0) || rtIsNaN(u1) || rtIsInf(u0)) {
    y = (rtNaN);
  } else if (u0 == 0.0) {
    y = 0.0 / u1;
  } else if (rtIsInf(u1)) {
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

real_T rt_hypotd_snf(real_T u0, real_T u1)
{
  real_T a;
  real_T b;
  real_T y;
  a = std::abs(u0);
  b = std::abs(u1);
  if (a < b) {
    a /= b;
    y = std::sqrt(a * a + 1.0) * b;
  } else if (a > b) {
    b /= a;
    y = std::sqrt(b * b + 1.0) * a;
  } else if (rtIsNaN(b)) {
    y = (rtNaN);
  } else {
    y = a * 1.4142135623730951;
  }

  return y;
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    int32_T tmp;
    int32_T tmp_0;
    if (u1 > 0.0) {
      tmp = 1;
    } else {
      tmp = -1;
    }

    if (u0 > 0.0) {
      tmp_0 = 1;
    } else {
      tmp_0 = -1;
    }

    y = std::atan2(static_cast<real_T>(tmp_0), static_cast<real_T>(tmp));
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

real_T rt_remd_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1) || rtIsInf(u0)) {
    y = (rtNaN);
  } else if (rtIsInf(u1)) {
    y = u0;
  } else {
    if (u1 < 0.0) {
      y = std::ceil(u1);
    } else {
      y = std::floor(u1);
    }

    if ((u1 != 0.0) && (u1 != y)) {
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
  }

  return y;
}

/* Model step function */
void sfc_6p_6p_topology_machine::step()
{
  if (rtmIsMajorTimeStep((&sfc_6p_6p_topology_machine_M))) {
    /* set solver stop time */
    if (!((&sfc_6p_6p_topology_machine_M)->Timing.clockTick0+1)) {
      rtsiSetSolverStopTime(&(&sfc_6p_6p_topology_machine_M)->solverInfo,
                            (((&sfc_6p_6p_topology_machine_M)
        ->Timing.clockTickH0 + 1) * (&sfc_6p_6p_topology_machine_M)
        ->Timing.stepSize0 * 4294967296.0));
    } else {
      rtsiSetSolverStopTime(&(&sfc_6p_6p_topology_machine_M)->solverInfo,
                            (((&sfc_6p_6p_topology_machine_M)->Timing.clockTick0
        + 1) * (&sfc_6p_6p_topology_machine_M)->Timing.stepSize0 +
        (&sfc_6p_6p_topology_machine_M)->Timing.clockTickH0 *
        (&sfc_6p_6p_topology_machine_M)->Timing.stepSize0 * 4294967296.0));
    }
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep((&sfc_6p_6p_topology_machine_M))) {
    (&sfc_6p_6p_topology_machine_M)->Timing.t[0] = rtsiGetT
      (&(&sfc_6p_6p_topology_machine_M)->solverInfo);
  }

  {
    real_T tmp_0[36];
    real_T tmp_1[36];
    real_T tmp[6];
    real_T tmpForInput_idx_0;
    real_T tmpForInput_idx_0_0;
    real_T tmpForInput_idx_1;
    real_T tmpForInput_idx_2;
    real_T tmpForInput_idx_3;
    int32_T i;
    int32_T i_0;
    int32_T i_1;
    int8_T tmp_2;
    boolean_T didZcEventOccur;
    static const int8_T tmp_3[3] = { 0, 4, 5 };

    if (rtmIsMajorTimeStep((&sfc_6p_6p_topology_machine_M))) {
      for (i = 0; i < 6; i++) {
        /* UnitDelay: '<S80>/fluxes' */
        sfc_6p_6p_topology_machine_B.xk1[i] =
          sfc_6p_6p_topology_machine_DW.fluxes_DSTATE[i];
      }

      /* Outputs for Enabled SubSystem: '<S69>/Saturation' incorporates:
       *  EnablePort: '<S79>/Enable'
       */
      /* Constant: '<S69>/Constant1' */
      if (sfc_6p_6p_topology_machine_P.Constant1_Value_l > 0.0) {
        /* Gain: '<S85>/1//Ll_d' */
        sfc_6p_6p_topology_machine_B.uLl_d_h[0] =
          sfc_6p_6p_topology_machine_P.uLl_d_Gain[0] *
          sfc_6p_6p_topology_machine_B.xk1[1];
        sfc_6p_6p_topology_machine_B.uLl_d_h[1] =
          sfc_6p_6p_topology_machine_P.uLl_d_Gain[1] *
          sfc_6p_6p_topology_machine_B.xk1[2];
        sfc_6p_6p_topology_machine_B.uLl_d_h[2] =
          sfc_6p_6p_topology_machine_P.uLl_d_Gain[2] *
          sfc_6p_6p_topology_machine_B.xk1[3];

        /* Sum: '<S85>/sum2' */
        tmpForInput_idx_0_0 = sfc_6p_6p_topology_machine_B.uLl_d_h[0];
        tmpForInput_idx_0_0 += sfc_6p_6p_topology_machine_B.uLl_d_h[1];
        tmpForInput_idx_0_0 += sfc_6p_6p_topology_machine_B.uLl_d_h[2];

        /* Sum: '<S85>/sum2' */
        sfc_6p_6p_topology_machine_B.phimd_g = tmpForInput_idx_0_0;

        /* UnitDelay: '<S81>/Lmd_sat' */
        sfc_6p_6p_topology_machine_B.Lmd_sat =
          sfc_6p_6p_topology_machine_DW.Lmd_sat_DSTATE;

        /* Math: '<S84>/Math Function2'
         *
         * About '<S84>/Math Function2':
         *  Operator: reciprocal
         */
        tmpForInput_idx_0_0 = sfc_6p_6p_topology_machine_B.Lmd_sat;

        /* Math: '<S84>/Math Function2'
         *
         * About '<S84>/Math Function2':
         *  Operator: reciprocal
         */
        sfc_6p_6p_topology_machine_B.MathFunction2 = 1.0 / tmpForInput_idx_0_0;

        /* Sum: '<S84>/Sum1' incorporates:
         *  Constant: '<S84>/u1'
         */
        tmpForInput_idx_0_0 = sfc_6p_6p_topology_machine_P.u1_Value[0];
        tmpForInput_idx_1 = sfc_6p_6p_topology_machine_P.u1_Value[1];
        tmpForInput_idx_2 = sfc_6p_6p_topology_machine_B.MathFunction2;
        tmpForInput_idx_0_0 += tmpForInput_idx_1;
        tmpForInput_idx_0_0 += tmpForInput_idx_2;

        /* Sum: '<S84>/Sum1' */
        sfc_6p_6p_topology_machine_B.Sum1_bq = tmpForInput_idx_0_0;

        /* Math: '<S84>/Math Function3'
         *
         * About '<S84>/Math Function3':
         *  Operator: reciprocal
         */
        tmpForInput_idx_0_0 = sfc_6p_6p_topology_machine_B.Sum1_bq;

        /* Math: '<S84>/Math Function3'
         *
         * About '<S84>/Math Function3':
         *  Operator: reciprocal
         */
        sfc_6p_6p_topology_machine_B.MathFunction3 = 1.0 / tmpForInput_idx_0_0;

        /* Product: '<S85>/Product4' */
        sfc_6p_6p_topology_machine_B.phimd_n =
          sfc_6p_6p_topology_machine_B.phimd_g *
          sfc_6p_6p_topology_machine_B.MathFunction3;

        /* Abs: '<S81>/Abs' */
        sfc_6p_6p_topology_machine_B.Phisat = std::abs
          (sfc_6p_6p_topology_machine_B.phimd_n);

        /* Lookup_n-D: '<S81>/1-D Lookup Table' incorporates:
         *  Abs: '<S81>/Abs'
         */
        sfc_6p_6p_topology_machine_B.Ifdsat = look1_pbinlxpw
          (sfc_6p_6p_topology_machine_B.Phisat,
           sfc_6p_6p_topology_machine_P.uDLookupTable_bp01Data_i,
           sfc_6p_6p_topology_machine_P.uDLookupTable_tableData_a,
           &sfc_6p_6p_topology_machine_DW.m_bpIndex, 1U);

        /* Switch: '<S81>/Switch' */
        if (sfc_6p_6p_topology_machine_B.Ifdsat != 0.0) {
          /* Product: '<S81>/Product' */
          sfc_6p_6p_topology_machine_B.Product_n =
            sfc_6p_6p_topology_machine_B.Phisat /
            sfc_6p_6p_topology_machine_B.Ifdsat;

          /* Switch: '<S81>/Switch' */
          sfc_6p_6p_topology_machine_B.Switch_d =
            sfc_6p_6p_topology_machine_B.Product_n;
        } else {
          /* Switch: '<S81>/Switch' incorporates:
           *  Constant: '<S81>/Constant1'
           */
          sfc_6p_6p_topology_machine_B.Switch_d =
            sfc_6p_6p_topology_machine_P.Constant1_Value_c;
        }

        /* End of Switch: '<S81>/Switch' */

        /* Gain: '<S81>/Lmd' */
        sfc_6p_6p_topology_machine_B.Lmsatd =
          sfc_6p_6p_topology_machine_P.Lmd_Gain *
          sfc_6p_6p_topology_machine_B.Switch_d;

        /* Outputs for Enabled SubSystem: '<S79>/Lmq_sat' incorporates:
         *  EnablePort: '<S82>/Enable'
         */
        /* Constant: '<S79>/Constant1' */
        if (sfc_6p_6p_topology_machine_P.Constant1_Value_n) {
          /* Gain: '<S87>/1//Ll_q' */
          sfc_6p_6p_topology_machine_B.uLl_q_i[0] =
            sfc_6p_6p_topology_machine_P.uLl_q_Gain[0] *
            sfc_6p_6p_topology_machine_B.xk1[0];
          sfc_6p_6p_topology_machine_B.uLl_q_i[1] =
            sfc_6p_6p_topology_machine_P.uLl_q_Gain[1] *
            sfc_6p_6p_topology_machine_B.xk1[4];
          sfc_6p_6p_topology_machine_B.uLl_q_i[2] =
            sfc_6p_6p_topology_machine_P.uLl_q_Gain[2] *
            sfc_6p_6p_topology_machine_B.xk1[5];

          /* UnitDelay: '<S82>/Lmq_sat' */
          sfc_6p_6p_topology_machine_B.Lmq_sat =
            sfc_6p_6p_topology_machine_DW.Lmq_sat_DSTATE;

          /* Sum: '<S87>/sum1' */
          tmpForInput_idx_0_0 = sfc_6p_6p_topology_machine_B.uLl_q_i[0];

          /* Sum: '<S86>/Sum2' incorporates:
           *  Constant: '<S86>/u2'
           */
          tmpForInput_idx_0 = sfc_6p_6p_topology_machine_P.u2_Value[0];

          /* Sum: '<S87>/sum1' */
          tmpForInput_idx_0_0 += sfc_6p_6p_topology_machine_B.uLl_q_i[1];

          /* Sum: '<S86>/Sum2' incorporates:
           *  Constant: '<S86>/u2'
           */
          tmpForInput_idx_1 = sfc_6p_6p_topology_machine_P.u2_Value[1];

          /* Sum: '<S87>/sum1' */
          tmpForInput_idx_0_0 += sfc_6p_6p_topology_machine_B.uLl_q_i[2];

          /* Sum: '<S86>/Sum2' incorporates:
           *  Constant: '<S86>/u2'
           */
          tmpForInput_idx_2 = sfc_6p_6p_topology_machine_P.u2_Value[2];

          /* Sum: '<S87>/sum1' */
          sfc_6p_6p_topology_machine_B.phimq_p = tmpForInput_idx_0_0;

          /* Math: '<S86>/Math Function'
           *
           * About '<S86>/Math Function':
           *  Operator: reciprocal
           */
          tmpForInput_idx_0_0 = sfc_6p_6p_topology_machine_B.Lmq_sat;

          /* Math: '<S86>/Math Function'
           *
           * About '<S86>/Math Function':
           *  Operator: reciprocal
           */
          sfc_6p_6p_topology_machine_B.MathFunction = 1.0 / tmpForInput_idx_0_0;

          /* Sum: '<S86>/Sum2' */
          tmpForInput_idx_3 = sfc_6p_6p_topology_machine_B.MathFunction;
          tmpForInput_idx_0_0 = tmpForInput_idx_0;
          tmpForInput_idx_0_0 += tmpForInput_idx_1;
          tmpForInput_idx_0_0 += tmpForInput_idx_2;
          tmpForInput_idx_0_0 += tmpForInput_idx_3;

          /* Sum: '<S86>/Sum2' */
          sfc_6p_6p_topology_machine_B.Sum2_d = tmpForInput_idx_0_0;

          /* Math: '<S86>/Math Function1'
           *
           * About '<S86>/Math Function1':
           *  Operator: reciprocal
           */
          tmpForInput_idx_0_0 = sfc_6p_6p_topology_machine_B.Sum2_d;

          /* Math: '<S86>/Math Function1'
           *
           * About '<S86>/Math Function1':
           *  Operator: reciprocal
           */
          sfc_6p_6p_topology_machine_B.MathFunction1 = 1.0 / tmpForInput_idx_0_0;

          /* Product: '<S87>/Product3' */
          sfc_6p_6p_topology_machine_B.phimq_i =
            sfc_6p_6p_topology_machine_B.phimq_p *
            sfc_6p_6p_topology_machine_B.MathFunction1;

          /* Abs: '<S82>/Abs' */
          sfc_6p_6p_topology_machine_B.Phisat_a = std::abs
            (sfc_6p_6p_topology_machine_B.phimq_i);

          /* Lookup_n-D: '<S82>/1-D Lookup Table' incorporates:
           *  Abs: '<S82>/Abs'
           */
          sfc_6p_6p_topology_machine_B.Ifqsat = look1_pbinlxpw
            (sfc_6p_6p_topology_machine_B.Phisat_a,
             sfc_6p_6p_topology_machine_P.uDLookupTable_bp01Data,
             sfc_6p_6p_topology_machine_P.uDLookupTable_tableData,
             &sfc_6p_6p_topology_machine_DW.m_bpIndex_i, 1U);

          /* Switch: '<S82>/Switch' */
          if (sfc_6p_6p_topology_machine_B.Ifqsat != 0.0) {
            /* Product: '<S82>/Product' */
            sfc_6p_6p_topology_machine_B.Product_i =
              sfc_6p_6p_topology_machine_B.Phisat_a /
              sfc_6p_6p_topology_machine_B.Ifqsat;

            /* Switch: '<S82>/Switch' */
            sfc_6p_6p_topology_machine_B.Switch_b =
              sfc_6p_6p_topology_machine_B.Product_i;
          } else {
            /* Switch: '<S82>/Switch' incorporates:
             *  Constant: '<S82>/Constant1'
             */
            sfc_6p_6p_topology_machine_B.Switch_b =
              sfc_6p_6p_topology_machine_P.Constant1_Value_i;
          }

          /* End of Switch: '<S82>/Switch' */

          /* Gain: '<S82>/Lmq' */
          sfc_6p_6p_topology_machine_B.Lmsatq =
            sfc_6p_6p_topology_machine_P.Lmq_Gain *
            sfc_6p_6p_topology_machine_B.Switch_b;

          /* Update for UnitDelay: '<S82>/Lmq_sat' */
          sfc_6p_6p_topology_machine_DW.Lmq_sat_DSTATE =
            sfc_6p_6p_topology_machine_B.Lmsatq;
        }

        /* End of Constant: '<S79>/Constant1' */
        /* End of Outputs for SubSystem: '<S79>/Lmq_sat' */

        /* Switch: '<S79>/Switch1' incorporates:
         *  Constant: '<S79>/Constant2'
         */
        if (sfc_6p_6p_topology_machine_P.Constant2_Value_io) {
          /* Switch: '<S79>/Switch1' */
          sfc_6p_6p_topology_machine_B.Switch1_j =
            sfc_6p_6p_topology_machine_B.Lmsatq;
        } else {
          /* Switch: '<S79>/Switch1' incorporates:
           *  Constant: '<S79>/u3'
           */
          sfc_6p_6p_topology_machine_B.Switch1_j =
            sfc_6p_6p_topology_machine_P.u3_Value;
        }

        /* End of Switch: '<S79>/Switch1' */

        /* Assignment: '<S83>/Update Lmq rows[1,5, (6)] col[1,5, (6)] ' incorporates:
         *  Constant: '<S83>/u1'
         */
        std::memcpy(&sfc_6p_6p_topology_machine_B.UpdateLmqrows156col156[0],
                    &sfc_6p_6p_topology_machine_P.u1_Value_l[0], 36U * sizeof
                    (real_T));

        /* Assignment: '<S83>/Update Lmq rows[1,5, (6)] col[1,5, (6)] ' */
        for (i_0 = 0; i_0 < 3; i_0++) {
          tmp_2 = tmp_3[i_0];
          i = 6 * tmp_2;
          sfc_6p_6p_topology_machine_B.UpdateLmqrows156col156[i] =
            sfc_6p_6p_topology_machine_B.Switch1_j;
          sfc_6p_6p_topology_machine_B.UpdateLmqrows156col156[i + 4] =
            sfc_6p_6p_topology_machine_B.Switch1_j;
          sfc_6p_6p_topology_machine_B.UpdateLmqrows156col156[i + 5] =
            sfc_6p_6p_topology_machine_B.Switch1_j;
        }

        /* Assignment: '<S83>/Update Lmd rows[2,3,4] col[2,3,4]' incorporates:
         *  Assignment: '<S83>/Update Lmq rows[1,5, (6)] col[1,5, (6)] '
         */
        std::memcpy(&sfc_6p_6p_topology_machine_B.UpdateLmdrows234col234[0],
                    &sfc_6p_6p_topology_machine_B.UpdateLmqrows156col156[0], 36U
                    * sizeof(real_T));

        /* Assignment: '<S83>/Update Lmd rows[2,3,4] col[2,3,4]' */
        for (i_0 = 0; i_0 < 3; i_0++) {
          i = (i_0 + 1) * 6;
          sfc_6p_6p_topology_machine_B.UpdateLmdrows234col234[i + 1] =
            sfc_6p_6p_topology_machine_B.Lmsatd;
          sfc_6p_6p_topology_machine_B.UpdateLmdrows234col234[i + 2] =
            sfc_6p_6p_topology_machine_B.Lmsatd;
          sfc_6p_6p_topology_machine_B.UpdateLmdrows234col234[i + 3] =
            sfc_6p_6p_topology_machine_B.Lmsatd;
        }

        for (i = 0; i < 36; i++) {
          /* Sum: '<S83>/Sum2' incorporates:
           *  Assignment: '<S83>/Update Lmd rows[2,3,4] col[2,3,4]'
           *  Constant: '<S83>/u5'
           */
          sfc_6p_6p_topology_machine_B.Sum2_h[i] =
            sfc_6p_6p_topology_machine_B.UpdateLmdrows234col234[i] +
            sfc_6p_6p_topology_machine_P.u5_Value[i];
        }

        /* Product: '<S79>/inversion' incorporates:
         *  Sum: '<S83>/Sum2'
         */
        rt_invd6x6_snf(sfc_6p_6p_topology_machine_B.Sum2_h,
                       sfc_6p_6p_topology_machine_B.Linv_d);

        /* Product: '<S79>/Product1' incorporates:
         *  Constant: '<S79>/u1'
         *  Product: '<S79>/inversion'
         */
        std::memcpy(&tmp_0[0], &sfc_6p_6p_topology_machine_P.u1_Value_a[0], 36U *
                    sizeof(real_T));
        std::memcpy(&tmp_1[0], &sfc_6p_6p_topology_machine_B.Linv_d[0], 36U *
                    sizeof(real_T));
        for (i_0 = 0; i_0 < 6; i_0++) {
          for (i = 0; i < 6; i++) {
            /* Product: '<S79>/Product1' */
            tmpForInput_idx_0_0 = 0.0;
            for (i_1 = 0; i_1 < 6; i_1++) {
              /* Product: '<S79>/Product1' */
              tmpForInput_idx_0_0 += tmp_0[6 * i_1 + i] * tmp_1[6 * i_0 + i_1];
            }

            /* Product: '<S79>/Product1' */
            sfc_6p_6p_topology_machine_B.RLinv[i + 6 * i_0] =
              tmpForInput_idx_0_0;
          }
        }

        /* Switch: '<S79>/Switch2' incorporates:
         *  Constant: '<S79>/Constant3'
         */
        if (sfc_6p_6p_topology_machine_P.Constant3_Value_p) {
          /* Switch: '<S79>/Switch2' */
          sfc_6p_6p_topology_machine_B.Switch2_a =
            sfc_6p_6p_topology_machine_B.MathFunction1;
        } else {
          /* Switch: '<S79>/Switch2' incorporates:
           *  Constant: '<S79>/u2'
           */
          sfc_6p_6p_topology_machine_B.Switch2_a =
            sfc_6p_6p_topology_machine_P.u2_Value_n;
        }

        /* End of Switch: '<S79>/Switch2' */

        /* Update for UnitDelay: '<S81>/Lmd_sat' */
        sfc_6p_6p_topology_machine_DW.Lmd_sat_DSTATE =
          sfc_6p_6p_topology_machine_B.Lmsatd;
      }

      /* End of Constant: '<S69>/Constant1' */
      /* End of Outputs for SubSystem: '<S69>/Saturation' */
      for (i = 0; i < 36; i++) {
        /* Switch: '<S69>/Switch' incorporates:
         *  Constant: '<S69>/Constant3'
         */
        if (sfc_6p_6p_topology_machine_P.Constant3_Value >=
            sfc_6p_6p_topology_machine_P.Switch_Threshold) {
          /* Switch: '<S69>/Switch' incorporates:
           *  Product: '<S79>/inversion'
           */
          tmpForInput_idx_0_0 = sfc_6p_6p_topology_machine_B.Linv_d[i];
        } else {
          /* Switch: '<S69>/Switch' incorporates:
           *  Constant: '<S69>/Constant4'
           */
          tmpForInput_idx_0_0 = sfc_6p_6p_topology_machine_P.Constant4_Value[i];
        }

        /* End of Switch: '<S69>/Switch' */

        /* Switch: '<S69>/Switch' */
        sfc_6p_6p_topology_machine_B.Linv[i] = tmpForInput_idx_0_0;

        /* Product: '<S69>/Product3' incorporates:
         *  Switch: '<S69>/Switch'
         */
        tmp_0[i] = tmpForInput_idx_0_0;
      }

      /* Product: '<S69>/Product3' */
      for (i = 0; i < 6; i++) {
        tmp[i] = sfc_6p_6p_topology_machine_B.xk1[i];
      }

      for (i = 0; i < 6; i++) {
        tmpForInput_idx_0_0 = 0.0;
        for (i_0 = 0; i_0 < 6; i_0++) {
          tmpForInput_idx_0_0 += tmp_0[6 * i_0 + i] * tmp[i_0];
        }

        /* Product: '<S69>/Product3' */
        sfc_6p_6p_topology_machine_B.Ik[i] = tmpForInput_idx_0_0;

        /* Gain: '<S69>/change Iq Id  current signs' */
        sfc_6p_6p_topology_machine_B.changeIqIdcurrentsigns[i] =
          sfc_6p_6p_topology_machine_P.changeIqIdcurrentsigns_Gain[i] *
          tmpForInput_idx_0_0;
      }

      /* DiscreteIntegrator: '<S67>/Rotor angle dthetae' */
      sfc_6p_6p_topology_machine_B.d_thetarad =
        sfc_6p_6p_topology_machine_DW.Rotorangledthetae_DSTATE;

      /* DigitalClock: '<S67>/Digital Clock' */
      sfc_6p_6p_topology_machine_B.DigitalClock =
        ((((&sfc_6p_6p_topology_machine_M)->Timing.clockTick1+
           (&sfc_6p_6p_topology_machine_M)->Timing.clockTickH1* 4294967296.0)) *
         1.25E-5);

      /* Gain: '<S67>/web2' */
      sfc_6p_6p_topology_machine_B.web2 = sfc_6p_6p_topology_machine_P.web2_Gain
        * sfc_6p_6p_topology_machine_B.DigitalClock;

      /* Sum: '<S67>/Sum3' */
      sfc_6p_6p_topology_machine_B.Sum3 =
        sfc_6p_6p_topology_machine_B.d_thetarad +
        sfc_6p_6p_topology_machine_B.web2;

      /* RelationalOperator: '<S99>/Compare' incorporates:
       *  Constant: '<S98>/Constant'
       *  Constant: '<S99>/Constant'
       */
      sfc_6p_6p_topology_machine_B.Compare =
        (sfc_6p_6p_topology_machine_P.dq0toAlphaBetaZero_Alignment ==
         sfc_6p_6p_topology_machine_P.CompareToConstant_const_b);

      /* Outputs for Enabled SubSystem: '<S98>/Subsystem1' incorporates:
       *  EnablePort: '<S102>/Enable'
       */
      if (sfc_6p_6p_topology_machine_B.Compare > 0) {
        /* Fcn: '<S102>/Fcn' */
        sfc_6p_6p_topology_machine_B.Fcn_m =
          sfc_6p_6p_topology_machine_B.changeIqIdcurrentsigns[1] * std::cos
          (sfc_6p_6p_topology_machine_B.Sum3) -
          sfc_6p_6p_topology_machine_B.changeIqIdcurrentsigns[0] * std::sin
          (sfc_6p_6p_topology_machine_B.Sum3);

        /* Fcn: '<S102>/Fcn1' */
        sfc_6p_6p_topology_machine_B.Fcn1 =
          sfc_6p_6p_topology_machine_B.changeIqIdcurrentsigns[1] * std::sin
          (sfc_6p_6p_topology_machine_B.Sum3) +
          sfc_6p_6p_topology_machine_B.changeIqIdcurrentsigns[0] * std::cos
          (sfc_6p_6p_topology_machine_B.Sum3);
      }

      /* End of Outputs for SubSystem: '<S98>/Subsystem1' */

      /* RelationalOperator: '<S100>/Compare' incorporates:
       *  Constant: '<S100>/Constant'
       *  Constant: '<S98>/Constant'
       */
      sfc_6p_6p_topology_machine_B.Compare_n =
        (sfc_6p_6p_topology_machine_P.dq0toAlphaBetaZero_Alignment ==
         sfc_6p_6p_topology_machine_P.CompareToConstant1_const_d);

      /* Outputs for Enabled SubSystem: '<S98>/Subsystem - pi//2 delay' incorporates:
       *  EnablePort: '<S101>/Enable'
       */
      if (sfc_6p_6p_topology_machine_B.Compare_n > 0) {
        /* Fcn: '<S101>/Fcn' */
        sfc_6p_6p_topology_machine_B.Fcn_p =
          sfc_6p_6p_topology_machine_B.changeIqIdcurrentsigns[1] * std::sin
          (sfc_6p_6p_topology_machine_B.Sum3) +
          sfc_6p_6p_topology_machine_B.changeIqIdcurrentsigns[0] * std::cos
          (sfc_6p_6p_topology_machine_B.Sum3);

        /* Fcn: '<S101>/Fcn1' */
        sfc_6p_6p_topology_machine_B.Fcn1_h =
          -sfc_6p_6p_topology_machine_B.changeIqIdcurrentsigns[1] * std::cos
          (sfc_6p_6p_topology_machine_B.Sum3) +
          sfc_6p_6p_topology_machine_B.changeIqIdcurrentsigns[0] * std::sin
          (sfc_6p_6p_topology_machine_B.Sum3);
      }

      /* End of Outputs for SubSystem: '<S98>/Subsystem - pi//2 delay' */

      /* Switch: '<S98>/Switch' */
      if (sfc_6p_6p_topology_machine_B.Compare != 0) {
        /* Switch: '<S98>/Switch' */
        sfc_6p_6p_topology_machine_B.Switch[0] =
          sfc_6p_6p_topology_machine_B.Fcn_m;
        sfc_6p_6p_topology_machine_B.Switch[1] =
          sfc_6p_6p_topology_machine_B.Fcn1;
      } else {
        /* Switch: '<S98>/Switch' */
        sfc_6p_6p_topology_machine_B.Switch[0] =
          sfc_6p_6p_topology_machine_B.Fcn_p;
        sfc_6p_6p_topology_machine_B.Switch[1] =
          sfc_6p_6p_topology_machine_B.Fcn1_h;
      }

      /* End of Switch: '<S98>/Switch' */

      /* SignalConversion generated from: '<S97>/Gain3' incorporates:
       *  Constant: '<S71>/Constant'
       */
      sfc_6p_6p_topology_machine_B.TmpSignalConversionAtGain3Inpor[0] =
        sfc_6p_6p_topology_machine_B.Switch[0];
      sfc_6p_6p_topology_machine_B.TmpSignalConversionAtGain3Inpor[1] =
        sfc_6p_6p_topology_machine_B.Switch[1];
      sfc_6p_6p_topology_machine_B.TmpSignalConversionAtGain3Inpor[2] =
        sfc_6p_6p_topology_machine_P.Constant_Value_k;

      /* Gain: '<S97>/Gain3' */
      tmpForInput_idx_0_0 =
        sfc_6p_6p_topology_machine_B.TmpSignalConversionAtGain3Inpor[0];
      tmpForInput_idx_1 =
        sfc_6p_6p_topology_machine_B.TmpSignalConversionAtGain3Inpor[1];
      tmpForInput_idx_2 =
        sfc_6p_6p_topology_machine_B.TmpSignalConversionAtGain3Inpor[2];
      for (i = 0; i < 3; i++) {
        /* Gain: '<S97>/Gain3' */
        tmpForInput_idx_0 = sfc_6p_6p_topology_machine_P.Gain3_Gain_l[i] *
          tmpForInput_idx_0_0;
        tmpForInput_idx_0 += sfc_6p_6p_topology_machine_P.Gain3_Gain_l[i + 3] *
          tmpForInput_idx_1;
        tmpForInput_idx_0 += sfc_6p_6p_topology_machine_P.Gain3_Gain_l[i + 6] *
          tmpForInput_idx_2;
        sfc_6p_6p_topology_machine_B.Gain3[i] = tmpForInput_idx_0;

        /* Gain: '<S71>/ib' */
        sfc_6p_6p_topology_machine_B.iabc[i] =
          sfc_6p_6p_topology_machine_P.ib_Gain * tmpForInput_idx_0;
      }

      /* Constant: '<S2>/Constant4' */
      sfc_6p_6p_topology_machine_B.Constant4 =
        sfc_6p_6p_topology_machine_P.Constant4_Value_d;

      /* DiscreteIntegrator: '<S34>/Integrator' */
      if ((sfc_6p_6p_topology_machine_B.Constant4 != 0.0) ||
          (sfc_6p_6p_topology_machine_DW.Integrator_PrevResetState != 0)) {
        sfc_6p_6p_topology_machine_DW.Integrator_DSTATE =
          sfc_6p_6p_topology_machine_B.Constant4;
        if (sfc_6p_6p_topology_machine_DW.Integrator_DSTATE >
            sfc_6p_6p_topology_machine_P.Integrator_UpperSat) {
          sfc_6p_6p_topology_machine_DW.Integrator_DSTATE =
            sfc_6p_6p_topology_machine_P.Integrator_UpperSat;
        } else if (sfc_6p_6p_topology_machine_DW.Integrator_DSTATE <
                   sfc_6p_6p_topology_machine_P.Integrator_LowerSat) {
          sfc_6p_6p_topology_machine_DW.Integrator_DSTATE =
            sfc_6p_6p_topology_machine_P.Integrator_LowerSat;
        }
      }

      /* DiscreteIntegrator: '<S34>/Integrator' */
      sfc_6p_6p_topology_machine_B.Integrator =
        sfc_6p_6p_topology_machine_DW.Integrator_DSTATE;

      /* Saturate: '<S34>/Saturation' */
      tmpForInput_idx_1 = sfc_6p_6p_topology_machine_B.Integrator;
      tmpForInput_idx_0_0 = sfc_6p_6p_topology_machine_P.Saturation_LowerSat_f;
      tmpForInput_idx_2 = sfc_6p_6p_topology_machine_P.Saturation_UpperSat_e;
      if (tmpForInput_idx_1 > tmpForInput_idx_2) {
        /* Saturate: '<S34>/Saturation' */
        sfc_6p_6p_topology_machine_B.Saturation = tmpForInput_idx_2;
      } else if (tmpForInput_idx_1 < tmpForInput_idx_0_0) {
        /* Saturate: '<S34>/Saturation' */
        sfc_6p_6p_topology_machine_B.Saturation = tmpForInput_idx_0_0;
      } else {
        /* Saturate: '<S34>/Saturation' */
        sfc_6p_6p_topology_machine_B.Saturation = tmpForInput_idx_1;
      }

      /* End of Saturate: '<S34>/Saturation' */

      /* Sum: '<S29>/Sum' incorporates:
       *  Constant: '<S29>/Constant5'
       */
      sfc_6p_6p_topology_machine_B.Sum = sfc_6p_6p_topology_machine_B.Saturation
        + sfc_6p_6p_topology_machine_P.Constant5_Value;

      /* Math: '<S29>/Mod' incorporates:
       *  Constant: '<S29>/Constant6'
       */
      sfc_6p_6p_topology_machine_B.Mod = rt_modd_snf
        (sfc_6p_6p_topology_machine_B.Sum,
         sfc_6p_6p_topology_machine_P.Constant6_Value_k);

      /* Outport: '<Root>/source_voltage_phase' incorporates:
       *  Constant: '<S29>/Constant5'
       *  Sum: '<S29>/Sum1'
       */
      sfc_6p_6p_topology_machine_Y.source_voltage_phase =
        sfc_6p_6p_topology_machine_B.Mod -
        sfc_6p_6p_topology_machine_P.Constant5_Value;

      /* Gain: '<S2>/Gain2' incorporates:
       *  Constant: '<Root>/Constant1'
       *  Constant: '<Root>/Constant2'
       *  Constant: '<Root>/Constant3'
       */
      sfc_6p_6p_topology_machine_B.Gain2[0] =
        sfc_6p_6p_topology_machine_P.Gain2_Gain *
        sfc_6p_6p_topology_machine_P.Constant1_Value_e;
      sfc_6p_6p_topology_machine_B.Gain2[1] =
        sfc_6p_6p_topology_machine_P.Gain2_Gain *
        sfc_6p_6p_topology_machine_P.Constant2_Value_e;
      sfc_6p_6p_topology_machine_B.Gain2[2] =
        sfc_6p_6p_topology_machine_P.Gain2_Gain *
        sfc_6p_6p_topology_machine_P.Constant3_Value_k;

      /* Sum: '<S2>/Add1' incorporates:
       *  Outport: '<Root>/source_voltage_phase'
       */
      sfc_6p_6p_topology_machine_B.Add1[0] =
        sfc_6p_6p_topology_machine_Y.source_voltage_phase +
        sfc_6p_6p_topology_machine_B.Gain2[0];

      /* Trigonometry: '<S2>/Trigonometric Function1' */
      tmpForInput_idx_0_0 = std::sin(sfc_6p_6p_topology_machine_B.Add1[0]);
      sfc_6p_6p_topology_machine_B.TrigonometricFunction1[0] =
        tmpForInput_idx_0_0;

      /* Gain: '<S2>/Gain1' */
      sfc_6p_6p_topology_machine_B.Gain1[0] =
        sfc_6p_6p_topology_machine_P.Gain1_Gain_j * tmpForInput_idx_0_0;

      /* Sum: '<S2>/Add1' incorporates:
       *  Outport: '<Root>/source_voltage_phase'
       */
      sfc_6p_6p_topology_machine_B.Add1[1] =
        sfc_6p_6p_topology_machine_Y.source_voltage_phase +
        sfc_6p_6p_topology_machine_B.Gain2[1];

      /* Trigonometry: '<S2>/Trigonometric Function1' */
      tmpForInput_idx_0_0 = std::sin(sfc_6p_6p_topology_machine_B.Add1[1]);
      sfc_6p_6p_topology_machine_B.TrigonometricFunction1[1] =
        tmpForInput_idx_0_0;

      /* Gain: '<S2>/Gain1' */
      sfc_6p_6p_topology_machine_B.Gain1[1] =
        sfc_6p_6p_topology_machine_P.Gain1_Gain_j * tmpForInput_idx_0_0;

      /* Sum: '<S2>/Add1' incorporates:
       *  Outport: '<Root>/source_voltage_phase'
       */
      sfc_6p_6p_topology_machine_B.Add1[2] =
        sfc_6p_6p_topology_machine_Y.source_voltage_phase +
        sfc_6p_6p_topology_machine_B.Gain2[2];

      /* Trigonometry: '<S2>/Trigonometric Function1' */
      tmpForInput_idx_0_0 = std::sin(sfc_6p_6p_topology_machine_B.Add1[2]);
      sfc_6p_6p_topology_machine_B.TrigonometricFunction1[2] =
        tmpForInput_idx_0_0;

      /* Gain: '<S2>/Gain1' */
      sfc_6p_6p_topology_machine_B.Gain1[2] =
        sfc_6p_6p_topology_machine_P.Gain1_Gain_j * tmpForInput_idx_0_0;

      /* Product: '<S2>/Product5' incorporates:
       *  Inport: '<Root>/voltage_magnitude_a'
       */
      sfc_6p_6p_topology_machine_B.Product5 =
        sfc_6p_6p_topology_machine_B.Gain1[0] *
        sfc_6p_6p_topology_machine_U.voltage_magnitude_a;

      /* Product: '<S2>/Product6' incorporates:
       *  Inport: '<Root>/voltage_magnitude_b'
       */
      sfc_6p_6p_topology_machine_B.Product6 =
        sfc_6p_6p_topology_machine_B.Gain1[1] *
        sfc_6p_6p_topology_machine_U.voltage_magnitude_b;

      /* Product: '<S2>/Product7' incorporates:
       *  Inport: '<Root>/voltage_magnitude_c'
       */
      sfc_6p_6p_topology_machine_B.Product7 =
        sfc_6p_6p_topology_machine_B.Gain1[2] *
        sfc_6p_6p_topology_machine_U.voltage_magnitude_c;

      /* S-Function (sfun_spssw_discc): '<S158>/State-Space' incorporates:
       *  Constant: '<S160>/SwitchCurrents'
       *  Inport: '<Root>/pulses_in_grid'
       *  Inport: '<Root>/pulses_in_motor'
       */

      /* S-Function block: <S158>/State-Space */
      {
        real_T accum;

        /* Circuit has switches */
        int_T *switch_status = (int_T*)
          sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.SWITCH_STATUS;
        int_T *switch_status_init = (int_T*)
          sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.SWITCH_STATUS_INIT;
        int_T *SwitchChange = (int_T*)
          sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.SW_CHG;
        int_T *gState = (int_T*)
          sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.G_STATE;
        real_T *yswitch = (real_T*)
          sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.Y_SWITCH;
        int_T *switchTypes = (int_T*)
          sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.SWITCH_TYPES;
        int_T *idxOutSw = (int_T*)
          sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.IDX_OUT_SW;
        real_T *DxCol = (real_T*)
          sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.DX_COL;
        real_T *tmp2 = (real_T*)
          sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.TMP2;
        real_T *BDcol = (real_T*)
          sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.BD_COL;
        real_T *tmp1 = (real_T*)
          sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.TMP1;
        real_T *uswlast = (real_T*)
          sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.USWLAST;
        int_T newState;
        int_T swChanged = 0;
        int loopsToDo = 20;
        real_T temp;

        /* keep an initial copy of switch_status*/
        memcpy(switch_status_init, switch_status, 12 * sizeof(int_T));
        memcpy(uswlast, &sfc_6p_6p_topology_machine_B.StateSpace_o1[0], 12*
               sizeof(real_T));
        do {
          if (loopsToDo == 1) {        /* Need to reset some variables: */
            swChanged = 0;

            /* return to the original switch status*/
            {
              int_T i1;
              for (i1=0; i1 < 12; i1++) {
                swChanged = ((SwitchChange[i1] = switch_status_init[i1] -
                              switch_status[i1]) != 0) ? 1 : swChanged;
                switch_status[i1] = switch_status_init[i1];
              }
            }
          } else {
            /*
             * Compute outputs:
             * ---------------
             */
            real_T *Cs = (real_T*)
              sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.CS;
            real_T *Ds = (real_T*)
              sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.DS;

            {
              int_T i1;
              real_T *y0 = &sfc_6p_6p_topology_machine_B.StateSpace_o1[0];
              for (i1=0; i1 < 35; i1++) {
                accum = 0.0;

                {
                  int_T i2;
                  real_T *xd = &sfc_6p_6p_topology_machine_DW.StateSpace_DSTATE
                    [0];
                  for (i2=0; i2 < 21; i2++) {
                    accum += *(Cs++) * xd[i2];
                  }
                }

                {
                  int_T i2;
                  const real_T *u0 =
                    &sfc_6p_6p_topology_machine_P.SwitchCurrents_Value[0];
                  for (i2=0; i2 < 12; i2++) {
                    accum += *(Ds++) * u0[i2];
                  }

                  accum += *(Ds++) * sfc_6p_6p_topology_machine_B.iabc[0];
                  accum += *(Ds++) * sfc_6p_6p_topology_machine_B.iabc[1];
                  accum += *(Ds++) * sfc_6p_6p_topology_machine_B.Product5;
                  accum += *(Ds++) * sfc_6p_6p_topology_machine_B.Product6;
                  accum += *(Ds++) * sfc_6p_6p_topology_machine_B.Product7;
                }

                y0[i1] = accum;
              }
            }

            swChanged = 0;

            {
              int_T i1;
              real_T *y0 = &sfc_6p_6p_topology_machine_B.StateSpace_o1[0];
              for (i1=0; i1 < 12; i1++) {
                newState = (y0[i1] > 0.0) && (gState[i1] > 0) ? 1 : ((y0[i1] <
                  0.0) ? 0 : switch_status[i1]);
                swChanged = ((SwitchChange[i1] = newState - switch_status[i1])
                             != 0) ? 1 : swChanged;
                switch_status[i1] = newState;/* Keep new state */
              }
            }
          }

          /*
           * Compute new As, Bs, Cs and Ds matrixes:
           * --------------------------------------
           */
          if (swChanged) {
            real_T *As = (real_T*)
              sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.AS;
            real_T *Cs = (real_T*)
              sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.CS;
            real_T *Bs = (real_T*)
              sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.BS;
            real_T *Ds = (real_T*)
              sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.DS;
            real_T a1;

            {
              int_T i1;
              for (i1=0; i1 < 12; i1++) {
                if (SwitchChange[i1] != 0) {
                  a1 = 1.0E+6*SwitchChange[i1];
                  temp = 1/(1-Ds[i1*18]*a1);

                  {
                    int_T i2;
                    for (i2=0; i2 < 35; i2++) {
                      DxCol[i2]= Ds[i2 * 17 + i1]*temp*a1;
                    }
                  }

                  DxCol[i1] = temp;

                  {
                    int_T i2;
                    for (i2=0; i2 < 21; i2++) {
                      BDcol[i2]= Bs[i2 * 17 + i1]*a1;
                    }
                  }

                  /* Copy row nSw of Cs into tmp1 and zero it out in Cs */
                  memcpy(tmp1, &Cs[i1 * 21], 21 * sizeof(real_T));
                  memset(&Cs[i1 * 21], '\0', 21 * sizeof(real_T));

                  /* Copy row nSw of Ds into tmp2 and zero it out in Ds */
                  memcpy(tmp2, &Ds[i1 * 17], 17 * sizeof(real_T));
                  memset(&Ds[i1 * 17], '\0', 17 * sizeof(real_T));

                  /* Cs = Cs + DxCol * tmp1, Ds = Ds + DxCol * tmp2 *******************/
                  {
                    int_T i2;
                    for (i2=0; i2 < 35; i2++) {
                      a1 = DxCol[i2];

                      {
                        int_T i3;
                        for (i3=0; i3 < 21; i3++) {
                          Cs[i2 * 21 + i3] += a1 * tmp1[i3];
                        }
                      }

                      {
                        int_T i3;
                        for (i3=0; i3 < 17; i3++) {
                          Ds[i2 * 17 + i3] += a1 * tmp2[i3];
                        }
                      }
                    }
                  }

                  /* As = As + BdCol*Cs(nSw,:), Bs = Bs + BdCol*Ds(nSw,:) *************/
                  {
                    int_T i2;
                    for (i2=0; i2 < 21; i2++) {
                      a1 = BDcol[i2];

                      {
                        int_T i3;
                        for (i3=0; i3 < 21; i3++) {
                          As[i2 * 21 + i3] += a1 * Cs[i1 * 21 + i3];
                        }
                      }

                      {
                        int_T i3;
                        for (i3=0; i3 < 17; i3++) {
                          Bs[i2 * 17 + i3] += a1 * Ds[i1 * 17 + i3];
                        }
                      }
                    }
                  }
                }
              }
            }
          }                            /* if (swChanged) */
        } while (swChanged > 0 && --loopsToDo > 0);

        if (loopsToDo == 0) {
          real_T *Cs = (real_T*)
            sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.CS;
          real_T *Ds = (real_T*)
            sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.DS;

          {
            int_T i1;
            real_T *y0 = &sfc_6p_6p_topology_machine_B.StateSpace_o1[0];
            for (i1=0; i1 < 35; i1++) {
              accum = 0.0;

              {
                int_T i2;
                real_T *xd = &sfc_6p_6p_topology_machine_DW.StateSpace_DSTATE[0];
                for (i2=0; i2 < 21; i2++) {
                  accum += *(Cs++) * xd[i2];
                }
              }

              {
                int_T i2;
                const real_T *u0 =
                  &sfc_6p_6p_topology_machine_P.SwitchCurrents_Value[0];
                for (i2=0; i2 < 12; i2++) {
                  accum += *(Ds++) * u0[i2];
                }

                accum += *(Ds++) * sfc_6p_6p_topology_machine_B.iabc[0];
                accum += *(Ds++) * sfc_6p_6p_topology_machine_B.iabc[1];
                accum += *(Ds++) * sfc_6p_6p_topology_machine_B.Product5;
                accum += *(Ds++) * sfc_6p_6p_topology_machine_B.Product6;
                accum += *(Ds++) * sfc_6p_6p_topology_machine_B.Product7;
              }

              y0[i1] = accum;
            }
          }
        }

        /* Output new switches states */
        {
          int_T i1;
          real_T *y1 = &sfc_6p_6p_topology_machine_B.StateSpace_o2[0];
          for (i1=0; i1 < 12; i1++) {
            y1[i1] = (real_T)switch_status[i1];
          }
        }
      }

      /* Gain: '<S25>/do not delete this gain' */
      sfc_6p_6p_topology_machine_B.donotdeletethisgain =
        sfc_6p_6p_topology_machine_P.donotdeletethisgain_Gain *
        sfc_6p_6p_topology_machine_B.StateSpace_o1[24];

      /* Outport: '<Root>/Udc' incorporates:
       *  Inport: '<Root>/grid_pt_ratio'
       *  Product: '<S8>/Divide'
       */
      sfc_6p_6p_topology_machine_Y.Udc =
        sfc_6p_6p_topology_machine_B.donotdeletethisgain /
        sfc_6p_6p_topology_machine_U.grid_pt_ratio;

      /* Gain: '<S3>/do not delete this gain' */
      sfc_6p_6p_topology_machine_B.donotdeletethisgain_o =
        sfc_6p_6p_topology_machine_P.donotdeletethisgain_Gain_d *
        sfc_6p_6p_topology_machine_B.StateSpace_o1[25];

      /* Outport: '<Root>/Idc' incorporates:
       *  Inport: '<Root>/grid_ct_ratio'
       *  Product: '<S9>/Divide'
       */
      sfc_6p_6p_topology_machine_Y.Idc =
        sfc_6p_6p_topology_machine_B.donotdeletethisgain_o /
        sfc_6p_6p_topology_machine_U.grid_ct_ratio;

      /* Gain: '<S127>/do not delete this gain' */
      sfc_6p_6p_topology_machine_B.donotdeletethisgain_c =
        sfc_6p_6p_topology_machine_P.donotdeletethisgain_Gain_i *
        sfc_6p_6p_topology_machine_B.StateSpace_o1[29];

      /* Gain: '<S128>/do not delete this gain' */
      sfc_6p_6p_topology_machine_B.donotdeletethisgain_p =
        sfc_6p_6p_topology_machine_P.donotdeletethisgain_Gain_o *
        sfc_6p_6p_topology_machine_B.StateSpace_o1[30];

      /* Gain: '<S129>/do not delete this gain' */
      sfc_6p_6p_topology_machine_B.donotdeletethisgain_cx =
        sfc_6p_6p_topology_machine_P.donotdeletethisgain_Gain_m *
        sfc_6p_6p_topology_machine_B.StateSpace_o1[31];

      /* Gain: '<S20>/Kv' */
      sfc_6p_6p_topology_machine_B.Kv[0] = sfc_6p_6p_topology_machine_P.Kv_Gain *
        sfc_6p_6p_topology_machine_B.donotdeletethisgain_c;
      sfc_6p_6p_topology_machine_B.Kv[1] = sfc_6p_6p_topology_machine_P.Kv_Gain *
        sfc_6p_6p_topology_machine_B.donotdeletethisgain_p;
      sfc_6p_6p_topology_machine_B.Kv[2] = sfc_6p_6p_topology_machine_P.Kv_Gain *
        sfc_6p_6p_topology_machine_B.donotdeletethisgain_cx;

      /* Gain: '<S130>/do not delete this gain' */
      sfc_6p_6p_topology_machine_B.donotdeletethisgain_f =
        sfc_6p_6p_topology_machine_P.donotdeletethisgain_Gain_k *
        sfc_6p_6p_topology_machine_B.StateSpace_o1[17];

      /* Gain: '<S131>/do not delete this gain' */
      sfc_6p_6p_topology_machine_B.donotdeletethisgain_k =
        sfc_6p_6p_topology_machine_P.donotdeletethisgain_Gain_h *
        sfc_6p_6p_topology_machine_B.StateSpace_o1[18];

      /* Gain: '<S132>/do not delete this gain' */
      sfc_6p_6p_topology_machine_B.donotdeletethisgain_n =
        sfc_6p_6p_topology_machine_P.donotdeletethisgain_Gain_n *
        sfc_6p_6p_topology_machine_B.StateSpace_o1[19];

      /* Gain: '<S20>/Kv1' */
      sfc_6p_6p_topology_machine_B.Kv1[0] =
        sfc_6p_6p_topology_machine_P.Kv1_Gain *
        sfc_6p_6p_topology_machine_B.donotdeletethisgain_f;
      sfc_6p_6p_topology_machine_B.Kv1[1] =
        sfc_6p_6p_topology_machine_P.Kv1_Gain *
        sfc_6p_6p_topology_machine_B.donotdeletethisgain_k;
      sfc_6p_6p_topology_machine_B.Kv1[2] =
        sfc_6p_6p_topology_machine_P.Kv1_Gain *
        sfc_6p_6p_topology_machine_B.donotdeletethisgain_n;

      /* Outport: '<Root>/Iph_grid' incorporates:
       *  Inport: '<Root>/grid_ct_ratio'
       *  Product: '<S10>/Divide'
       */
      sfc_6p_6p_topology_machine_Y.Iph_grid[0] =
        sfc_6p_6p_topology_machine_B.Kv[0] /
        sfc_6p_6p_topology_machine_U.grid_ct_ratio;

      /* Outport: '<Root>/Uph_ph_grid' incorporates:
       *  Inport: '<Root>/grid_pt_ratio'
       *  Product: '<S13>/Divide'
       */
      sfc_6p_6p_topology_machine_Y.Uph_ph_grid[0] =
        sfc_6p_6p_topology_machine_B.Kv1[0] /
        sfc_6p_6p_topology_machine_U.grid_pt_ratio;

      /* Outport: '<Root>/Iph_grid' incorporates:
       *  Inport: '<Root>/grid_ct_ratio'
       *  Product: '<S10>/Divide'
       */
      sfc_6p_6p_topology_machine_Y.Iph_grid[1] =
        sfc_6p_6p_topology_machine_B.Kv[1] /
        sfc_6p_6p_topology_machine_U.grid_ct_ratio;

      /* Outport: '<Root>/Uph_ph_grid' incorporates:
       *  Inport: '<Root>/grid_pt_ratio'
       *  Product: '<S13>/Divide'
       */
      sfc_6p_6p_topology_machine_Y.Uph_ph_grid[1] =
        sfc_6p_6p_topology_machine_B.Kv1[1] /
        sfc_6p_6p_topology_machine_U.grid_pt_ratio;

      /* Outport: '<Root>/Iph_grid' incorporates:
       *  Inport: '<Root>/grid_ct_ratio'
       *  Product: '<S10>/Divide'
       */
      sfc_6p_6p_topology_machine_Y.Iph_grid[2] =
        sfc_6p_6p_topology_machine_B.Kv[2] /
        sfc_6p_6p_topology_machine_U.grid_ct_ratio;

      /* Outport: '<Root>/Uph_ph_grid' incorporates:
       *  Inport: '<Root>/grid_pt_ratio'
       *  Product: '<S13>/Divide'
       */
      sfc_6p_6p_topology_machine_Y.Uph_ph_grid[2] =
        sfc_6p_6p_topology_machine_B.Kv1[2] /
        sfc_6p_6p_topology_machine_U.grid_pt_ratio;
    }

    /* RelationalOperator: '<S36>/Relational Operator' incorporates:
     *  Constant: '<S36>/Constant2'
     */
    sfc_6p_6p_topology_machine_B.RelationalOperator =
      (sfc_6p_6p_topology_machine_X.Integrator_CSTATE >
       sfc_6p_6p_topology_machine_P.Constant2_Value_ez);

    /* Sum: '<S36>/Subtract' incorporates:
     *  Constant: '<S36>/Constant2'
     */
    sfc_6p_6p_topology_machine_B.Subtract =
      sfc_6p_6p_topology_machine_X.Integrator_CSTATE -
      sfc_6p_6p_topology_machine_P.Constant2_Value_ez;

    /* InitialCondition: '<S36>/Initial' */
    if ((sfc_6p_6p_topology_machine_DW.Initial_FirstOutputTime == (rtMinusInf)) ||
        (sfc_6p_6p_topology_machine_DW.Initial_FirstOutputTime ==
         (&sfc_6p_6p_topology_machine_M)->Timing.t[0])) {
      sfc_6p_6p_topology_machine_DW.Initial_FirstOutputTime =
        (&sfc_6p_6p_topology_machine_M)->Timing.t[0];

      /* InitialCondition: '<S36>/Initial' */
      sfc_6p_6p_topology_machine_B.Initial =
        sfc_6p_6p_topology_machine_P.Initial_Value;
    } else {
      /* InitialCondition: '<S36>/Initial' */
      sfc_6p_6p_topology_machine_B.Initial =
        sfc_6p_6p_topology_machine_B.Subtract;
    }

    /* End of InitialCondition: '<S36>/Initial' */

    /* Integrator: '<S36>/Integrator' */
    /* Limited  Integrator  */
    if (rtsiIsModeUpdateTimeStep(&(&sfc_6p_6p_topology_machine_M)->solverInfo))
    {
      didZcEventOccur = (sfc_6p_6p_topology_machine_B.RelationalOperator &&
                         (sfc_6p_6p_topology_mach_PrevZCX.Integrator_Reset_ZCE
                          != POS_ZCSIG));
      sfc_6p_6p_topology_mach_PrevZCX.Integrator_Reset_ZCE =
        sfc_6p_6p_topology_machine_B.RelationalOperator;

      /* evaluate zero-crossings */
      if (didZcEventOccur || (sfc_6p_6p_topology_machine_DW.Integrator_IWORK !=
                              0)) {
        sfc_6p_6p_topology_machine_X.Integrator_CSTATE =
          sfc_6p_6p_topology_machine_B.Initial;
        rtsiSetBlockStateForSolverChangedAtMajorStep
          (&(&sfc_6p_6p_topology_machine_M)->solverInfo, true);
        rtsiSetContTimeOutputInconsistentWithStateAtMajorStep
          (&(&sfc_6p_6p_topology_machine_M)->solverInfo, true);
      }
    }

    if (sfc_6p_6p_topology_machine_X.Integrator_CSTATE >=
        sfc_6p_6p_topology_machine_P.Integrator_UpperSat_l) {
      if (sfc_6p_6p_topology_machine_X.Integrator_CSTATE !=
          sfc_6p_6p_topology_machine_P.Integrator_UpperSat_l) {
        rtsiSetBlockStateForSolverChangedAtMajorStep
          (&(&sfc_6p_6p_topology_machine_M)->solverInfo, true);
        rtsiSetContTimeOutputInconsistentWithStateAtMajorStep
          (&(&sfc_6p_6p_topology_machine_M)->solverInfo, true);
      }

      sfc_6p_6p_topology_machine_X.Integrator_CSTATE =
        sfc_6p_6p_topology_machine_P.Integrator_UpperSat_l;
    } else if (sfc_6p_6p_topology_machine_X.Integrator_CSTATE <=
               sfc_6p_6p_topology_machine_P.Integrator_LowerSat_n) {
      if (sfc_6p_6p_topology_machine_X.Integrator_CSTATE !=
          sfc_6p_6p_topology_machine_P.Integrator_LowerSat_n) {
        rtsiSetBlockStateForSolverChangedAtMajorStep
          (&(&sfc_6p_6p_topology_machine_M)->solverInfo, true);
        rtsiSetContTimeOutputInconsistentWithStateAtMajorStep
          (&(&sfc_6p_6p_topology_machine_M)->solverInfo, true);
      }

      sfc_6p_6p_topology_machine_X.Integrator_CSTATE =
        sfc_6p_6p_topology_machine_P.Integrator_LowerSat_n;
    }

    /* Integrator: '<S36>/Integrator' */
    sfc_6p_6p_topology_machine_B.Integrator_d =
      sfc_6p_6p_topology_machine_X.Integrator_CSTATE;

    /* Outport: '<Root>/wt' */
    sfc_6p_6p_topology_machine_Y.wt = sfc_6p_6p_topology_machine_B.Integrator_d;
    if (rtmIsMajorTimeStep((&sfc_6p_6p_topology_machine_M))) {
      /* Memory: '<S36>/Memory' */
      sfc_6p_6p_topology_machine_B.Memory =
        sfc_6p_6p_topology_machine_DW.Memory_PreviousInput;

      /* Gain: '<S145>/do not delete this gain' */
      sfc_6p_6p_topology_machine_B.donotdeletethisgain_c2 =
        sfc_6p_6p_topology_machine_P.donotdeletethisgain_Gain_m0 *
        sfc_6p_6p_topology_machine_B.StateSpace_o1[20];

      /* Gain: '<S146>/do not delete this gain' */
      sfc_6p_6p_topology_machine_B.donotdeletethisgain_g =
        sfc_6p_6p_topology_machine_P.donotdeletethisgain_Gain_ip *
        sfc_6p_6p_topology_machine_B.StateSpace_o1[21];

      /* Gain: '<S147>/do not delete this gain' */
      sfc_6p_6p_topology_machine_B.donotdeletethisgain_f1 =
        sfc_6p_6p_topology_machine_P.donotdeletethisgain_Gain_c *
        sfc_6p_6p_topology_machine_B.StateSpace_o1[22];

      /* Gain: '<S21>/Kv1' */
      sfc_6p_6p_topology_machine_B.Kv1_j[0] =
        sfc_6p_6p_topology_machine_P.Kv1_Gain_a *
        sfc_6p_6p_topology_machine_B.donotdeletethisgain_c2;
      sfc_6p_6p_topology_machine_B.Kv1_j[1] =
        sfc_6p_6p_topology_machine_P.Kv1_Gain_a *
        sfc_6p_6p_topology_machine_B.donotdeletethisgain_g;
      sfc_6p_6p_topology_machine_B.Kv1_j[2] =
        sfc_6p_6p_topology_machine_P.Kv1_Gain_a *
        sfc_6p_6p_topology_machine_B.donotdeletethisgain_f1;

      /* Outputs for Enabled SubSystem: '<S36>/Automatic Gain Control' incorporates:
       *  EnablePort: '<S37>/Enable'
       */
      if (rtsiIsModeUpdateTimeStep(&(&sfc_6p_6p_topology_machine_M)->solverInfo))
      {
        /* Constant: '<S36>/Constant1' */
        if (sfc_6p_6p_topology_machine_P.Constant1_Value_a > 0.0) {
          if (!sfc_6p_6p_topology_machine_DW.AutomaticGainControl_MODE) {
            (void) std::memset
              (&(sfc_6p_6p_topology_machine_XDis.VariableTransportDelay_CSTATE_c),
               0,
               4*sizeof(boolean_T));
            sfc_6p_6p_topology_machine_DW.AutomaticGainControl_MODE = true;
          }
        } else {
          tmpForInput_idx_0_0 = rtmGetTStart((&sfc_6p_6p_topology_machine_M));
          if (((((&sfc_6p_6p_topology_machine_M)->Timing.clockTick1+
                 (&sfc_6p_6p_topology_machine_M)->Timing.clockTickH1*
                 4294967296.0)) * 1.25E-5) == tmpForInput_idx_0_0) {
            (void) std::memset
              (&(sfc_6p_6p_topology_machine_XDis.VariableTransportDelay_CSTATE_c),
               1,
               4*sizeof(boolean_T));
          }

          if (sfc_6p_6p_topology_machine_DW.AutomaticGainControl_MODE) {
            (void) std::memset
              (&(sfc_6p_6p_topology_machine_XDis.VariableTransportDelay_CSTATE_c),
               1,
               4*sizeof(boolean_T));

            /* Disable for Enabled SubSystem: '<S48>/Subsystem - pi//2 delay' */
            if (sfc_6p_6p_topology_machine_DW.Subsystempi2delay_k.Subsystempi2delay_MODE)
            {
              sfc_6_Subsystempi2delay_Disable
                (&sfc_6p_6p_topology_machine_DW.Subsystempi2delay_k);
            }

            /* End of Disable for SubSystem: '<S48>/Subsystem - pi//2 delay' */

            /* Disable for Enabled SubSystem: '<S48>/Subsystem1' */
            if (sfc_6p_6p_topology_machine_DW.Subsystem1_ka.Subsystem1_MODE) {
              sfc_6p_6p_to_Subsystem1_Disable
                (&sfc_6p_6p_topology_machine_DW.Subsystem1_ka);
            }

            /* End of Disable for SubSystem: '<S48>/Subsystem1' */
            sfc_6p_6p_topology_machine_DW.AutomaticGainControl_MODE = false;
          }
        }

        /* End of Constant: '<S36>/Constant1' */
      }

      /* End of Outputs for SubSystem: '<S36>/Automatic Gain Control' */
    }

    /* Outputs for Enabled SubSystem: '<S36>/Automatic Gain Control' incorporates:
     *  EnablePort: '<S37>/Enable'
     */
    if (sfc_6p_6p_topology_machine_DW.AutomaticGainControl_MODE) {
      /* VariableTransportDelay: '<S46>/Variable Transport Delay' */
      {
        real_T **uBuffer = (real_T**)
          &sfc_6p_6p_topology_machine_DW.VariableTransportDelay_PWORK_g.TUbufferPtrs
          [0];
        real_T simTime = (&sfc_6p_6p_topology_machine_M)->Timing.t[0];
        real_T appliedDelay;

        /* For variable transport dealy, find the real applied dealy
         * here and then output
         */
        sfc_6p_6p_topology_machine_B.VariableTransportDelay_p=
          rt_VTDelayfindtDInterpolate
          (sfc_6p_6p_topology_machine_X.VariableTransportDelay_CSTATE_c,
           *uBuffer,
           sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK_g.CircularBufSize,
           sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK_g.Head,
           sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK_g.Tail,
           &sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK_g.Last,
           simTime, 0.0,0,
           0, (sfc_6p_6p_topology_machine_P.VariableTransportDelay_InitOutp),
           &appliedDelay);
      }

      /* Integrator: '<S46>/integrator' */
      sfc_6p_6p_topology_machine_B.integrator_d =
        sfc_6p_6p_topology_machine_X.integrator_CSTATE_g;

      /* Clock: '<S46>/Clock' */
      sfc_6p_6p_topology_machine_B.Clock_c = (&sfc_6p_6p_topology_machine_M)
        ->Timing.t[0];

      /* RelationalOperator: '<S46>/Relational Operator' incorporates:
       *  Constant: '<S46>/Constant'
       */
      sfc_6p_6p_topology_machine_B.RelationalOperator_h =
        (sfc_6p_6p_topology_machine_B.Clock_c >=
         sfc_6p_6p_topology_machine_P.Constant_Value);

      /* Switch: '<S46>/Switch' */
      if (sfc_6p_6p_topology_machine_B.RelationalOperator_h) {
        /* Sum: '<S46>/Sum7' */
        sfc_6p_6p_topology_machine_B.Sum7_h =
          sfc_6p_6p_topology_machine_B.integrator_d -
          sfc_6p_6p_topology_machine_B.VariableTransportDelay_p;

        /* Product: '<S46>/Product' */
        sfc_6p_6p_topology_machine_B.Meanvalue_o =
          sfc_6p_6p_topology_machine_B.Sum7_h *
          sfc_6p_6p_topology_machine_B.Memory;

        /* Switch: '<S46>/Switch' */
        sfc_6p_6p_topology_machine_B.Switch_n =
          sfc_6p_6p_topology_machine_B.Meanvalue_o;
      } else {
        /* Switch: '<S46>/Switch' incorporates:
         *  Constant: '<S46>/Constant1'
         */
        sfc_6p_6p_topology_machine_B.Switch_n =
          sfc_6p_6p_topology_machine_P.Constant1_Value;
      }

      /* End of Switch: '<S46>/Switch' */

      /* VariableTransportDelay: '<S47>/Variable Transport Delay' */
      {
        real_T **uBuffer = (real_T**)
          &sfc_6p_6p_topology_machine_DW.VariableTransportDelay_PWORK_j.TUbufferPtrs
          [0];
        real_T simTime = (&sfc_6p_6p_topology_machine_M)->Timing.t[0];
        real_T appliedDelay;

        /* For variable transport dealy, find the real applied dealy
         * here and then output
         */
        sfc_6p_6p_topology_machine_B.VariableTransportDelay_e=
          rt_VTDelayfindtDInterpolate
          (sfc_6p_6p_topology_machine_X.VariableTransportDelay_CSTATE_l,
           *uBuffer,
           sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK_a.CircularBufSize,
           sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK_a.Head,
           sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK_a.Tail,
           &sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK_a.Last,
           simTime, 0.0,0,
           0, (sfc_6p_6p_topology_machine_P.VariableTransportDelay_InitOu_g),
           &appliedDelay);
      }

      /* Integrator: '<S47>/integrator' */
      sfc_6p_6p_topology_machine_B.integrator_de =
        sfc_6p_6p_topology_machine_X.integrator_CSTATE_b;

      /* Clock: '<S47>/Clock' */
      sfc_6p_6p_topology_machine_B.Clock_b = (&sfc_6p_6p_topology_machine_M)
        ->Timing.t[0];

      /* RelationalOperator: '<S47>/Relational Operator' incorporates:
       *  Constant: '<S47>/Constant'
       */
      sfc_6p_6p_topology_machine_B.RelationalOperator_g1 =
        (sfc_6p_6p_topology_machine_B.Clock_b >=
         sfc_6p_6p_topology_machine_P.Constant_Value_p);

      /* Switch: '<S47>/Switch' */
      if (sfc_6p_6p_topology_machine_B.RelationalOperator_g1) {
        /* Sum: '<S47>/Sum7' */
        sfc_6p_6p_topology_machine_B.Sum7_i =
          sfc_6p_6p_topology_machine_B.integrator_de -
          sfc_6p_6p_topology_machine_B.VariableTransportDelay_e;

        /* Product: '<S47>/Product' */
        sfc_6p_6p_topology_machine_B.Meanvalue_j =
          sfc_6p_6p_topology_machine_B.Sum7_i *
          sfc_6p_6p_topology_machine_B.Memory;

        /* Switch: '<S47>/Switch' */
        sfc_6p_6p_topology_machine_B.Switch_m =
          sfc_6p_6p_topology_machine_B.Meanvalue_j;
      } else {
        /* Switch: '<S47>/Switch' incorporates:
         *  Constant: '<S47>/Constant1'
         */
        sfc_6p_6p_topology_machine_B.Switch_m =
          sfc_6p_6p_topology_machine_P.Constant1_Value_g;
      }

      /* End of Switch: '<S47>/Switch' */

      /* RealImagToComplex: '<S42>/Real-Imag to Complex' */
      sfc_6p_6p_topology_machine_B.RealImagtoComplex.re =
        sfc_6p_6p_topology_machine_B.Switch_n;
      sfc_6p_6p_topology_machine_B.RealImagtoComplex.im =
        sfc_6p_6p_topology_machine_B.Switch_m;

      /* ComplexToMagnitudeAngle: '<S42>/Complex to Magnitude-Angle' incorporates:
       *  RealImagToComplex: '<S42>/Real-Imag to Complex'
       */
      sfc_6p_6p_topology_machine_B.ComplextoMagnitudeAngle_o1 = rt_hypotd_snf
        (sfc_6p_6p_topology_machine_B.RealImagtoComplex.re,
         sfc_6p_6p_topology_machine_B.RealImagtoComplex.im);

      /* ComplexToMagnitudeAngle: '<S42>/Complex to Magnitude-Angle' incorporates:
       *  RealImagToComplex: '<S42>/Real-Imag to Complex'
       */
      sfc_6p_6p_topology_machine_B.ComplextoMagnitudeAngle_o2 = rt_atan2d_snf
        (sfc_6p_6p_topology_machine_B.RealImagtoComplex.im,
         sfc_6p_6p_topology_machine_B.RealImagtoComplex.re);

      /* Saturate: '<S37>/Saturation' */
      tmpForInput_idx_1 =
        sfc_6p_6p_topology_machine_B.ComplextoMagnitudeAngle_o1;
      tmpForInput_idx_0_0 = sfc_6p_6p_topology_machine_P.Saturation_LowerSat;
      tmpForInput_idx_2 = sfc_6p_6p_topology_machine_P.Saturation_UpperSat;
      if (tmpForInput_idx_1 > tmpForInput_idx_2) {
        /* Saturate: '<S37>/Saturation' */
        sfc_6p_6p_topology_machine_B.Saturation_a = tmpForInput_idx_2;
      } else if (tmpForInput_idx_1 < tmpForInput_idx_0_0) {
        /* Saturate: '<S37>/Saturation' */
        sfc_6p_6p_topology_machine_B.Saturation_a = tmpForInput_idx_0_0;
      } else {
        /* Saturate: '<S37>/Saturation' */
        sfc_6p_6p_topology_machine_B.Saturation_a = tmpForInput_idx_1;
      }

      /* End of Saturate: '<S37>/Saturation' */

      /* Math: '<S37>/Math Function'
       *
       * About '<S37>/Math Function':
       *  Operator: reciprocal
       */
      tmpForInput_idx_0_0 = sfc_6p_6p_topology_machine_B.Saturation_a;

      /* Math: '<S37>/Math Function'
       *
       * About '<S37>/Math Function':
       *  Operator: reciprocal
       */
      sfc_6p_6p_topology_machine_B.MathFunction_c = 1.0 / tmpForInput_idx_0_0;

      /* Saturate: '<S46>/To avoid division  by zero' */
      tmpForInput_idx_1 = sfc_6p_6p_topology_machine_B.Memory;
      tmpForInput_idx_0_0 =
        sfc_6p_6p_topology_machine_P.Toavoiddivisionbyzero_LowerSat;
      tmpForInput_idx_2 =
        sfc_6p_6p_topology_machine_P.Toavoiddivisionbyzero_UpperSat;
      if (tmpForInput_idx_1 > tmpForInput_idx_2) {
        /* Saturate: '<S46>/To avoid division  by zero' */
        sfc_6p_6p_topology_machine_B.Freq_o = tmpForInput_idx_2;
      } else if (tmpForInput_idx_1 < tmpForInput_idx_0_0) {
        /* Saturate: '<S46>/To avoid division  by zero' */
        sfc_6p_6p_topology_machine_B.Freq_o = tmpForInput_idx_0_0;
      } else {
        /* Saturate: '<S46>/To avoid division  by zero' */
        sfc_6p_6p_topology_machine_B.Freq_o = tmpForInput_idx_1;
      }

      /* End of Saturate: '<S46>/To avoid division  by zero' */

      /* Fcn: '<S46>/period' */
      sfc_6p_6p_topology_machine_B.period_i = 1.0 /
        sfc_6p_6p_topology_machine_B.Freq_o;

      /* Saturate: '<S47>/To avoid division  by zero' */
      tmpForInput_idx_1 = sfc_6p_6p_topology_machine_B.Memory;
      tmpForInput_idx_0_0 =
        sfc_6p_6p_topology_machine_P.Toavoiddivisionbyzero_LowerSa_l;
      tmpForInput_idx_2 =
        sfc_6p_6p_topology_machine_P.Toavoiddivisionbyzero_UpperSa_h;
      if (tmpForInput_idx_1 > tmpForInput_idx_2) {
        /* Saturate: '<S47>/To avoid division  by zero' */
        sfc_6p_6p_topology_machine_B.Freq_l = tmpForInput_idx_2;
      } else if (tmpForInput_idx_1 < tmpForInput_idx_0_0) {
        /* Saturate: '<S47>/To avoid division  by zero' */
        sfc_6p_6p_topology_machine_B.Freq_l = tmpForInput_idx_0_0;
      } else {
        /* Saturate: '<S47>/To avoid division  by zero' */
        sfc_6p_6p_topology_machine_B.Freq_l = tmpForInput_idx_1;
      }

      /* End of Saturate: '<S47>/To avoid division  by zero' */

      /* Fcn: '<S47>/period' */
      sfc_6p_6p_topology_machine_B.period_a = 1.0 /
        sfc_6p_6p_topology_machine_B.Freq_l;
      if (rtmIsMajorTimeStep((&sfc_6p_6p_topology_machine_M))) {
        /* RelationalOperator: '<S50>/Compare' incorporates:
         *  Constant: '<S48>/Constant'
         *  Constant: '<S50>/Constant'
         */
        sfc_6p_6p_topology_machine_B.Compare_j0 =
          (sfc_6p_6p_topology_machine_P.AlphaBetaZerotodq0_Alignment ==
           sfc_6p_6p_topology_machine_P.CompareToConstant_const);

        /* RelationalOperator: '<S51>/Compare' incorporates:
         *  Constant: '<S48>/Constant'
         *  Constant: '<S51>/Constant'
         */
        sfc_6p_6p_topology_machine_B.Compare_p =
          (sfc_6p_6p_topology_machine_P.AlphaBetaZerotodq0_Alignment ==
           sfc_6p_6p_topology_machine_P.CompareToConstant1_const);
      }

      /* Gain: '<S49>/Gain3' */
      tmpForInput_idx_0_0 = sfc_6p_6p_topology_machine_B.Kv1_j[0];
      tmpForInput_idx_1 = sfc_6p_6p_topology_machine_B.Kv1_j[1];
      tmpForInput_idx_2 = sfc_6p_6p_topology_machine_B.Kv1_j[2];
      for (i = 0; i < 3; i++) {
        /* Gain: '<S49>/Gain3' */
        tmpForInput_idx_0 = sfc_6p_6p_topology_machine_P.Gain3_Gain[i] *
          tmpForInput_idx_0_0;
        tmpForInput_idx_0 += sfc_6p_6p_topology_machine_P.Gain3_Gain[i + 3] *
          tmpForInput_idx_1;
        tmpForInput_idx_0 += sfc_6p_6p_topology_machine_P.Gain3_Gain[i + 6] *
          tmpForInput_idx_2;
        sfc_6p_6p_topology_machine_B.Gain3_o4[i] = tmpForInput_idx_0;

        /* Gain: '<S49>/Gain1' */
        sfc_6p_6p_topology_machine_B.Gain1_c[i] =
          sfc_6p_6p_topology_machine_P.Gain1_Gain * tmpForInput_idx_0;
      }

      /* Outputs for Enabled SubSystem: '<S48>/Subsystem - pi//2 delay' */
      /* Gain: '<S49>/Gain1' */
      sfc_6p_6p_top_Subsystempi2delay(sfc_6p_6p_topology_machine_B.Compare_p,
        &sfc_6p_6p_topology_machine_B.Gain1_c[0],
        sfc_6p_6p_topology_machine_B.Integrator_d,
        &sfc_6p_6p_topology_machine_B.Subsystempi2delay_k,
        &sfc_6p_6p_topology_machine_DW.Subsystempi2delay_k);

      /* End of Outputs for SubSystem: '<S48>/Subsystem - pi//2 delay' */

      /* Outputs for Enabled SubSystem: '<S48>/Subsystem1' */
      sfc_6p_6p_topology_m_Subsystem1(sfc_6p_6p_topology_machine_B.Compare_j0,
        &sfc_6p_6p_topology_machine_B.Gain1_c[0],
        sfc_6p_6p_topology_machine_B.Integrator_d,
        &sfc_6p_6p_topology_machine_B.Subsystem1_ka,
        &sfc_6p_6p_topology_machine_DW.Subsystem1_ka);

      /* End of Outputs for SubSystem: '<S48>/Subsystem1' */

      /* Switch: '<S48>/Switch' */
      if (sfc_6p_6p_topology_machine_B.Compare_j0 != 0) {
        /* Switch: '<S48>/Switch' */
        sfc_6p_6p_topology_machine_B.Switch_dq[0] =
          sfc_6p_6p_topology_machine_B.Subsystem1_ka.Fcn;
        sfc_6p_6p_topology_machine_B.Switch_dq[1] =
          sfc_6p_6p_topology_machine_B.Subsystem1_ka.Fcn1;
      } else {
        /* Switch: '<S48>/Switch' */
        sfc_6p_6p_topology_machine_B.Switch_dq[0] =
          sfc_6p_6p_topology_machine_B.Subsystempi2delay_k.Fcn;
        sfc_6p_6p_topology_machine_B.Switch_dq[1] =
          sfc_6p_6p_topology_machine_B.Subsystempi2delay_k.Fcn1;
      }

      /* End of Switch: '<S48>/Switch' */
    }

    /* End of Outputs for SubSystem: '<S36>/Automatic Gain Control' */

    /* Integrator: '<S38>/Integrator' */
    /* Limited  Integrator  */
    tmpForInput_idx_0_0 = sfc_6p_6p_topology_machine_P.Continuous_Par_Limits[0];
    if (sfc_6p_6p_topology_machine_X.Integrator_CSTATE_i >= tmpForInput_idx_0_0)
    {
      sfc_6p_6p_topology_machine_X.Integrator_CSTATE_i =
        sfc_6p_6p_topology_machine_P.Continuous_Par_Limits[0];
    } else {
      tmpForInput_idx_0_0 = sfc_6p_6p_topology_machine_P.Continuous_Par_Limits[1];
      if (sfc_6p_6p_topology_machine_X.Integrator_CSTATE_i <=
          tmpForInput_idx_0_0) {
        sfc_6p_6p_topology_machine_X.Integrator_CSTATE_i =
          sfc_6p_6p_topology_machine_P.Continuous_Par_Limits[1];
      }
    }

    /* Integrator: '<S38>/Integrator' */
    sfc_6p_6p_topology_machine_B.Integrator_o =
      sfc_6p_6p_topology_machine_X.Integrator_CSTATE_i;

    /* VariableTransportDelay: '<S58>/Variable Transport Delay' */
    {
      real_T **uBuffer = (real_T**)
        &sfc_6p_6p_topology_machine_DW.VariableTransportDelay_PWORK.TUbufferPtrs[
        0];
      real_T simTime = (&sfc_6p_6p_topology_machine_M)->Timing.t[0];
      real_T appliedDelay;

      /* For variable transport dealy, find the real applied dealy
       * here and then output
       */
      sfc_6p_6p_topology_machine_B.VariableTransportDelay=
        rt_VTDelayfindtDInterpolate
        (sfc_6p_6p_topology_machine_X.VariableTransportDelay_CSTATE, *uBuffer,
         sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK.CircularBufSize,
         sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK.Head,
         sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK.Tail,
         &sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK.Last,
         simTime, 0.0,0,
         0, (sfc_6p_6p_topology_machine_P.VariableTransportDelay_InitOu_b),
         &appliedDelay);
    }

    /* Integrator: '<S58>/integrator' */
    sfc_6p_6p_topology_machine_B.integrator =
      sfc_6p_6p_topology_machine_X.integrator_CSTATE;

    /* Clock: '<S58>/Clock' */
    sfc_6p_6p_topology_machine_B.Clock = (&sfc_6p_6p_topology_machine_M)
      ->Timing.t[0];

    /* RelationalOperator: '<S58>/Relational Operator' incorporates:
     *  Constant: '<S58>/Constant'
     */
    sfc_6p_6p_topology_machine_B.RelationalOperator_g =
      (sfc_6p_6p_topology_machine_B.Clock >=
       sfc_6p_6p_topology_machine_P.Constant_Value_f);

    /* Switch: '<S58>/Switch' */
    if (sfc_6p_6p_topology_machine_B.RelationalOperator_g) {
      /* Sum: '<S58>/Sum7' */
      sfc_6p_6p_topology_machine_B.Sum7 =
        sfc_6p_6p_topology_machine_B.integrator -
        sfc_6p_6p_topology_machine_B.VariableTransportDelay;

      /* Product: '<S58>/Product' */
      sfc_6p_6p_topology_machine_B.Meanvalue = sfc_6p_6p_topology_machine_B.Sum7
        * sfc_6p_6p_topology_machine_B.Memory;

      /* Switch: '<S58>/Switch' */
      sfc_6p_6p_topology_machine_B.Switch_g =
        sfc_6p_6p_topology_machine_B.Meanvalue;
    } else {
      /* Switch: '<S58>/Switch' incorporates:
       *  Constant: '<S58>/Constant1'
       */
      sfc_6p_6p_topology_machine_B.Switch_g =
        sfc_6p_6p_topology_machine_P.Constant1_Value_ij;
    }

    /* End of Switch: '<S58>/Switch' */

    /* Product: '<S36>/Divide' */
    sfc_6p_6p_topology_machine_B.Divide_g =
      sfc_6p_6p_topology_machine_B.Switch_g *
      sfc_6p_6p_topology_machine_B.MathFunction_c;

    /* Gain: '<S38>/Kp4' */
    sfc_6p_6p_topology_machine_B.Kp4 =
      sfc_6p_6p_topology_machine_P.Continuous_Kp *
      sfc_6p_6p_topology_machine_B.Divide_g;

    /* Gain: '<S38>/Kp5' */
    sfc_6p_6p_topology_machine_B.Kp5 =
      sfc_6p_6p_topology_machine_P.Continuous_Ki *
      sfc_6p_6p_topology_machine_B.Divide_g;

    /* Gain: '<S38>/Kp6' */
    sfc_6p_6p_topology_machine_B.Kp6 =
      sfc_6p_6p_topology_machine_P.Continuous_Kd *
      sfc_6p_6p_topology_machine_B.Divide_g;

    /* TransferFcn: '<S38>/Transfer Fcn' */
    sfc_6p_6p_topology_machine_B.TransferFcn = 0.0;
    sfc_6p_6p_topology_machine_B.TransferFcn +=
      sfc_6p_6p_topology_machine_P.TransferFcn_C *
      sfc_6p_6p_topology_machine_X.TransferFcn_CSTATE;
    sfc_6p_6p_topology_machine_B.TransferFcn +=
      sfc_6p_6p_topology_machine_P.TransferFcn_D *
      sfc_6p_6p_topology_machine_B.Kp6;

    /* Sum: '<S38>/Sum6' */
    sfc_6p_6p_topology_machine_B.Sum6 = (sfc_6p_6p_topology_machine_B.Kp4 +
      sfc_6p_6p_topology_machine_B.Integrator_o) +
      sfc_6p_6p_topology_machine_B.TransferFcn;

    /* Saturate: '<S38>/Saturation2' */
    tmpForInput_idx_0_0 = sfc_6p_6p_topology_machine_P.Continuous_Par_Limits[1];
    tmpForInput_idx_2 = sfc_6p_6p_topology_machine_P.Continuous_Par_Limits[0];
    tmpForInput_idx_1 = sfc_6p_6p_topology_machine_B.Sum6;
    if (tmpForInput_idx_1 > tmpForInput_idx_2) {
      /* Saturate: '<S38>/Saturation2' */
      sfc_6p_6p_topology_machine_B.Saturation2 = tmpForInput_idx_2;
    } else if (tmpForInput_idx_1 < tmpForInput_idx_0_0) {
      /* Saturate: '<S38>/Saturation2' */
      sfc_6p_6p_topology_machine_B.Saturation2 = tmpForInput_idx_0_0;
    } else {
      /* Saturate: '<S38>/Saturation2' */
      sfc_6p_6p_topology_machine_B.Saturation2 = tmpForInput_idx_1;
    }

    /* End of Saturate: '<S38>/Saturation2' */

    /* Gain: '<S36>/Gain10' */
    sfc_6p_6p_topology_machine_B.Gain10 =
      sfc_6p_6p_topology_machine_P.Gain10_Gain *
      sfc_6p_6p_topology_machine_B.Saturation2;

    /* RateLimiter: '<S36>/Rate Limiter' */
    if (sfc_6p_6p_topology_machine_DW.LastMajorTime == (rtInf)) {
      /* RateLimiter: '<S36>/Rate Limiter' */
      sfc_6p_6p_topology_machine_B.RateLimiter =
        sfc_6p_6p_topology_machine_B.Gain10;
    } else {
      tmpForInput_idx_2 = (&sfc_6p_6p_topology_machine_M)->Timing.t[0] -
        sfc_6p_6p_topology_machine_DW.LastMajorTime;
      if (sfc_6p_6p_topology_machine_DW.LastMajorTime ==
          (&sfc_6p_6p_topology_machine_M)->Timing.t[0]) {
        if (sfc_6p_6p_topology_machine_DW.PrevLimited) {
          /* RateLimiter: '<S36>/Rate Limiter' */
          sfc_6p_6p_topology_machine_B.RateLimiter =
            sfc_6p_6p_topology_machine_DW.PrevY;
        } else {
          /* RateLimiter: '<S36>/Rate Limiter' */
          sfc_6p_6p_topology_machine_B.RateLimiter =
            sfc_6p_6p_topology_machine_B.Gain10;
        }
      } else {
        tmpForInput_idx_1 = tmpForInput_idx_2 *
          sfc_6p_6p_topology_machine_P.RateLimiter_RisingLim;
        tmpForInput_idx_0_0 = sfc_6p_6p_topology_machine_B.Gain10 -
          sfc_6p_6p_topology_machine_DW.PrevY;
        if (tmpForInput_idx_0_0 > tmpForInput_idx_1) {
          /* RateLimiter: '<S36>/Rate Limiter' */
          sfc_6p_6p_topology_machine_B.RateLimiter =
            sfc_6p_6p_topology_machine_DW.PrevY + tmpForInput_idx_1;
          didZcEventOccur = true;
        } else {
          tmpForInput_idx_2 *=
            sfc_6p_6p_topology_machine_P.RateLimiter_FallingLim;
          if (tmpForInput_idx_0_0 < tmpForInput_idx_2) {
            /* RateLimiter: '<S36>/Rate Limiter' */
            sfc_6p_6p_topology_machine_B.RateLimiter =
              sfc_6p_6p_topology_machine_DW.PrevY + tmpForInput_idx_2;
            didZcEventOccur = true;
          } else {
            /* RateLimiter: '<S36>/Rate Limiter' */
            sfc_6p_6p_topology_machine_B.RateLimiter =
              sfc_6p_6p_topology_machine_B.Gain10;
            didZcEventOccur = false;
          }
        }

        if (rtsiIsModeUpdateTimeStep(&(&sfc_6p_6p_topology_machine_M)
             ->solverInfo)) {
          sfc_6p_6p_topology_machine_DW.PrevLimited = didZcEventOccur;
        }
      }
    }

    /* End of RateLimiter: '<S36>/Rate Limiter' */

    /* Integrator: '<S54>/Integrator_x1' */
    sfc_6p_6p_topology_machine_B.x1 =
      sfc_6p_6p_topology_machine_X.Integrator_x1_CSTATE;

    /* Gain: '<S55>/A11' */
    sfc_6p_6p_topology_machine_B.A11 = sfc_6p_6p_topology_machine_P.A11_Gain *
      sfc_6p_6p_topology_machine_B.x1;

    /* Integrator: '<S54>/Integrator_x2' */
    sfc_6p_6p_topology_machine_B.x2 =
      sfc_6p_6p_topology_machine_X.Integrator_x2_CSTATE;

    /* Gain: '<S55>/A12' */
    sfc_6p_6p_topology_machine_B.A12 = sfc_6p_6p_topology_machine_P.A12_Gain *
      sfc_6p_6p_topology_machine_B.x2;

    /* Gain: '<S55>/A21' */
    sfc_6p_6p_topology_machine_B.A21 = sfc_6p_6p_topology_machine_P.A21_Gain *
      sfc_6p_6p_topology_machine_B.x1;

    /* Gain: '<S55>/A22' */
    sfc_6p_6p_topology_machine_B.A22 = sfc_6p_6p_topology_machine_P.A22_Gain *
      sfc_6p_6p_topology_machine_B.x2;

    /* Sum: '<S55>/sum2' */
    sfc_6p_6p_topology_machine_B.sum2 = sfc_6p_6p_topology_machine_B.A11 +
      sfc_6p_6p_topology_machine_B.A12;

    /* Sum: '<S55>/sum3' */
    sfc_6p_6p_topology_machine_B.sum3 = sfc_6p_6p_topology_machine_B.A21 +
      sfc_6p_6p_topology_machine_B.A22;

    /* Gain: '<S56>/B11' */
    sfc_6p_6p_topology_machine_B.B11 = sfc_6p_6p_topology_machine_P.B11_Gain *
      sfc_6p_6p_topology_machine_B.RateLimiter;

    /* Sum: '<S54>/A*x1+ B* u1' */
    sfc_6p_6p_topology_machine_B.x1_i = sfc_6p_6p_topology_machine_B.sum2 +
      sfc_6p_6p_topology_machine_B.B11;

    /* Gain: '<S56>/B21' */
    sfc_6p_6p_topology_machine_B.B21 = sfc_6p_6p_topology_machine_P.B21_Gain *
      sfc_6p_6p_topology_machine_B.RateLimiter;

    /* Sum: '<S54>/A*x2+ B*u2' */
    sfc_6p_6p_topology_machine_B.x2_m = sfc_6p_6p_topology_machine_B.sum3 +
      sfc_6p_6p_topology_machine_B.B21;

    /* Gain: '<S57>/C11' */
    sfc_6p_6p_topology_machine_B.C11 = sfc_6p_6p_topology_machine_P.C11_Gain *
      sfc_6p_6p_topology_machine_B.x1;

    /* Gain: '<S57>/C12' */
    sfc_6p_6p_topology_machine_B.C12 = sfc_6p_6p_topology_machine_P.C12_Gain *
      sfc_6p_6p_topology_machine_B.x2;

    /* Sum: '<S57>/sum2' */
    sfc_6p_6p_topology_machine_B.sum2_a = sfc_6p_6p_topology_machine_B.C11 +
      sfc_6p_6p_topology_machine_B.C12;

    /* Gain: '<S54>/D*u' */
    sfc_6p_6p_topology_machine_B.Du = sfc_6p_6p_topology_machine_P.Du_Gain *
      sfc_6p_6p_topology_machine_B.RateLimiter;

    /* Sum: '<S54>/C*x + D*u' */
    sfc_6p_6p_topology_machine_B.y = sfc_6p_6p_topology_machine_B.Du +
      sfc_6p_6p_topology_machine_B.sum2_a;

    /* Saturate: '<S58>/To avoid division  by zero' */
    tmpForInput_idx_1 = sfc_6p_6p_topology_machine_B.Memory;
    tmpForInput_idx_0_0 =
      sfc_6p_6p_topology_machine_P.Toavoiddivisionbyzero_LowerSa_n;
    tmpForInput_idx_2 =
      sfc_6p_6p_topology_machine_P.Toavoiddivisionbyzero_UpperSa_o;
    if (tmpForInput_idx_1 > tmpForInput_idx_2) {
      /* Saturate: '<S58>/To avoid division  by zero' */
      sfc_6p_6p_topology_machine_B.Freq = tmpForInput_idx_2;
    } else if (tmpForInput_idx_1 < tmpForInput_idx_0_0) {
      /* Saturate: '<S58>/To avoid division  by zero' */
      sfc_6p_6p_topology_machine_B.Freq = tmpForInput_idx_0_0;
    } else {
      /* Saturate: '<S58>/To avoid division  by zero' */
      sfc_6p_6p_topology_machine_B.Freq = tmpForInput_idx_1;
    }

    /* End of Saturate: '<S58>/To avoid division  by zero' */

    /* Fcn: '<S58>/period' */
    sfc_6p_6p_topology_machine_B.period = 1.0 /
      sfc_6p_6p_topology_machine_B.Freq;
    if (rtmIsMajorTimeStep((&sfc_6p_6p_topology_machine_M))) {
      /* RelationalOperator: '<S61>/Compare' incorporates:
       *  Constant: '<S59>/Constant'
       *  Constant: '<S61>/Constant'
       */
      sfc_6p_6p_topology_machine_B.Compare_e =
        (sfc_6p_6p_topology_machine_P.AlphaBetaZerotodq0_Alignment_n ==
         sfc_6p_6p_topology_machine_P.CompareToConstant_const_c);

      /* RelationalOperator: '<S62>/Compare' incorporates:
       *  Constant: '<S59>/Constant'
       *  Constant: '<S62>/Constant'
       */
      sfc_6p_6p_topology_machine_B.Compare_j =
        (sfc_6p_6p_topology_machine_P.AlphaBetaZerotodq0_Alignment_n ==
         sfc_6p_6p_topology_machine_P.CompareToConstant1_const_g);
    }

    /* Gain: '<S60>/Gain3' */
    tmpForInput_idx_0_0 = sfc_6p_6p_topology_machine_B.Kv1_j[0];
    tmpForInput_idx_1 = sfc_6p_6p_topology_machine_B.Kv1_j[1];
    tmpForInput_idx_2 = sfc_6p_6p_topology_machine_B.Kv1_j[2];
    for (i = 0; i < 3; i++) {
      /* Gain: '<S60>/Gain3' */
      tmpForInput_idx_0 = sfc_6p_6p_topology_machine_P.Gain3_Gain_j[i] *
        tmpForInput_idx_0_0;
      tmpForInput_idx_0 += sfc_6p_6p_topology_machine_P.Gain3_Gain_j[i + 3] *
        tmpForInput_idx_1;
      tmpForInput_idx_0 += sfc_6p_6p_topology_machine_P.Gain3_Gain_j[i + 6] *
        tmpForInput_idx_2;
      sfc_6p_6p_topology_machine_B.Gain3_o[i] = tmpForInput_idx_0;

      /* Gain: '<S60>/Gain1' */
      sfc_6p_6p_topology_machine_B.Gain1_f[i] =
        sfc_6p_6p_topology_machine_P.Gain1_Gain_o * tmpForInput_idx_0;
    }

    /* Outputs for Enabled SubSystem: '<S59>/Subsystem - pi//2 delay' */
    /* Gain: '<S60>/Gain1' */
    sfc_6p_6p_top_Subsystempi2delay(sfc_6p_6p_topology_machine_B.Compare_j,
      &sfc_6p_6p_topology_machine_B.Gain1_f[0],
      sfc_6p_6p_topology_machine_B.Integrator_d,
      &sfc_6p_6p_topology_machine_B.Subsystempi2delay,
      &sfc_6p_6p_topology_machine_DW.Subsystempi2delay);

    /* End of Outputs for SubSystem: '<S59>/Subsystem - pi//2 delay' */

    /* Outputs for Enabled SubSystem: '<S59>/Subsystem1' */
    sfc_6p_6p_topology_m_Subsystem1(sfc_6p_6p_topology_machine_B.Compare_e,
      &sfc_6p_6p_topology_machine_B.Gain1_f[0],
      sfc_6p_6p_topology_machine_B.Integrator_d,
      &sfc_6p_6p_topology_machine_B.Subsystem1,
      &sfc_6p_6p_topology_machine_DW.Subsystem1);

    /* End of Outputs for SubSystem: '<S59>/Subsystem1' */

    /* Switch: '<S59>/Switch' */
    if (sfc_6p_6p_topology_machine_B.Compare_e != 0) {
      /* Switch: '<S59>/Switch' */
      sfc_6p_6p_topology_machine_B.Switch_a[0] =
        sfc_6p_6p_topology_machine_B.Subsystem1.Fcn;
      sfc_6p_6p_topology_machine_B.Switch_a[1] =
        sfc_6p_6p_topology_machine_B.Subsystem1.Fcn1;
    } else {
      /* Switch: '<S59>/Switch' */
      sfc_6p_6p_topology_machine_B.Switch_a[0] =
        sfc_6p_6p_topology_machine_B.Subsystempi2delay.Fcn;
      sfc_6p_6p_topology_machine_B.Switch_a[1] =
        sfc_6p_6p_topology_machine_B.Subsystempi2delay.Fcn1;
    }

    /* End of Switch: '<S59>/Switch' */
    if (rtmIsMajorTimeStep((&sfc_6p_6p_topology_machine_M))) {
      /* Gain: '<S142>/do not delete this gain' */
      sfc_6p_6p_topology_machine_B.donotdeletethisgain_h =
        sfc_6p_6p_topology_machine_P.donotdeletethisgain_Gain_b *
        sfc_6p_6p_topology_machine_B.StateSpace_o1[32];

      /* Gain: '<S143>/do not delete this gain' */
      sfc_6p_6p_topology_machine_B.donotdeletethisgain_d =
        sfc_6p_6p_topology_machine_P.donotdeletethisgain_Gain_km *
        sfc_6p_6p_topology_machine_B.StateSpace_o1[33];

      /* Gain: '<S144>/do not delete this gain' */
      sfc_6p_6p_topology_machine_B.donotdeletethisgain_kk =
        sfc_6p_6p_topology_machine_P.donotdeletethisgain_Gain_p *
        sfc_6p_6p_topology_machine_B.StateSpace_o1[34];

      /* Gain: '<S21>/Kv' */
      sfc_6p_6p_topology_machine_B.Kv_n[0] =
        sfc_6p_6p_topology_machine_P.Kv_Gain_e *
        sfc_6p_6p_topology_machine_B.donotdeletethisgain_h;
      sfc_6p_6p_topology_machine_B.Kv_n[1] =
        sfc_6p_6p_topology_machine_P.Kv_Gain_e *
        sfc_6p_6p_topology_machine_B.donotdeletethisgain_d;
      sfc_6p_6p_topology_machine_B.Kv_n[2] =
        sfc_6p_6p_topology_machine_P.Kv_Gain_e *
        sfc_6p_6p_topology_machine_B.donotdeletethisgain_kk;

      /* Gain: '<S112>/do not delete this gain' */
      sfc_6p_6p_topology_machine_B.donotdeletethisgain_m =
        sfc_6p_6p_topology_machine_P.donotdeletethisgain_Gain_f *
        sfc_6p_6p_topology_machine_B.StateSpace_o1[26];

      /* Gain: '<S113>/do not delete this gain' */
      sfc_6p_6p_topology_machine_B.donotdeletethisgain_b =
        sfc_6p_6p_topology_machine_P.donotdeletethisgain_Gain_hv *
        sfc_6p_6p_topology_machine_B.StateSpace_o1[27];

      /* Gain: '<S114>/do not delete this gain' */
      sfc_6p_6p_topology_machine_B.donotdeletethisgain_n2 =
        sfc_6p_6p_topology_machine_P.donotdeletethisgain_Gain_j *
        sfc_6p_6p_topology_machine_B.StateSpace_o1[28];

      /* Gain: '<S19>/Kv' */
      sfc_6p_6p_topology_machine_B.Kv_n3[0] =
        sfc_6p_6p_topology_machine_P.Kv_Gain_ej *
        sfc_6p_6p_topology_machine_B.donotdeletethisgain_m;
      sfc_6p_6p_topology_machine_B.Kv_n3[1] =
        sfc_6p_6p_topology_machine_P.Kv_Gain_ej *
        sfc_6p_6p_topology_machine_B.donotdeletethisgain_b;
      sfc_6p_6p_topology_machine_B.Kv_n3[2] =
        sfc_6p_6p_topology_machine_P.Kv_Gain_ej *
        sfc_6p_6p_topology_machine_B.donotdeletethisgain_n2;

      /* Gain: '<S115>/do not delete this gain' */
      sfc_6p_6p_topology_machine_B.donotdeletethisgain_de =
        sfc_6p_6p_topology_machine_P.donotdeletethisgain_Gain_fx *
        sfc_6p_6p_topology_machine_B.StateSpace_o1[14];

      /* Gain: '<S116>/do not delete this gain' */
      sfc_6p_6p_topology_machine_B.donotdeletethisgain_kv =
        sfc_6p_6p_topology_machine_P.donotdeletethisgain_Gain_ka *
        sfc_6p_6p_topology_machine_B.StateSpace_o1[15];

      /* Gain: '<S117>/do not delete this gain' */
      sfc_6p_6p_topology_machine_B.donotdeletethisgain_g0 =
        sfc_6p_6p_topology_machine_P.donotdeletethisgain_Gain_fm *
        sfc_6p_6p_topology_machine_B.StateSpace_o1[16];

      /* Gain: '<S19>/Kv1' */
      sfc_6p_6p_topology_machine_B.Kv1_l[0] =
        sfc_6p_6p_topology_machine_P.Kv1_Gain_k *
        sfc_6p_6p_topology_machine_B.donotdeletethisgain_de;
      sfc_6p_6p_topology_machine_B.Kv1_l[1] =
        sfc_6p_6p_topology_machine_P.Kv1_Gain_k *
        sfc_6p_6p_topology_machine_B.donotdeletethisgain_kv;
      sfc_6p_6p_topology_machine_B.Kv1_l[2] =
        sfc_6p_6p_topology_machine_P.Kv1_Gain_k *
        sfc_6p_6p_topology_machine_B.donotdeletethisgain_g0;

      /* Outport: '<Root>/Uph_n_source' incorporates:
       *  Inport: '<Root>/grid_pt_ratio'
       *  Product: '<S14>/Divide'
       */
      sfc_6p_6p_topology_machine_Y.Uph_n_source[0] =
        sfc_6p_6p_topology_machine_B.Kv1_j[0] /
        sfc_6p_6p_topology_machine_U.grid_pt_ratio;

      /* Outport: '<Root>/Iph_source' incorporates:
       *  Inport: '<Root>/grid_ct_ratio'
       *  Product: '<S11>/Divide'
       */
      sfc_6p_6p_topology_machine_Y.Iph_source[0] =
        sfc_6p_6p_topology_machine_B.Kv_n[0] /
        sfc_6p_6p_topology_machine_U.grid_ct_ratio;

      /* Outport: '<Root>/Iph_motor' incorporates:
       *  Inport: '<Root>/motor_ct_ratio'
       *  Product: '<S12>/Divide'
       */
      sfc_6p_6p_topology_machine_Y.Iph_motor[0] =
        sfc_6p_6p_topology_machine_B.Kv_n3[0] /
        sfc_6p_6p_topology_machine_U.motor_ct_ratio;

      /* Outport: '<Root>/Uph_ph_motor' incorporates:
       *  Inport: '<Root>/motor_pt_ratio'
       *  Product: '<S15>/Divide'
       */
      sfc_6p_6p_topology_machine_Y.Uph_ph_motor[0] =
        sfc_6p_6p_topology_machine_B.Kv1_l[0] /
        sfc_6p_6p_topology_machine_U.motor_pt_ratio;

      /* Outport: '<Root>/Uph_n_source' incorporates:
       *  Inport: '<Root>/grid_pt_ratio'
       *  Product: '<S14>/Divide'
       */
      sfc_6p_6p_topology_machine_Y.Uph_n_source[1] =
        sfc_6p_6p_topology_machine_B.Kv1_j[1] /
        sfc_6p_6p_topology_machine_U.grid_pt_ratio;

      /* Outport: '<Root>/Iph_source' incorporates:
       *  Inport: '<Root>/grid_ct_ratio'
       *  Product: '<S11>/Divide'
       */
      sfc_6p_6p_topology_machine_Y.Iph_source[1] =
        sfc_6p_6p_topology_machine_B.Kv_n[1] /
        sfc_6p_6p_topology_machine_U.grid_ct_ratio;

      /* Outport: '<Root>/Iph_motor' incorporates:
       *  Inport: '<Root>/motor_ct_ratio'
       *  Product: '<S12>/Divide'
       */
      sfc_6p_6p_topology_machine_Y.Iph_motor[1] =
        sfc_6p_6p_topology_machine_B.Kv_n3[1] /
        sfc_6p_6p_topology_machine_U.motor_ct_ratio;

      /* Outport: '<Root>/Uph_ph_motor' incorporates:
       *  Inport: '<Root>/motor_pt_ratio'
       *  Product: '<S15>/Divide'
       */
      sfc_6p_6p_topology_machine_Y.Uph_ph_motor[1] =
        sfc_6p_6p_topology_machine_B.Kv1_l[1] /
        sfc_6p_6p_topology_machine_U.motor_pt_ratio;

      /* Outport: '<Root>/Uph_n_source' incorporates:
       *  Inport: '<Root>/grid_pt_ratio'
       *  Product: '<S14>/Divide'
       */
      sfc_6p_6p_topology_machine_Y.Uph_n_source[2] =
        sfc_6p_6p_topology_machine_B.Kv1_j[2] /
        sfc_6p_6p_topology_machine_U.grid_pt_ratio;

      /* Outport: '<Root>/Iph_source' incorporates:
       *  Inport: '<Root>/grid_ct_ratio'
       *  Product: '<S11>/Divide'
       */
      sfc_6p_6p_topology_machine_Y.Iph_source[2] =
        sfc_6p_6p_topology_machine_B.Kv_n[2] /
        sfc_6p_6p_topology_machine_U.grid_ct_ratio;

      /* Outport: '<Root>/Iph_motor' incorporates:
       *  Inport: '<Root>/motor_ct_ratio'
       *  Product: '<S12>/Divide'
       */
      sfc_6p_6p_topology_machine_Y.Iph_motor[2] =
        sfc_6p_6p_topology_machine_B.Kv_n3[2] /
        sfc_6p_6p_topology_machine_U.motor_ct_ratio;

      /* Outport: '<Root>/Uph_ph_motor' incorporates:
       *  Inport: '<Root>/motor_pt_ratio'
       *  Product: '<S15>/Divide'
       */
      sfc_6p_6p_topology_machine_Y.Uph_ph_motor[2] =
        sfc_6p_6p_topology_machine_B.Kv1_l[2] /
        sfc_6p_6p_topology_machine_U.motor_pt_ratio;

      /* UnitDelay: '<S104>/dw_delay' */
      sfc_6p_6p_topology_machine_B.dw_delay =
        sfc_6p_6p_topology_machine_DW.dw_delay_DSTATE;

      /* Gain: '<S104>/F2' */
      sfc_6p_6p_topology_machine_B.F2 = sfc_6p_6p_topology_machine_P.F2_Gain *
        sfc_6p_6p_topology_machine_B.dw_delay;

      /* UnitDelay: '<S104>/dw_predict' */
      sfc_6p_6p_topology_machine_B.dw_predict =
        sfc_6p_6p_topology_machine_DW.dw_predict_DSTATE;

      /* Sum: '<S104>/Sum1' */
      sfc_6p_6p_topology_machine_B.Sum1_h = sfc_6p_6p_topology_machine_B.F2 -
        sfc_6p_6p_topology_machine_B.dw_predict;

      /* Sum: '<S67>/Sum' incorporates:
       *  Constant: '<S67>/nominal speed'
       */
      sfc_6p_6p_topology_machine_B.n =
        sfc_6p_6p_topology_machine_P.nominalspeed_Value +
        sfc_6p_6p_topology_machine_B.Sum1_h;

      /* Switch: '<S67>/Switch' incorporates:
       *  Inport: '<Root>/enable_forced_speed'
       */
      if (sfc_6p_6p_topology_machine_U.enable_forced_speed >
          sfc_6p_6p_topology_machine_P.Switch_Threshold_k) {
        /* Switch: '<S67>/Switch' incorporates:
         *  Inport: '<Root>/forced_speed_pu'
         */
        sfc_6p_6p_topology_machine_B.Switch_e =
          sfc_6p_6p_topology_machine_U.forced_speed_pu;
      } else {
        /* Switch: '<S67>/Switch' */
        sfc_6p_6p_topology_machine_B.Switch_e = sfc_6p_6p_topology_machine_B.n;
      }

      /* End of Switch: '<S67>/Switch' */

      /* Outport: '<Root>/machine_speed' incorporates:
       *  Gain: '<S67>/units'
       */
      sfc_6p_6p_topology_machine_Y.machine_speed =
        sfc_6p_6p_topology_machine_P.units_Gain *
        sfc_6p_6p_topology_machine_B.Switch_e;

      /* Gain: '<S76>/1-1' */
      sfc_6p_6p_topology_machine_B.iqsids[0] =
        sfc_6p_6p_topology_machine_P.u1_Gain[0] *
        sfc_6p_6p_topology_machine_B.changeIqIdcurrentsigns[0];
      sfc_6p_6p_topology_machine_B.iqsids[1] =
        sfc_6p_6p_topology_machine_P.u1_Gain[1] *
        sfc_6p_6p_topology_machine_B.changeIqIdcurrentsigns[1];

      /* Product: '<S76>/Mult1' */
      sfc_6p_6p_topology_machine_B.Mult1[0] =
        sfc_6p_6p_topology_machine_B.iqsids[0] *
        sfc_6p_6p_topology_machine_B.xk1[1];
      sfc_6p_6p_topology_machine_B.Mult1[1] = sfc_6p_6p_topology_machine_B.xk1[0]
        * sfc_6p_6p_topology_machine_B.iqsids[1];

      /* DiscreteIntegrator: '<S67>/theta' */
      sfc_6p_6p_topology_machine_B.theta =
        sfc_6p_6p_topology_machine_DW.theta_DSTATE;

      /* Fcn: '<S67>/Fcn' */
      tmpForInput_idx_2 = sfc_6p_6p_topology_machine_B.theta;

      /* Signum: '<S5>/Sign' incorporates:
       *  Outport: '<Root>/machine_speed'
       */
      tmpForInput_idx_0_0 = sfc_6p_6p_topology_machine_Y.machine_speed;
      if (rtIsNaN(tmpForInput_idx_0_0)) {
        /* Signum: '<S5>/Sign' */
        sfc_6p_6p_topology_machine_B.Sign = (rtNaN);
      } else if (tmpForInput_idx_0_0 < 0.0) {
        /* Signum: '<S5>/Sign' */
        sfc_6p_6p_topology_machine_B.Sign = -1.0;
      } else {
        /* Signum: '<S5>/Sign' */
        sfc_6p_6p_topology_machine_B.Sign = (tmpForInput_idx_0_0 > 0.0);
      }

      /* End of Signum: '<S5>/Sign' */

      /* Abs: '<S5>/Abs' incorporates:
       *  Outport: '<Root>/machine_speed'
       */
      sfc_6p_6p_topology_machine_B.Abs = std::abs
        (sfc_6p_6p_topology_machine_Y.machine_speed);

      /* Product: '<S5>/Product' incorporates:
       *  Inport: '<Root>/load_coefficients'
       */
      sfc_6p_6p_topology_machine_B.Product = sfc_6p_6p_topology_machine_B.Abs *
        sfc_6p_6p_topology_machine_U.load_coefficients[2];

      /* Sum: '<S5>/Sum' incorporates:
       *  Inport: '<Root>/load_coefficients'
       */
      sfc_6p_6p_topology_machine_B.Sum_c =
        sfc_6p_6p_topology_machine_U.load_coefficients[1] +
        sfc_6p_6p_topology_machine_B.Product;

      /* Product: '<S5>/Product1' */
      sfc_6p_6p_topology_machine_B.Product1 = sfc_6p_6p_topology_machine_B.Abs *
        sfc_6p_6p_topology_machine_B.Sum_c;

      /* Sum: '<S5>/Sum1' incorporates:
       *  Inport: '<Root>/load_coefficients'
       */
      sfc_6p_6p_topology_machine_B.Sum1_l =
        sfc_6p_6p_topology_machine_U.load_coefficients[0] +
        sfc_6p_6p_topology_machine_B.Product1;

      /* Outport: '<Root>/machine_load' incorporates:
       *  Product: '<S5>/Product2'
       */
      sfc_6p_6p_topology_machine_Y.machine_load =
        sfc_6p_6p_topology_machine_B.Sign * sfc_6p_6p_topology_machine_B.Sum1_l;

      /* Switch: '<S68>/Switch' incorporates:
       *  Constant: '<S68>/Constant1'
       */
      if (sfc_6p_6p_topology_machine_P.Constant1_Value_cr) {
        /* Switch: '<S68>/Switch' */
        sfc_6p_6p_topology_machine_B.ikq2 =
          sfc_6p_6p_topology_machine_B.changeIqIdcurrentsigns[5];
      } else {
        /* Switch: '<S68>/Switch' incorporates:
         *  Constant: '<S68>/Constant2'
         */
        sfc_6p_6p_topology_machine_B.ikq2 =
          sfc_6p_6p_topology_machine_P.Constant2_Value;
      }

      /* End of Switch: '<S68>/Switch' */

      /* Gain: '<S78>/1//Ll_q' */
      sfc_6p_6p_topology_machine_B.uLl_q[0] =
        sfc_6p_6p_topology_machine_P.uLl_q_Gain_h[0] *
        sfc_6p_6p_topology_machine_B.xk1[0];

      /* Sum: '<S76>/Sum2' */
      tmpForInput_idx_0_0 = sfc_6p_6p_topology_machine_B.Mult1[0];

      /* Gain: '<S78>/1//Ll_q' */
      sfc_6p_6p_topology_machine_B.uLl_q[1] =
        sfc_6p_6p_topology_machine_P.uLl_q_Gain_h[1] *
        sfc_6p_6p_topology_machine_B.xk1[4];

      /* Sum: '<S76>/Sum2' */
      tmpForInput_idx_0_0 += sfc_6p_6p_topology_machine_B.Mult1[1];

      /* Gain: '<S78>/1//Ll_q' */
      sfc_6p_6p_topology_machine_B.uLl_q[2] =
        sfc_6p_6p_topology_machine_P.uLl_q_Gain_h[2] *
        sfc_6p_6p_topology_machine_B.xk1[5];

      /* Sum: '<S76>/Sum2' */
      sfc_6p_6p_topology_machine_B.Sum2 = tmpForInput_idx_0_0;

      /* Outport: '<Root>/machine_electrical_torque' incorporates:
       *  Gain: '<S67>/units2'
       */
      sfc_6p_6p_topology_machine_Y.machine_electrical_torque =
        sfc_6p_6p_topology_machine_P.units2_Gain *
        sfc_6p_6p_topology_machine_B.Sum2;

      /* Fcn: '<S67>/Fcn' */
      tmpForInput_idx_0_0 = rt_remd_snf(tmpForInput_idx_2, 6.2831853071795862);

      /* Fcn: '<S67>/Fcn' */
      sfc_6p_6p_topology_machine_B.Fcn = tmpForInput_idx_0_0;

      /* Outport: '<Root>/machine_mechanical_angle' incorporates:
       *  Gain: '<S67>/t'
       */
      sfc_6p_6p_topology_machine_Y.machine_mechanical_angle =
        sfc_6p_6p_topology_machine_P.t_Gain * sfc_6p_6p_topology_machine_B.Fcn;

      /* Product: '<S67>/Product' */
      sfc_6p_6p_topology_machine_B.Pe = sfc_6p_6p_topology_machine_B.Switch_e *
        sfc_6p_6p_topology_machine_B.Sum2;

      /* Outport: '<Root>/machine_electrical_power' incorporates:
       *  Gain: '<S67>/units1'
       */
      sfc_6p_6p_topology_machine_Y.machine_electrical_power =
        sfc_6p_6p_topology_machine_P.units1_Gain *
        sfc_6p_6p_topology_machine_B.Pe;

      /* Switch: '<S69>/Switch3' incorporates:
       *  Constant: '<S69>/Constant8'
       */
      if (sfc_6p_6p_topology_machine_P.Constant8_Value >=
          sfc_6p_6p_topology_machine_P.Switch3_Threshold) {
        /* Switch: '<S69>/Switch3' */
        sfc_6p_6p_topology_machine_B.Switch3[0] =
          sfc_6p_6p_topology_machine_B.Switch2_a;
        sfc_6p_6p_topology_machine_B.Switch3[1] =
          sfc_6p_6p_topology_machine_B.MathFunction3;
      } else {
        /* Switch: '<S69>/Switch3' incorporates:
         *  Constant: '<S69>/Laqd_nosat'
         */
        sfc_6p_6p_topology_machine_B.Switch3[0] =
          sfc_6p_6p_topology_machine_P.Laqd_nosat_Value[0];
        sfc_6p_6p_topology_machine_B.Switch3[1] =
          sfc_6p_6p_topology_machine_P.Laqd_nosat_Value[1];
      }

      /* End of Switch: '<S69>/Switch3' */

      /* Product: '<S78>/Product' */
      sfc_6p_6p_topology_machine_B.Product_k[0] =
        sfc_6p_6p_topology_machine_B.uLl_q[0] *
        sfc_6p_6p_topology_machine_B.Switch3[0];
      sfc_6p_6p_topology_machine_B.Product_k[1] =
        sfc_6p_6p_topology_machine_B.Switch3[0] *
        sfc_6p_6p_topology_machine_B.uLl_q[1];
      sfc_6p_6p_topology_machine_B.Product_k[2] =
        sfc_6p_6p_topology_machine_B.Switch3[0] *
        sfc_6p_6p_topology_machine_B.uLl_q[2];

      /* Sum: '<S78>/sum2' */
      tmpForInput_idx_0_0 = sfc_6p_6p_topology_machine_B.Product_k[0];

      /* Gain: '<S78>/1//Ll_d' */
      tmpForInput_idx_1 = sfc_6p_6p_topology_machine_P.uLl_d_Gain_p[0] *
        sfc_6p_6p_topology_machine_B.xk1[1];
      sfc_6p_6p_topology_machine_B.uLl_d[0] = tmpForInput_idx_1;

      /* Product: '<S78>/Product1' */
      sfc_6p_6p_topology_machine_B.Product1_h[0] = tmpForInput_idx_1 *
        sfc_6p_6p_topology_machine_B.Switch3[1];

      /* Sum: '<S78>/sum2' */
      tmpForInput_idx_0_0 += sfc_6p_6p_topology_machine_B.Product_k[1];

      /* Gain: '<S78>/1//Ll_d' */
      tmpForInput_idx_1 = sfc_6p_6p_topology_machine_P.uLl_d_Gain_p[1] *
        sfc_6p_6p_topology_machine_B.xk1[2];
      sfc_6p_6p_topology_machine_B.uLl_d[1] = tmpForInput_idx_1;

      /* Product: '<S78>/Product1' */
      sfc_6p_6p_topology_machine_B.Product1_h[1] = tmpForInput_idx_1 *
        sfc_6p_6p_topology_machine_B.Switch3[1];

      /* Sum: '<S78>/sum2' */
      tmpForInput_idx_0_0 += sfc_6p_6p_topology_machine_B.Product_k[2];

      /* Gain: '<S78>/1//Ll_d' */
      tmpForInput_idx_1 = sfc_6p_6p_topology_machine_P.uLl_d_Gain_p[2] *
        sfc_6p_6p_topology_machine_B.xk1[3];
      sfc_6p_6p_topology_machine_B.uLl_d[2] = tmpForInput_idx_1;

      /* Product: '<S78>/Product1' */
      sfc_6p_6p_topology_machine_B.Product1_h[2] = tmpForInput_idx_1 *
        sfc_6p_6p_topology_machine_B.Switch3[1];

      /* Sum: '<S78>/sum2' */
      sfc_6p_6p_topology_machine_B.phimq = tmpForInput_idx_0_0;

      /* Sum: '<S78>/sum1' */
      tmpForInput_idx_0_0 = sfc_6p_6p_topology_machine_B.Product1_h[0];
      tmpForInput_idx_0_0 += sfc_6p_6p_topology_machine_B.Product1_h[1];
      tmpForInput_idx_0_0 += sfc_6p_6p_topology_machine_B.Product1_h[2];

      /* Sum: '<S78>/sum1' */
      sfc_6p_6p_topology_machine_B.phimd = tmpForInput_idx_0_0;

      /* Gain: '<S70>/1_Vb' */
      sfc_6p_6p_topology_machine_B.u_Vb[0] =
        sfc_6p_6p_topology_machine_P.u_Vb_Gain *
        sfc_6p_6p_topology_machine_B.StateSpace_o1[12];
      sfc_6p_6p_topology_machine_B.u_Vb[1] =
        sfc_6p_6p_topology_machine_P.u_Vb_Gain *
        sfc_6p_6p_topology_machine_B.StateSpace_o1[13];

      /* Sum: '<S70>/Sum' */
      sfc_6p_6p_topology_machine_B.Sum_h = sfc_6p_6p_topology_machine_B.u_Vb[1]
        - sfc_6p_6p_topology_machine_B.u_Vb[0];

      /* Gain: '<S70>/Gain' */
      sfc_6p_6p_topology_machine_B.B = sfc_6p_6p_topology_machine_P.Gain_Gain *
        sfc_6p_6p_topology_machine_B.Sum_h;

      /* Sum: '<S70>/Sum1' */
      sfc_6p_6p_topology_machine_B.A = sfc_6p_6p_topology_machine_B.B +
        sfc_6p_6p_topology_machine_B.u_Vb[0];

      /* Sum: '<S70>/Sum2' */
      sfc_6p_6p_topology_machine_B.C = (0.0 - sfc_6p_6p_topology_machine_B.B) -
        sfc_6p_6p_topology_machine_B.A;

      /* SignalConversion generated from: '<S91>/Gain3' */
      sfc_6p_6p_topology_machine_B.TmpSignalConversionAtGain3Inp_a[0] =
        sfc_6p_6p_topology_machine_B.A;
      sfc_6p_6p_topology_machine_B.TmpSignalConversionAtGain3Inp_a[1] =
        sfc_6p_6p_topology_machine_B.B;
      sfc_6p_6p_topology_machine_B.TmpSignalConversionAtGain3Inp_a[2] =
        sfc_6p_6p_topology_machine_B.C;

      /* Gain: '<S91>/Gain3' */
      tmpForInput_idx_0_0 =
        sfc_6p_6p_topology_machine_B.TmpSignalConversionAtGain3Inp_a[0];
      tmpForInput_idx_1 =
        sfc_6p_6p_topology_machine_B.TmpSignalConversionAtGain3Inp_a[1];
      tmpForInput_idx_2 =
        sfc_6p_6p_topology_machine_B.TmpSignalConversionAtGain3Inp_a[2];
      for (i = 0; i < 3; i++) {
        /* Gain: '<S91>/Gain3' */
        tmpForInput_idx_0 = sfc_6p_6p_topology_machine_P.Gain3_Gain_o[i] *
          tmpForInput_idx_0_0;
        tmpForInput_idx_0 += sfc_6p_6p_topology_machine_P.Gain3_Gain_o[i + 3] *
          tmpForInput_idx_1;
        tmpForInput_idx_0 += sfc_6p_6p_topology_machine_P.Gain3_Gain_o[i + 6] *
          tmpForInput_idx_2;
        sfc_6p_6p_topology_machine_B.Gain3_d[i] = tmpForInput_idx_0;

        /* Gain: '<S91>/Gain1' */
        sfc_6p_6p_topology_machine_B.Gain1_p[i] =
          sfc_6p_6p_topology_machine_P.Gain1_Gain_gd * tmpForInput_idx_0;
      }

      /* RelationalOperator: '<S92>/Compare' incorporates:
       *  Constant: '<S90>/Constant'
       *  Constant: '<S92>/Constant'
       */
      sfc_6p_6p_topology_machine_B.Compare_l =
        (sfc_6p_6p_topology_machine_P.AlphaBetaZerotodq0_Alignment_b ==
         sfc_6p_6p_topology_machine_P.CompareToConstant_const_d);

      /* Outputs for Enabled SubSystem: '<S90>/Subsystem1' incorporates:
       *  EnablePort: '<S95>/Enable'
       */
      if (sfc_6p_6p_topology_machine_B.Compare_l > 0) {
        /* Fcn: '<S95>/Fcn' */
        sfc_6p_6p_topology_machine_B.Fcn_b =
          sfc_6p_6p_topology_machine_B.Gain1_p[0] * std::cos
          (sfc_6p_6p_topology_machine_B.Sum3) +
          sfc_6p_6p_topology_machine_B.Gain1_p[1] * std::sin
          (sfc_6p_6p_topology_machine_B.Sum3);

        /* Fcn: '<S95>/Fcn1' */
        sfc_6p_6p_topology_machine_B.Fcn1_j =
          -sfc_6p_6p_topology_machine_B.Gain1_p[0] * std::sin
          (sfc_6p_6p_topology_machine_B.Sum3) +
          sfc_6p_6p_topology_machine_B.Gain1_p[1] * std::cos
          (sfc_6p_6p_topology_machine_B.Sum3);
      }

      /* End of Outputs for SubSystem: '<S90>/Subsystem1' */

      /* RelationalOperator: '<S93>/Compare' incorporates:
       *  Constant: '<S90>/Constant'
       *  Constant: '<S93>/Constant'
       */
      sfc_6p_6p_topology_machine_B.Compare_ld =
        (sfc_6p_6p_topology_machine_P.AlphaBetaZerotodq0_Alignment_b ==
         sfc_6p_6p_topology_machine_P.CompareToConstant1_const_o);

      /* Outputs for Enabled SubSystem: '<S90>/Subsystem - pi//2 delay' incorporates:
       *  EnablePort: '<S94>/Enable'
       */
      if (sfc_6p_6p_topology_machine_B.Compare_ld > 0) {
        /* Fcn: '<S94>/Fcn' */
        sfc_6p_6p_topology_machine_B.Fcn_bn =
          sfc_6p_6p_topology_machine_B.Gain1_p[0] * std::sin
          (sfc_6p_6p_topology_machine_B.Sum3) -
          sfc_6p_6p_topology_machine_B.Gain1_p[1] * std::cos
          (sfc_6p_6p_topology_machine_B.Sum3);

        /* Fcn: '<S94>/Fcn1' */
        sfc_6p_6p_topology_machine_B.Fcn1_p =
          sfc_6p_6p_topology_machine_B.Gain1_p[0] * std::cos
          (sfc_6p_6p_topology_machine_B.Sum3) +
          sfc_6p_6p_topology_machine_B.Gain1_p[1] * std::sin
          (sfc_6p_6p_topology_machine_B.Sum3);
      }

      /* End of Outputs for SubSystem: '<S90>/Subsystem - pi//2 delay' */

      /* Switch: '<S90>/Switch' */
      if (sfc_6p_6p_topology_machine_B.Compare_l != 0) {
        /* Switch: '<S90>/Switch' */
        sfc_6p_6p_topology_machine_B.Switch_i[0] =
          sfc_6p_6p_topology_machine_B.Fcn_b;
        sfc_6p_6p_topology_machine_B.Switch_i[1] =
          sfc_6p_6p_topology_machine_B.Fcn1_j;
      } else {
        /* Switch: '<S90>/Switch' */
        sfc_6p_6p_topology_machine_B.Switch_i[0] =
          sfc_6p_6p_topology_machine_B.Fcn_bn;
        sfc_6p_6p_topology_machine_B.Switch_i[1] =
          sfc_6p_6p_topology_machine_B.Fcn1_p;
      }

      /* End of Switch: '<S90>/Switch' */

      /* Switch: '<S69>/Switch2' incorporates:
       *  Constant: '<S69>/Constant5'
       */
      if (sfc_6p_6p_topology_machine_P.Constant5_Value_c >=
          sfc_6p_6p_topology_machine_P.Switch2_Threshold) {
        /* Switch: '<S69>/Switch2' */
        sfc_6p_6p_topology_machine_B.Switch2[0] =
          sfc_6p_6p_topology_machine_B.Switch1_j;
        sfc_6p_6p_topology_machine_B.Switch2[1] =
          sfc_6p_6p_topology_machine_B.Lmsatd;
      } else {
        /* Switch: '<S69>/Switch2' incorporates:
         *  Constant: '<S69>/Lmqd_nosat'
         */
        sfc_6p_6p_topology_machine_B.Switch2[0] =
          sfc_6p_6p_topology_machine_P.Lmqd_nosat_Value[0];
        sfc_6p_6p_topology_machine_B.Switch2[1] =
          sfc_6p_6p_topology_machine_P.Lmqd_nosat_Value[1];
      }

      /* End of Switch: '<S69>/Switch2' */

      /* Fcn: '<S74>/x->theta' */
      sfc_6p_6p_topology_machine_B.xtheta = rt_atan2d_snf
        (sfc_6p_6p_topology_machine_B.Switch_i[0],
         sfc_6p_6p_topology_machine_B.Switch_i[1]);

      /* Gain: '<S75>/Gain' */
      sfc_6p_6p_topology_machine_B.Gain =
        sfc_6p_6p_topology_machine_P.Gain_Gain_d *
        sfc_6p_6p_topology_machine_B.xtheta;

      /* Product: '<S73>/Product2' */
      sfc_6p_6p_topology_machine_B.Product2_e =
        sfc_6p_6p_topology_machine_B.changeIqIdcurrentsigns[0] *
        sfc_6p_6p_topology_machine_B.Switch_i[1];

      /* Product: '<S73>/Product3' */
      sfc_6p_6p_topology_machine_B.Product3 =
        sfc_6p_6p_topology_machine_B.Switch_i[0] *
        sfc_6p_6p_topology_machine_B.changeIqIdcurrentsigns[1];

      /* Sum: '<S73>/Sum' */
      sfc_6p_6p_topology_machine_B.Peo = sfc_6p_6p_topology_machine_B.Product2_e
        + sfc_6p_6p_topology_machine_B.Product3;

      /* Gain: '<S73>/Gain' */
      sfc_6p_6p_topology_machine_B.Gain_d =
        sfc_6p_6p_topology_machine_P.Gain_Gain_m *
        sfc_6p_6p_topology_machine_B.Peo;

      /* Product: '<S73>/Product4' */
      sfc_6p_6p_topology_machine_B.Product4 =
        sfc_6p_6p_topology_machine_B.Switch_i[1] *
        sfc_6p_6p_topology_machine_B.changeIqIdcurrentsigns[1];

      /* Product: '<S73>/Product5' */
      sfc_6p_6p_topology_machine_B.Product5_d =
        sfc_6p_6p_topology_machine_B.Switch_i[0] *
        sfc_6p_6p_topology_machine_B.changeIqIdcurrentsigns[0];

      /* Sum: '<S73>/Sum1' */
      sfc_6p_6p_topology_machine_B.Qeo = sfc_6p_6p_topology_machine_B.Product4 -
        sfc_6p_6p_topology_machine_B.Product5_d;

      /* Gain: '<S73>/Gain1' */
      sfc_6p_6p_topology_machine_B.Gain1_j =
        sfc_6p_6p_topology_machine_P.Gain1_Gain_f *
        sfc_6p_6p_topology_machine_B.Qeo;

      /* Gain: '<S68>/output formatting' */
      sfc_6p_6p_topology_machine_B.outputformatting[0] =
        sfc_6p_6p_topology_machine_P.outputformatting_Gain[0] *
        sfc_6p_6p_topology_machine_B.Gain3[0];
      sfc_6p_6p_topology_machine_B.outputformatting[3] =
        sfc_6p_6p_topology_machine_B.changeIqIdcurrentsigns[0] *
        sfc_6p_6p_topology_machine_P.outputformatting_Gain[3];
      sfc_6p_6p_topology_machine_B.outputformatting[1] =
        sfc_6p_6p_topology_machine_P.outputformatting_Gain[1] *
        sfc_6p_6p_topology_machine_B.Gain3[1];
      sfc_6p_6p_topology_machine_B.outputformatting[4] =
        sfc_6p_6p_topology_machine_B.changeIqIdcurrentsigns[1] *
        sfc_6p_6p_topology_machine_P.outputformatting_Gain[4];
      sfc_6p_6p_topology_machine_B.outputformatting[2] =
        sfc_6p_6p_topology_machine_P.outputformatting_Gain[2] *
        sfc_6p_6p_topology_machine_B.Gain3[2];
      sfc_6p_6p_topology_machine_B.outputformatting[5] =
        sfc_6p_6p_topology_machine_B.changeIqIdcurrentsigns[2] *
        sfc_6p_6p_topology_machine_P.outputformatting_Gain[5];
      sfc_6p_6p_topology_machine_B.outputformatting[6] =
        sfc_6p_6p_topology_machine_B.changeIqIdcurrentsigns[4] *
        sfc_6p_6p_topology_machine_P.outputformatting_Gain[6];
      sfc_6p_6p_topology_machine_B.outputformatting[7] =
        sfc_6p_6p_topology_machine_P.outputformatting_Gain[7] *
        sfc_6p_6p_topology_machine_B.ikq2;
      sfc_6p_6p_topology_machine_B.outputformatting[8] =
        sfc_6p_6p_topology_machine_B.changeIqIdcurrentsigns[3] *
        sfc_6p_6p_topology_machine_P.outputformatting_Gain[8];
      sfc_6p_6p_topology_machine_B.outputformatting[9] =
        sfc_6p_6p_topology_machine_P.outputformatting_Gain[9] *
        sfc_6p_6p_topology_machine_B.phimq;
      sfc_6p_6p_topology_machine_B.outputformatting[10] =
        sfc_6p_6p_topology_machine_P.outputformatting_Gain[10] *
        sfc_6p_6p_topology_machine_B.phimd;
      sfc_6p_6p_topology_machine_B.outputformatting[11] =
        sfc_6p_6p_topology_machine_B.Switch_i[1] *
        sfc_6p_6p_topology_machine_P.outputformatting_Gain[11];
      sfc_6p_6p_topology_machine_B.outputformatting[12] =
        sfc_6p_6p_topology_machine_B.Switch_i[0] *
        sfc_6p_6p_topology_machine_P.outputformatting_Gain[12];
      sfc_6p_6p_topology_machine_B.outputformatting[13] =
        sfc_6p_6p_topology_machine_B.Switch2[0] *
        sfc_6p_6p_topology_machine_P.outputformatting_Gain[13];
      sfc_6p_6p_topology_machine_B.outputformatting[14] =
        sfc_6p_6p_topology_machine_B.Switch2[1] *
        sfc_6p_6p_topology_machine_P.outputformatting_Gain[14];
      sfc_6p_6p_topology_machine_B.outputformatting[15] =
        sfc_6p_6p_topology_machine_P.outputformatting_Gain[15] *
        sfc_6p_6p_topology_machine_B.Gain;
      sfc_6p_6p_topology_machine_B.outputformatting[16] =
        sfc_6p_6p_topology_machine_P.outputformatting_Gain[16] *
        sfc_6p_6p_topology_machine_B.Gain_d;
      sfc_6p_6p_topology_machine_B.outputformatting[17] =
        sfc_6p_6p_topology_machine_P.outputformatting_Gain[17] *
        sfc_6p_6p_topology_machine_B.Gain1_j;

      /* Outport: '<Root>/machine_flux_d' */
      sfc_6p_6p_topology_machine_Y.machine_flux_d =
        sfc_6p_6p_topology_machine_B.outputformatting[10];

      /* Outport: '<Root>/machine_flux_q' */
      sfc_6p_6p_topology_machine_Y.machine_flux_q =
        sfc_6p_6p_topology_machine_B.outputformatting[9];

      /* Outport: '<Root>/machine_stator_current_d' */
      sfc_6p_6p_topology_machine_Y.machine_stator_current_d =
        sfc_6p_6p_topology_machine_B.outputformatting[4];

      /* Outport: '<Root>/machine_stator_current_q' */
      sfc_6p_6p_topology_machine_Y.machine_stator_current_q =
        sfc_6p_6p_topology_machine_B.outputformatting[3];

      /* Outport: '<Root>/machine_field_current_d' */
      sfc_6p_6p_topology_machine_Y.machine_field_current_d =
        sfc_6p_6p_topology_machine_B.outputformatting[5];

      /* Outport: '<Root>/machine_damper_current_q1' */
      sfc_6p_6p_topology_machine_Y.machine_damper_current_q1 =
        sfc_6p_6p_topology_machine_B.outputformatting[6];

      /* Outport: '<Root>/machine_damper_current_q2' */
      sfc_6p_6p_topology_machine_Y.machine_damper_current_q2 =
        sfc_6p_6p_topology_machine_B.outputformatting[7];

      /* Outport: '<Root>/machine_damper_current_d' */
      sfc_6p_6p_topology_machine_Y.machine_damper_current_d =
        sfc_6p_6p_topology_machine_B.outputformatting[8];

      /* Outport: '<Root>/machine_stator_voltage_q' */
      sfc_6p_6p_topology_machine_Y.machine_stator_voltage_q =
        sfc_6p_6p_topology_machine_B.outputformatting[11];

      /* Outport: '<Root>/machine_stator_voltage_d' */
      sfc_6p_6p_topology_machine_Y.machine_stator_voltage_d =
        sfc_6p_6p_topology_machine_B.outputformatting[12];

      /* Outport: '<Root>/machine_phase_current_a' */
      sfc_6p_6p_topology_machine_Y.machine_phase_current_a =
        sfc_6p_6p_topology_machine_B.outputformatting[0];

      /* Outport: '<Root>/machine_phase_current_b' */
      sfc_6p_6p_topology_machine_Y.machine_phase_current_b =
        sfc_6p_6p_topology_machine_B.outputformatting[1];

      /* Outport: '<Root>/machine_phase_current_c' */
      sfc_6p_6p_topology_machine_Y.machine_phase_current_c =
        sfc_6p_6p_topology_machine_B.outputformatting[2];

      /* Gain: '<S65>/N' incorporates:
       *  Inport: '<Root>/voltage_excitation'
       */
      sfc_6p_6p_topology_machine_B.Vfd = sfc_6p_6p_topology_machine_P.N_Gain *
        sfc_6p_6p_topology_machine_U.voltage_excitation;
      for (i = 0; i < 6; i++) {
        /* UnitDelay: '<S80>/voltages' */
        sfc_6p_6p_topology_machine_B.voltages[i] =
          sfc_6p_6p_topology_machine_DW.voltages_DSTATE[i];
      }

      /* DigitalClock: '<S80>/Digital Clock' */
      sfc_6p_6p_topology_machine_B.DigitalClock_g =
        ((((&sfc_6p_6p_topology_machine_M)->Timing.clockTick1+
           (&sfc_6p_6p_topology_machine_M)->Timing.clockTickH1* 4294967296.0)) *
         1.25E-5);

      /* Switch: '<S80>/IC' incorporates:
       *  Constant: '<S69>/Constant2'
       *  Switch: '<S69>/Switch1'
       */
      if (sfc_6p_6p_topology_machine_B.DigitalClock_g >=
          sfc_6p_6p_topology_machine_P.IC_Threshold) {
        /* Sum: '<S80>/sum' incorporates:
         *  Constant: '<S65>/[ Vkd =0 Vkq1=0  Vkq2=0 ]'
         */
        sfc_6p_6p_topology_machine_B.ukuk1[0] =
          sfc_6p_6p_topology_machine_B.voltages[0] +
          sfc_6p_6p_topology_machine_B.Switch_i[1];
        sfc_6p_6p_topology_machine_B.ukuk1[1] =
          sfc_6p_6p_topology_machine_B.Switch_i[0] +
          sfc_6p_6p_topology_machine_B.voltages[1];
        sfc_6p_6p_topology_machine_B.ukuk1[2] = sfc_6p_6p_topology_machine_B.Vfd
          + sfc_6p_6p_topology_machine_B.voltages[2];
        sfc_6p_6p_topology_machine_B.ukuk1[3] =
          sfc_6p_6p_topology_machine_P.Vkd0Vkq10Vkq20_Value[0] +
          sfc_6p_6p_topology_machine_B.voltages[3];
        sfc_6p_6p_topology_machine_B.ukuk1[4] =
          sfc_6p_6p_topology_machine_P.Vkd0Vkq10Vkq20_Value[1] +
          sfc_6p_6p_topology_machine_B.voltages[4];
        sfc_6p_6p_topology_machine_B.ukuk1[5] =
          sfc_6p_6p_topology_machine_P.Vkd0Vkq10Vkq20_Value[2] +
          sfc_6p_6p_topology_machine_B.voltages[5];
        if (sfc_6p_6p_topology_machine_P.Constant2_Value_i >=
            sfc_6p_6p_topology_machine_P.Switch1_Threshold) {
          /* Switch: '<S69>/Switch1' incorporates:
           *  Product: '<S79>/Product1'
           */
          std::memcpy(&sfc_6p_6p_topology_machine_B.Switch1[0],
                      &sfc_6p_6p_topology_machine_B.RLinv[0], 36U * sizeof
                      (real_T));
        } else {
          /* Switch: '<S69>/Switch1' incorporates:
           *  Constant: '<S69>/Constant6'
           */
          std::memcpy(&sfc_6p_6p_topology_machine_B.Switch1[0],
                      &sfc_6p_6p_topology_machine_P.Constant6_Value[0], 36U *
                      sizeof(real_T));
        }

        for (i = 0; i < 36; i++) {
          /* Assignment: '<S77>/W(1,2)=wr' incorporates:
           *  Constant: '<S77>/u1'
           */
          sfc_6p_6p_topology_machine_B.W12wr[i] =
            sfc_6p_6p_topology_machine_P.u1_Value_p[i];
        }

        /* Assignment: '<S77>/W(1,2)=wr' incorporates:
         *  Constant: '<S69>/Constant2'
         *  Switch: '<S69>/Switch1'
         */
        sfc_6p_6p_topology_machine_B.W12wr[6] =
          sfc_6p_6p_topology_machine_B.Switch_e;

        /* Gain: '<S77>/Gain1' */
        sfc_6p_6p_topology_machine_B.Gain1_e =
          sfc_6p_6p_topology_machine_P.Gain1_Gain_g *
          sfc_6p_6p_topology_machine_B.Switch_e;

        /* Assignment: '<S77>/W(2,1)=-wr' incorporates:
         *  Assignment: '<S77>/W(1,2)=wr'
         */
        std::memcpy(&sfc_6p_6p_topology_machine_B.W21wr[0],
                    &sfc_6p_6p_topology_machine_B.W12wr[0], 36U * sizeof(real_T));

        /* Assignment: '<S77>/W(2,1)=-wr' */
        sfc_6p_6p_topology_machine_B.W21wr[1] =
          sfc_6p_6p_topology_machine_B.Gain1_e;
        for (i = 0; i < 36; i++) {
          /* Sum: '<S69>/Sum1' incorporates:
           *  Assignment: '<S77>/W(2,1)=-wr'
           *  Switch: '<S69>/Switch1'
           */
          tmpForInput_idx_0_0 = (0.0 - sfc_6p_6p_topology_machine_B.W21wr[i]) -
            sfc_6p_6p_topology_machine_B.Switch1[i];
          sfc_6p_6p_topology_machine_B.A_m[i] = tmpForInput_idx_0_0;

          /* Gain: '<S88>/wbase*Ts//2' incorporates:
           *  Sum: '<S69>/Sum1'
           */
          tmpForInput_idx_0_0 *= sfc_6p_6p_topology_machine_P.wbaseTs2_Gain;
          sfc_6p_6p_topology_machine_B.wbaseTs2[i] = tmpForInput_idx_0_0;

          /* Sum: '<S88>/Sum1' incorporates:
           *  Constant: '<S88>/u5'
           *  Gain: '<S88>/wbase*Ts//2'
           */
          sfc_6p_6p_topology_machine_B.Sum1_b[i] =
            sfc_6p_6p_topology_machine_P.u5_Value_o[i] - tmpForInput_idx_0_0;
        }

        /* Product: '<S88>/inversion' incorporates:
         *  Sum: '<S88>/Sum1'
         */
        rt_invd6x6_snf(sfc_6p_6p_topology_machine_B.Sum1_b,
                       sfc_6p_6p_topology_machine_B.inversion);
        for (i = 0; i < 36; i++) {
          /* Gain: '<S88>/wbase*Ts//2 ' incorporates:
           *  Product: '<S88>/inversion'
           */
          tmpForInput_idx_0_0 = sfc_6p_6p_topology_machine_P.wbaseTs2_Gain_g *
            sfc_6p_6p_topology_machine_B.inversion[i];
          sfc_6p_6p_topology_machine_B.wbaseTs2_c[i] = tmpForInput_idx_0_0;

          /* Product: '<S80>/Product1' incorporates:
           *  Gain: '<S88>/wbase*Ts//2 '
           */
          tmp_0[i] = tmpForInput_idx_0_0;
        }

        /* Product: '<S80>/Product1' */
        for (i = 0; i < 6; i++) {
          tmp[i] = sfc_6p_6p_topology_machine_B.ukuk1[i];
        }

        for (i_0 = 0; i_0 < 6; i_0++) {
          tmpForInput_idx_0_0 = 0.0;
          for (i = 0; i < 6; i++) {
            tmpForInput_idx_0_0 += tmp_0[6 * i + i_0] * tmp[i];
          }

          /* Product: '<S80>/Product1' */
          sfc_6p_6p_topology_machine_B.Product1_b[i_0] = tmpForInput_idx_0_0;
        }

        for (i = 0; i < 36; i++) {
          /* Sum: '<S88>/Sum5' incorporates:
           *  Constant: '<S88>/u5'
           *  Gain: '<S88>/wbase*Ts//2'
           */
          tmpForInput_idx_0_0 = sfc_6p_6p_topology_machine_P.u5_Value_o[i] +
            sfc_6p_6p_topology_machine_B.wbaseTs2[i];
          sfc_6p_6p_topology_machine_B.Sum5[i] = tmpForInput_idx_0_0;

          /* Product: '<S88>/Product4' incorporates:
           *  Product: '<S88>/inversion'
           *  Sum: '<S88>/Sum5'
           */
          tmp_0[i] = sfc_6p_6p_topology_machine_B.inversion[i];
          tmp_1[i] = tmpForInput_idx_0_0;
        }

        for (i_0 = 0; i_0 < 6; i_0++) {
          for (i = 0; i < 6; i++) {
            /* Product: '<S88>/Product4' */
            tmpForInput_idx_0_0 = 0.0;
            for (i_1 = 0; i_1 < 6; i_1++) {
              /* Product: '<S88>/Product4' */
              tmpForInput_idx_0_0 += tmp_0[6 * i_1 + i] * tmp_1[6 * i_0 + i_1];
            }

            /* Product: '<S88>/Product4' */
            sfc_6p_6p_topology_machine_B.Product4_e[i + 6 * i_0] =
              tmpForInput_idx_0_0;
          }
        }

        /* Product: '<S80>/Product2' incorporates:
         *  Product: '<S88>/Product4'
         */
        std::memcpy(&tmp_0[0], &sfc_6p_6p_topology_machine_B.Product4_e[0], 36U *
                    sizeof(real_T));
        for (i = 0; i < 6; i++) {
          tmp[i] = sfc_6p_6p_topology_machine_B.xk1[i];
        }

        for (i = 0; i < 6; i++) {
          tmpForInput_idx_0_0 = 0.0;
          for (i_0 = 0; i_0 < 6; i_0++) {
            tmpForInput_idx_0_0 += tmp_0[6 * i_0 + i] * tmp[i_0];
          }

          /* Product: '<S80>/Product2' */
          sfc_6p_6p_topology_machine_B.Product2_a[i] = tmpForInput_idx_0_0;

          /* Sum: '<S80>/Ad*x(k-1) + Bd*( u(k-1) + u(k))' */
          tmpForInput_idx_0_0 += sfc_6p_6p_topology_machine_B.Product1_b[i];
          sfc_6p_6p_topology_machine_B.xk[i] = tmpForInput_idx_0_0;

          /* Switch: '<S80>/IC' */
          sfc_6p_6p_topology_machine_B.IC[i] = tmpForInput_idx_0_0;
        }

        /* End of Product: '<S80>/Product2' */
      } else {
        /* Switch: '<S80>/IC' */
        for (i = 0; i < 6; i++) {
          sfc_6p_6p_topology_machine_B.IC[i] =
            sfc_6p_6p_topology_machine_B.xk1[i];
        }
      }

      /* End of Switch: '<S80>/IC' */

      /* Gain: '<S67>/1_Pb' incorporates:
       *  Constant: '<Root>/Constant6'
       */
      sfc_6p_6p_topology_machine_B.u_Pb = sfc_6p_6p_topology_machine_P.u_Pb_Gain
        * sfc_6p_6p_topology_machine_P.Constant6_Value_m;

      /* Fcn: '<S67>/div' */
      sfc_6p_6p_topology_machine_B.div = sfc_6p_6p_topology_machine_B.u_Pb /
        sfc_6p_6p_topology_machine_B.Switch_e;

      /* Gain: '<S67>/F' */
      sfc_6p_6p_topology_machine_B.F = sfc_6p_6p_topology_machine_P.F_Gain *
        sfc_6p_6p_topology_machine_B.n;

      /* Gain: '<S67>/1_Pb1' incorporates:
       *  Outport: '<Root>/machine_load'
       */
      sfc_6p_6p_topology_machine_B.u_Pb1 =
        sfc_6p_6p_topology_machine_P.u_Pb1_Gain *
        sfc_6p_6p_topology_machine_Y.machine_load;

      /* Sum: '<S67>/Sum2' */
      sfc_6p_6p_topology_machine_B.Sum2_o = ((sfc_6p_6p_topology_machine_B.div -
        sfc_6p_6p_topology_machine_B.Sum2) - sfc_6p_6p_topology_machine_B.F) -
        sfc_6p_6p_topology_machine_B.u_Pb1;

      /* Gain: '<S67>/1 ----- 2H' */
      sfc_6p_6p_topology_machine_B.u2H = sfc_6p_6p_topology_machine_P.u2H_Gain *
        sfc_6p_6p_topology_machine_B.Sum2_o;

      /* Sum: '<S67>/Sum1' incorporates:
       *  Constant: '<S67>/nominal speed'
       */
      sfc_6p_6p_topology_machine_B.Sum1_p =
        sfc_6p_6p_topology_machine_B.Switch_e -
        sfc_6p_6p_topology_machine_P.nominalspeed_Value;

      /* DiscreteIntegrator: '<S103>/Rotor spee<d deviation (dw)' incorporates:
       *  Inport: '<Root>/enable_forced_speed'
       */
      if (sfc_6p_6p_topology_machine_DW.Rotorspeeddeviationdw_IC_LOADIN != 0) {
        sfc_6p_6p_topology_machine_DW.Rotorspeeddeviationdw_DSTATE =
          sfc_6p_6p_topology_machine_B.Sum1_p;

        /* DiscreteIntegrator: '<S103>/Rotor spee<d deviation (dw)' */
        sfc_6p_6p_topology_machine_B.dw =
          sfc_6p_6p_topology_machine_DW.Rotorspeeddeviationdw_DSTATE;
      } else if ((sfc_6p_6p_topology_machine_U.enable_forced_speed != 0.0) ||
                 (sfc_6p_6p_topology_machine_DW.Rotorspeeddeviationdw_PrevReset
                  != 0)) {
        sfc_6p_6p_topology_machine_DW.Rotorspeeddeviationdw_DSTATE =
          sfc_6p_6p_topology_machine_B.Sum1_p;

        /* DiscreteIntegrator: '<S103>/Rotor spee<d deviation (dw)' */
        sfc_6p_6p_topology_machine_B.dw =
          sfc_6p_6p_topology_machine_DW.Rotorspeeddeviationdw_DSTATE;
      } else {
        /* DiscreteIntegrator: '<S103>/Rotor spee<d deviation (dw)' */
        sfc_6p_6p_topology_machine_B.dw =
          sfc_6p_6p_topology_machine_P.Rotorspeeddeviationdw_gainval *
          sfc_6p_6p_topology_machine_B.u2H +
          sfc_6p_6p_topology_machine_DW.Rotorspeeddeviationdw_DSTATE;
      }

      /* End of DiscreteIntegrator: '<S103>/Rotor spee<d deviation (dw)' */

      /* Gain: '<S67>/we base' */
      sfc_6p_6p_topology_machine_B.webase =
        sfc_6p_6p_topology_machine_P.webase_Gain *
        sfc_6p_6p_topology_machine_B.Sum1_h;

      /* Gain: '<S67>/web3' */
      sfc_6p_6p_topology_machine_B.web3 = sfc_6p_6p_topology_machine_P.web3_Gain
        * sfc_6p_6p_topology_machine_B.Switch_e;

      /* Gain: '<S24>/do not delete this gain' */
      sfc_6p_6p_topology_machine_B.donotdeletethisgain_mj =
        sfc_6p_6p_topology_machine_P.donotdeletethisgain_Gain_on *
        sfc_6p_6p_topology_machine_B.StateSpace_o1[23];

      /* Outport: '<Root>/Udc_machine' incorporates:
       *  Inport: '<Root>/grid_pt_ratio'
       *  Product: '<S16>/Divide'
       */
      sfc_6p_6p_topology_machine_Y.Udc_machine =
        sfc_6p_6p_topology_machine_B.donotdeletethisgain_mj /
        sfc_6p_6p_topology_machine_U.grid_pt_ratio;

      /* Product: '<S2>/Product4' incorporates:
       *  Constant: '<S2>/Constant9'
       *  Inport: '<Root>/voltage_frequency'
       */
      sfc_6p_6p_topology_machine_B.Product4_m =
        sfc_6p_6p_topology_machine_P.Constant9_Value *
        sfc_6p_6p_topology_machine_U.voltage_frequency;
    }
  }

  if (rtmIsMajorTimeStep((&sfc_6p_6p_topology_machine_M))) {
    int32_T i;
    if (rtmIsMajorTimeStep((&sfc_6p_6p_topology_machine_M))) {
      /* Update for UnitDelay: '<S80>/fluxes' */
      for (i = 0; i < 6; i++) {
        sfc_6p_6p_topology_machine_DW.fluxes_DSTATE[i] =
          sfc_6p_6p_topology_machine_B.IC[i];
      }

      /* End of Update for UnitDelay: '<S80>/fluxes' */

      /* Update for DiscreteIntegrator: '<S67>/Rotor angle dthetae' */
      sfc_6p_6p_topology_machine_DW.Rotorangledthetae_DSTATE +=
        sfc_6p_6p_topology_machine_P.Rotorangledthetae_gainval *
        sfc_6p_6p_topology_machine_B.webase;

      /* Update for DiscreteIntegrator: '<S34>/Integrator' */
      sfc_6p_6p_topology_machine_DW.Integrator_DSTATE +=
        sfc_6p_6p_topology_machine_P.Integrator_gainval *
        sfc_6p_6p_topology_machine_B.Product4_m;
      if (sfc_6p_6p_topology_machine_DW.Integrator_DSTATE >
          sfc_6p_6p_topology_machine_P.Integrator_UpperSat) {
        sfc_6p_6p_topology_machine_DW.Integrator_DSTATE =
          sfc_6p_6p_topology_machine_P.Integrator_UpperSat;
      } else if (sfc_6p_6p_topology_machine_DW.Integrator_DSTATE <
                 sfc_6p_6p_topology_machine_P.Integrator_LowerSat) {
        sfc_6p_6p_topology_machine_DW.Integrator_DSTATE =
          sfc_6p_6p_topology_machine_P.Integrator_LowerSat;
      }

      if (sfc_6p_6p_topology_machine_B.Constant4 > 0.0) {
        sfc_6p_6p_topology_machine_DW.Integrator_PrevResetState = 1;
      } else if (sfc_6p_6p_topology_machine_B.Constant4 < 0.0) {
        sfc_6p_6p_topology_machine_DW.Integrator_PrevResetState = -1;
      } else if (sfc_6p_6p_topology_machine_B.Constant4 == 0.0) {
        sfc_6p_6p_topology_machine_DW.Integrator_PrevResetState = 0;
      } else {
        sfc_6p_6p_topology_machine_DW.Integrator_PrevResetState = 2;
      }

      /* End of Update for DiscreteIntegrator: '<S34>/Integrator' */

      /* Update for S-Function (sfun_spssw_discc): '<S158>/State-Space' incorporates:
       *  Constant: '<S160>/SwitchCurrents'
       *  Inport: '<Root>/pulses_in_grid'
       *  Inport: '<Root>/pulses_in_motor'
       */

      /* S-Function block: <S158>/State-Space */
      {
        const real_T *As = (real_T*)
          sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.AS;
        const real_T *Bs = (real_T*)
          sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.BS;
        real_T *xtmp = (real_T*)
          sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.XTMP;
        real_T accum;

        /* Calculate new states... */
        {
          int_T i1;
          real_T *xd = &sfc_6p_6p_topology_machine_DW.StateSpace_DSTATE[0];
          for (i1=0; i1 < 21; i1++) {
            accum = 0.0;

            {
              int_T i2;
              real_T *xd = &sfc_6p_6p_topology_machine_DW.StateSpace_DSTATE[0];
              for (i2=0; i2 < 21; i2++) {
                accum += *(As++) * xd[i2];
              }
            }

            {
              int_T i2;
              const real_T *u0 =
                &sfc_6p_6p_topology_machine_P.SwitchCurrents_Value[0];
              for (i2=0; i2 < 12; i2++) {
                accum += *(Bs++) * u0[i2];
              }

              accum += *(Bs++) * sfc_6p_6p_topology_machine_B.iabc[0];
              accum += *(Bs++) * sfc_6p_6p_topology_machine_B.iabc[1];
              accum += *(Bs++) * sfc_6p_6p_topology_machine_B.Product5;
              accum += *(Bs++) * sfc_6p_6p_topology_machine_B.Product6;
              accum += *(Bs++) * sfc_6p_6p_topology_machine_B.Product7;
            }

            xtmp[i1] = accum;
          }
        }

        {
          int_T i1;
          real_T *xd = &sfc_6p_6p_topology_machine_DW.StateSpace_DSTATE[0];
          for (i1=0; i1 < 21; i1++) {
            xd[i1] = xtmp[i1];
          }
        }

        {
          int_T *gState = (int_T*)
            sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.G_STATE;

          /* Store switch gates values for next step */
          *(gState++) = (int_T) sfc_6p_6p_topology_machine_U.pulses_in_grid[0];
          *(gState++) = (int_T) sfc_6p_6p_topology_machine_U.pulses_in_grid[5];
          *(gState++) = (int_T) sfc_6p_6p_topology_machine_U.pulses_in_grid[1];
          *(gState++) = (int_T) sfc_6p_6p_topology_machine_U.pulses_in_grid[3];
          *(gState++) = (int_T) sfc_6p_6p_topology_machine_U.pulses_in_grid[2];
          *(gState++) = (int_T) sfc_6p_6p_topology_machine_U.pulses_in_grid[4];
          *(gState++) = (int_T) sfc_6p_6p_topology_machine_U.pulses_in_motor[0];
          *(gState++) = (int_T) sfc_6p_6p_topology_machine_U.pulses_in_motor[5];
          *(gState++) = (int_T) sfc_6p_6p_topology_machine_U.pulses_in_motor[1];
          *(gState++) = (int_T) sfc_6p_6p_topology_machine_U.pulses_in_motor[3];
          *(gState++) = (int_T) sfc_6p_6p_topology_machine_U.pulses_in_motor[2];
          *(gState++) = (int_T) sfc_6p_6p_topology_machine_U.pulses_in_motor[4];
        }
      }

      /* Update for Memory: '<S36>/Memory' */
      sfc_6p_6p_topology_machine_DW.Memory_PreviousInput =
        sfc_6p_6p_topology_machine_B.y;
    }

    /* Update for Integrator: '<S36>/Integrator' */
    sfc_6p_6p_topology_machine_DW.Integrator_IWORK = 0;

    /* Update for Enabled SubSystem: '<S36>/Automatic Gain Control' incorporates:
     *  EnablePort: '<S37>/Enable'
     */
    if (sfc_6p_6p_topology_machine_DW.AutomaticGainControl_MODE) {
      /* Update for VariableTransportDelay: '<S46>/Variable Transport Delay' */
      {
        real_T **uBuffer = (real_T**)
          &sfc_6p_6p_topology_machine_DW.VariableTransportDelay_PWORK_g.TUbufferPtrs
          [0];
        real_T simTime = (&sfc_6p_6p_topology_machine_M)->Timing.t[0];
        sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK_g.Head =
          ((sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK_g.Head <
            (sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK_g.CircularBufSize
             -1)) ?
           (sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK_g.Head+1)
           : 0);
        if (sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK_g.Head ==
            sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK_g.Tail) {
          sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK_g.Tail =
            ((sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK_g.Tail <
              (sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK_g.CircularBufSize
               -1)) ?
             (sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK_g.Tail+
              1) : 0);
        }

        (*uBuffer +
          sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK_g.CircularBufSize)
          [sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK_g.Head] =
          simTime;
        (*uBuffer + 2 *
          sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK_g.CircularBufSize)
          [sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK_g.Head] =
          sfc_6p_6p_topology_machine_X.VariableTransportDelay_CSTATE_c;
        (*uBuffer)
          [sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK_g.Head] =
          sfc_6p_6p_topology_machine_B.integrator_d;

        /* when use fixed buffer, reset solver at when buffer is updated
         * to avoid output consistency fail.
         */
      }

      /* Update for VariableTransportDelay: '<S47>/Variable Transport Delay' */
      {
        real_T **uBuffer = (real_T**)
          &sfc_6p_6p_topology_machine_DW.VariableTransportDelay_PWORK_j.TUbufferPtrs
          [0];
        real_T simTime = (&sfc_6p_6p_topology_machine_M)->Timing.t[0];
        sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK_a.Head =
          ((sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK_a.Head <
            (sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK_a.CircularBufSize
             -1)) ?
           (sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK_a.Head+1)
           : 0);
        if (sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK_a.Head ==
            sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK_a.Tail) {
          sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK_a.Tail =
            ((sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK_a.Tail <
              (sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK_a.CircularBufSize
               -1)) ?
             (sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK_a.Tail+
              1) : 0);
        }

        (*uBuffer +
          sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK_a.CircularBufSize)
          [sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK_a.Head] =
          simTime;
        (*uBuffer + 2 *
          sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK_a.CircularBufSize)
          [sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK_a.Head] =
          sfc_6p_6p_topology_machine_X.VariableTransportDelay_CSTATE_l;
        (*uBuffer)
          [sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK_a.Head] =
          sfc_6p_6p_topology_machine_B.integrator_de;

        /* when use fixed buffer, reset solver at when buffer is updated
         * to avoid output consistency fail.
         */
      }
    }

    /* End of Update for SubSystem: '<S36>/Automatic Gain Control' */

    /* Update for VariableTransportDelay: '<S58>/Variable Transport Delay' */
    {
      real_T **uBuffer = (real_T**)
        &sfc_6p_6p_topology_machine_DW.VariableTransportDelay_PWORK.TUbufferPtrs[
        0];
      real_T simTime = (&sfc_6p_6p_topology_machine_M)->Timing.t[0];
      sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK.Head =
        ((sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK.Head <
          (sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK.CircularBufSize
           -1)) ?
         (sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK.Head+1) : 0);
      if (sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK.Head ==
          sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK.Tail) {
        sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK.Tail =
          ((sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK.Tail <
            (sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK.CircularBufSize
             -1)) ?
           (sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK.Tail+1) :
           0);
      }

      (*uBuffer +
        sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK.CircularBufSize)
        [sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK.Head] =
        simTime;
      (*uBuffer + 2 *
        sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK.CircularBufSize)
        [sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK.Head] =
        sfc_6p_6p_topology_machine_X.VariableTransportDelay_CSTATE;
      (*uBuffer)[sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK.Head]
        = sfc_6p_6p_topology_machine_B.integrator;

      /* when use fixed buffer, reset solver at when buffer is updated
       * to avoid output consistency fail.
       */
    }

    /* Update for RateLimiter: '<S36>/Rate Limiter' */
    sfc_6p_6p_topology_machine_DW.PrevY =
      sfc_6p_6p_topology_machine_B.RateLimiter;
    sfc_6p_6p_topology_machine_DW.LastMajorTime = (&sfc_6p_6p_topology_machine_M)
      ->Timing.t[0];
    if (rtmIsMajorTimeStep((&sfc_6p_6p_topology_machine_M))) {
      /* Update for UnitDelay: '<S104>/dw_delay' */
      sfc_6p_6p_topology_machine_DW.dw_delay_DSTATE =
        sfc_6p_6p_topology_machine_B.dw;

      /* Update for UnitDelay: '<S104>/dw_predict' */
      sfc_6p_6p_topology_machine_DW.dw_predict_DSTATE =
        sfc_6p_6p_topology_machine_B.dw_delay;

      /* Update for DiscreteIntegrator: '<S67>/theta' */
      sfc_6p_6p_topology_machine_DW.theta_DSTATE +=
        sfc_6p_6p_topology_machine_P.theta_gainval *
        sfc_6p_6p_topology_machine_B.web3;

      /* Update for UnitDelay: '<S80>/voltages' incorporates:
       *  Constant: '<S65>/[ Vkd =0 Vkq1=0  Vkq2=0 ]'
       */
      sfc_6p_6p_topology_machine_DW.voltages_DSTATE[0] =
        sfc_6p_6p_topology_machine_B.Switch_i[1];
      sfc_6p_6p_topology_machine_DW.voltages_DSTATE[1] =
        sfc_6p_6p_topology_machine_B.Switch_i[0];
      sfc_6p_6p_topology_machine_DW.voltages_DSTATE[2] =
        sfc_6p_6p_topology_machine_B.Vfd;
      sfc_6p_6p_topology_machine_DW.voltages_DSTATE[3] =
        sfc_6p_6p_topology_machine_P.Vkd0Vkq10Vkq20_Value[0];
      sfc_6p_6p_topology_machine_DW.voltages_DSTATE[4] =
        sfc_6p_6p_topology_machine_P.Vkd0Vkq10Vkq20_Value[1];
      sfc_6p_6p_topology_machine_DW.voltages_DSTATE[5] =
        sfc_6p_6p_topology_machine_P.Vkd0Vkq10Vkq20_Value[2];

      /* Update for DiscreteIntegrator: '<S103>/Rotor spee<d deviation (dw)' incorporates:
       *  Inport: '<Root>/enable_forced_speed'
       */
      sfc_6p_6p_topology_machine_DW.Rotorspeeddeviationdw_IC_LOADIN = 0U;
      sfc_6p_6p_topology_machine_DW.Rotorspeeddeviationdw_DSTATE =
        sfc_6p_6p_topology_machine_P.Rotorspeeddeviationdw_gainval *
        sfc_6p_6p_topology_machine_B.u2H + sfc_6p_6p_topology_machine_B.dw;
      if (sfc_6p_6p_topology_machine_U.enable_forced_speed > 0.0) {
        sfc_6p_6p_topology_machine_DW.Rotorspeeddeviationdw_PrevReset = 1;
      } else if (sfc_6p_6p_topology_machine_U.enable_forced_speed < 0.0) {
        sfc_6p_6p_topology_machine_DW.Rotorspeeddeviationdw_PrevReset = -1;
      } else if (sfc_6p_6p_topology_machine_U.enable_forced_speed == 0.0) {
        sfc_6p_6p_topology_machine_DW.Rotorspeeddeviationdw_PrevReset = 0;
      } else {
        sfc_6p_6p_topology_machine_DW.Rotorspeeddeviationdw_PrevReset = 2;
      }

      /* End of Update for DiscreteIntegrator: '<S103>/Rotor spee<d deviation (dw)' */
    }

    /* ContTimeOutputInconsistentWithStateAtMajorOutputFlag is set, need to run a minor output */
    if (rtmIsMajorTimeStep((&sfc_6p_6p_topology_machine_M))) {
      if (rtsiGetContTimeOutputInconsistentWithStateAtMajorStep
          (&(&sfc_6p_6p_topology_machine_M)->solverInfo)) {
        rtsiSetSimTimeStep(&(&sfc_6p_6p_topology_machine_M)->solverInfo,
                           MINOR_TIME_STEP);
        rtsiSetContTimeOutputInconsistentWithStateAtMajorStep
          (&(&sfc_6p_6p_topology_machine_M)->solverInfo, false);
        sfc_6p_6p_topology_machine::step();
        rtsiSetSimTimeStep(&(&sfc_6p_6p_topology_machine_M)->solverInfo,
                           MAJOR_TIME_STEP);
      }
    }
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep((&sfc_6p_6p_topology_machine_M))) {
    rt_ertODEUpdateContinuousStates(&(&sfc_6p_6p_topology_machine_M)->solverInfo);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick0 and the high bits
     * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
     */
    if (!(++(&sfc_6p_6p_topology_machine_M)->Timing.clockTick0)) {
      ++(&sfc_6p_6p_topology_machine_M)->Timing.clockTickH0;
    }

    (&sfc_6p_6p_topology_machine_M)->Timing.t[0] = rtsiGetSolverStopTime
      (&(&sfc_6p_6p_topology_machine_M)->solverInfo);

    {
      /* Update absolute timer for sample time: [1.25E-5s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 1.25E-5, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       * Timer of this task consists of two 32 bit unsigned integers.
       * The two integers represent the low bits Timing.clockTick1 and the high bits
       * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
       */
      (&sfc_6p_6p_topology_machine_M)->Timing.clockTick1++;
      if (!(&sfc_6p_6p_topology_machine_M)->Timing.clockTick1) {
        (&sfc_6p_6p_topology_machine_M)->Timing.clockTickH1++;
      }
    }
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void sfc_6p_6p_topology_machine::sfc_6p_6p_topology_machine_derivatives()
{
  XDot_sfc_6p_6p_topology_machi_T *_rtXdot;
  real_T tmp;
  boolean_T lsat;
  boolean_T usat;
  _rtXdot = ((XDot_sfc_6p_6p_topology_machi_T *) (&sfc_6p_6p_topology_machine_M
             )->derivs);

  /* Derivatives for Integrator: '<S36>/Integrator' */
  lsat = (sfc_6p_6p_topology_machine_X.Integrator_CSTATE <=
          sfc_6p_6p_topology_machine_P.Integrator_LowerSat_n);
  usat = (sfc_6p_6p_topology_machine_X.Integrator_CSTATE >=
          sfc_6p_6p_topology_machine_P.Integrator_UpperSat_l);
  if (((!lsat) && (!usat)) || (lsat && (sfc_6p_6p_topology_machine_B.Saturation2
        > 0.0)) || (usat && (sfc_6p_6p_topology_machine_B.Saturation2 < 0.0))) {
    _rtXdot->Integrator_CSTATE = sfc_6p_6p_topology_machine_B.Saturation2;
  } else {
    /* in saturation */
    _rtXdot->Integrator_CSTATE = 0.0;
  }

  /* End of Derivatives for Integrator: '<S36>/Integrator' */

  /* Derivatives for Enabled SubSystem: '<S36>/Automatic Gain Control' */
  if (sfc_6p_6p_topology_machine_DW.AutomaticGainControl_MODE) {
    /* Derivatives for VariableTransportDelay: '<S46>/Variable Transport Delay' */
    {
      real_T instantDelay;
      instantDelay = sfc_6p_6p_topology_machine_B.period_i;
      if (instantDelay >
          (sfc_6p_6p_topology_machine_P.VariableTransportDelay_MaxDelay)) {
        instantDelay =
          (sfc_6p_6p_topology_machine_P.VariableTransportDelay_MaxDelay);
      }

      if (instantDelay < 0.0) {
        ((XDot_sfc_6p_6p_topology_machi_T *) (&sfc_6p_6p_topology_machine_M)
          ->derivs)->VariableTransportDelay_CSTATE_c = 0;
      } else {
        ((XDot_sfc_6p_6p_topology_machi_T *) (&sfc_6p_6p_topology_machine_M)
          ->derivs)->VariableTransportDelay_CSTATE_c = 1.0/instantDelay;
      }
    }

    /* Derivatives for Integrator: '<S46>/integrator' */
    _rtXdot->integrator_CSTATE_g = sfc_6p_6p_topology_machine_B.Switch_dq[0];

    /* Derivatives for VariableTransportDelay: '<S47>/Variable Transport Delay' */
    {
      real_T instantDelay;
      instantDelay = sfc_6p_6p_topology_machine_B.period_a;
      if (instantDelay >
          (sfc_6p_6p_topology_machine_P.VariableTransportDelay_MaxDel_i)) {
        instantDelay =
          (sfc_6p_6p_topology_machine_P.VariableTransportDelay_MaxDel_i);
      }

      if (instantDelay < 0.0) {
        ((XDot_sfc_6p_6p_topology_machi_T *) (&sfc_6p_6p_topology_machine_M)
          ->derivs)->VariableTransportDelay_CSTATE_l = 0;
      } else {
        ((XDot_sfc_6p_6p_topology_machi_T *) (&sfc_6p_6p_topology_machine_M)
          ->derivs)->VariableTransportDelay_CSTATE_l = 1.0/instantDelay;
      }
    }

    /* Derivatives for Integrator: '<S47>/integrator' */
    _rtXdot->integrator_CSTATE_b = sfc_6p_6p_topology_machine_B.Switch_dq[1];
  } else {
    {
      real_T *dx;
      int_T i;
      dx = &(((XDot_sfc_6p_6p_topology_machi_T *) (&sfc_6p_6p_topology_machine_M)
              ->derivs)->VariableTransportDelay_CSTATE_c);
      for (i=0; i < 4; i++) {
        dx[i] = 0.0;
      }
    }
  }

  /* End of Derivatives for SubSystem: '<S36>/Automatic Gain Control' */

  /* Derivatives for Integrator: '<S38>/Integrator' */
  tmp = sfc_6p_6p_topology_machine_P.Continuous_Par_Limits[1];
  lsat = (sfc_6p_6p_topology_machine_X.Integrator_CSTATE_i <= tmp);
  tmp = sfc_6p_6p_topology_machine_P.Continuous_Par_Limits[0];
  usat = (sfc_6p_6p_topology_machine_X.Integrator_CSTATE_i >= tmp);
  if (((!lsat) && (!usat)) || (lsat && (sfc_6p_6p_topology_machine_B.Kp5 > 0.0))
      || (usat && (sfc_6p_6p_topology_machine_B.Kp5 < 0.0))) {
    _rtXdot->Integrator_CSTATE_i = sfc_6p_6p_topology_machine_B.Kp5;
  } else {
    /* in saturation */
    _rtXdot->Integrator_CSTATE_i = 0.0;
  }

  /* End of Derivatives for Integrator: '<S38>/Integrator' */

  /* Derivatives for VariableTransportDelay: '<S58>/Variable Transport Delay' */
  {
    real_T instantDelay;
    instantDelay = sfc_6p_6p_topology_machine_B.period;
    if (instantDelay >
        (sfc_6p_6p_topology_machine_P.VariableTransportDelay_MaxDel_l)) {
      instantDelay =
        (sfc_6p_6p_topology_machine_P.VariableTransportDelay_MaxDel_l);
    }

    if (instantDelay < 0.0) {
      ((XDot_sfc_6p_6p_topology_machi_T *) (&sfc_6p_6p_topology_machine_M)
        ->derivs)->VariableTransportDelay_CSTATE = 0;
    } else {
      ((XDot_sfc_6p_6p_topology_machi_T *) (&sfc_6p_6p_topology_machine_M)
        ->derivs)->VariableTransportDelay_CSTATE = 1.0/instantDelay;
    }
  }

  /* Derivatives for Integrator: '<S58>/integrator' */
  _rtXdot->integrator_CSTATE = sfc_6p_6p_topology_machine_B.Switch_a[1];

  /* Derivatives for TransferFcn: '<S38>/Transfer Fcn' */
  _rtXdot->TransferFcn_CSTATE = 0.0;
  _rtXdot->TransferFcn_CSTATE += sfc_6p_6p_topology_machine_P.TransferFcn_A *
    sfc_6p_6p_topology_machine_X.TransferFcn_CSTATE;
  _rtXdot->TransferFcn_CSTATE += sfc_6p_6p_topology_machine_B.Kp6;

  /* Derivatives for Integrator: '<S54>/Integrator_x1' */
  _rtXdot->Integrator_x1_CSTATE = sfc_6p_6p_topology_machine_B.x1_i;

  /* Derivatives for Integrator: '<S54>/Integrator_x2' */
  _rtXdot->Integrator_x2_CSTATE = sfc_6p_6p_topology_machine_B.x2_m;
}

/* Model initialize function */
void sfc_6p_6p_topology_machine::initialize()
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* non-finite (run-time) assignments */
  sfc_6p_6p_topology_machine_P.Continuous_Par_Limits[0] = rtInf;
  sfc_6p_6p_topology_machine_P.Saturation_UpperSat = rtInf;
  sfc_6p_6p_topology_machine_P.Integrator_UpperSat = rtInf;
  sfc_6p_6p_topology_machine_P.Integrator_LowerSat = rtMinusInf;
  sfc_6p_6p_topology_machine_P.Saturation_UpperSat_e = rtInf;
  sfc_6p_6p_topology_machine_P.Saturation_LowerSat_f = rtMinusInf;
  sfc_6p_6p_topology_machine_P.Integrator_UpperSat_l = rtInf;
  sfc_6p_6p_topology_machine_P.Integrator_LowerSat_n = rtMinusInf;

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&(&sfc_6p_6p_topology_machine_M)->solverInfo,
                          &(&sfc_6p_6p_topology_machine_M)->Timing.simTimeStep);
    rtsiSetTPtr(&(&sfc_6p_6p_topology_machine_M)->solverInfo, &rtmGetTPtr
                ((&sfc_6p_6p_topology_machine_M)));
    rtsiSetStepSizePtr(&(&sfc_6p_6p_topology_machine_M)->solverInfo,
                       &(&sfc_6p_6p_topology_machine_M)->Timing.stepSize0);
    rtsiSetdXPtr(&(&sfc_6p_6p_topology_machine_M)->solverInfo,
                 &(&sfc_6p_6p_topology_machine_M)->derivs);
    rtsiSetContStatesPtr(&(&sfc_6p_6p_topology_machine_M)->solverInfo, (real_T **)
                         &(&sfc_6p_6p_topology_machine_M)->contStates);
    rtsiSetNumContStatesPtr(&(&sfc_6p_6p_topology_machine_M)->solverInfo,
      &(&sfc_6p_6p_topology_machine_M)->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&(&sfc_6p_6p_topology_machine_M)->solverInfo,
      &(&sfc_6p_6p_topology_machine_M)->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&(&sfc_6p_6p_topology_machine_M)
      ->solverInfo, &(&sfc_6p_6p_topology_machine_M)->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&(&sfc_6p_6p_topology_machine_M)
      ->solverInfo, &(&sfc_6p_6p_topology_machine_M)->periodicContStateRanges);
    rtsiSetContStateDisabledPtr(&(&sfc_6p_6p_topology_machine_M)->solverInfo,
      (boolean_T**) &(&sfc_6p_6p_topology_machine_M)->contStateDisabled);
    rtsiSetErrorStatusPtr(&(&sfc_6p_6p_topology_machine_M)->solverInfo,
                          (&rtmGetErrorStatus((&sfc_6p_6p_topology_machine_M))));
    rtsiSetRTModelPtr(&(&sfc_6p_6p_topology_machine_M)->solverInfo,
                      (&sfc_6p_6p_topology_machine_M));
  }

  rtsiSetSimTimeStep(&(&sfc_6p_6p_topology_machine_M)->solverInfo,
                     MAJOR_TIME_STEP);
  (&sfc_6p_6p_topology_machine_M)->intgData.f[0] =
    (&sfc_6p_6p_topology_machine_M)->odeF[0];
  (&sfc_6p_6p_topology_machine_M)->contStates = ((X_sfc_6p_6p_topology_machine_T
    *) &sfc_6p_6p_topology_machine_X);
  (&sfc_6p_6p_topology_machine_M)->contStateDisabled =
    ((XDis_sfc_6p_6p_topology_machi_T *) &sfc_6p_6p_topology_machine_XDis);
  (&sfc_6p_6p_topology_machine_M)->Timing.tStart = (0.0);
  rtsiSetSolverData(&(&sfc_6p_6p_topology_machine_M)->solverInfo, static_cast<
                    void *>(&(&sfc_6p_6p_topology_machine_M)->intgData));
  rtsiSetIsMinorTimeStepWithModeChange(&(&sfc_6p_6p_topology_machine_M)
    ->solverInfo, false);
  rtsiSetSolverName(&(&sfc_6p_6p_topology_machine_M)->solverInfo,"ode1");
  rtmSetTPtr((&sfc_6p_6p_topology_machine_M), &(&sfc_6p_6p_topology_machine_M)
             ->Timing.tArray[0]);
  (&sfc_6p_6p_topology_machine_M)->Timing.stepSize0 = 1.25E-5;
  rtmSetFirstInitCond((&sfc_6p_6p_topology_machine_M), 1);

  /* Start for Constant: '<S2>/Constant4' */
  sfc_6p_6p_topology_machine_B.Constant4 =
    sfc_6p_6p_topology_machine_P.Constant4_Value_d;

  /* Start for S-Function (sfun_spssw_discc): '<S158>/State-Space' incorporates:
   *  Constant: '<S160>/SwitchCurrents'
   *  Inport: '<Root>/pulses_in_grid'
   *  Inport: '<Root>/pulses_in_motor'
   */

  /* S-Function block: <S158>/State-Space */
  {
    sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.AS = (real_T*)calloc(21 * 21,
      sizeof(real_T));
    sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.BS = (real_T*)calloc(21 * 17,
      sizeof(real_T));
    sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.CS = (real_T*)calloc(35 * 21,
      sizeof(real_T));
    sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.DS = (real_T*)calloc(35 * 17,
      sizeof(real_T));
    sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.DX_COL = (real_T*)calloc(35,
      sizeof(real_T));
    sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.TMP2 = (real_T*)calloc(17,
      sizeof(real_T));
    sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.BD_COL = (real_T*)calloc(21,
      sizeof(real_T));
    sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.TMP1 = (real_T*)calloc(21,
      sizeof(real_T));
    sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.XTMP = (real_T*)calloc(21,
      sizeof(real_T));
    sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.SWITCH_STATUS = (int_T*)
      calloc(12, sizeof(int_T));
    sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.SW_CHG = (int_T*)calloc(12,
      sizeof(int_T));
    sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.G_STATE = (int_T*)calloc(12,
      sizeof(int_T));
    sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.Y_SWITCH = (real_T*)calloc(12,
      sizeof(real_T));
    sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.SWITCH_TYPES = (int_T*)calloc
      (12, sizeof(int_T));
    sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.IDX_OUT_SW = (int_T*)calloc
      (12, sizeof(int_T));
    sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.SWITCH_STATUS_INIT = (int_T*)
      calloc(12, sizeof(int_T));
    sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.USWLAST = (real_T*)calloc(12,
      sizeof(real_T));
  }

  /* Start for InitialCondition: '<S36>/Initial' */
  sfc_6p_6p_topology_machine_B.Initial =
    sfc_6p_6p_topology_machine_P.Initial_Value;

  /* Start for InitialCondition: '<S36>/Initial' */
  sfc_6p_6p_topology_machine_DW.Initial_FirstOutputTime = (rtMinusInf);

  /* Start for VariableTransportDelay: '<S58>/Variable Transport Delay' */
  {
    real_T *pBuffer =
      &sfc_6p_6p_topology_machine_DW.VariableTransportDelay_RWORK.TUbufferArea[0];
    int_T j;
    sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK.Tail = 0;
    sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK.Head = 0;
    sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK.Last = 0;
    sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK.CircularBufSize =
      8192;
    for (j=0; j < 8192; j++) {
      pBuffer[j] = (sfc_6p_6p_topology_machine_P.VariableTransportDelay_InitOu_b);
      pBuffer[8192 + j] = (&sfc_6p_6p_topology_machine_M)->Timing.t[0];
    }

    pBuffer[2*8192] = 0.0;
    sfc_6p_6p_topology_machine_DW.VariableTransportDelay_PWORK.TUbufferPtrs[0] =
      (void *) &pBuffer[0];
  }

  sfc_6p_6p_topology_mach_PrevZCX.Integrator_Reset_ZCE = UNINITIALIZED_ZCSIG;

  {
    int32_T i;

    /* InitializeConditions for UnitDelay: '<S80>/fluxes' */
    for (i = 0; i < 6; i++) {
      sfc_6p_6p_topology_machine_DW.fluxes_DSTATE[i] =
        sfc_6p_6p_topology_machine_P.fluxes_InitialCondition[i];
    }

    /* End of InitializeConditions for UnitDelay: '<S80>/fluxes' */

    /* InitializeConditions for DiscreteIntegrator: '<S67>/Rotor angle dthetae' */
    sfc_6p_6p_topology_machine_DW.Rotorangledthetae_DSTATE =
      sfc_6p_6p_topology_machine_P.Rotorangledthetae_IC;

    /* InitializeConditions for DiscreteIntegrator: '<S34>/Integrator' */
    sfc_6p_6p_topology_machine_DW.Integrator_DSTATE =
      sfc_6p_6p_topology_machine_B.Constant4;
    if (sfc_6p_6p_topology_machine_DW.Integrator_DSTATE >
        sfc_6p_6p_topology_machine_P.Integrator_UpperSat) {
      sfc_6p_6p_topology_machine_DW.Integrator_DSTATE =
        sfc_6p_6p_topology_machine_P.Integrator_UpperSat;
    } else if (sfc_6p_6p_topology_machine_DW.Integrator_DSTATE <
               sfc_6p_6p_topology_machine_P.Integrator_LowerSat) {
      sfc_6p_6p_topology_machine_DW.Integrator_DSTATE =
        sfc_6p_6p_topology_machine_P.Integrator_LowerSat;
    }

    /* End of InitializeConditions for DiscreteIntegrator: '<S34>/Integrator' */

    /* InitializeConditions for S-Function (sfun_spssw_discc): '<S158>/State-Space' incorporates:
     *  Constant: '<S160>/SwitchCurrents'
     *  Inport: '<Root>/pulses_in_grid'
     *  Inport: '<Root>/pulses_in_motor'
     */
    {
      int32_T i, j;
      real_T *As = (real_T*)sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.AS;
      real_T *Bs = (real_T*)sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.BS;
      real_T *Cs = (real_T*)sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.CS;
      real_T *Ds = (real_T*)sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.DS;
      real_T *X0 = (real_T*)&sfc_6p_6p_topology_machine_DW.StateSpace_DSTATE[0];
      for (i = 0; i < 21; i++) {
        X0[i] = (sfc_6p_6p_topology_machine_P.StateSpace_X0_param[i]);
      }

      /* Copy and transpose A and B */
      for (i=0; i<21; i++) {
        for (j=0; j<21; j++)
          As[i*21 + j] = (sfc_6p_6p_topology_machine_P.StateSpace_AS_param[i + j*
                          21]);
        for (j=0; j<17; j++)
          Bs[i*17 + j] = (sfc_6p_6p_topology_machine_P.StateSpace_BS_param[i + j*
                          21]);
      }

      /* Copy and transpose C */
      for (i=0; i<35; i++) {
        for (j=0; j<21; j++)
          Cs[i*21 + j] = (sfc_6p_6p_topology_machine_P.StateSpace_CS_param[i + j*
                          35]);
      }

      /* Copy and transpose D */
      for (i=0; i<35; i++) {
        for (j=0; j<17; j++)
          Ds[i*17 + j] = (sfc_6p_6p_topology_machine_P.StateSpace_DS_param[i + j*
                          35]);
      }

      {
        /* Switches work vectors */
        int_T *switch_status = (int_T*)
          sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.SWITCH_STATUS;
        int_T *gState = (int_T*)
          sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.G_STATE;
        real_T *yswitch = (real_T*)
          sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.Y_SWITCH;
        int_T *switchTypes = (int_T*)
          sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.SWITCH_TYPES;
        int_T *idxOutSw = (int_T*)
          sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.IDX_OUT_SW;
        int_T *switch_status_init = (int_T*)
          sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.SWITCH_STATUS_INIT;

        /* Initialize work vectors */
        switch_status[0] = 0;
        switch_status_init[0] = 0;
        gState[0] = (int_T) 0.0;
        yswitch[0] = 1/1.0E-6;
        switchTypes[0] = (int_T)4.0;
        idxOutSw[0] = ((int_T)0.0) - 1;
        switch_status[1] = 0;
        switch_status_init[1] = 0;
        gState[1] = (int_T) 0.0;
        yswitch[1] = 1/1.0E-6;
        switchTypes[1] = (int_T)4.0;
        idxOutSw[1] = ((int_T)0.0) - 1;
        switch_status[2] = 0;
        switch_status_init[2] = 0;
        gState[2] = (int_T) 0.0;
        yswitch[2] = 1/1.0E-6;
        switchTypes[2] = (int_T)4.0;
        idxOutSw[2] = ((int_T)0.0) - 1;
        switch_status[3] = 0;
        switch_status_init[3] = 0;
        gState[3] = (int_T) 0.0;
        yswitch[3] = 1/1.0E-6;
        switchTypes[3] = (int_T)4.0;
        idxOutSw[3] = ((int_T)0.0) - 1;
        switch_status[4] = 0;
        switch_status_init[4] = 0;
        gState[4] = (int_T) 0.0;
        yswitch[4] = 1/1.0E-6;
        switchTypes[4] = (int_T)4.0;
        idxOutSw[4] = ((int_T)0.0) - 1;
        switch_status[5] = 0;
        switch_status_init[5] = 0;
        gState[5] = (int_T) 0.0;
        yswitch[5] = 1/1.0E-6;
        switchTypes[5] = (int_T)4.0;
        idxOutSw[5] = ((int_T)0.0) - 1;
        switch_status[6] = 0;
        switch_status_init[6] = 0;
        gState[6] = (int_T) 0.0;
        yswitch[6] = 1/1.0E-6;
        switchTypes[6] = (int_T)4.0;
        idxOutSw[6] = ((int_T)0.0) - 1;
        switch_status[7] = 0;
        switch_status_init[7] = 0;
        gState[7] = (int_T) 0.0;
        yswitch[7] = 1/1.0E-6;
        switchTypes[7] = (int_T)4.0;
        idxOutSw[7] = ((int_T)0.0) - 1;
        switch_status[8] = 0;
        switch_status_init[8] = 0;
        gState[8] = (int_T) 0.0;
        yswitch[8] = 1/1.0E-6;
        switchTypes[8] = (int_T)4.0;
        idxOutSw[8] = ((int_T)0.0) - 1;
        switch_status[9] = 0;
        switch_status_init[9] = 0;
        gState[9] = (int_T) 0.0;
        yswitch[9] = 1/1.0E-6;
        switchTypes[9] = (int_T)4.0;
        idxOutSw[9] = ((int_T)0.0) - 1;
        switch_status[10] = 0;
        switch_status_init[10] = 0;
        gState[10] = (int_T) 0.0;
        yswitch[10] = 1/1.0E-6;
        switchTypes[10] = (int_T)4.0;
        idxOutSw[10] = ((int_T)0.0) - 1;
        switch_status[11] = 0;
        switch_status_init[11] = 0;
        gState[11] = (int_T) 0.0;
        yswitch[11] = 1/1.0E-6;
        switchTypes[11] = (int_T)4.0;
        idxOutSw[11] = ((int_T)0.0) - 1;
      }
    }

    /* InitializeConditions for Integrator: '<S36>/Integrator' */
    if (rtmIsFirstInitCond((&sfc_6p_6p_topology_machine_M))) {
      sfc_6p_6p_topology_machine_X.Integrator_CSTATE = 0.0;
    }

    sfc_6p_6p_topology_machine_DW.Integrator_IWORK = 1;

    /* End of InitializeConditions for Integrator: '<S36>/Integrator' */

    /* InitializeConditions for Memory: '<S36>/Memory' */
    sfc_6p_6p_topology_machine_DW.Memory_PreviousInput =
      sfc_6p_6p_topology_machine_P.Memory_InitialCondition;

    /* InitializeConditions for Integrator: '<S38>/Integrator' */
    sfc_6p_6p_topology_machine_X.Integrator_CSTATE_i =
      sfc_6p_6p_topology_machine_P.Continuous_Init;

    /* InitializeConditions for VariableTransportDelay: '<S58>/Variable Transport Delay' */
    sfc_6p_6p_topology_machine_X.VariableTransportDelay_CSTATE = 0.0;

    /* InitializeConditions for Integrator: '<S58>/integrator' */
    sfc_6p_6p_topology_machine_X.integrator_CSTATE =
      sfc_6p_6p_topology_machine_P.integrator_IC_n;

    /* InitializeConditions for TransferFcn: '<S38>/Transfer Fcn' */
    sfc_6p_6p_topology_machine_X.TransferFcn_CSTATE = 0.0;

    /* InitializeConditions for RateLimiter: '<S36>/Rate Limiter' */
    sfc_6p_6p_topology_machine_DW.LastMajorTime = (rtInf);

    /* InitializeConditions for Integrator: '<S54>/Integrator_x1' */
    sfc_6p_6p_topology_machine_X.Integrator_x1_CSTATE =
      sfc_6p_6p_topology_machine_P.Integrator_x1_IC;

    /* InitializeConditions for Integrator: '<S54>/Integrator_x2' */
    sfc_6p_6p_topology_machine_X.Integrator_x2_CSTATE =
      sfc_6p_6p_topology_machine_P.Integrator_x2_IC;

    /* InitializeConditions for UnitDelay: '<S104>/dw_delay' */
    sfc_6p_6p_topology_machine_DW.dw_delay_DSTATE =
      sfc_6p_6p_topology_machine_P.dw_delay_InitialCondition;

    /* InitializeConditions for UnitDelay: '<S104>/dw_predict' */
    sfc_6p_6p_topology_machine_DW.dw_predict_DSTATE =
      sfc_6p_6p_topology_machine_P.dw_predict_InitialCondition;

    /* InitializeConditions for DiscreteIntegrator: '<S67>/theta' */
    sfc_6p_6p_topology_machine_DW.theta_DSTATE =
      sfc_6p_6p_topology_machine_P.theta_IC;

    /* InitializeConditions for UnitDelay: '<S80>/voltages' */
    for (i = 0; i < 6; i++) {
      sfc_6p_6p_topology_machine_DW.voltages_DSTATE[i] =
        sfc_6p_6p_topology_machine_P.voltages_InitialCondition;
    }

    /* End of InitializeConditions for UnitDelay: '<S80>/voltages' */

    /* InitializeConditions for DiscreteIntegrator: '<S103>/Rotor spee<d deviation (dw)' */
    sfc_6p_6p_topology_machine_DW.Rotorspeeddeviationdw_IC_LOADIN = 1U;

    /* SystemInitialize for Enabled SubSystem: '<S69>/Saturation' */
    /* InitializeConditions for UnitDelay: '<S81>/Lmd_sat' */
    sfc_6p_6p_topology_machine_DW.Lmd_sat_DSTATE =
      sfc_6p_6p_topology_machine_P.Lmd_sat_InitialCondition;

    /* SystemInitialize for Enabled SubSystem: '<S79>/Lmq_sat' */
    /* InitializeConditions for UnitDelay: '<S82>/Lmq_sat' */
    sfc_6p_6p_topology_machine_DW.Lmq_sat_DSTATE =
      sfc_6p_6p_topology_machine_P.Lmq_sat_InitialCondition;

    /* SystemInitialize for Gain: '<S82>/Lmq' incorporates:
     *  Outport: '<S82>/Lmq_sat '
     */
    sfc_6p_6p_topology_machine_B.Lmsatq =
      sfc_6p_6p_topology_machine_P.Lmq_sat_Y0;

    /* SystemInitialize for Math: '<S86>/Math Function1' incorporates:
     *  Outport: '<S82>/Laq_'
     *
     * About '<S86>/Math Function1':
     *  Operator: reciprocal
     */
    sfc_6p_6p_topology_machine_B.MathFunction1 =
      sfc_6p_6p_topology_machine_P.Laq_Y0;

    /* End of SystemInitialize for SubSystem: '<S79>/Lmq_sat' */
    for (i = 0; i < 36; i++) {
      /* SystemInitialize for Product: '<S79>/inversion' incorporates:
       *  Outport: '<S79>/Linv'
       */
      sfc_6p_6p_topology_machine_B.Linv_d[i] =
        sfc_6p_6p_topology_machine_P.Linv_Y0;

      /* SystemInitialize for Product: '<S79>/Product1' incorporates:
       *  Outport: '<S79>/R*Linv'
       */
      sfc_6p_6p_topology_machine_B.RLinv[i] =
        sfc_6p_6p_topology_machine_P.RLinv_Y0;
    }

    /* SystemInitialize for Switch: '<S79>/Switch1' incorporates:
     *  Outport: '<S79>/Lmqd'
     */
    sfc_6p_6p_topology_machine_B.Switch1_j =
      sfc_6p_6p_topology_machine_P.Lmqd_Y0;

    /* SystemInitialize for Gain: '<S81>/Lmd' incorporates:
     *  Outport: '<S79>/Lmqd'
     */
    sfc_6p_6p_topology_machine_B.Lmsatd = sfc_6p_6p_topology_machine_P.Lmqd_Y0;

    /* SystemInitialize for Switch: '<S79>/Switch2' incorporates:
     *  Outport: '<S79>/Laqd'
     */
    sfc_6p_6p_topology_machine_B.Switch2_a =
      sfc_6p_6p_topology_machine_P.Laqd_Y0;

    /* SystemInitialize for Math: '<S84>/Math Function3' incorporates:
     *  Outport: '<S79>/Laqd'
     *
     * About '<S84>/Math Function3':
     *  Operator: reciprocal
     */
    sfc_6p_6p_topology_machine_B.MathFunction3 =
      sfc_6p_6p_topology_machine_P.Laqd_Y0;

    /* End of SystemInitialize for SubSystem: '<S69>/Saturation' */

    /* SystemInitialize for Enabled SubSystem: '<S98>/Subsystem1' */
    /* SystemInitialize for Fcn: '<S102>/Fcn' incorporates:
     *  Outport: '<S102>/alpha_beta'
     */
    sfc_6p_6p_topology_machine_B.Fcn_m =
      sfc_6p_6p_topology_machine_P.alpha_beta_Y0_l[0];

    /* SystemInitialize for Fcn: '<S102>/Fcn1' incorporates:
     *  Outport: '<S102>/alpha_beta'
     */
    sfc_6p_6p_topology_machine_B.Fcn1 =
      sfc_6p_6p_topology_machine_P.alpha_beta_Y0_l[1];

    /* End of SystemInitialize for SubSystem: '<S98>/Subsystem1' */

    /* SystemInitialize for Enabled SubSystem: '<S98>/Subsystem - pi//2 delay' */
    /* SystemInitialize for Fcn: '<S101>/Fcn' incorporates:
     *  Outport: '<S101>/alpha_beta'
     */
    sfc_6p_6p_topology_machine_B.Fcn_p =
      sfc_6p_6p_topology_machine_P.alpha_beta_Y0[0];

    /* SystemInitialize for Fcn: '<S101>/Fcn1' incorporates:
     *  Outport: '<S101>/alpha_beta'
     */
    sfc_6p_6p_topology_machine_B.Fcn1_h =
      sfc_6p_6p_topology_machine_P.alpha_beta_Y0[1];

    /* End of SystemInitialize for SubSystem: '<S98>/Subsystem - pi//2 delay' */

    /* SystemInitialize for Enabled SubSystem: '<S36>/Automatic Gain Control' */
    /* Start for VariableTransportDelay: '<S46>/Variable Transport Delay' */
    {
      real_T *pBuffer =
        &sfc_6p_6p_topology_machine_DW.VariableTransportDelay_RWORK_b.TUbufferArea
        [0];
      int_T j;
      sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK_g.Tail = 0;
      sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK_g.Head = 0;
      sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK_g.Last = 0;
      sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK_g.CircularBufSize
        = 8192;
      for (j=0; j < 8192; j++) {
        pBuffer[j] =
          (sfc_6p_6p_topology_machine_P.VariableTransportDelay_InitOutp);
        pBuffer[8192 + j] = (&sfc_6p_6p_topology_machine_M)->Timing.t[0];
      }

      pBuffer[2*8192] = 0.0;
      sfc_6p_6p_topology_machine_DW.VariableTransportDelay_PWORK_g.TUbufferPtrs
        [0] = (void *) &pBuffer[0];
    }

    /* Start for VariableTransportDelay: '<S47>/Variable Transport Delay' */
    {
      real_T *pBuffer =
        &sfc_6p_6p_topology_machine_DW.VariableTransportDelay_RWORK_p.TUbufferArea
        [0];
      int_T j;
      sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK_a.Tail = 0;
      sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK_a.Head = 0;
      sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK_a.Last = 0;
      sfc_6p_6p_topology_machine_DW.VariableTransportDelay_IWORK_a.CircularBufSize
        = 8192;
      for (j=0; j < 8192; j++) {
        pBuffer[j] =
          (sfc_6p_6p_topology_machine_P.VariableTransportDelay_InitOu_g);
        pBuffer[8192 + j] = (&sfc_6p_6p_topology_machine_M)->Timing.t[0];
      }

      pBuffer[2*8192] = 0.0;
      sfc_6p_6p_topology_machine_DW.VariableTransportDelay_PWORK_j.TUbufferPtrs
        [0] = (void *) &pBuffer[0];
    }

    /* InitializeConditions for VariableTransportDelay: '<S46>/Variable Transport Delay' */
    sfc_6p_6p_topology_machine_X.VariableTransportDelay_CSTATE_c = 0.0;

    /* InitializeConditions for Integrator: '<S46>/integrator' */
    sfc_6p_6p_topology_machine_X.integrator_CSTATE_g =
      sfc_6p_6p_topology_machine_P.integrator_IC;

    /* InitializeConditions for VariableTransportDelay: '<S47>/Variable Transport Delay' */
    sfc_6p_6p_topology_machine_X.VariableTransportDelay_CSTATE_l = 0.0;

    /* InitializeConditions for Integrator: '<S47>/integrator' */
    sfc_6p_6p_topology_machine_X.integrator_CSTATE_b =
      sfc_6p_6p_topology_machine_P.integrator_IC_m;

    /* SystemInitialize for Enabled SubSystem: '<S48>/Subsystem - pi//2 delay' */
    sfc_6p_6_Subsystempi2delay_Init
      (&sfc_6p_6p_topology_machine_B.Subsystempi2delay_k,
       &sfc_6p_6p_topology_machine_P.Subsystempi2delay_k);

    /* End of SystemInitialize for SubSystem: '<S48>/Subsystem - pi//2 delay' */

    /* SystemInitialize for Enabled SubSystem: '<S48>/Subsystem1' */
    sfc_6p_6p_topol_Subsystem1_Init(&sfc_6p_6p_topology_machine_B.Subsystem1_ka,
      &sfc_6p_6p_topology_machine_P.Subsystem1_ka);

    /* End of SystemInitialize for SubSystem: '<S48>/Subsystem1' */

    /* SystemInitialize for Math: '<S37>/Math Function' incorporates:
     *  Outport: '<S37>/Gain'
     *
     * About '<S37>/Math Function':
     *  Operator: reciprocal
     */
    sfc_6p_6p_topology_machine_B.MathFunction_c =
      sfc_6p_6p_topology_machine_P.Gain_Y0;

    /* End of SystemInitialize for SubSystem: '<S36>/Automatic Gain Control' */

    /* SystemInitialize for Enabled SubSystem: '<S59>/Subsystem - pi//2 delay' */
    sfc_6p_6_Subsystempi2delay_Init
      (&sfc_6p_6p_topology_machine_B.Subsystempi2delay,
       &sfc_6p_6p_topology_machine_P.Subsystempi2delay);

    /* End of SystemInitialize for SubSystem: '<S59>/Subsystem - pi//2 delay' */

    /* SystemInitialize for Enabled SubSystem: '<S59>/Subsystem1' */
    sfc_6p_6p_topol_Subsystem1_Init(&sfc_6p_6p_topology_machine_B.Subsystem1,
      &sfc_6p_6p_topology_machine_P.Subsystem1);

    /* End of SystemInitialize for SubSystem: '<S59>/Subsystem1' */

    /* SystemInitialize for Enabled SubSystem: '<S90>/Subsystem1' */
    /* SystemInitialize for Fcn: '<S95>/Fcn' incorporates:
     *  Outport: '<S95>/dq'
     */
    sfc_6p_6p_topology_machine_B.Fcn_b = sfc_6p_6p_topology_machine_P.dq_Y0_m[0];

    /* SystemInitialize for Fcn: '<S95>/Fcn1' incorporates:
     *  Outport: '<S95>/dq'
     */
    sfc_6p_6p_topology_machine_B.Fcn1_j = sfc_6p_6p_topology_machine_P.dq_Y0_m[1];

    /* End of SystemInitialize for SubSystem: '<S90>/Subsystem1' */

    /* SystemInitialize for Enabled SubSystem: '<S90>/Subsystem - pi//2 delay' */
    /* SystemInitialize for Fcn: '<S94>/Fcn' incorporates:
     *  Outport: '<S94>/dq'
     */
    sfc_6p_6p_topology_machine_B.Fcn_bn = sfc_6p_6p_topology_machine_P.dq_Y0[0];

    /* SystemInitialize for Fcn: '<S94>/Fcn1' incorporates:
     *  Outport: '<S94>/dq'
     */
    sfc_6p_6p_topology_machine_B.Fcn1_p = sfc_6p_6p_topology_machine_P.dq_Y0[1];

    /* End of SystemInitialize for SubSystem: '<S90>/Subsystem - pi//2 delay' */

    /* set "at time zero" to false */
    if (rtmIsFirstInitCond((&sfc_6p_6p_topology_machine_M))) {
      rtmSetFirstInitCond((&sfc_6p_6p_topology_machine_M), 0);
    }
  }
}

/* Model terminate function */
void sfc_6p_6p_topology_machine::terminate()
{
  /* Terminate for S-Function (sfun_spssw_discc): '<S158>/State-Space' incorporates:
   *  Constant: '<S160>/SwitchCurrents'
   *  Inport: '<Root>/pulses_in_grid'
   *  Inport: '<Root>/pulses_in_motor'
   */

  /* S-Function block: <S158>/State-Space */
  {
    /* Free memory */
    free(sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.AS);
    free(sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.BS);
    free(sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.CS);
    free(sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.DS);
    free(sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.DX_COL);
    free(sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.TMP2);
    free(sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.BD_COL);
    free(sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.TMP1);
    free(sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.XTMP);

    /*
     * Circuit has switches*/
    free(sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.G_STATE);
    free(sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.SWITCH_STATUS);
    free(sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.SW_CHG);
    free(sfc_6p_6p_topology_machine_DW.StateSpace_PWORK.SWITCH_STATUS_INIT);
  }
}

/* Constructor */
sfc_6p_6p_topology_machine::sfc_6p_6p_topology_machine() :
  sfc_6p_6p_topology_machine_U(),
  sfc_6p_6p_topology_machine_Y(),
  sfc_6p_6p_topology_machine_B(),
  sfc_6p_6p_topology_machine_DW(),
  sfc_6p_6p_topology_machine_X(),
  sfc_6p_6p_topology_machine_XDis(),
  sfc_6p_6p_topology_mach_PrevZCX(),
  sfc_6p_6p_topology_machine_M()
{
  /* Currently there is no constructor body generated.*/
}

/* Destructor */
sfc_6p_6p_topology_machine::~sfc_6p_6p_topology_machine()
{
  /* Currently there is no destructor body generated.*/
}

/* Real-Time Model get method */
RT_MODEL_sfc_6p_6p_topology_m_T * sfc_6p_6p_topology_machine::getRTM()
{
  return (&sfc_6p_6p_topology_machine_M);
}
