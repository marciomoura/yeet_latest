/*
 * LCITopology6x6Minimal.cpp
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

#include "LCITopology6x6Minimal.h"
#include "LCITopology6x6Minimal_private.h"

/* user code (top of source file) */
#include <string.h>

/* For variable transport delay block, find the real delay time */
static real_T rt_VTDelayfindtDInterpolate(
  real_T x,real_T* tBuf,real_T* uBuf,real_T* xBuf,int_T bufSz,int_T head,int_T
  tail,int_T* pLast,real_T t,real_T tStart,boolean_T discrete,boolean_T
  minorStepAndTAtLastMajorOutput,real_T initOutput,real_T* appliedDelay)
{
  int_T n, k;
  real_T f;
  int_T kp1;
  real_T tminustD, tL, tR, uD, uL, uR, fU;
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
  uint32_T found;
  uint32_T iLeft;
  uint32_T iRght;

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
void LCITopology6x6MinimalModelClass::rt_ertODEUpdateContinuousStates
  (RTWSolverInfo *si )
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
  LCITopology6x6Minimal_derivatives();
  rtsiSetT(si, tnew);
  for (i = 0; i < nXc; ++i) {
    x[i] += h * f0[i];
  }

  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

/*
 * System initialize for enable system:
 *    '<S54>/Subsystem - pi//2 delay'
 *    '<S65>/Subsystem - pi//2 delay'
 */
void LCITopology6x6MinimalModelClass::LCITopol_Subsystempi2delay_Init
  (B_Subsystempi2delay_LCITopolo_T *localB, P_Subsystempi2delay_LCITopolo_T
   *localP)
{
  /* SystemInitialize for Fcn: '<S58>/Fcn' incorporates:
   *  Outport: '<S58>/dq'
   */
  localB->Fcn = localP->dq_Y0[0];

  /* SystemInitialize for Fcn: '<S58>/Fcn1' incorporates:
   *  Outport: '<S58>/dq'
   */
  localB->Fcn1 = localP->dq_Y0[1];
}

/*
 * Disable for enable system:
 *    '<S54>/Subsystem - pi//2 delay'
 *    '<S65>/Subsystem - pi//2 delay'
 */
void LCITopology6x6MinimalModelClass::LCITo_Subsystempi2delay_Disable
  (DW_Subsystempi2delay_LCITopol_T *localDW)
{
  localDW->Subsystempi2delay_MODE = false;
}

/*
 * Output and update for enable system:
 *    '<S54>/Subsystem - pi//2 delay'
 *    '<S65>/Subsystem - pi//2 delay'
 */
void LCITopology6x6MinimalModelClass::LCITopology6x_Subsystempi2delay(uint8_T
  rtu_Enable, const real_T rtu_alpha_beta[2], real_T rtu_wt,
  B_Subsystempi2delay_LCITopolo_T *localB, DW_Subsystempi2delay_LCITopol_T
  *localDW)
{
  /* Outputs for Enabled SubSystem: '<S54>/Subsystem - pi//2 delay' incorporates:
   *  EnablePort: '<S58>/Enable'
   */
  if (rtmIsMajorTimeStep((&LCITopology6x6Minimal_M)) && rtmIsMajorTimeStep
      ((&LCITopology6x6Minimal_M))) {
    if (rtu_Enable > 0) {
      localDW->Subsystempi2delay_MODE = true;
    } else {
      if (localDW->Subsystempi2delay_MODE) {
        LCITo_Subsystempi2delay_Disable(localDW);
      }
    }
  }

  if (localDW->Subsystempi2delay_MODE) {
    /* Fcn: '<S58>/Fcn' */
    localB->Fcn = rtu_alpha_beta[0] * std::sin(rtu_wt) - rtu_alpha_beta[1] * std::
      cos(rtu_wt);

    /* Fcn: '<S58>/Fcn1' */
    localB->Fcn1 = rtu_alpha_beta[0] * std::cos(rtu_wt) + rtu_alpha_beta[1] *
      std::sin(rtu_wt);
  }

  /* End of Outputs for SubSystem: '<S54>/Subsystem - pi//2 delay' */
}

/*
 * System initialize for enable system:
 *    '<S54>/Subsystem1'
 *    '<S65>/Subsystem1'
 */
void LCITopology6x6MinimalModelClass::LCITopology6x6M_Subsystem1_Init
  (B_Subsystem1_LCITopology6x6Mi_T *localB, P_Subsystem1_LCITopology6x6Mi_T
   *localP)
{
  /* SystemInitialize for Fcn: '<S59>/Fcn' incorporates:
   *  Outport: '<S59>/dq'
   */
  localB->Fcn = localP->dq_Y0[0];

  /* SystemInitialize for Fcn: '<S59>/Fcn1' incorporates:
   *  Outport: '<S59>/dq'
   */
  localB->Fcn1 = localP->dq_Y0[1];
}

/*
 * Disable for enable system:
 *    '<S54>/Subsystem1'
 *    '<S65>/Subsystem1'
 */
void LCITopology6x6MinimalModelClass::LCITopology6_Subsystem1_Disable
  (DW_Subsystem1_LCITopology6x6M_T *localDW)
{
  localDW->Subsystem1_MODE = false;
}

/*
 * Output and update for enable system:
 *    '<S54>/Subsystem1'
 *    '<S65>/Subsystem1'
 */
void LCITopology6x6MinimalModelClass::LCITopology6x6Minima_Subsystem1(uint8_T
  rtu_Enable, const real_T rtu_alpha_beta[2], real_T rtu_wt,
  B_Subsystem1_LCITopology6x6Mi_T *localB, DW_Subsystem1_LCITopology6x6M_T
  *localDW)
{
  /* Outputs for Enabled SubSystem: '<S54>/Subsystem1' incorporates:
   *  EnablePort: '<S59>/Enable'
   */
  if (rtmIsMajorTimeStep((&LCITopology6x6Minimal_M)) && rtmIsMajorTimeStep
      ((&LCITopology6x6Minimal_M))) {
    if (rtu_Enable > 0) {
      localDW->Subsystem1_MODE = true;
    } else {
      if (localDW->Subsystem1_MODE) {
        LCITopology6_Subsystem1_Disable(localDW);
      }
    }
  }

  if (localDW->Subsystem1_MODE) {
    /* Fcn: '<S59>/Fcn' */
    localB->Fcn = rtu_alpha_beta[0] * std::cos(rtu_wt) + rtu_alpha_beta[1] * std::
      sin(rtu_wt);

    /* Fcn: '<S59>/Fcn1' */
    localB->Fcn1 = -rtu_alpha_beta[0] * std::sin(rtu_wt) + rtu_alpha_beta[1] *
      std::cos(rtu_wt);
  }

  /* End of Outputs for SubSystem: '<S54>/Subsystem1' */
}

void rt_invd5x5_snf(const real_T u[25], real_T y[25])
{
  real_T u_0[25];
  real_T x[25];
  real_T s;
  real_T smax;
  real_T u_1;
  int32_T Aoffset;
  int32_T ONE;
  int32_T b;
  int32_T ipk;
  int32_T ix;
  int32_T jj;
  int32_T jm1;
  int32_T jp1j;
  int32_T jpiv_offset;
  int32_T mmj;
  int32_T pipk;
  int8_T ipiv[5];
  int8_T p[5];
  for (ix = 0; ix < 25; ix++) {
    u_1 = u[ix];
    y[ix] = 0.0;
    u_0[ix] = u_1;
  }

  for (ix = 0; ix < 5; ix++) {
    ipiv[ix] = static_cast<int8_T>(ix + 1);
  }

  for (ipk = 0; ipk < 4; ipk++) {
    pipk = ipk + 1;
    jm1 = pipk - 1;
    mmj = 5 - pipk;
    Aoffset = jm1 * 6;
    ONE = 1;
    jj = Aoffset + 1;
    jp1j = jj + 1;
    Aoffset = mmj + 1;
    for (ix = 0; ix < 25; ix++) {
      u_1 = u_0[ix];
      x[ix] = u_1;
    }

    ix = jj - 1;
    u_1 = x[jj - 1];
    s = std::abs(u_1);
    smax = s;
    for (jpiv_offset = 2; jpiv_offset <= Aoffset; jpiv_offset++) {
      ix++;
      u_1 = x[ix];
      s = std::abs(u_1);
      if (s > smax) {
        ONE = jpiv_offset;
        smax = s;
      }
    }

    jpiv_offset = ONE - 1;
    ONE = (jj + jpiv_offset) - 1;
    if (u_0[ONE] != 0.0) {
      if (jpiv_offset != 0) {
        Aoffset = pipk + jpiv_offset;
        ipiv[pipk - 1] = static_cast<int8_T>(Aoffset);
        ONE = jm1 + jpiv_offset;
        for (jpiv_offset = 0; jpiv_offset < 5; jpiv_offset++) {
          s = u_0[jm1];
          u_0[jm1] = u_0[ONE];
          u_0[ONE] = s;
          jm1 += 5;
          ONE += 5;
        }
      }

      Aoffset = mmj - 1;
      jm1 = jp1j + Aoffset;
      for (b = jp1j; b <= jm1; b++) {
        u_1 = u_0[b - 1];
        s = u_0[jj - 1];
        smax = u_1 / s;
        u_0[b - 1] = smax;
      }
    }

    Aoffset = 4 - pipk;
    jpiv_offset = jj + 4;
    pipk = jj + 6;
    ONE = pipk - 1;
    for (pipk = 0; pipk <= Aoffset; pipk++) {
      s = u_0[jpiv_offset];
      if (s != 0.0) {
        s = -s;
        ix = jp1j - 1;
        jm1 = ONE + 1;
        jj = mmj + ONE;
        for (b = jm1; b <= jj; b++) {
          u_0[b - 1] += u_0[ix] * s;
          ix++;
        }
      }

      jpiv_offset += 5;
      ONE += 5;
    }
  }

  for (ix = 0; ix < 5; ix++) {
    p[ix] = static_cast<int8_T>(ix + 1);
  }

  ipk = ipiv[0] - 1;
  if (ipk + 1 > 1) {
    pipk = p[ipk];
    p[ipk] = p[0];
    p[0] = static_cast<int8_T>(pipk);
  }

  ipk = ipiv[1] - 1;
  if (ipk + 1 > 2) {
    pipk = p[ipk];
    p[ipk] = p[1];
    p[1] = static_cast<int8_T>(pipk);
  }

  ipk = ipiv[2] - 1;
  if (ipk + 1 > 3) {
    pipk = p[ipk];
    p[ipk] = p[2];
    p[2] = static_cast<int8_T>(pipk);
  }

  ipk = ipiv[3] - 1;
  if (ipk + 1 > 4) {
    pipk = p[ipk];
    p[ipk] = p[3];
    p[3] = static_cast<int8_T>(pipk);
  }

  for (jpiv_offset = 0; jpiv_offset < 5; jpiv_offset++) {
    pipk = jpiv_offset;
    Aoffset = p[pipk] - 1;
    y[pipk + 5 * Aoffset] = 1.0;
    for (ipk = pipk + 1; ipk < 6; ipk++) {
      if (y[(5 * Aoffset + ipk) - 1] != 0.0) {
        jm1 = ipk + 1;
        for (b = jm1; b < 6; b++) {
          y[(b + 5 * Aoffset) - 1] -= u_0[((ipk - 1) * 5 + b) - 1] * y[(5 *
            Aoffset + ipk) - 1];
        }
      }
    }
  }

  for (ipk = 0; ipk < 5; ipk++) {
    pipk = ipk;
    pipk *= 5;
    for (jpiv_offset = 4; jpiv_offset >= 0; jpiv_offset--) {
      ix = 5 * jpiv_offset;
      if (y[jpiv_offset + pipk] != 0.0) {
        y[jpiv_offset + pipk] /= u_0[jpiv_offset + ix];
        jm1 = jpiv_offset - 1;
        for (b = 0; b <= jm1; b++) {
          y[b + pipk] -= y[jpiv_offset + pipk] * u_0[b + ix];
        }
      }
    }
  }
}

static real_T rt_modd_snf(real_T u0, real_T u1)
{
  real_T q;
  real_T y;
  boolean_T yEq;
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
    y = std::fmod(u0, u1);
    yEq = (y == 0.0);
    if ((!yEq) && (u1 > std::floor(u1))) {
      q = std::abs(u0 / u1);
      yEq = !(std::abs(q - std::floor(q + 0.5)) > DBL_EPSILON * q);
    }

    if (yEq) {
      y = u1 * 0.0;
    } else {
      if ((u0 < 0.0) != (u1 < 0.0)) {
        y += u1;
      }
    }
  }

  return y;
}

static real_T rt_hypotd_snf(real_T u0, real_T u1)
{
  real_T a;
  real_T y;
  a = std::abs(u0);
  y = std::abs(u1);
  if (a < y) {
    a /= y;
    y *= std::sqrt(a * a + 1.0);
  } else if (a > y) {
    y /= a;
    y = std::sqrt(y * y + 1.0) * a;
  } else {
    if (!rtIsNaN(y)) {
      y = a * 1.4142135623730951;
    }
  }

  return y;
}

static real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  int32_T tmp;
  int32_T tmp_0;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
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

static real_T rt_remd_snf(real_T u0, real_T u1)
{
  real_T q;
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
void LCITopology6x6MinimalModelClass::step()
{
  if (rtmIsMajorTimeStep((&LCITopology6x6Minimal_M))) {
    /* set solver stop time */
    if (!((&LCITopology6x6Minimal_M)->Timing.clockTick0+1)) {
      rtsiSetSolverStopTime(&(&LCITopology6x6Minimal_M)->solverInfo,
                            (((&LCITopology6x6Minimal_M)->Timing.clockTickH0 + 1)
        * (&LCITopology6x6Minimal_M)->Timing.stepSize0 * 4294967296.0));
    } else {
      rtsiSetSolverStopTime(&(&LCITopology6x6Minimal_M)->solverInfo,
                            (((&LCITopology6x6Minimal_M)->Timing.clockTick0 + 1)
        * (&LCITopology6x6Minimal_M)->Timing.stepSize0 +
        (&LCITopology6x6Minimal_M)->Timing.clockTickH0 *
        (&LCITopology6x6Minimal_M)->Timing.stepSize0 * 4294967296.0));
    }
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep((&LCITopology6x6Minimal_M))) {
    (&LCITopology6x6Minimal_M)->Timing.t[0] = rtsiGetT
      (&(&LCITopology6x6Minimal_M)->solverInfo);
  }

  {
    real_T tmp_1[25];
    real_T tmp_2[25];
    real_T tmp[5];
    real_T tmp_0[5];
    real_T tmpForInput_idx_0;
    real_T tmpForInput_idx_1;
    real_T tmpForInput_idx_2;
    real_T tmpForInput_idx_3;
    int32_T i;
    int32_T i_0;
    int32_T i_1;
    boolean_T didZcEventOccur;
    if (rtmIsMajorTimeStep((&LCITopology6x6Minimal_M))) {
      for (i = 0; i < 5; i++) {
        /* UnitDelay: '<S86>/fluxes' */
        LCITopology6x6Minimal_B.xk1[i] =
          LCITopology6x6Minimal_DW.fluxes_DSTATE[i];
      }

      /* Outputs for Enabled SubSystem: '<S75>/Saturation' incorporates:
       *  EnablePort: '<S85>/Enable'
       */
      /* Constant: '<S75>/Constant1' */
      if (LCITopology6x6Minimal_P.Constant1_Value_l > 0.0) {
        /* Gain: '<S91>/1//Ll_d' */
        LCITopology6x6Minimal_B.uLl_d_h[0] = LCITopology6x6Minimal_P.uLl_d_Gain
          [0] * LCITopology6x6Minimal_B.xk1[1];
        LCITopology6x6Minimal_B.uLl_d_h[1] = LCITopology6x6Minimal_P.uLl_d_Gain
          [1] * LCITopology6x6Minimal_B.xk1[2];
        LCITopology6x6Minimal_B.uLl_d_h[2] = LCITopology6x6Minimal_P.uLl_d_Gain
          [2] * LCITopology6x6Minimal_B.xk1[3];

        /* Sum: '<S91>/sum2' */
        tmpForInput_idx_0 = LCITopology6x6Minimal_B.uLl_d_h[0];
        tmpForInput_idx_0 += LCITopology6x6Minimal_B.uLl_d_h[1];
        tmpForInput_idx_0 += LCITopology6x6Minimal_B.uLl_d_h[2];

        /* Sum: '<S91>/sum2' */
        LCITopology6x6Minimal_B.phimd_g = tmpForInput_idx_0;

        /* UnitDelay: '<S87>/Lmd_sat' */
        LCITopology6x6Minimal_B.Lmd_sat =
          LCITopology6x6Minimal_DW.Lmd_sat_DSTATE;

        /* Math: '<S90>/Math Function2'
         *
         * About '<S90>/Math Function2':
         *  Operator: reciprocal
         */
        tmpForInput_idx_0 = LCITopology6x6Minimal_B.Lmd_sat;

        /* Math: '<S90>/Math Function2'
         *
         * About '<S90>/Math Function2':
         *  Operator: reciprocal
         */
        LCITopology6x6Minimal_B.MathFunction2 = 1.0 / tmpForInput_idx_0;

        /* Sum: '<S90>/Sum1' incorporates:
         *  Constant: '<S90>/u1'
         */
        tmpForInput_idx_0 = LCITopology6x6Minimal_P.u1_Value[0];
        tmpForInput_idx_1 = LCITopology6x6Minimal_P.u1_Value[1];
        tmpForInput_idx_2 = LCITopology6x6Minimal_B.MathFunction2;
        tmpForInput_idx_0 += tmpForInput_idx_1;
        tmpForInput_idx_0 += tmpForInput_idx_2;

        /* Sum: '<S90>/Sum1' */
        LCITopology6x6Minimal_B.Sum1_bq = tmpForInput_idx_0;

        /* Math: '<S90>/Math Function3'
         *
         * About '<S90>/Math Function3':
         *  Operator: reciprocal
         */
        tmpForInput_idx_0 = LCITopology6x6Minimal_B.Sum1_bq;

        /* Math: '<S90>/Math Function3'
         *
         * About '<S90>/Math Function3':
         *  Operator: reciprocal
         */
        LCITopology6x6Minimal_B.MathFunction3 = 1.0 / tmpForInput_idx_0;

        /* Product: '<S91>/Product4' */
        LCITopology6x6Minimal_B.phimd_n = LCITopology6x6Minimal_B.phimd_g *
          LCITopology6x6Minimal_B.MathFunction3;

        /* Abs: '<S87>/Abs' */
        LCITopology6x6Minimal_B.Phisat = std::abs
          (LCITopology6x6Minimal_B.phimd_n);

        /* Lookup_n-D: '<S87>/1-D Lookup Table' incorporates:
         *  Abs: '<S87>/Abs'
         */
        LCITopology6x6Minimal_B.Ifdsat = look1_pbinlxpw
          (LCITopology6x6Minimal_B.Phisat,
           LCITopology6x6Minimal_P.uDLookupTable_bp01Data_i,
           LCITopology6x6Minimal_P.uDLookupTable_tableData_a,
           &LCITopology6x6Minimal_DW.m_bpIndex, 1U);

        /* Switch: '<S87>/Switch' */
        if (LCITopology6x6Minimal_B.Ifdsat != 0.0) {
          /* Product: '<S87>/Product' */
          LCITopology6x6Minimal_B.Product_n = LCITopology6x6Minimal_B.Phisat /
            LCITopology6x6Minimal_B.Ifdsat;

          /* Switch: '<S87>/Switch' */
          LCITopology6x6Minimal_B.Switch_d = LCITopology6x6Minimal_B.Product_n;
        } else {
          /* Switch: '<S87>/Switch' incorporates:
           *  Constant: '<S87>/Constant1'
           */
          LCITopology6x6Minimal_B.Switch_d =
            LCITopology6x6Minimal_P.Constant1_Value;
        }

        /* End of Switch: '<S87>/Switch' */

        /* Gain: '<S87>/Lmd' */
        LCITopology6x6Minimal_B.Lmsatd = LCITopology6x6Minimal_P.Lmd_Gain *
          LCITopology6x6Minimal_B.Switch_d;

        /* Outputs for Enabled SubSystem: '<S85>/Lmq_sat' incorporates:
         *  EnablePort: '<S88>/Enable'
         */
        /* Constant: '<S85>/Constant1' */
        if (LCITopology6x6Minimal_P.Constant1_Value_n) {
          /* Gain: '<S93>/1//Ll_q' */
          LCITopology6x6Minimal_B.uLl_q_i[0] =
            LCITopology6x6Minimal_P.uLl_q_Gain[0] * LCITopology6x6Minimal_B.xk1
            [0];
          LCITopology6x6Minimal_B.uLl_q_i[1] =
            LCITopology6x6Minimal_P.uLl_q_Gain[1] * LCITopology6x6Minimal_B.xk1
            [4];

          /* Sum: '<S93>/sum1' */
          tmpForInput_idx_0 = LCITopology6x6Minimal_B.uLl_q_i[0];
          tmpForInput_idx_0 += LCITopology6x6Minimal_B.uLl_q_i[1];

          /* Sum: '<S93>/sum1' */
          LCITopology6x6Minimal_B.phimq_p = tmpForInput_idx_0;

          /* UnitDelay: '<S88>/Lmq_sat' */
          LCITopology6x6Minimal_B.Lmq_sat =
            LCITopology6x6Minimal_DW.Lmq_sat_DSTATE;

          /* Math: '<S92>/Math Function'
           *
           * About '<S92>/Math Function':
           *  Operator: reciprocal
           */
          tmpForInput_idx_0 = LCITopology6x6Minimal_B.Lmq_sat;

          /* Math: '<S92>/Math Function'
           *
           * About '<S92>/Math Function':
           *  Operator: reciprocal
           */
          LCITopology6x6Minimal_B.MathFunction = 1.0 / tmpForInput_idx_0;

          /* Sum: '<S92>/Sum2' incorporates:
           *  Constant: '<S92>/u2'
           */
          tmpForInput_idx_0 = LCITopology6x6Minimal_P.u2_Value[0];
          tmpForInput_idx_1 = LCITopology6x6Minimal_P.u2_Value[1];
          tmpForInput_idx_2 = LCITopology6x6Minimal_P.u2_Value[2];
          tmpForInput_idx_3 = LCITopology6x6Minimal_B.MathFunction;
          tmpForInput_idx_0 += tmpForInput_idx_1;
          tmpForInput_idx_0 += tmpForInput_idx_2;
          tmpForInput_idx_0 += tmpForInput_idx_3;

          /* Sum: '<S92>/Sum2' */
          LCITopology6x6Minimal_B.Sum2_d = tmpForInput_idx_0;

          /* Math: '<S92>/Math Function1'
           *
           * About '<S92>/Math Function1':
           *  Operator: reciprocal
           */
          tmpForInput_idx_0 = LCITopology6x6Minimal_B.Sum2_d;

          /* Math: '<S92>/Math Function1'
           *
           * About '<S92>/Math Function1':
           *  Operator: reciprocal
           */
          LCITopology6x6Minimal_B.MathFunction1 = 1.0 / tmpForInput_idx_0;

          /* Product: '<S93>/Product3' */
          LCITopology6x6Minimal_B.phimq_i = LCITopology6x6Minimal_B.phimq_p *
            LCITopology6x6Minimal_B.MathFunction1;

          /* Abs: '<S88>/Abs' */
          LCITopology6x6Minimal_B.Phisat_a = std::abs
            (LCITopology6x6Minimal_B.phimq_i);

          /* Lookup_n-D: '<S88>/1-D Lookup Table' incorporates:
           *  Abs: '<S88>/Abs'
           */
          LCITopology6x6Minimal_B.Ifqsat = look1_pbinlxpw
            (LCITopology6x6Minimal_B.Phisat_a,
             LCITopology6x6Minimal_P.uDLookupTable_bp01Data,
             LCITopology6x6Minimal_P.uDLookupTable_tableData,
             &LCITopology6x6Minimal_DW.m_bpIndex_i, 1U);

          /* Switch: '<S88>/Switch' */
          if (LCITopology6x6Minimal_B.Ifqsat != 0.0) {
            /* Product: '<S88>/Product' */
            LCITopology6x6Minimal_B.Product_i = LCITopology6x6Minimal_B.Phisat_a
              / LCITopology6x6Minimal_B.Ifqsat;

            /* Switch: '<S88>/Switch' */
            LCITopology6x6Minimal_B.Switch_b = LCITopology6x6Minimal_B.Product_i;
          } else {
            /* Switch: '<S88>/Switch' incorporates:
             *  Constant: '<S88>/Constant1'
             */
            LCITopology6x6Minimal_B.Switch_b =
              LCITopology6x6Minimal_P.Constant1_Value_i;
          }

          /* End of Switch: '<S88>/Switch' */

          /* Gain: '<S88>/Lmq' */
          LCITopology6x6Minimal_B.Lmsatq = LCITopology6x6Minimal_P.Lmq_Gain *
            LCITopology6x6Minimal_B.Switch_b;

          /* Update for UnitDelay: '<S88>/Lmq_sat' */
          LCITopology6x6Minimal_DW.Lmq_sat_DSTATE =
            LCITopology6x6Minimal_B.Lmsatq;
        }

        /* End of Constant: '<S85>/Constant1' */
        /* End of Outputs for SubSystem: '<S85>/Lmq_sat' */

        /* Switch: '<S85>/Switch1' incorporates:
         *  Constant: '<S85>/Constant2'
         */
        if (LCITopology6x6Minimal_P.Constant2_Value_io) {
          /* Switch: '<S85>/Switch1' */
          LCITopology6x6Minimal_B.Switch1_j = LCITopology6x6Minimal_B.Lmsatq;
        } else {
          /* Switch: '<S85>/Switch1' incorporates:
           *  Constant: '<S85>/u3'
           */
          LCITopology6x6Minimal_B.Switch1_j = LCITopology6x6Minimal_P.u3_Value;
        }

        /* End of Switch: '<S85>/Switch1' */

        /* Assignment: '<S89>/Update Lmq rows[1,5, (6)] col[1,5, (6)] ' incorporates:
         *  Constant: '<S89>/u1'
         */
        std::memcpy(&LCITopology6x6Minimal_B.UpdateLmqrows156col156[0],
                    &LCITopology6x6Minimal_P.u1_Value_l[0], 25U * sizeof(real_T));

        /* Assignment: '<S89>/Update Lmq rows[1,5, (6)] col[1,5, (6)] ' */
        LCITopology6x6Minimal_B.UpdateLmqrows156col156[0] =
          LCITopology6x6Minimal_B.Switch1_j;
        LCITopology6x6Minimal_B.UpdateLmqrows156col156[4] =
          LCITopology6x6Minimal_B.Switch1_j;
        LCITopology6x6Minimal_B.UpdateLmqrows156col156[20] =
          LCITopology6x6Minimal_B.Switch1_j;
        LCITopology6x6Minimal_B.UpdateLmqrows156col156[24] =
          LCITopology6x6Minimal_B.Switch1_j;

        /* Assignment: '<S89>/Update Lmd rows[2,3,4] col[2,3,4]' incorporates:
         *  Assignment: '<S89>/Update Lmq rows[1,5, (6)] col[1,5, (6)] '
         */
        std::memcpy(&LCITopology6x6Minimal_B.UpdateLmdrows234col234[0],
                    &LCITopology6x6Minimal_B.UpdateLmqrows156col156[0], 25U *
                    sizeof(real_T));

        /* Assignment: '<S89>/Update Lmd rows[2,3,4] col[2,3,4]' */
        for (i_0 = 0; i_0 < 3; i_0++) {
          LCITopology6x6Minimal_B.UpdateLmdrows234col234[5 * (i_0 + 1) + 1] =
            LCITopology6x6Minimal_B.Lmsatd;
          LCITopology6x6Minimal_B.UpdateLmdrows234col234[5 * (i_0 + 1) + 2] =
            LCITopology6x6Minimal_B.Lmsatd;
          LCITopology6x6Minimal_B.UpdateLmdrows234col234[5 * (i_0 + 1) + 3] =
            LCITopology6x6Minimal_B.Lmsatd;
        }

        for (i = 0; i < 25; i++) {
          /* Sum: '<S89>/Sum2' incorporates:
           *  Assignment: '<S89>/Update Lmd rows[2,3,4] col[2,3,4]'
           *  Constant: '<S89>/u5'
           */
          LCITopology6x6Minimal_B.Sum2_h[i] =
            LCITopology6x6Minimal_B.UpdateLmdrows234col234[i] +
            LCITopology6x6Minimal_P.u5_Value[i];
        }

        /* Product: '<S85>/inversion' incorporates:
         *  Sum: '<S89>/Sum2'
         */
        rt_invd5x5_snf(LCITopology6x6Minimal_B.Sum2_h,
                       LCITopology6x6Minimal_B.Linv_d);

        /* Product: '<S85>/Product1' incorporates:
         *  Constant: '<S85>/u1'
         *  Product: '<S85>/inversion'
         */
        std::memcpy(&tmp_1[0], &LCITopology6x6Minimal_P.u1_Value_a[0], 25U *
                    sizeof(real_T));
        std::memcpy(&tmp_2[0], &LCITopology6x6Minimal_B.Linv_d[0], 25U * sizeof
                    (real_T));
        for (i_0 = 0; i_0 < 5; i_0++) {
          for (i = 0; i < 5; i++) {
            /* Product: '<S85>/Product1' */
            LCITopology6x6Minimal_B.RLinv[i + 5 * i_0] = 0.0;
            for (i_1 = 0; i_1 < 5; i_1++) {
              LCITopology6x6Minimal_B.RLinv[i + 5 * i_0] += tmp_1[5 * i_1 + i] *
                tmp_2[5 * i_0 + i_1];
            }
          }
        }

        /* Switch: '<S85>/Switch2' incorporates:
         *  Constant: '<S85>/Constant3'
         */
        if (LCITopology6x6Minimal_P.Constant3_Value_p) {
          /* Switch: '<S85>/Switch2' */
          LCITopology6x6Minimal_B.Switch2_a =
            LCITopology6x6Minimal_B.MathFunction1;
        } else {
          /* Switch: '<S85>/Switch2' incorporates:
           *  Constant: '<S85>/u2'
           */
          LCITopology6x6Minimal_B.Switch2_a = LCITopology6x6Minimal_P.u2_Value_n;
        }

        /* End of Switch: '<S85>/Switch2' */

        /* Update for UnitDelay: '<S87>/Lmd_sat' */
        LCITopology6x6Minimal_DW.Lmd_sat_DSTATE = LCITopology6x6Minimal_B.Lmsatd;
      }

      /* End of Constant: '<S75>/Constant1' */
      /* End of Outputs for SubSystem: '<S75>/Saturation' */
      for (i = 0; i < 25; i++) {
        /* Switch: '<S75>/Switch' incorporates:
         *  Constant: '<S75>/Constant3'
         */
        if (LCITopology6x6Minimal_P.Constant3_Value >=
            LCITopology6x6Minimal_P.Switch_Threshold) {
          /* Switch: '<S75>/Switch' incorporates:
           *  Product: '<S85>/inversion'
           */
          LCITopology6x6Minimal_B.Linv[i] = LCITopology6x6Minimal_B.Linv_d[i];
        } else {
          /* Switch: '<S75>/Switch' incorporates:
           *  Constant: '<S75>/Constant4'
           */
          LCITopology6x6Minimal_B.Linv[i] =
            LCITopology6x6Minimal_P.Constant4_Value[i];
        }

        /* End of Switch: '<S75>/Switch' */

        /* Product: '<S75>/Product3' incorporates:
         *  Switch: '<S75>/Switch'
         */
        tmp_1[i] = LCITopology6x6Minimal_B.Linv[i];
      }

      /* Product: '<S75>/Product3' */
      for (i = 0; i < 5; i++) {
        tmp[i] = LCITopology6x6Minimal_B.xk1[i];
      }

      for (i = 0; i < 5; i++) {
        tmp_0[i] = 0.0;
        for (i_0 = 0; i_0 < 5; i_0++) {
          tmpForInput_idx_1 = tmp_0[i];
          tmpForInput_idx_1 += tmp_1[5 * i_0 + i] * tmp[i_0];
          tmp_0[i] = tmpForInput_idx_1;
        }

        /* Product: '<S75>/Product3' */
        LCITopology6x6Minimal_B.Ik[i] = tmp_0[i];

        /* Gain: '<S75>/change Iq Id  current signs' */
        LCITopology6x6Minimal_B.changeIqIdcurrentsigns[i] =
          LCITopology6x6Minimal_P.changeIqIdcurrentsigns_Gain[i] *
          LCITopology6x6Minimal_B.Ik[i];
      }

      /* DiscreteIntegrator: '<S73>/Rotor angle dthetae' */
      LCITopology6x6Minimal_B.d_thetarad =
        LCITopology6x6Minimal_DW.Rotorangledthetae_DSTATE;

      /* DigitalClock: '<S73>/Digital Clock' */
      LCITopology6x6Minimal_B.DigitalClock = ((((&LCITopology6x6Minimal_M)
        ->Timing.clockTick1+(&LCITopology6x6Minimal_M)->Timing.clockTickH1*
        4294967296.0)) * 1.25E-5);

      /* Gain: '<S73>/web2' */
      LCITopology6x6Minimal_B.web2 = LCITopology6x6Minimal_P.web2_Gain *
        LCITopology6x6Minimal_B.DigitalClock;

      /* Sum: '<S73>/Sum3' */
      LCITopology6x6Minimal_B.Sum3 = LCITopology6x6Minimal_B.d_thetarad +
        LCITopology6x6Minimal_B.web2;

      /* RelationalOperator: '<S105>/Compare' incorporates:
       *  Constant: '<S104>/Constant'
       *  Constant: '<S105>/Constant'
       */
      LCITopology6x6Minimal_B.Compare =
        (LCITopology6x6Minimal_P.dq0toAlphaBetaZero_Alignment ==
         LCITopology6x6Minimal_P.CompareToConstant_const_b);

      /* Outputs for Enabled SubSystem: '<S104>/Subsystem1' incorporates:
       *  EnablePort: '<S108>/Enable'
       */
      if (LCITopology6x6Minimal_B.Compare > 0) {
        /* Fcn: '<S108>/Fcn' */
        LCITopology6x6Minimal_B.Fcn_m =
          LCITopology6x6Minimal_B.changeIqIdcurrentsigns[1] * std::cos
          (LCITopology6x6Minimal_B.Sum3) -
          LCITopology6x6Minimal_B.changeIqIdcurrentsigns[0] * std::sin
          (LCITopology6x6Minimal_B.Sum3);

        /* Fcn: '<S108>/Fcn1' */
        LCITopology6x6Minimal_B.Fcn1 =
          LCITopology6x6Minimal_B.changeIqIdcurrentsigns[1] * std::sin
          (LCITopology6x6Minimal_B.Sum3) +
          LCITopology6x6Minimal_B.changeIqIdcurrentsigns[0] * std::cos
          (LCITopology6x6Minimal_B.Sum3);
      }

      /* End of Outputs for SubSystem: '<S104>/Subsystem1' */

      /* RelationalOperator: '<S106>/Compare' incorporates:
       *  Constant: '<S104>/Constant'
       *  Constant: '<S106>/Constant'
       */
      LCITopology6x6Minimal_B.Compare_n =
        (LCITopology6x6Minimal_P.dq0toAlphaBetaZero_Alignment ==
         LCITopology6x6Minimal_P.CompareToConstant1_const_d);

      /* Outputs for Enabled SubSystem: '<S104>/Subsystem - pi//2 delay' incorporates:
       *  EnablePort: '<S107>/Enable'
       */
      if (LCITopology6x6Minimal_B.Compare_n > 0) {
        /* Fcn: '<S107>/Fcn' */
        LCITopology6x6Minimal_B.Fcn_p =
          LCITopology6x6Minimal_B.changeIqIdcurrentsigns[1] * std::sin
          (LCITopology6x6Minimal_B.Sum3) +
          LCITopology6x6Minimal_B.changeIqIdcurrentsigns[0] * std::cos
          (LCITopology6x6Minimal_B.Sum3);

        /* Fcn: '<S107>/Fcn1' */
        LCITopology6x6Minimal_B.Fcn1_h =
          -LCITopology6x6Minimal_B.changeIqIdcurrentsigns[1] * std::cos
          (LCITopology6x6Minimal_B.Sum3) +
          LCITopology6x6Minimal_B.changeIqIdcurrentsigns[0] * std::sin
          (LCITopology6x6Minimal_B.Sum3);
      }

      /* End of Outputs for SubSystem: '<S104>/Subsystem - pi//2 delay' */

      /* Switch: '<S104>/Switch' */
      if (LCITopology6x6Minimal_B.Compare != 0) {
        /* Switch: '<S104>/Switch' */
        LCITopology6x6Minimal_B.Switch[0] = LCITopology6x6Minimal_B.Fcn_m;
        LCITopology6x6Minimal_B.Switch[1] = LCITopology6x6Minimal_B.Fcn1;
      } else {
        /* Switch: '<S104>/Switch' */
        LCITopology6x6Minimal_B.Switch[0] = LCITopology6x6Minimal_B.Fcn_p;
        LCITopology6x6Minimal_B.Switch[1] = LCITopology6x6Minimal_B.Fcn1_h;
      }

      /* End of Switch: '<S104>/Switch' */

      /* SignalConversion generated from: '<S103>/Gain3' incorporates:
       *  Constant: '<S77>/Constant'
       */
      LCITopology6x6Minimal_B.TmpSignalConversionAtGain3Inpor[0] =
        LCITopology6x6Minimal_B.Switch[0];
      LCITopology6x6Minimal_B.TmpSignalConversionAtGain3Inpor[1] =
        LCITopology6x6Minimal_B.Switch[1];
      LCITopology6x6Minimal_B.TmpSignalConversionAtGain3Inpor[2] =
        LCITopology6x6Minimal_P.Constant_Value_k;
      for (i = 0; i < 3; i++) {
        /* Gain: '<S103>/Gain3' */
        LCITopology6x6Minimal_B.Gain3[i] = 0.0;
        LCITopology6x6Minimal_B.Gain3[i] +=
          LCITopology6x6Minimal_P.Gain3_Gain_l[i] *
          LCITopology6x6Minimal_B.TmpSignalConversionAtGain3Inpor[0];
        LCITopology6x6Minimal_B.Gain3[i] +=
          LCITopology6x6Minimal_P.Gain3_Gain_l[i + 3] *
          LCITopology6x6Minimal_B.TmpSignalConversionAtGain3Inpor[1];
        LCITopology6x6Minimal_B.Gain3[i] +=
          LCITopology6x6Minimal_P.Gain3_Gain_l[i + 6] *
          LCITopology6x6Minimal_B.TmpSignalConversionAtGain3Inpor[2];

        /* Gain: '<S77>/ib' */
        LCITopology6x6Minimal_B.iabc[i] = LCITopology6x6Minimal_P.ib_Gain *
          LCITopology6x6Minimal_B.Gain3[i];
      }

      /* Constant: '<S2>/Constant4' */
      LCITopology6x6Minimal_B.Constant4 =
        LCITopology6x6Minimal_P.Constant4_Value_d;

      /* DiscreteIntegrator: '<S36>/Integrator' */
      if ((LCITopology6x6Minimal_B.Constant4 != 0.0) ||
          (LCITopology6x6Minimal_DW.Integrator_PrevResetState != 0)) {
        LCITopology6x6Minimal_DW.Integrator_DSTATE =
          LCITopology6x6Minimal_B.Constant4;
        if (LCITopology6x6Minimal_DW.Integrator_DSTATE >=
            LCITopology6x6Minimal_P.Integrator_UpperSat) {
          LCITopology6x6Minimal_DW.Integrator_DSTATE =
            LCITopology6x6Minimal_P.Integrator_UpperSat;
        } else {
          if (LCITopology6x6Minimal_DW.Integrator_DSTATE <=
              LCITopology6x6Minimal_P.Integrator_LowerSat) {
            LCITopology6x6Minimal_DW.Integrator_DSTATE =
              LCITopology6x6Minimal_P.Integrator_LowerSat;
          }
        }
      }

      if (LCITopology6x6Minimal_DW.Integrator_DSTATE >=
          LCITopology6x6Minimal_P.Integrator_UpperSat) {
        LCITopology6x6Minimal_DW.Integrator_DSTATE =
          LCITopology6x6Minimal_P.Integrator_UpperSat;
      } else {
        if (LCITopology6x6Minimal_DW.Integrator_DSTATE <=
            LCITopology6x6Minimal_P.Integrator_LowerSat) {
          LCITopology6x6Minimal_DW.Integrator_DSTATE =
            LCITopology6x6Minimal_P.Integrator_LowerSat;
        }
      }

      /* DiscreteIntegrator: '<S36>/Integrator' */
      LCITopology6x6Minimal_B.Integrator =
        LCITopology6x6Minimal_DW.Integrator_DSTATE;

      /* Saturate: '<S36>/Saturation' */
      tmpForInput_idx_0 = LCITopology6x6Minimal_B.Integrator;
      tmpForInput_idx_1 = LCITopology6x6Minimal_P.Saturation_LowerSat_f;
      tmpForInput_idx_2 = LCITopology6x6Minimal_P.Saturation_UpperSat_e;
      if (tmpForInput_idx_0 > tmpForInput_idx_2) {
        /* Saturate: '<S36>/Saturation' */
        LCITopology6x6Minimal_B.Saturation = tmpForInput_idx_2;
      } else if (tmpForInput_idx_0 < tmpForInput_idx_1) {
        /* Saturate: '<S36>/Saturation' */
        LCITopology6x6Minimal_B.Saturation = tmpForInput_idx_1;
      } else {
        /* Saturate: '<S36>/Saturation' */
        LCITopology6x6Minimal_B.Saturation = tmpForInput_idx_0;
      }

      /* End of Saturate: '<S36>/Saturation' */

      /* Sum: '<S31>/Sum' incorporates:
       *  Constant: '<S31>/Constant5'
       */
      LCITopology6x6Minimal_B.Sum = LCITopology6x6Minimal_B.Saturation +
        LCITopology6x6Minimal_P.Constant5_Value;

      /* Math: '<S31>/Mod' incorporates:
       *  Constant: '<S31>/Constant6'
       */
      LCITopology6x6Minimal_B.Mod = rt_modd_snf(LCITopology6x6Minimal_B.Sum,
        LCITopology6x6Minimal_P.Constant6_Value_k);

      /* Outport: '<Root>/source_voltage_phase' incorporates:
       *  Constant: '<S31>/Constant5'
       *  Sum: '<S31>/Sum1'
       */
      LCITopology6x6Minimal_Y.source_voltage_phase = LCITopology6x6Minimal_B.Mod
        - LCITopology6x6Minimal_P.Constant5_Value;

      /* Gain: '<S2>/Gain2' incorporates:
       *  Constant: '<Root>/Constant1'
       *  Constant: '<Root>/Constant2'
       *  Constant: '<Root>/Constant3'
       */
      LCITopology6x6Minimal_B.Gain2[0] = LCITopology6x6Minimal_P.Gain2_Gain *
        LCITopology6x6Minimal_P.Constant1_Value_e;
      LCITopology6x6Minimal_B.Gain2[1] = LCITopology6x6Minimal_P.Gain2_Gain *
        LCITopology6x6Minimal_P.Constant2_Value_e;
      LCITopology6x6Minimal_B.Gain2[2] = LCITopology6x6Minimal_P.Gain2_Gain *
        LCITopology6x6Minimal_P.Constant3_Value_k;

      /* Sum: '<S2>/Add1' incorporates:
       *  Outport: '<Root>/source_voltage_phase'
       */
      LCITopology6x6Minimal_B.Add1[0] =
        LCITopology6x6Minimal_Y.source_voltage_phase +
        LCITopology6x6Minimal_B.Gain2[0];

      /* Trigonometry: '<S2>/Trigonometric Function1' */
      LCITopology6x6Minimal_B.TrigonometricFunction1[0] = std::sin
        (LCITopology6x6Minimal_B.Add1[0]);

      /* Gain: '<S2>/Gain1' */
      LCITopology6x6Minimal_B.Gain1[0] = LCITopology6x6Minimal_P.Gain1_Gain_j *
        LCITopology6x6Minimal_B.TrigonometricFunction1[0];

      /* Sum: '<S2>/Add1' incorporates:
       *  Outport: '<Root>/source_voltage_phase'
       */
      LCITopology6x6Minimal_B.Add1[1] =
        LCITopology6x6Minimal_Y.source_voltage_phase +
        LCITopology6x6Minimal_B.Gain2[1];

      /* Trigonometry: '<S2>/Trigonometric Function1' */
      LCITopology6x6Minimal_B.TrigonometricFunction1[1] = std::sin
        (LCITopology6x6Minimal_B.Add1[1]);

      /* Gain: '<S2>/Gain1' */
      LCITopology6x6Minimal_B.Gain1[1] = LCITopology6x6Minimal_P.Gain1_Gain_j *
        LCITopology6x6Minimal_B.TrigonometricFunction1[1];

      /* Sum: '<S2>/Add1' incorporates:
       *  Outport: '<Root>/source_voltage_phase'
       */
      LCITopology6x6Minimal_B.Add1[2] =
        LCITopology6x6Minimal_Y.source_voltage_phase +
        LCITopology6x6Minimal_B.Gain2[2];

      /* Trigonometry: '<S2>/Trigonometric Function1' */
      LCITopology6x6Minimal_B.TrigonometricFunction1[2] = std::sin
        (LCITopology6x6Minimal_B.Add1[2]);

      /* Gain: '<S2>/Gain1' */
      LCITopology6x6Minimal_B.Gain1[2] = LCITopology6x6Minimal_P.Gain1_Gain_j *
        LCITopology6x6Minimal_B.TrigonometricFunction1[2];

      /* Product: '<S2>/Product5' incorporates:
       *  Inport: '<Root>/voltage_magnitude_a'
       */
      LCITopology6x6Minimal_B.Product5 = LCITopology6x6Minimal_B.Gain1[0] *
        LCITopology6x6Minimal_U.voltage_magnitude_a;

      /* Product: '<S2>/Product6' incorporates:
       *  Inport: '<Root>/voltage_magnitude_b'
       */
      LCITopology6x6Minimal_B.Product6 = LCITopology6x6Minimal_B.Gain1[1] *
        LCITopology6x6Minimal_U.voltage_magnitude_b;

      /* Product: '<S2>/Product7' incorporates:
       *  Inport: '<Root>/voltage_magnitude_c'
       */
      LCITopology6x6Minimal_B.Product7 = LCITopology6x6Minimal_B.Gain1[2] *
        LCITopology6x6Minimal_U.voltage_magnitude_c;

      /* S-Function (sfun_spssw_discc): '<S164>/State-Space' incorporates:
       *  Constant: '<S166>/SwitchCurrents'
       *  Inport: '<Root>/pulses_in_grid'
       *  Inport: '<Root>/pulses_in_motor'
       */

      /* S-Function block: <S164>/State-Space */
      {
        real_T accum;

        /* Circuit has switches */
        int_T *switch_status = (int_T*)
          LCITopology6x6Minimal_DW.StateSpace_PWORK.SWITCH_STATUS;
        int_T *switch_status_init = (int_T*)
          LCITopology6x6Minimal_DW.StateSpace_PWORK.SWITCH_STATUS_INIT;
        int_T *SwitchChange = (int_T*)
          LCITopology6x6Minimal_DW.StateSpace_PWORK.SW_CHG;
        int_T *gState = (int_T*)
          LCITopology6x6Minimal_DW.StateSpace_PWORK.G_STATE;
        real_T *yswitch = (real_T*)
          LCITopology6x6Minimal_DW.StateSpace_PWORK.Y_SWITCH;
        int_T *switchTypes = (int_T*)
          LCITopology6x6Minimal_DW.StateSpace_PWORK.SWITCH_TYPES;
        int_T *idxOutSw = (int_T*)
          LCITopology6x6Minimal_DW.StateSpace_PWORK.IDX_OUT_SW;
        real_T *DxCol = (real_T*)
          LCITopology6x6Minimal_DW.StateSpace_PWORK.DX_COL;
        real_T *tmp2 = (real_T*)LCITopology6x6Minimal_DW.StateSpace_PWORK.TMP2;
        real_T *BDcol = (real_T*)
          LCITopology6x6Minimal_DW.StateSpace_PWORK.BD_COL;
        real_T *tmp1 = (real_T*)LCITopology6x6Minimal_DW.StateSpace_PWORK.TMP1;
        real_T *uswlast = (real_T*)
          LCITopology6x6Minimal_DW.StateSpace_PWORK.USWLAST;
        int_T newState;
        int_T swChanged = 0;
        int loopsToDo = 20;
        real_T temp;

        /* keep an initial copy of switch_status*/
        memcpy(switch_status_init, switch_status, 14 * sizeof(int_T));
        memcpy(uswlast, &LCITopology6x6Minimal_B.StateSpace_o1[0], 14*sizeof
               (real_T));
        do {
          if (loopsToDo == 1) {        /* Need to reset some variables: */
            swChanged = 0;

            /* return to the original switch status*/
            {
              int_T i1;
              for (i1=0; i1 < 14; i1++) {
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
            real_T *Cs = (real_T*)LCITopology6x6Minimal_DW.StateSpace_PWORK.CS;
            real_T *Ds = (real_T*)LCITopology6x6Minimal_DW.StateSpace_PWORK.DS;

            {
              int_T i1;
              real_T *y0 = &LCITopology6x6Minimal_B.StateSpace_o1[0];
              for (i1=0; i1 < 37; i1++) {
                accum = 0.0;

                {
                  int_T i2;
                  real_T *xd = &LCITopology6x6Minimal_DW.StateSpace_DSTATE[0];
                  for (i2=0; i2 < 21; i2++) {
                    accum += *(Cs++) * xd[i2];
                  }
                }

                {
                  int_T i2;
                  const real_T *u0 =
                    &LCITopology6x6Minimal_P.SwitchCurrents_Value[0];
                  for (i2=0; i2 < 14; i2++) {
                    accum += *(Ds++) * u0[i2];
                  }

                  accum += *(Ds++) * LCITopology6x6Minimal_B.iabc[0];
                  accum += *(Ds++) * LCITopology6x6Minimal_B.iabc[1];
                  accum += *(Ds++) * LCITopology6x6Minimal_B.Product5;
                  accum += *(Ds++) * LCITopology6x6Minimal_B.Product6;
                  accum += *(Ds++) * LCITopology6x6Minimal_B.Product7;
                }

                y0[i1] = accum;
              }
            }

            swChanged = 0;

            {
              int_T i1;
              real_T *y0 = &LCITopology6x6Minimal_B.StateSpace_o1[0];
              for (i1=0; i1 < 14; i1++) {
                switch (switchTypes[i1]) {
                 case 1 :              /* Ideal switch */
                  newState = gState[i1] > 0 ? 1 : 0;
                  break;

                 case 4 :              /* Thyristor */
                  newState = (y0[i1] > 0.0) && (gState[i1] > 0) ? 1 : ((y0[i1] <
                    0.0) ? 0 : switch_status[i1]);
                  break;
                }

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
            real_T *As = (real_T*)LCITopology6x6Minimal_DW.StateSpace_PWORK.AS;
            real_T *Cs = (real_T*)LCITopology6x6Minimal_DW.StateSpace_PWORK.CS;
            real_T *Bs = (real_T*)LCITopology6x6Minimal_DW.StateSpace_PWORK.BS;
            real_T *Ds = (real_T*)LCITopology6x6Minimal_DW.StateSpace_PWORK.DS;
            real_T a1;

            {
              int_T i1;
              for (i1=0; i1 < 14; i1++) {
                if (SwitchChange[i1] != 0) {
                  a1 = yswitch[i1]*SwitchChange[i1];
                  temp = 1/(1-Ds[i1*20]*a1);

                  {
                    int_T i2;
                    for (i2=0; i2 < 37; i2++) {
                      DxCol[i2]= Ds[i2 * 19 + i1]*temp*a1;
                    }
                  }

                  DxCol[i1] = temp;

                  {
                    int_T i2;
                    for (i2=0; i2 < 21; i2++) {
                      BDcol[i2]= Bs[i2 * 19 + i1]*a1;
                    }
                  }

                  /* Copy row nSw of Cs into tmp1 and zero it out in Cs */
                  memcpy(tmp1, &Cs[i1 * 21], 21 * sizeof(real_T));
                  memset(&Cs[i1 * 21], '\0', 21 * sizeof(real_T));

                  /* Copy row nSw of Ds into tmp2 and zero it out in Ds */
                  memcpy(tmp2, &Ds[i1 * 19], 19 * sizeof(real_T));
                  memset(&Ds[i1 * 19], '\0', 19 * sizeof(real_T));

                  /* Cs = Cs + DxCol * tmp1, Ds = Ds + DxCol * tmp2 *******************/
                  {
                    int_T i2;
                    for (i2=0; i2 < 37; i2++) {
                      a1 = DxCol[i2];

                      {
                        int_T i3;
                        for (i3=0; i3 < 21; i3++) {
                          Cs[i2 * 21 + i3] += a1 * tmp1[i3];
                        }
                      }

                      {
                        int_T i3;
                        for (i3=0; i3 < 19; i3++) {
                          Ds[i2 * 19 + i3] += a1 * tmp2[i3];
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
                        for (i3=0; i3 < 19; i3++) {
                          Bs[i2 * 19 + i3] += a1 * Ds[i1 * 19 + i3];
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
          real_T *Cs = (real_T*)LCITopology6x6Minimal_DW.StateSpace_PWORK.CS;
          real_T *Ds = (real_T*)LCITopology6x6Minimal_DW.StateSpace_PWORK.DS;

          {
            int_T i1;
            real_T *y0 = &LCITopology6x6Minimal_B.StateSpace_o1[0];
            for (i1=0; i1 < 37; i1++) {
              accum = 0.0;

              {
                int_T i2;
                real_T *xd = &LCITopology6x6Minimal_DW.StateSpace_DSTATE[0];
                for (i2=0; i2 < 21; i2++) {
                  accum += *(Cs++) * xd[i2];
                }
              }

              {
                int_T i2;
                const real_T *u0 =
                  &LCITopology6x6Minimal_P.SwitchCurrents_Value[0];
                for (i2=0; i2 < 14; i2++) {
                  accum += *(Ds++) * u0[i2];
                }

                accum += *(Ds++) * LCITopology6x6Minimal_B.iabc[0];
                accum += *(Ds++) * LCITopology6x6Minimal_B.iabc[1];
                accum += *(Ds++) * LCITopology6x6Minimal_B.Product5;
                accum += *(Ds++) * LCITopology6x6Minimal_B.Product6;
                accum += *(Ds++) * LCITopology6x6Minimal_B.Product7;
              }

              y0[i1] = accum;
            }
          }
        }

        /* Output new switches states */
        {
          int_T i1;
          real_T *y1 = &LCITopology6x6Minimal_B.StateSpace_o2[0];
          for (i1=0; i1 < 14; i1++) {
            y1[i1] = (real_T)switch_status[i1];
          }
        }
      }

      /* Gain: '<S27>/do not delete this gain' */
      LCITopology6x6Minimal_B.donotdeletethisgain =
        LCITopology6x6Minimal_P.donotdeletethisgain_Gain *
        LCITopology6x6Minimal_B.StateSpace_o1[26];

      /* Outport: '<Root>/Udc' incorporates:
       *  Inport: '<Root>/grid_pt_ratio'
       *  Product: '<S10>/Divide'
       */
      LCITopology6x6Minimal_Y.Udc = LCITopology6x6Minimal_B.donotdeletethisgain /
        LCITopology6x6Minimal_U.grid_pt_ratio;

      /* Gain: '<S3>/do not delete this gain' */
      LCITopology6x6Minimal_B.donotdeletethisgain_o =
        LCITopology6x6Minimal_P.donotdeletethisgain_Gain_d *
        LCITopology6x6Minimal_B.StateSpace_o1[27];

      /* Outport: '<Root>/Idc' incorporates:
       *  Inport: '<Root>/grid_ct_ratio'
       *  Product: '<S11>/Divide'
       */
      LCITopology6x6Minimal_Y.Idc =
        LCITopology6x6Minimal_B.donotdeletethisgain_o /
        LCITopology6x6Minimal_U.grid_ct_ratio;

      /* Gain: '<S133>/do not delete this gain' */
      LCITopology6x6Minimal_B.donotdeletethisgain_c =
        LCITopology6x6Minimal_P.donotdeletethisgain_Gain_i *
        LCITopology6x6Minimal_B.StateSpace_o1[31];

      /* Gain: '<S134>/do not delete this gain' */
      LCITopology6x6Minimal_B.donotdeletethisgain_p =
        LCITopology6x6Minimal_P.donotdeletethisgain_Gain_o *
        LCITopology6x6Minimal_B.StateSpace_o1[32];

      /* Gain: '<S135>/do not delete this gain' */
      LCITopology6x6Minimal_B.donotdeletethisgain_cx =
        LCITopology6x6Minimal_P.donotdeletethisgain_Gain_m *
        LCITopology6x6Minimal_B.StateSpace_o1[33];

      /* Gain: '<S22>/Kv' */
      LCITopology6x6Minimal_B.Kv[0] = LCITopology6x6Minimal_P.Kv_Gain *
        LCITopology6x6Minimal_B.donotdeletethisgain_c;
      LCITopology6x6Minimal_B.Kv[1] = LCITopology6x6Minimal_P.Kv_Gain *
        LCITopology6x6Minimal_B.donotdeletethisgain_p;
      LCITopology6x6Minimal_B.Kv[2] = LCITopology6x6Minimal_P.Kv_Gain *
        LCITopology6x6Minimal_B.donotdeletethisgain_cx;

      /* Gain: '<S136>/do not delete this gain' */
      LCITopology6x6Minimal_B.donotdeletethisgain_f =
        LCITopology6x6Minimal_P.donotdeletethisgain_Gain_k *
        LCITopology6x6Minimal_B.StateSpace_o1[19];

      /* Gain: '<S137>/do not delete this gain' */
      LCITopology6x6Minimal_B.donotdeletethisgain_k =
        LCITopology6x6Minimal_P.donotdeletethisgain_Gain_h *
        LCITopology6x6Minimal_B.StateSpace_o1[20];

      /* Gain: '<S138>/do not delete this gain' */
      LCITopology6x6Minimal_B.donotdeletethisgain_n =
        LCITopology6x6Minimal_P.donotdeletethisgain_Gain_n *
        LCITopology6x6Minimal_B.StateSpace_o1[21];

      /* Gain: '<S22>/Kv1' */
      LCITopology6x6Minimal_B.Kv1[0] = LCITopology6x6Minimal_P.Kv1_Gain *
        LCITopology6x6Minimal_B.donotdeletethisgain_f;
      LCITopology6x6Minimal_B.Kv1[1] = LCITopology6x6Minimal_P.Kv1_Gain *
        LCITopology6x6Minimal_B.donotdeletethisgain_k;
      LCITopology6x6Minimal_B.Kv1[2] = LCITopology6x6Minimal_P.Kv1_Gain *
        LCITopology6x6Minimal_B.donotdeletethisgain_n;

      /* Outport: '<Root>/Iph_grid' incorporates:
       *  Inport: '<Root>/grid_ct_ratio'
       *  Product: '<S12>/Divide'
       */
      LCITopology6x6Minimal_Y.Iph_grid[0] = LCITopology6x6Minimal_B.Kv[0] /
        LCITopology6x6Minimal_U.grid_ct_ratio;

      /* Outport: '<Root>/Uph_ph_grid' incorporates:
       *  Inport: '<Root>/grid_pt_ratio'
       *  Product: '<S15>/Divide'
       */
      LCITopology6x6Minimal_Y.Uph_ph_grid[0] = LCITopology6x6Minimal_B.Kv1[0] /
        LCITopology6x6Minimal_U.grid_pt_ratio;

      /* Outport: '<Root>/Iph_grid' incorporates:
       *  Inport: '<Root>/grid_ct_ratio'
       *  Product: '<S12>/Divide'
       */
      LCITopology6x6Minimal_Y.Iph_grid[1] = LCITopology6x6Minimal_B.Kv[1] /
        LCITopology6x6Minimal_U.grid_ct_ratio;

      /* Outport: '<Root>/Uph_ph_grid' incorporates:
       *  Inport: '<Root>/grid_pt_ratio'
       *  Product: '<S15>/Divide'
       */
      LCITopology6x6Minimal_Y.Uph_ph_grid[1] = LCITopology6x6Minimal_B.Kv1[1] /
        LCITopology6x6Minimal_U.grid_pt_ratio;

      /* Outport: '<Root>/Iph_grid' incorporates:
       *  Inport: '<Root>/grid_ct_ratio'
       *  Product: '<S12>/Divide'
       */
      LCITopology6x6Minimal_Y.Iph_grid[2] = LCITopology6x6Minimal_B.Kv[2] /
        LCITopology6x6Minimal_U.grid_ct_ratio;

      /* Outport: '<Root>/Uph_ph_grid' incorporates:
       *  Inport: '<Root>/grid_pt_ratio'
       *  Product: '<S15>/Divide'
       */
      LCITopology6x6Minimal_Y.Uph_ph_grid[2] = LCITopology6x6Minimal_B.Kv1[2] /
        LCITopology6x6Minimal_U.grid_pt_ratio;
    }

    /* RelationalOperator: '<S42>/Relational Operator' incorporates:
     *  Constant: '<S42>/Constant2'
     */
    LCITopology6x6Minimal_B.RelationalOperator =
      (LCITopology6x6Minimal_X.Integrator_CSTATE >
       LCITopology6x6Minimal_P.Constant2_Value_ez);

    /* Sum: '<S42>/Subtract' incorporates:
     *  Constant: '<S42>/Constant2'
     */
    LCITopology6x6Minimal_B.Subtract = LCITopology6x6Minimal_X.Integrator_CSTATE
      - LCITopology6x6Minimal_P.Constant2_Value_ez;

    /* InitialCondition: '<S42>/Initial' */
    if ((LCITopology6x6Minimal_DW.Initial_FirstOutputTime == (rtMinusInf)) ||
        (LCITopology6x6Minimal_DW.Initial_FirstOutputTime ==
         (&LCITopology6x6Minimal_M)->Timing.t[0])) {
      LCITopology6x6Minimal_DW.Initial_FirstOutputTime =
        (&LCITopology6x6Minimal_M)->Timing.t[0];

      /* InitialCondition: '<S42>/Initial' */
      LCITopology6x6Minimal_B.Initial = LCITopology6x6Minimal_P.Initial_Value;
    } else {
      /* InitialCondition: '<S42>/Initial' */
      LCITopology6x6Minimal_B.Initial = LCITopology6x6Minimal_B.Subtract;
    }

    /* End of InitialCondition: '<S42>/Initial' */

    /* Integrator: '<S42>/Integrator' */
    /* Limited  Integrator  */
    if (rtmIsMajorTimeStep((&LCITopology6x6Minimal_M))) {
      didZcEventOccur = (LCITopology6x6Minimal_B.RelationalOperator &&
                         (LCITopology6x6Minimal_PrevZCX.Integrator_Reset_ZCE !=
                          1));
      LCITopology6x6Minimal_PrevZCX.Integrator_Reset_ZCE =
        LCITopology6x6Minimal_B.RelationalOperator;

      /* evaluate zero-crossings */
      if (didZcEventOccur || (LCITopology6x6Minimal_DW.Integrator_IWORK != 0)) {
        LCITopology6x6Minimal_X.Integrator_CSTATE =
          LCITopology6x6Minimal_B.Initial;
        rtsiSetBlockStateForSolverChangedAtMajorStep(&(&LCITopology6x6Minimal_M
          )->solverInfo, true);
        rtsiSetContTimeOutputInconsistentWithStateAtMajorStep
          (&(&LCITopology6x6Minimal_M)->solverInfo, true);
      }
    }

    if (LCITopology6x6Minimal_X.Integrator_CSTATE >=
        LCITopology6x6Minimal_P.Integrator_UpperSat_l) {
      if (LCITopology6x6Minimal_X.Integrator_CSTATE !=
          LCITopology6x6Minimal_P.Integrator_UpperSat_l) {
        rtsiSetBlockStateForSolverChangedAtMajorStep(&(&LCITopology6x6Minimal_M
          )->solverInfo, true);
        rtsiSetContTimeOutputInconsistentWithStateAtMajorStep
          (&(&LCITopology6x6Minimal_M)->solverInfo, true);
      }

      LCITopology6x6Minimal_X.Integrator_CSTATE =
        LCITopology6x6Minimal_P.Integrator_UpperSat_l;
    } else {
      if (LCITopology6x6Minimal_X.Integrator_CSTATE <=
          LCITopology6x6Minimal_P.Integrator_LowerSat_n) {
        if (LCITopology6x6Minimal_X.Integrator_CSTATE !=
            LCITopology6x6Minimal_P.Integrator_LowerSat_n) {
          rtsiSetBlockStateForSolverChangedAtMajorStep
            (&(&LCITopology6x6Minimal_M)->solverInfo, true);
          rtsiSetContTimeOutputInconsistentWithStateAtMajorStep
            (&(&LCITopology6x6Minimal_M)->solverInfo, true);
        }

        LCITopology6x6Minimal_X.Integrator_CSTATE =
          LCITopology6x6Minimal_P.Integrator_LowerSat_n;
      }
    }

    /* Integrator: '<S42>/Integrator' */
    LCITopology6x6Minimal_B.Integrator_d =
      LCITopology6x6Minimal_X.Integrator_CSTATE;

    /* Outport: '<Root>/wt' */
    LCITopology6x6Minimal_Y.wt = LCITopology6x6Minimal_B.Integrator_d;
    if (rtmIsMajorTimeStep((&LCITopology6x6Minimal_M))) {
      /* Memory: '<S42>/Memory' */
      LCITopology6x6Minimal_B.Memory =
        LCITopology6x6Minimal_DW.Memory_PreviousInput;

      /* Gain: '<S151>/do not delete this gain' */
      LCITopology6x6Minimal_B.donotdeletethisgain_c2 =
        LCITopology6x6Minimal_P.donotdeletethisgain_Gain_m0 *
        LCITopology6x6Minimal_B.StateSpace_o1[22];

      /* Gain: '<S152>/do not delete this gain' */
      LCITopology6x6Minimal_B.donotdeletethisgain_g =
        LCITopology6x6Minimal_P.donotdeletethisgain_Gain_ip *
        LCITopology6x6Minimal_B.StateSpace_o1[23];

      /* Gain: '<S153>/do not delete this gain' */
      LCITopology6x6Minimal_B.donotdeletethisgain_f1 =
        LCITopology6x6Minimal_P.donotdeletethisgain_Gain_c *
        LCITopology6x6Minimal_B.StateSpace_o1[24];

      /* Gain: '<S23>/Kv1' */
      LCITopology6x6Minimal_B.Kv1_j[0] = LCITopology6x6Minimal_P.Kv1_Gain_a *
        LCITopology6x6Minimal_B.donotdeletethisgain_c2;
      LCITopology6x6Minimal_B.Kv1_j[1] = LCITopology6x6Minimal_P.Kv1_Gain_a *
        LCITopology6x6Minimal_B.donotdeletethisgain_g;
      LCITopology6x6Minimal_B.Kv1_j[2] = LCITopology6x6Minimal_P.Kv1_Gain_a *
        LCITopology6x6Minimal_B.donotdeletethisgain_f1;

      /* Outputs for Enabled SubSystem: '<S42>/Automatic Gain Control' incorporates:
       *  EnablePort: '<S43>/Enable'
       */
      if (rtmIsMajorTimeStep((&LCITopology6x6Minimal_M))) {
        /* Constant: '<S42>/Constant1' */
        if (LCITopology6x6Minimal_P.Constant1_Value_a > 0.0) {
          LCITopology6x6Minimal_DW.AutomaticGainControl_MODE = true;
        } else {
          if (LCITopology6x6Minimal_DW.AutomaticGainControl_MODE) {
            /* Disable for Enabled SubSystem: '<S54>/Subsystem - pi//2 delay' */
            if (LCITopology6x6Minimal_DW.Subsystempi2delay_k.Subsystempi2delay_MODE)
            {
              LCITo_Subsystempi2delay_Disable
                (&LCITopology6x6Minimal_DW.Subsystempi2delay_k);
            }

            /* End of Disable for SubSystem: '<S54>/Subsystem - pi//2 delay' */

            /* Disable for Enabled SubSystem: '<S54>/Subsystem1' */
            if (LCITopology6x6Minimal_DW.Subsystem1_ka.Subsystem1_MODE) {
              LCITopology6_Subsystem1_Disable
                (&LCITopology6x6Minimal_DW.Subsystem1_ka);
            }

            /* End of Disable for SubSystem: '<S54>/Subsystem1' */
            LCITopology6x6Minimal_DW.AutomaticGainControl_MODE = false;
          }
        }

        /* End of Constant: '<S42>/Constant1' */
      }

      /* End of Outputs for SubSystem: '<S42>/Automatic Gain Control' */
    }

    /* Outputs for Enabled SubSystem: '<S42>/Automatic Gain Control' incorporates:
     *  EnablePort: '<S43>/Enable'
     */
    if (LCITopology6x6Minimal_DW.AutomaticGainControl_MODE) {
      /* VariableTransportDelay: '<S52>/Variable Transport Delay' */
      {
        real_T **uBuffer = (real_T**)
          &LCITopology6x6Minimal_DW.VariableTransportDelay_PWORK_g.TUbufferPtrs
          [0];
        real_T **tBuffer = (real_T**)
          &LCITopology6x6Minimal_DW.VariableTransportDelay_PWORK_g.TUbufferPtrs
          [1];
        real_T **xBuffer = (real_T**)
          &LCITopology6x6Minimal_DW.VariableTransportDelay_PWORK_g.TUbufferPtrs
          [2];
        real_T simTime = (&LCITopology6x6Minimal_M)->Timing.t[0];
        real_T appliedDelay;

        /* For variable transport dealy, find the real applied dealy
         * here and then output
         */
        LCITopology6x6Minimal_B.VariableTransportDelay_p=
          rt_VTDelayfindtDInterpolate
          (LCITopology6x6Minimal_X.VariableTransportDelay_CSTATE_c,*tBuffer,
           *uBuffer, *xBuffer,
           LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK_g.CircularBufSize,
           LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK_g.Head,
           LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK_g.Tail,
           &LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK_g.Last,
           simTime, 0.0,0,
           0, LCITopology6x6Minimal_P.VariableTransportDelay_InitOutp,
           &appliedDelay);
      }

      /* Integrator: '<S52>/integrator' */
      LCITopology6x6Minimal_B.integrator_d =
        LCITopology6x6Minimal_X.integrator_CSTATE_g;

      /* Clock: '<S52>/Clock' */
      LCITopology6x6Minimal_B.Clock_c = (&LCITopology6x6Minimal_M)->Timing.t[0];

      /* RelationalOperator: '<S52>/Relational Operator' incorporates:
       *  Constant: '<S52>/Constant'
       */
      LCITopology6x6Minimal_B.RelationalOperator_h =
        (LCITopology6x6Minimal_B.Clock_c >=
         LCITopology6x6Minimal_P.Constant_Value);
      if (rtmIsMajorTimeStep((&LCITopology6x6Minimal_M))) {
        /* Memory: '<S52>/Memory' */
        LCITopology6x6Minimal_B.Memory_f =
          LCITopology6x6Minimal_DW.Memory_PreviousInput_d;
      }

      /* Switch: '<S52>/Switch' */
      if (LCITopology6x6Minimal_B.RelationalOperator_h) {
        /* Sum: '<S52>/Sum7' */
        LCITopology6x6Minimal_B.Sum7_h = LCITopology6x6Minimal_B.integrator_d -
          LCITopology6x6Minimal_B.VariableTransportDelay_p;

        /* Product: '<S52>/Product' */
        LCITopology6x6Minimal_B.Meanvalue_o = LCITopology6x6Minimal_B.Sum7_h *
          LCITopology6x6Minimal_B.Memory;

        /* Switch: '<S52>/Switch' */
        LCITopology6x6Minimal_B.Switch_n = LCITopology6x6Minimal_B.Meanvalue_o;
      } else {
        /* Switch: '<S52>/Switch' */
        LCITopology6x6Minimal_B.Switch_n = LCITopology6x6Minimal_B.Memory_f;
      }

      /* End of Switch: '<S52>/Switch' */

      /* VariableTransportDelay: '<S53>/Variable Transport Delay' */
      {
        real_T **uBuffer = (real_T**)
          &LCITopology6x6Minimal_DW.VariableTransportDelay_PWORK_j.TUbufferPtrs
          [0];
        real_T **tBuffer = (real_T**)
          &LCITopology6x6Minimal_DW.VariableTransportDelay_PWORK_j.TUbufferPtrs
          [1];
        real_T **xBuffer = (real_T**)
          &LCITopology6x6Minimal_DW.VariableTransportDelay_PWORK_j.TUbufferPtrs
          [2];
        real_T simTime = (&LCITopology6x6Minimal_M)->Timing.t[0];
        real_T appliedDelay;

        /* For variable transport dealy, find the real applied dealy
         * here and then output
         */
        LCITopology6x6Minimal_B.VariableTransportDelay_e=
          rt_VTDelayfindtDInterpolate
          (LCITopology6x6Minimal_X.VariableTransportDelay_CSTATE_l,*tBuffer,
           *uBuffer, *xBuffer,
           LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK_a.CircularBufSize,
           LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK_a.Head,
           LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK_a.Tail,
           &LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK_a.Last,
           simTime, 0.0,0,
           0, LCITopology6x6Minimal_P.VariableTransportDelay_InitOu_g,
           &appliedDelay);
      }

      /* Integrator: '<S53>/integrator' */
      LCITopology6x6Minimal_B.integrator_de =
        LCITopology6x6Minimal_X.integrator_CSTATE_b;

      /* Clock: '<S53>/Clock' */
      LCITopology6x6Minimal_B.Clock_b = (&LCITopology6x6Minimal_M)->Timing.t[0];

      /* RelationalOperator: '<S53>/Relational Operator' incorporates:
       *  Constant: '<S53>/Constant'
       */
      LCITopology6x6Minimal_B.RelationalOperator_g1 =
        (LCITopology6x6Minimal_B.Clock_b >=
         LCITopology6x6Minimal_P.Constant_Value_p);
      if (rtmIsMajorTimeStep((&LCITopology6x6Minimal_M))) {
        /* Memory: '<S53>/Memory' */
        LCITopology6x6Minimal_B.Memory_c =
          LCITopology6x6Minimal_DW.Memory_PreviousInput_h;
      }

      /* Switch: '<S53>/Switch' */
      if (LCITopology6x6Minimal_B.RelationalOperator_g1) {
        /* Sum: '<S53>/Sum7' */
        LCITopology6x6Minimal_B.Sum7_i = LCITopology6x6Minimal_B.integrator_de -
          LCITopology6x6Minimal_B.VariableTransportDelay_e;

        /* Product: '<S53>/Product' */
        LCITopology6x6Minimal_B.Meanvalue_j = LCITopology6x6Minimal_B.Sum7_i *
          LCITopology6x6Minimal_B.Memory;

        /* Switch: '<S53>/Switch' */
        LCITopology6x6Minimal_B.Switch_m = LCITopology6x6Minimal_B.Meanvalue_j;
      } else {
        /* Switch: '<S53>/Switch' */
        LCITopology6x6Minimal_B.Switch_m = LCITopology6x6Minimal_B.Memory_c;
      }

      /* End of Switch: '<S53>/Switch' */

      /* RealImagToComplex: '<S48>/Real-Imag to Complex' */
      LCITopology6x6Minimal_B.RealImagtoComplex.re =
        LCITopology6x6Minimal_B.Switch_n;
      LCITopology6x6Minimal_B.RealImagtoComplex.im =
        LCITopology6x6Minimal_B.Switch_m;

      /* ComplexToMagnitudeAngle: '<S48>/Complex to Magnitude-Angle' incorporates:
       *  RealImagToComplex: '<S48>/Real-Imag to Complex'
       */
      LCITopology6x6Minimal_B.ComplextoMagnitudeAngle_o1 = rt_hypotd_snf
        (LCITopology6x6Minimal_B.RealImagtoComplex.re,
         LCITopology6x6Minimal_B.RealImagtoComplex.im);

      /* ComplexToMagnitudeAngle: '<S48>/Complex to Magnitude-Angle' incorporates:
       *  RealImagToComplex: '<S48>/Real-Imag to Complex'
       */
      LCITopology6x6Minimal_B.ComplextoMagnitudeAngle_o2 = rt_atan2d_snf
        (LCITopology6x6Minimal_B.RealImagtoComplex.im,
         LCITopology6x6Minimal_B.RealImagtoComplex.re);

      /* Saturate: '<S43>/Saturation' */
      tmpForInput_idx_0 = LCITopology6x6Minimal_B.ComplextoMagnitudeAngle_o1;
      tmpForInput_idx_1 = LCITopology6x6Minimal_P.Saturation_LowerSat;
      tmpForInput_idx_2 = LCITopology6x6Minimal_P.Saturation_UpperSat;
      if (tmpForInput_idx_0 > tmpForInput_idx_2) {
        /* Saturate: '<S43>/Saturation' */
        LCITopology6x6Minimal_B.Saturation_a = tmpForInput_idx_2;
      } else if (tmpForInput_idx_0 < tmpForInput_idx_1) {
        /* Saturate: '<S43>/Saturation' */
        LCITopology6x6Minimal_B.Saturation_a = tmpForInput_idx_1;
      } else {
        /* Saturate: '<S43>/Saturation' */
        LCITopology6x6Minimal_B.Saturation_a = tmpForInput_idx_0;
      }

      /* End of Saturate: '<S43>/Saturation' */

      /* Math: '<S43>/Math Function'
       *
       * About '<S43>/Math Function':
       *  Operator: reciprocal
       */
      tmpForInput_idx_0 = LCITopology6x6Minimal_B.Saturation_a;

      /* Math: '<S43>/Math Function'
       *
       * About '<S43>/Math Function':
       *  Operator: reciprocal
       */
      LCITopology6x6Minimal_B.MathFunction_c = 1.0 / tmpForInput_idx_0;

      /* Saturate: '<S52>/To avoid division  by zero' */
      tmpForInput_idx_0 = LCITopology6x6Minimal_B.Memory;
      tmpForInput_idx_1 = LCITopology6x6Minimal_P.Toavoiddivisionbyzero_LowerSat;
      tmpForInput_idx_2 = LCITopology6x6Minimal_P.Toavoiddivisionbyzero_UpperSat;
      if (tmpForInput_idx_0 > tmpForInput_idx_2) {
        /* Saturate: '<S52>/To avoid division  by zero' */
        LCITopology6x6Minimal_B.Freq_o = tmpForInput_idx_2;
      } else if (tmpForInput_idx_0 < tmpForInput_idx_1) {
        /* Saturate: '<S52>/To avoid division  by zero' */
        LCITopology6x6Minimal_B.Freq_o = tmpForInput_idx_1;
      } else {
        /* Saturate: '<S52>/To avoid division  by zero' */
        LCITopology6x6Minimal_B.Freq_o = tmpForInput_idx_0;
      }

      /* End of Saturate: '<S52>/To avoid division  by zero' */

      /* Fcn: '<S52>/period' */
      LCITopology6x6Minimal_B.period_i = 1.0 / LCITopology6x6Minimal_B.Freq_o;

      /* Saturate: '<S53>/To avoid division  by zero' */
      tmpForInput_idx_0 = LCITopology6x6Minimal_B.Memory;
      tmpForInput_idx_1 =
        LCITopology6x6Minimal_P.Toavoiddivisionbyzero_LowerSa_l;
      tmpForInput_idx_2 =
        LCITopology6x6Minimal_P.Toavoiddivisionbyzero_UpperSa_h;
      if (tmpForInput_idx_0 > tmpForInput_idx_2) {
        /* Saturate: '<S53>/To avoid division  by zero' */
        LCITopology6x6Minimal_B.Freq_l = tmpForInput_idx_2;
      } else if (tmpForInput_idx_0 < tmpForInput_idx_1) {
        /* Saturate: '<S53>/To avoid division  by zero' */
        LCITopology6x6Minimal_B.Freq_l = tmpForInput_idx_1;
      } else {
        /* Saturate: '<S53>/To avoid division  by zero' */
        LCITopology6x6Minimal_B.Freq_l = tmpForInput_idx_0;
      }

      /* End of Saturate: '<S53>/To avoid division  by zero' */

      /* Fcn: '<S53>/period' */
      LCITopology6x6Minimal_B.period_a = 1.0 / LCITopology6x6Minimal_B.Freq_l;
      if (rtmIsMajorTimeStep((&LCITopology6x6Minimal_M))) {
        /* RelationalOperator: '<S56>/Compare' incorporates:
         *  Constant: '<S54>/Constant'
         *  Constant: '<S56>/Constant'
         */
        LCITopology6x6Minimal_B.Compare_j0 =
          (LCITopology6x6Minimal_P.AlphaBetaZerotodq0_Alignment ==
           LCITopology6x6Minimal_P.CompareToConstant_const);

        /* RelationalOperator: '<S57>/Compare' incorporates:
         *  Constant: '<S54>/Constant'
         *  Constant: '<S57>/Constant'
         */
        LCITopology6x6Minimal_B.Compare_p =
          (LCITopology6x6Minimal_P.AlphaBetaZerotodq0_Alignment ==
           LCITopology6x6Minimal_P.CompareToConstant1_const);
      }

      for (i = 0; i < 3; i++) {
        /* Gain: '<S55>/Gain3' */
        LCITopology6x6Minimal_B.Gain3_o4[i] = 0.0;
        LCITopology6x6Minimal_B.Gain3_o4[i] +=
          LCITopology6x6Minimal_P.Gain3_Gain[i] * LCITopology6x6Minimal_B.Kv1_j
          [0];
        LCITopology6x6Minimal_B.Gain3_o4[i] +=
          LCITopology6x6Minimal_P.Gain3_Gain[i + 3] *
          LCITopology6x6Minimal_B.Kv1_j[1];
        LCITopology6x6Minimal_B.Gain3_o4[i] +=
          LCITopology6x6Minimal_P.Gain3_Gain[i + 6] *
          LCITopology6x6Minimal_B.Kv1_j[2];

        /* Gain: '<S55>/Gain1' */
        LCITopology6x6Minimal_B.Gain1_c[i] = LCITopology6x6Minimal_P.Gain1_Gain *
          LCITopology6x6Minimal_B.Gain3_o4[i];
      }

      /* Outputs for Enabled SubSystem: '<S54>/Subsystem - pi//2 delay' */
      LCITopology6x_Subsystempi2delay(LCITopology6x6Minimal_B.Compare_p,
        &LCITopology6x6Minimal_B.Gain1_c[0],
        LCITopology6x6Minimal_B.Integrator_d,
        &LCITopology6x6Minimal_B.Subsystempi2delay_k,
        &LCITopology6x6Minimal_DW.Subsystempi2delay_k);

      /* End of Outputs for SubSystem: '<S54>/Subsystem - pi//2 delay' */

      /* Outputs for Enabled SubSystem: '<S54>/Subsystem1' */
      LCITopology6x6Minima_Subsystem1(LCITopology6x6Minimal_B.Compare_j0,
        &LCITopology6x6Minimal_B.Gain1_c[0],
        LCITopology6x6Minimal_B.Integrator_d,
        &LCITopology6x6Minimal_B.Subsystem1_ka,
        &LCITopology6x6Minimal_DW.Subsystem1_ka);

      /* End of Outputs for SubSystem: '<S54>/Subsystem1' */

      /* Switch: '<S54>/Switch' */
      if (LCITopology6x6Minimal_B.Compare_j0 != 0) {
        /* Switch: '<S54>/Switch' */
        LCITopology6x6Minimal_B.Switch_dq[0] =
          LCITopology6x6Minimal_B.Subsystem1_ka.Fcn;
        LCITopology6x6Minimal_B.Switch_dq[1] =
          LCITopology6x6Minimal_B.Subsystem1_ka.Fcn1;
      } else {
        /* Switch: '<S54>/Switch' */
        LCITopology6x6Minimal_B.Switch_dq[0] =
          LCITopology6x6Minimal_B.Subsystempi2delay_k.Fcn;
        LCITopology6x6Minimal_B.Switch_dq[1] =
          LCITopology6x6Minimal_B.Subsystempi2delay_k.Fcn1;
      }

      /* End of Switch: '<S54>/Switch' */
    }

    /* End of Outputs for SubSystem: '<S42>/Automatic Gain Control' */

    /* Integrator: '<S44>/Integrator' */
    /* Limited  Integrator  */
    if (LCITopology6x6Minimal_X.Integrator_CSTATE_i >=
        LCITopology6x6Minimal_P.Integrator_UpperSat_j) {
      LCITopology6x6Minimal_X.Integrator_CSTATE_i =
        LCITopology6x6Minimal_P.Integrator_UpperSat_j;
    } else {
      if (LCITopology6x6Minimal_X.Integrator_CSTATE_i <=
          LCITopology6x6Minimal_P.Integrator_LowerSat_c) {
        LCITopology6x6Minimal_X.Integrator_CSTATE_i =
          LCITopology6x6Minimal_P.Integrator_LowerSat_c;
      }
    }

    /* Integrator: '<S44>/Integrator' */
    LCITopology6x6Minimal_B.Integrator_o =
      LCITopology6x6Minimal_X.Integrator_CSTATE_i;

    /* VariableTransportDelay: '<S64>/Variable Transport Delay' */
    {
      real_T **uBuffer = (real_T**)
        &LCITopology6x6Minimal_DW.VariableTransportDelay_PWORK.TUbufferPtrs[0];
      real_T **tBuffer = (real_T**)
        &LCITopology6x6Minimal_DW.VariableTransportDelay_PWORK.TUbufferPtrs[1];
      real_T **xBuffer = (real_T**)
        &LCITopology6x6Minimal_DW.VariableTransportDelay_PWORK.TUbufferPtrs[2];
      real_T simTime = (&LCITopology6x6Minimal_M)->Timing.t[0];
      real_T appliedDelay;

      /* For variable transport dealy, find the real applied dealy
       * here and then output
       */
      LCITopology6x6Minimal_B.VariableTransportDelay=
        rt_VTDelayfindtDInterpolate
        (LCITopology6x6Minimal_X.VariableTransportDelay_CSTATE,*tBuffer,*uBuffer,
         *xBuffer,
         LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK.CircularBufSize,
         LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK.Head,
         LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK.Tail,
         &LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK.Last, simTime,
         0.0,0,
         0, LCITopology6x6Minimal_P.VariableTransportDelay_InitOu_b,
         &appliedDelay);
    }

    /* Integrator: '<S64>/integrator' */
    LCITopology6x6Minimal_B.integrator =
      LCITopology6x6Minimal_X.integrator_CSTATE;

    /* Clock: '<S64>/Clock' */
    LCITopology6x6Minimal_B.Clock = (&LCITopology6x6Minimal_M)->Timing.t[0];

    /* RelationalOperator: '<S64>/Relational Operator' incorporates:
     *  Constant: '<S64>/Constant'
     */
    LCITopology6x6Minimal_B.RelationalOperator_g =
      (LCITopology6x6Minimal_B.Clock >= LCITopology6x6Minimal_P.Constant_Value_f);
    if (rtmIsMajorTimeStep((&LCITopology6x6Minimal_M))) {
      /* Memory: '<S64>/Memory' */
      LCITopology6x6Minimal_B.Memory_l =
        LCITopology6x6Minimal_DW.Memory_PreviousInput_p;
    }

    /* Switch: '<S64>/Switch' */
    if (LCITopology6x6Minimal_B.RelationalOperator_g) {
      /* Sum: '<S64>/Sum7' */
      LCITopology6x6Minimal_B.Sum7 = LCITopology6x6Minimal_B.integrator -
        LCITopology6x6Minimal_B.VariableTransportDelay;

      /* Product: '<S64>/Product' */
      LCITopology6x6Minimal_B.Meanvalue = LCITopology6x6Minimal_B.Sum7 *
        LCITopology6x6Minimal_B.Memory;

      /* Switch: '<S64>/Switch' */
      LCITopology6x6Minimal_B.Switch_g = LCITopology6x6Minimal_B.Meanvalue;
    } else {
      /* Switch: '<S64>/Switch' */
      LCITopology6x6Minimal_B.Switch_g = LCITopology6x6Minimal_B.Memory_l;
    }

    /* End of Switch: '<S64>/Switch' */

    /* Product: '<S42>/Divide' */
    LCITopology6x6Minimal_B.Divide_g = LCITopology6x6Minimal_B.Switch_g *
      LCITopology6x6Minimal_B.MathFunction_c;

    /* Gain: '<S44>/Kp4' */
    LCITopology6x6Minimal_B.Kp4 = LCITopology6x6Minimal_P.Continuous_Kp *
      LCITopology6x6Minimal_B.Divide_g;

    /* Gain: '<S44>/Kp5' */
    LCITopology6x6Minimal_B.Kp5 = LCITopology6x6Minimal_P.Continuous_Ki *
      LCITopology6x6Minimal_B.Divide_g;

    /* Gain: '<S44>/Kp6' */
    LCITopology6x6Minimal_B.Kp6 = LCITopology6x6Minimal_P.Continuous_Kd *
      LCITopology6x6Minimal_B.Divide_g;

    /* TransferFcn: '<S44>/Transfer Fcn' */
    LCITopology6x6Minimal_B.TransferFcn = 0.0;
    LCITopology6x6Minimal_B.TransferFcn += LCITopology6x6Minimal_P.TransferFcn_C
      * LCITopology6x6Minimal_X.TransferFcn_CSTATE;
    LCITopology6x6Minimal_B.TransferFcn += LCITopology6x6Minimal_P.TransferFcn_D
      * LCITopology6x6Minimal_B.Kp6;

    /* Sum: '<S44>/Sum6' */
    LCITopology6x6Minimal_B.Sum6 = (LCITopology6x6Minimal_B.Kp4 +
      LCITopology6x6Minimal_B.Integrator_o) +
      LCITopology6x6Minimal_B.TransferFcn;

    /* Saturate: '<S44>/Saturation2' */
    tmpForInput_idx_0 = LCITopology6x6Minimal_B.Sum6;
    tmpForInput_idx_1 = LCITopology6x6Minimal_P.Saturation2_LowerSat;
    tmpForInput_idx_2 = LCITopology6x6Minimal_P.Saturation2_UpperSat;
    if (tmpForInput_idx_0 > tmpForInput_idx_2) {
      /* Saturate: '<S44>/Saturation2' */
      LCITopology6x6Minimal_B.Saturation2 = tmpForInput_idx_2;
    } else if (tmpForInput_idx_0 < tmpForInput_idx_1) {
      /* Saturate: '<S44>/Saturation2' */
      LCITopology6x6Minimal_B.Saturation2 = tmpForInput_idx_1;
    } else {
      /* Saturate: '<S44>/Saturation2' */
      LCITopology6x6Minimal_B.Saturation2 = tmpForInput_idx_0;
    }

    /* End of Saturate: '<S44>/Saturation2' */

    /* Gain: '<S42>/Gain10' */
    LCITopology6x6Minimal_B.Gain10 = LCITopology6x6Minimal_P.Gain10_Gain *
      LCITopology6x6Minimal_B.Saturation2;

    /* RateLimiter: '<S42>/Rate Limiter' */
    if (LCITopology6x6Minimal_DW.LastMajorTime == (rtInf)) {
      /* RateLimiter: '<S42>/Rate Limiter' */
      LCITopology6x6Minimal_B.RateLimiter = LCITopology6x6Minimal_B.Gain10;
    } else {
      tmpForInput_idx_1 = (&LCITopology6x6Minimal_M)->Timing.t[0] -
        LCITopology6x6Minimal_DW.LastMajorTime;
      tmpForInput_idx_2 = tmpForInput_idx_1 *
        LCITopology6x6Minimal_P.RateLimiter_RisingLim;
      tmpForInput_idx_0 = LCITopology6x6Minimal_B.Gain10 -
        LCITopology6x6Minimal_DW.PrevY;
      if (tmpForInput_idx_0 > tmpForInput_idx_2) {
        /* RateLimiter: '<S42>/Rate Limiter' */
        LCITopology6x6Minimal_B.RateLimiter = LCITopology6x6Minimal_DW.PrevY +
          tmpForInput_idx_2;
      } else {
        tmpForInput_idx_1 *= LCITopology6x6Minimal_P.RateLimiter_FallingLim;
        if (tmpForInput_idx_0 < tmpForInput_idx_1) {
          /* RateLimiter: '<S42>/Rate Limiter' */
          LCITopology6x6Minimal_B.RateLimiter = LCITopology6x6Minimal_DW.PrevY +
            tmpForInput_idx_1;
        } else {
          /* RateLimiter: '<S42>/Rate Limiter' */
          LCITopology6x6Minimal_B.RateLimiter = LCITopology6x6Minimal_B.Gain10;
        }
      }
    }

    /* End of RateLimiter: '<S42>/Rate Limiter' */

    /* Integrator: '<S60>/Integrator_x1' */
    LCITopology6x6Minimal_B.x1 = LCITopology6x6Minimal_X.Integrator_x1_CSTATE;

    /* Gain: '<S61>/A11' */
    LCITopology6x6Minimal_B.A11 = LCITopology6x6Minimal_P.A11_Gain *
      LCITopology6x6Minimal_B.x1;

    /* Integrator: '<S60>/Integrator_x2' */
    LCITopology6x6Minimal_B.x2 = LCITopology6x6Minimal_X.Integrator_x2_CSTATE;

    /* Gain: '<S61>/A12' */
    LCITopology6x6Minimal_B.A12 = LCITopology6x6Minimal_P.A12_Gain *
      LCITopology6x6Minimal_B.x2;

    /* Gain: '<S61>/A21' */
    LCITopology6x6Minimal_B.A21 = LCITopology6x6Minimal_P.A21_Gain *
      LCITopology6x6Minimal_B.x1;

    /* Gain: '<S61>/A22' */
    LCITopology6x6Minimal_B.A22 = LCITopology6x6Minimal_P.A22_Gain *
      LCITopology6x6Minimal_B.x2;

    /* Sum: '<S61>/sum2' */
    LCITopology6x6Minimal_B.sum2 = LCITopology6x6Minimal_B.A11 +
      LCITopology6x6Minimal_B.A12;

    /* Sum: '<S61>/sum3' */
    LCITopology6x6Minimal_B.sum3 = LCITopology6x6Minimal_B.A21 +
      LCITopology6x6Minimal_B.A22;

    /* Gain: '<S62>/B11' */
    LCITopology6x6Minimal_B.B11 = LCITopology6x6Minimal_P.B11_Gain *
      LCITopology6x6Minimal_B.RateLimiter;

    /* Sum: '<S60>/A*x1+ B* u1' */
    LCITopology6x6Minimal_B.x1_i = LCITopology6x6Minimal_B.sum2 +
      LCITopology6x6Minimal_B.B11;

    /* Gain: '<S62>/B21' */
    LCITopology6x6Minimal_B.B21 = LCITopology6x6Minimal_P.B21_Gain *
      LCITopology6x6Minimal_B.RateLimiter;

    /* Sum: '<S60>/A*x2+ B*u2' */
    LCITopology6x6Minimal_B.x2_m = LCITopology6x6Minimal_B.sum3 +
      LCITopology6x6Minimal_B.B21;

    /* Gain: '<S63>/C11' */
    LCITopology6x6Minimal_B.C11 = LCITopology6x6Minimal_P.C11_Gain *
      LCITopology6x6Minimal_B.x1;

    /* Gain: '<S63>/C12' */
    LCITopology6x6Minimal_B.C12 = LCITopology6x6Minimal_P.C12_Gain *
      LCITopology6x6Minimal_B.x2;

    /* Sum: '<S63>/sum2' */
    LCITopology6x6Minimal_B.sum2_a = LCITopology6x6Minimal_B.C11 +
      LCITopology6x6Minimal_B.C12;

    /* Gain: '<S60>/D*u' */
    LCITopology6x6Minimal_B.Du = LCITopology6x6Minimal_P.Du_Gain *
      LCITopology6x6Minimal_B.RateLimiter;

    /* Sum: '<S60>/C*x + D*u' */
    LCITopology6x6Minimal_B.y = LCITopology6x6Minimal_B.Du +
      LCITopology6x6Minimal_B.sum2_a;

    /* Saturate: '<S64>/To avoid division  by zero' */
    tmpForInput_idx_0 = LCITopology6x6Minimal_B.Memory;
    tmpForInput_idx_1 = LCITopology6x6Minimal_P.Toavoiddivisionbyzero_LowerSa_n;
    tmpForInput_idx_2 = LCITopology6x6Minimal_P.Toavoiddivisionbyzero_UpperSa_o;
    if (tmpForInput_idx_0 > tmpForInput_idx_2) {
      /* Saturate: '<S64>/To avoid division  by zero' */
      LCITopology6x6Minimal_B.Freq = tmpForInput_idx_2;
    } else if (tmpForInput_idx_0 < tmpForInput_idx_1) {
      /* Saturate: '<S64>/To avoid division  by zero' */
      LCITopology6x6Minimal_B.Freq = tmpForInput_idx_1;
    } else {
      /* Saturate: '<S64>/To avoid division  by zero' */
      LCITopology6x6Minimal_B.Freq = tmpForInput_idx_0;
    }

    /* End of Saturate: '<S64>/To avoid division  by zero' */

    /* Fcn: '<S64>/period' */
    LCITopology6x6Minimal_B.period = 1.0 / LCITopology6x6Minimal_B.Freq;
    if (rtmIsMajorTimeStep((&LCITopology6x6Minimal_M))) {
      /* RelationalOperator: '<S67>/Compare' incorporates:
       *  Constant: '<S65>/Constant'
       *  Constant: '<S67>/Constant'
       */
      LCITopology6x6Minimal_B.Compare_e =
        (LCITopology6x6Minimal_P.AlphaBetaZerotodq0_Alignment_n ==
         LCITopology6x6Minimal_P.CompareToConstant_const_c);

      /* RelationalOperator: '<S68>/Compare' incorporates:
       *  Constant: '<S65>/Constant'
       *  Constant: '<S68>/Constant'
       */
      LCITopology6x6Minimal_B.Compare_j =
        (LCITopology6x6Minimal_P.AlphaBetaZerotodq0_Alignment_n ==
         LCITopology6x6Minimal_P.CompareToConstant1_const_g);
    }

    for (i = 0; i < 3; i++) {
      /* Gain: '<S66>/Gain3' */
      LCITopology6x6Minimal_B.Gain3_o[i] = 0.0;
      LCITopology6x6Minimal_B.Gain3_o[i] +=
        LCITopology6x6Minimal_P.Gain3_Gain_j[i] * LCITopology6x6Minimal_B.Kv1_j
        [0];
      LCITopology6x6Minimal_B.Gain3_o[i] +=
        LCITopology6x6Minimal_P.Gain3_Gain_j[i + 3] *
        LCITopology6x6Minimal_B.Kv1_j[1];
      LCITopology6x6Minimal_B.Gain3_o[i] +=
        LCITopology6x6Minimal_P.Gain3_Gain_j[i + 6] *
        LCITopology6x6Minimal_B.Kv1_j[2];

      /* Gain: '<S66>/Gain1' */
      LCITopology6x6Minimal_B.Gain1_f[i] = LCITopology6x6Minimal_P.Gain1_Gain_o *
        LCITopology6x6Minimal_B.Gain3_o[i];
    }

    /* Outputs for Enabled SubSystem: '<S65>/Subsystem - pi//2 delay' */
    LCITopology6x_Subsystempi2delay(LCITopology6x6Minimal_B.Compare_j,
      &LCITopology6x6Minimal_B.Gain1_f[0], LCITopology6x6Minimal_B.Integrator_d,
      &LCITopology6x6Minimal_B.Subsystempi2delay,
      &LCITopology6x6Minimal_DW.Subsystempi2delay);

    /* End of Outputs for SubSystem: '<S65>/Subsystem - pi//2 delay' */

    /* Outputs for Enabled SubSystem: '<S65>/Subsystem1' */
    LCITopology6x6Minima_Subsystem1(LCITopology6x6Minimal_B.Compare_e,
      &LCITopology6x6Minimal_B.Gain1_f[0], LCITopology6x6Minimal_B.Integrator_d,
      &LCITopology6x6Minimal_B.Subsystem1, &LCITopology6x6Minimal_DW.Subsystem1);

    /* End of Outputs for SubSystem: '<S65>/Subsystem1' */

    /* Switch: '<S65>/Switch' */
    if (LCITopology6x6Minimal_B.Compare_e != 0) {
      /* Switch: '<S65>/Switch' */
      LCITopology6x6Minimal_B.Switch_a[0] =
        LCITopology6x6Minimal_B.Subsystem1.Fcn;
      LCITopology6x6Minimal_B.Switch_a[1] =
        LCITopology6x6Minimal_B.Subsystem1.Fcn1;
    } else {
      /* Switch: '<S65>/Switch' */
      LCITopology6x6Minimal_B.Switch_a[0] =
        LCITopology6x6Minimal_B.Subsystempi2delay.Fcn;
      LCITopology6x6Minimal_B.Switch_a[1] =
        LCITopology6x6Minimal_B.Subsystempi2delay.Fcn1;
    }

    /* End of Switch: '<S65>/Switch' */
    if (rtmIsMajorTimeStep((&LCITopology6x6Minimal_M))) {
      /* Gain: '<S148>/do not delete this gain' */
      LCITopology6x6Minimal_B.donotdeletethisgain_h =
        LCITopology6x6Minimal_P.donotdeletethisgain_Gain_b *
        LCITopology6x6Minimal_B.StateSpace_o1[34];

      /* Gain: '<S149>/do not delete this gain' */
      LCITopology6x6Minimal_B.donotdeletethisgain_d =
        LCITopology6x6Minimal_P.donotdeletethisgain_Gain_km *
        LCITopology6x6Minimal_B.StateSpace_o1[35];

      /* Gain: '<S150>/do not delete this gain' */
      LCITopology6x6Minimal_B.donotdeletethisgain_kk =
        LCITopology6x6Minimal_P.donotdeletethisgain_Gain_p *
        LCITopology6x6Minimal_B.StateSpace_o1[36];

      /* Gain: '<S23>/Kv' */
      LCITopology6x6Minimal_B.Kv_n[0] = LCITopology6x6Minimal_P.Kv_Gain_e *
        LCITopology6x6Minimal_B.donotdeletethisgain_h;
      LCITopology6x6Minimal_B.Kv_n[1] = LCITopology6x6Minimal_P.Kv_Gain_e *
        LCITopology6x6Minimal_B.donotdeletethisgain_d;
      LCITopology6x6Minimal_B.Kv_n[2] = LCITopology6x6Minimal_P.Kv_Gain_e *
        LCITopology6x6Minimal_B.donotdeletethisgain_kk;

      /* Gain: '<S118>/do not delete this gain' */
      LCITopology6x6Minimal_B.donotdeletethisgain_m =
        LCITopology6x6Minimal_P.donotdeletethisgain_Gain_f *
        LCITopology6x6Minimal_B.StateSpace_o1[28];

      /* Gain: '<S119>/do not delete this gain' */
      LCITopology6x6Minimal_B.donotdeletethisgain_b =
        LCITopology6x6Minimal_P.donotdeletethisgain_Gain_hv *
        LCITopology6x6Minimal_B.StateSpace_o1[29];

      /* Gain: '<S120>/do not delete this gain' */
      LCITopology6x6Minimal_B.donotdeletethisgain_n2 =
        LCITopology6x6Minimal_P.donotdeletethisgain_Gain_j *
        LCITopology6x6Minimal_B.StateSpace_o1[30];

      /* Gain: '<S21>/Kv' */
      LCITopology6x6Minimal_B.Kv_n3[0] = LCITopology6x6Minimal_P.Kv_Gain_ej *
        LCITopology6x6Minimal_B.donotdeletethisgain_m;
      LCITopology6x6Minimal_B.Kv_n3[1] = LCITopology6x6Minimal_P.Kv_Gain_ej *
        LCITopology6x6Minimal_B.donotdeletethisgain_b;
      LCITopology6x6Minimal_B.Kv_n3[2] = LCITopology6x6Minimal_P.Kv_Gain_ej *
        LCITopology6x6Minimal_B.donotdeletethisgain_n2;

      /* Gain: '<S121>/do not delete this gain' */
      LCITopology6x6Minimal_B.donotdeletethisgain_de =
        LCITopology6x6Minimal_P.donotdeletethisgain_Gain_fx *
        LCITopology6x6Minimal_B.StateSpace_o1[16];

      /* Gain: '<S122>/do not delete this gain' */
      LCITopology6x6Minimal_B.donotdeletethisgain_kv =
        LCITopology6x6Minimal_P.donotdeletethisgain_Gain_ka *
        LCITopology6x6Minimal_B.StateSpace_o1[17];

      /* Gain: '<S123>/do not delete this gain' */
      LCITopology6x6Minimal_B.donotdeletethisgain_g0 =
        LCITopology6x6Minimal_P.donotdeletethisgain_Gain_fm *
        LCITopology6x6Minimal_B.StateSpace_o1[18];

      /* Gain: '<S21>/Kv1' */
      LCITopology6x6Minimal_B.Kv1_l[0] = LCITopology6x6Minimal_P.Kv1_Gain_k *
        LCITopology6x6Minimal_B.donotdeletethisgain_de;
      LCITopology6x6Minimal_B.Kv1_l[1] = LCITopology6x6Minimal_P.Kv1_Gain_k *
        LCITopology6x6Minimal_B.donotdeletethisgain_kv;
      LCITopology6x6Minimal_B.Kv1_l[2] = LCITopology6x6Minimal_P.Kv1_Gain_k *
        LCITopology6x6Minimal_B.donotdeletethisgain_g0;

      /* Outport: '<Root>/Uph_n_source' incorporates:
       *  Inport: '<Root>/grid_pt_ratio'
       *  Product: '<S16>/Divide'
       */
      LCITopology6x6Minimal_Y.Uph_n_source[0] = LCITopology6x6Minimal_B.Kv1_j[0]
        / LCITopology6x6Minimal_U.grid_pt_ratio;

      /* Outport: '<Root>/Iph_source' incorporates:
       *  Inport: '<Root>/grid_ct_ratio'
       *  Product: '<S13>/Divide'
       */
      LCITopology6x6Minimal_Y.Iph_source[0] = LCITopology6x6Minimal_B.Kv_n[0] /
        LCITopology6x6Minimal_U.grid_ct_ratio;

      /* Outport: '<Root>/Iph_motor' incorporates:
       *  Inport: '<Root>/motor_ct_ratio'
       *  Product: '<S14>/Divide'
       */
      LCITopology6x6Minimal_Y.Iph_motor[0] = LCITopology6x6Minimal_B.Kv_n3[0] /
        LCITopology6x6Minimal_U.motor_ct_ratio;

      /* Outport: '<Root>/Uph_ph_motor' incorporates:
       *  Inport: '<Root>/motor_pt_ratio'
       *  Product: '<S17>/Divide'
       */
      LCITopology6x6Minimal_Y.Uph_ph_motor[0] = LCITopology6x6Minimal_B.Kv1_l[0]
        / LCITopology6x6Minimal_U.motor_pt_ratio;

      /* Outport: '<Root>/Uph_n_source' incorporates:
       *  Inport: '<Root>/grid_pt_ratio'
       *  Product: '<S16>/Divide'
       */
      LCITopology6x6Minimal_Y.Uph_n_source[1] = LCITopology6x6Minimal_B.Kv1_j[1]
        / LCITopology6x6Minimal_U.grid_pt_ratio;

      /* Outport: '<Root>/Iph_source' incorporates:
       *  Inport: '<Root>/grid_ct_ratio'
       *  Product: '<S13>/Divide'
       */
      LCITopology6x6Minimal_Y.Iph_source[1] = LCITopology6x6Minimal_B.Kv_n[1] /
        LCITopology6x6Minimal_U.grid_ct_ratio;

      /* Outport: '<Root>/Iph_motor' incorporates:
       *  Inport: '<Root>/motor_ct_ratio'
       *  Product: '<S14>/Divide'
       */
      LCITopology6x6Minimal_Y.Iph_motor[1] = LCITopology6x6Minimal_B.Kv_n3[1] /
        LCITopology6x6Minimal_U.motor_ct_ratio;

      /* Outport: '<Root>/Uph_ph_motor' incorporates:
       *  Inport: '<Root>/motor_pt_ratio'
       *  Product: '<S17>/Divide'
       */
      LCITopology6x6Minimal_Y.Uph_ph_motor[1] = LCITopology6x6Minimal_B.Kv1_l[1]
        / LCITopology6x6Minimal_U.motor_pt_ratio;

      /* Outport: '<Root>/Uph_n_source' incorporates:
       *  Inport: '<Root>/grid_pt_ratio'
       *  Product: '<S16>/Divide'
       */
      LCITopology6x6Minimal_Y.Uph_n_source[2] = LCITopology6x6Minimal_B.Kv1_j[2]
        / LCITopology6x6Minimal_U.grid_pt_ratio;

      /* Outport: '<Root>/Iph_source' incorporates:
       *  Inport: '<Root>/grid_ct_ratio'
       *  Product: '<S13>/Divide'
       */
      LCITopology6x6Minimal_Y.Iph_source[2] = LCITopology6x6Minimal_B.Kv_n[2] /
        LCITopology6x6Minimal_U.grid_ct_ratio;

      /* Outport: '<Root>/Iph_motor' incorporates:
       *  Inport: '<Root>/motor_ct_ratio'
       *  Product: '<S14>/Divide'
       */
      LCITopology6x6Minimal_Y.Iph_motor[2] = LCITopology6x6Minimal_B.Kv_n3[2] /
        LCITopology6x6Minimal_U.motor_ct_ratio;

      /* Outport: '<Root>/Uph_ph_motor' incorporates:
       *  Inport: '<Root>/motor_pt_ratio'
       *  Product: '<S17>/Divide'
       */
      LCITopology6x6Minimal_Y.Uph_ph_motor[2] = LCITopology6x6Minimal_B.Kv1_l[2]
        / LCITopology6x6Minimal_U.motor_pt_ratio;

      /* UnitDelay: '<S110>/dw_delay' */
      LCITopology6x6Minimal_B.dw_delay =
        LCITopology6x6Minimal_DW.dw_delay_DSTATE;

      /* Gain: '<S110>/F2' */
      LCITopology6x6Minimal_B.F2 = LCITopology6x6Minimal_P.F2_Gain *
        LCITopology6x6Minimal_B.dw_delay;

      /* UnitDelay: '<S110>/dw_predict' */
      LCITopology6x6Minimal_B.dw_predict =
        LCITopology6x6Minimal_DW.dw_predict_DSTATE;

      /* Sum: '<S110>/Sum1' */
      LCITopology6x6Minimal_B.Sum1_h = LCITopology6x6Minimal_B.F2 -
        LCITopology6x6Minimal_B.dw_predict;

      /* Sum: '<S73>/Sum' incorporates:
       *  Constant: '<S73>/nominal speed'
       */
      LCITopology6x6Minimal_B.n = LCITopology6x6Minimal_P.nominalspeed_Value +
        LCITopology6x6Minimal_B.Sum1_h;

      /* Switch: '<S73>/Switch' incorporates:
       *  Inport: '<Root>/enable_forced_speed'
       */
      if (LCITopology6x6Minimal_U.enable_forced_speed >
          LCITopology6x6Minimal_P.Switch_Threshold_k) {
        /* Switch: '<S73>/Switch' incorporates:
         *  Inport: '<Root>/forced_speed_pu'
         */
        LCITopology6x6Minimal_B.Switch_e =
          LCITopology6x6Minimal_U.forced_speed_pu;
      } else {
        /* Switch: '<S73>/Switch' */
        LCITopology6x6Minimal_B.Switch_e = LCITopology6x6Minimal_B.n;
      }

      /* End of Switch: '<S73>/Switch' */

      /* Outport: '<Root>/machine_speed' incorporates:
       *  Gain: '<S73>/units'
       */
      LCITopology6x6Minimal_Y.machine_speed = LCITopology6x6Minimal_P.units_Gain
        * LCITopology6x6Minimal_B.Switch_e;

      /* Gain: '<S82>/1-1' */
      LCITopology6x6Minimal_B.iqsids[0] = LCITopology6x6Minimal_P.u1_Gain[0] *
        LCITopology6x6Minimal_B.changeIqIdcurrentsigns[0];
      LCITopology6x6Minimal_B.iqsids[1] = LCITopology6x6Minimal_P.u1_Gain[1] *
        LCITopology6x6Minimal_B.changeIqIdcurrentsigns[1];

      /* Product: '<S82>/Mult1' */
      LCITopology6x6Minimal_B.Mult1[0] = LCITopology6x6Minimal_B.xk1[1] *
        LCITopology6x6Minimal_B.iqsids[0];
      LCITopology6x6Minimal_B.Mult1[1] = LCITopology6x6Minimal_B.xk1[0] *
        LCITopology6x6Minimal_B.iqsids[1];

      /* DiscreteIntegrator: '<S73>/theta' */
      LCITopology6x6Minimal_B.theta = LCITopology6x6Minimal_DW.theta_DSTATE;

      /* Fcn: '<S73>/Fcn' */
      tmpForInput_idx_1 = LCITopology6x6Minimal_B.theta;

      /* Signum: '<S7>/Sign' incorporates:
       *  Outport: '<Root>/machine_speed'
       */
      tmpForInput_idx_0 = LCITopology6x6Minimal_Y.machine_speed;
      if (tmpForInput_idx_0 < 0.0) {
        /* Signum: '<S7>/Sign' */
        LCITopology6x6Minimal_B.Sign = -1.0;
      } else if (tmpForInput_idx_0 > 0.0) {
        /* Signum: '<S7>/Sign' */
        LCITopology6x6Minimal_B.Sign = 1.0;
      } else if (tmpForInput_idx_0 == 0.0) {
        /* Signum: '<S7>/Sign' */
        LCITopology6x6Minimal_B.Sign = 0.0;
      } else {
        /* Signum: '<S7>/Sign' */
        LCITopology6x6Minimal_B.Sign = (rtNaN);
      }

      /* End of Signum: '<S7>/Sign' */

      /* Abs: '<S7>/Abs' incorporates:
       *  Outport: '<Root>/machine_speed'
       */
      LCITopology6x6Minimal_B.Abs = std::abs
        (LCITopology6x6Minimal_Y.machine_speed);

      /* Product: '<S7>/Product' incorporates:
       *  Inport: '<Root>/load_coefficients'
       */
      LCITopology6x6Minimal_B.Product = LCITopology6x6Minimal_B.Abs *
        LCITopology6x6Minimal_U.load_coefficients[2];

      /* Sum: '<S7>/Sum' incorporates:
       *  Inport: '<Root>/load_coefficients'
       */
      LCITopology6x6Minimal_B.Sum_c = LCITopology6x6Minimal_U.load_coefficients
        [1] + LCITopology6x6Minimal_B.Product;

      /* Product: '<S7>/Product1' */
      LCITopology6x6Minimal_B.Product1 = LCITopology6x6Minimal_B.Abs *
        LCITopology6x6Minimal_B.Sum_c;

      /* Sum: '<S7>/Sum1' incorporates:
       *  Inport: '<Root>/load_coefficients'
       */
      LCITopology6x6Minimal_B.Sum1_l =
        LCITopology6x6Minimal_U.load_coefficients[0] +
        LCITopology6x6Minimal_B.Product1;

      /* Outport: '<Root>/machine_load' incorporates:
       *  Product: '<S7>/Product2'
       */
      LCITopology6x6Minimal_Y.machine_load = LCITopology6x6Minimal_B.Sign *
        LCITopology6x6Minimal_B.Sum1_l;

      /* Switch: '<S74>/Switch' incorporates:
       *  Constant: '<S74>/Constant1'
       */
      if (LCITopology6x6Minimal_P.Constant1_Value_c) {
        /* Switch: '<S74>/Switch' */
        LCITopology6x6Minimal_B.ikq2 =
          LCITopology6x6Minimal_B.changeIqIdcurrentsigns[0];
      } else {
        /* Switch: '<S74>/Switch' incorporates:
         *  Constant: '<S74>/Constant2'
         */
        LCITopology6x6Minimal_B.ikq2 = LCITopology6x6Minimal_P.Constant2_Value;
      }

      /* End of Switch: '<S74>/Switch' */

      /* Gain: '<S84>/1//Ll_q' */
      LCITopology6x6Minimal_B.uLl_q[0] = LCITopology6x6Minimal_P.uLl_q_Gain_h[0]
        * LCITopology6x6Minimal_B.xk1[0];
      LCITopology6x6Minimal_B.uLl_q[1] = LCITopology6x6Minimal_P.uLl_q_Gain_h[1]
        * LCITopology6x6Minimal_B.xk1[4];

      /* Switch: '<S75>/Switch3' incorporates:
       *  Constant: '<S75>/Constant8'
       */
      if (LCITopology6x6Minimal_P.Constant8_Value >=
          LCITopology6x6Minimal_P.Switch3_Threshold) {
        /* Switch: '<S75>/Switch3' */
        LCITopology6x6Minimal_B.Switch3[0] = LCITopology6x6Minimal_B.Switch2_a;
        LCITopology6x6Minimal_B.Switch3[1] =
          LCITopology6x6Minimal_B.MathFunction3;
      } else {
        /* Switch: '<S75>/Switch3' incorporates:
         *  Constant: '<S75>/Laqd_nosat'
         */
        LCITopology6x6Minimal_B.Switch3[0] =
          LCITopology6x6Minimal_P.Laqd_nosat_Value[0];
        LCITopology6x6Minimal_B.Switch3[1] =
          LCITopology6x6Minimal_P.Laqd_nosat_Value[1];
      }

      /* End of Switch: '<S75>/Switch3' */

      /* Sum: '<S82>/Sum2' */
      tmpForInput_idx_0 = LCITopology6x6Minimal_B.Mult1[0];

      /* Product: '<S84>/Product' */
      LCITopology6x6Minimal_B.Product_k[0] = LCITopology6x6Minimal_B.uLl_q[0] *
        LCITopology6x6Minimal_B.Switch3[0];

      /* Sum: '<S82>/Sum2' */
      tmpForInput_idx_0 += LCITopology6x6Minimal_B.Mult1[1];

      /* Product: '<S84>/Product' */
      LCITopology6x6Minimal_B.Product_k[1] = LCITopology6x6Minimal_B.uLl_q[1] *
        LCITopology6x6Minimal_B.Switch3[0];

      /* Sum: '<S82>/Sum2' */
      LCITopology6x6Minimal_B.Sum2 = tmpForInput_idx_0;

      /* Outport: '<Root>/machine_electrical_torque' incorporates:
       *  Gain: '<S73>/units2'
       */
      LCITopology6x6Minimal_Y.machine_electrical_torque =
        LCITopology6x6Minimal_P.units2_Gain * LCITopology6x6Minimal_B.Sum2;

      /* Fcn: '<S73>/Fcn' */
      tmpForInput_idx_0 = rt_remd_snf(tmpForInput_idx_1, 6.2831853071795862);

      /* Fcn: '<S73>/Fcn' */
      LCITopology6x6Minimal_B.Fcn = tmpForInput_idx_0;

      /* Outport: '<Root>/machine_mechanical_angle' incorporates:
       *  Gain: '<S73>/t'
       */
      LCITopology6x6Minimal_Y.machine_mechanical_angle =
        LCITopology6x6Minimal_P.t_Gain * LCITopology6x6Minimal_B.Fcn;

      /* Product: '<S73>/Product' */
      LCITopology6x6Minimal_B.Pe = LCITopology6x6Minimal_B.Switch_e *
        LCITopology6x6Minimal_B.Sum2;

      /* Outport: '<Root>/machine_electrical_power' incorporates:
       *  Gain: '<S73>/units1'
       */
      LCITopology6x6Minimal_Y.machine_electrical_power =
        LCITopology6x6Minimal_P.units1_Gain * LCITopology6x6Minimal_B.Pe;

      /* Sum: '<S84>/sum2' */
      tmpForInput_idx_0 = LCITopology6x6Minimal_B.Product_k[0];
      tmpForInput_idx_0 += LCITopology6x6Minimal_B.Product_k[1];

      /* Sum: '<S84>/sum2' */
      LCITopology6x6Minimal_B.phimq = tmpForInput_idx_0;

      /* Gain: '<S84>/1//Ll_d' */
      LCITopology6x6Minimal_B.uLl_d[0] = LCITopology6x6Minimal_P.uLl_d_Gain_p[0]
        * LCITopology6x6Minimal_B.xk1[1];

      /* Product: '<S84>/Product1' */
      LCITopology6x6Minimal_B.Product1_h[0] = LCITopology6x6Minimal_B.uLl_d[0] *
        LCITopology6x6Minimal_B.Switch3[1];

      /* Gain: '<S84>/1//Ll_d' */
      LCITopology6x6Minimal_B.uLl_d[1] = LCITopology6x6Minimal_P.uLl_d_Gain_p[1]
        * LCITopology6x6Minimal_B.xk1[2];

      /* Product: '<S84>/Product1' */
      LCITopology6x6Minimal_B.Product1_h[1] = LCITopology6x6Minimal_B.uLl_d[1] *
        LCITopology6x6Minimal_B.Switch3[1];

      /* Gain: '<S84>/1//Ll_d' */
      LCITopology6x6Minimal_B.uLl_d[2] = LCITopology6x6Minimal_P.uLl_d_Gain_p[2]
        * LCITopology6x6Minimal_B.xk1[3];

      /* Product: '<S84>/Product1' */
      LCITopology6x6Minimal_B.Product1_h[2] = LCITopology6x6Minimal_B.uLl_d[2] *
        LCITopology6x6Minimal_B.Switch3[1];

      /* Sum: '<S84>/sum1' */
      tmpForInput_idx_0 = LCITopology6x6Minimal_B.Product1_h[0];
      tmpForInput_idx_0 += LCITopology6x6Minimal_B.Product1_h[1];
      tmpForInput_idx_0 += LCITopology6x6Minimal_B.Product1_h[2];

      /* Sum: '<S84>/sum1' */
      LCITopology6x6Minimal_B.phimd = tmpForInput_idx_0;

      /* Gain: '<S76>/1_Vb' */
      LCITopology6x6Minimal_B.u_Vb[0] = LCITopology6x6Minimal_P.u_Vb_Gain *
        LCITopology6x6Minimal_B.StateSpace_o1[14];
      LCITopology6x6Minimal_B.u_Vb[1] = LCITopology6x6Minimal_P.u_Vb_Gain *
        LCITopology6x6Minimal_B.StateSpace_o1[15];

      /* Sum: '<S76>/Sum' */
      LCITopology6x6Minimal_B.Sum_h = LCITopology6x6Minimal_B.u_Vb[1] -
        LCITopology6x6Minimal_B.u_Vb[0];

      /* Gain: '<S76>/Gain' */
      LCITopology6x6Minimal_B.B = LCITopology6x6Minimal_P.Gain_Gain *
        LCITopology6x6Minimal_B.Sum_h;

      /* Sum: '<S76>/Sum1' */
      LCITopology6x6Minimal_B.A = LCITopology6x6Minimal_B.B +
        LCITopology6x6Minimal_B.u_Vb[0];

      /* Sum: '<S76>/Sum2' */
      LCITopology6x6Minimal_B.C = (0.0 - LCITopology6x6Minimal_B.B) -
        LCITopology6x6Minimal_B.A;

      /* SignalConversion generated from: '<S97>/Gain3' */
      LCITopology6x6Minimal_B.TmpSignalConversionAtGain3Inp_a[0] =
        LCITopology6x6Minimal_B.A;
      LCITopology6x6Minimal_B.TmpSignalConversionAtGain3Inp_a[1] =
        LCITopology6x6Minimal_B.B;
      LCITopology6x6Minimal_B.TmpSignalConversionAtGain3Inp_a[2] =
        LCITopology6x6Minimal_B.C;
      for (i = 0; i < 3; i++) {
        /* Gain: '<S97>/Gain3' */
        LCITopology6x6Minimal_B.Gain3_d[i] = 0.0;
        LCITopology6x6Minimal_B.Gain3_d[i] +=
          LCITopology6x6Minimal_P.Gain3_Gain_o[i] *
          LCITopology6x6Minimal_B.TmpSignalConversionAtGain3Inp_a[0];
        LCITopology6x6Minimal_B.Gain3_d[i] +=
          LCITopology6x6Minimal_P.Gain3_Gain_o[i + 3] *
          LCITopology6x6Minimal_B.TmpSignalConversionAtGain3Inp_a[1];
        LCITopology6x6Minimal_B.Gain3_d[i] +=
          LCITopology6x6Minimal_P.Gain3_Gain_o[i + 6] *
          LCITopology6x6Minimal_B.TmpSignalConversionAtGain3Inp_a[2];

        /* Gain: '<S97>/Gain1' */
        LCITopology6x6Minimal_B.Gain1_p[i] =
          LCITopology6x6Minimal_P.Gain1_Gain_gd *
          LCITopology6x6Minimal_B.Gain3_d[i];
      }

      /* RelationalOperator: '<S98>/Compare' incorporates:
       *  Constant: '<S96>/Constant'
       *  Constant: '<S98>/Constant'
       */
      LCITopology6x6Minimal_B.Compare_l =
        (LCITopology6x6Minimal_P.AlphaBetaZerotodq0_Alignment_b ==
         LCITopology6x6Minimal_P.CompareToConstant_const_d);

      /* Outputs for Enabled SubSystem: '<S96>/Subsystem1' incorporates:
       *  EnablePort: '<S101>/Enable'
       */
      if (LCITopology6x6Minimal_B.Compare_l > 0) {
        /* Fcn: '<S101>/Fcn' */
        LCITopology6x6Minimal_B.Fcn_b = LCITopology6x6Minimal_B.Gain1_p[0] * std::
          cos(LCITopology6x6Minimal_B.Sum3) + LCITopology6x6Minimal_B.Gain1_p[1]
          * std::sin(LCITopology6x6Minimal_B.Sum3);

        /* Fcn: '<S101>/Fcn1' */
        LCITopology6x6Minimal_B.Fcn1_j = -LCITopology6x6Minimal_B.Gain1_p[0] *
          std::sin(LCITopology6x6Minimal_B.Sum3) +
          LCITopology6x6Minimal_B.Gain1_p[1] * std::cos
          (LCITopology6x6Minimal_B.Sum3);
      }

      /* End of Outputs for SubSystem: '<S96>/Subsystem1' */

      /* RelationalOperator: '<S99>/Compare' incorporates:
       *  Constant: '<S96>/Constant'
       *  Constant: '<S99>/Constant'
       */
      LCITopology6x6Minimal_B.Compare_ld =
        (LCITopology6x6Minimal_P.AlphaBetaZerotodq0_Alignment_b ==
         LCITopology6x6Minimal_P.CompareToConstant1_const_o);

      /* Outputs for Enabled SubSystem: '<S96>/Subsystem - pi//2 delay' incorporates:
       *  EnablePort: '<S100>/Enable'
       */
      if (LCITopology6x6Minimal_B.Compare_ld > 0) {
        /* Fcn: '<S100>/Fcn' */
        LCITopology6x6Minimal_B.Fcn_bn = LCITopology6x6Minimal_B.Gain1_p[0] *
          std::sin(LCITopology6x6Minimal_B.Sum3) -
          LCITopology6x6Minimal_B.Gain1_p[1] * std::cos
          (LCITopology6x6Minimal_B.Sum3);

        /* Fcn: '<S100>/Fcn1' */
        LCITopology6x6Minimal_B.Fcn1_p = LCITopology6x6Minimal_B.Gain1_p[0] *
          std::cos(LCITopology6x6Minimal_B.Sum3) +
          LCITopology6x6Minimal_B.Gain1_p[1] * std::sin
          (LCITopology6x6Minimal_B.Sum3);
      }

      /* End of Outputs for SubSystem: '<S96>/Subsystem - pi//2 delay' */

      /* Switch: '<S96>/Switch' */
      if (LCITopology6x6Minimal_B.Compare_l != 0) {
        /* Switch: '<S96>/Switch' */
        LCITopology6x6Minimal_B.Switch_i[0] = LCITopology6x6Minimal_B.Fcn_b;
        LCITopology6x6Minimal_B.Switch_i[1] = LCITopology6x6Minimal_B.Fcn1_j;
      } else {
        /* Switch: '<S96>/Switch' */
        LCITopology6x6Minimal_B.Switch_i[0] = LCITopology6x6Minimal_B.Fcn_bn;
        LCITopology6x6Minimal_B.Switch_i[1] = LCITopology6x6Minimal_B.Fcn1_p;
      }

      /* End of Switch: '<S96>/Switch' */

      /* Switch: '<S75>/Switch2' incorporates:
       *  Constant: '<S75>/Constant5'
       */
      if (LCITopology6x6Minimal_P.Constant5_Value_c >=
          LCITopology6x6Minimal_P.Switch2_Threshold) {
        /* Switch: '<S75>/Switch2' */
        LCITopology6x6Minimal_B.Switch2[0] = LCITopology6x6Minimal_B.Switch1_j;
        LCITopology6x6Minimal_B.Switch2[1] = LCITopology6x6Minimal_B.Lmsatd;
      } else {
        /* Switch: '<S75>/Switch2' incorporates:
         *  Constant: '<S75>/Lmqd_nosat'
         */
        LCITopology6x6Minimal_B.Switch2[0] =
          LCITopology6x6Minimal_P.Lmqd_nosat_Value[0];
        LCITopology6x6Minimal_B.Switch2[1] =
          LCITopology6x6Minimal_P.Lmqd_nosat_Value[1];
      }

      /* End of Switch: '<S75>/Switch2' */

      /* Fcn: '<S80>/x->theta' */
      LCITopology6x6Minimal_B.xtheta = rt_atan2d_snf
        (LCITopology6x6Minimal_B.Switch_i[0], LCITopology6x6Minimal_B.Switch_i[1]);

      /* Gain: '<S81>/Gain' */
      LCITopology6x6Minimal_B.Gain = LCITopology6x6Minimal_P.Gain_Gain_d *
        LCITopology6x6Minimal_B.xtheta;

      /* Product: '<S79>/Product2' */
      LCITopology6x6Minimal_B.Product2_e = LCITopology6x6Minimal_B.Switch_i[1] *
        LCITopology6x6Minimal_B.changeIqIdcurrentsigns[0];

      /* Product: '<S79>/Product3' */
      LCITopology6x6Minimal_B.Product3 = LCITopology6x6Minimal_B.Switch_i[0] *
        LCITopology6x6Minimal_B.changeIqIdcurrentsigns[1];

      /* Sum: '<S79>/Sum' */
      LCITopology6x6Minimal_B.Peo = LCITopology6x6Minimal_B.Product2_e +
        LCITopology6x6Minimal_B.Product3;

      /* Gain: '<S79>/Gain' */
      LCITopology6x6Minimal_B.Gain_d = LCITopology6x6Minimal_P.Gain_Gain_m *
        LCITopology6x6Minimal_B.Peo;

      /* Product: '<S79>/Product4' */
      LCITopology6x6Minimal_B.Product4 = LCITopology6x6Minimal_B.Switch_i[1] *
        LCITopology6x6Minimal_B.changeIqIdcurrentsigns[1];

      /* Product: '<S79>/Product5' */
      LCITopology6x6Minimal_B.Product5_d = LCITopology6x6Minimal_B.Switch_i[0] *
        LCITopology6x6Minimal_B.changeIqIdcurrentsigns[0];

      /* Sum: '<S79>/Sum1' */
      LCITopology6x6Minimal_B.Qeo = LCITopology6x6Minimal_B.Product4 -
        LCITopology6x6Minimal_B.Product5_d;

      /* Gain: '<S79>/Gain1' */
      LCITopology6x6Minimal_B.Gain1_j = LCITopology6x6Minimal_P.Gain1_Gain_f *
        LCITopology6x6Minimal_B.Qeo;

      /* Gain: '<S74>/output formatting' */
      LCITopology6x6Minimal_B.outputformatting[0] =
        LCITopology6x6Minimal_P.outputformatting_Gain[0] *
        LCITopology6x6Minimal_B.Gain3[0];
      LCITopology6x6Minimal_B.outputformatting[3] =
        LCITopology6x6Minimal_P.outputformatting_Gain[3] *
        LCITopology6x6Minimal_B.changeIqIdcurrentsigns[0];
      LCITopology6x6Minimal_B.outputformatting[1] =
        LCITopology6x6Minimal_P.outputformatting_Gain[1] *
        LCITopology6x6Minimal_B.Gain3[1];
      LCITopology6x6Minimal_B.outputformatting[4] =
        LCITopology6x6Minimal_P.outputformatting_Gain[4] *
        LCITopology6x6Minimal_B.changeIqIdcurrentsigns[1];
      LCITopology6x6Minimal_B.outputformatting[2] =
        LCITopology6x6Minimal_P.outputformatting_Gain[2] *
        LCITopology6x6Minimal_B.Gain3[2];
      LCITopology6x6Minimal_B.outputformatting[5] =
        LCITopology6x6Minimal_P.outputformatting_Gain[5] *
        LCITopology6x6Minimal_B.changeIqIdcurrentsigns[2];
      LCITopology6x6Minimal_B.outputformatting[6] =
        LCITopology6x6Minimal_P.outputformatting_Gain[6] *
        LCITopology6x6Minimal_B.changeIqIdcurrentsigns[4];
      LCITopology6x6Minimal_B.outputformatting[7] =
        LCITopology6x6Minimal_P.outputformatting_Gain[7] *
        LCITopology6x6Minimal_B.ikq2;
      LCITopology6x6Minimal_B.outputformatting[8] =
        LCITopology6x6Minimal_P.outputformatting_Gain[8] *
        LCITopology6x6Minimal_B.changeIqIdcurrentsigns[3];
      LCITopology6x6Minimal_B.outputformatting[9] =
        LCITopology6x6Minimal_P.outputformatting_Gain[9] *
        LCITopology6x6Minimal_B.phimq;
      LCITopology6x6Minimal_B.outputformatting[10] =
        LCITopology6x6Minimal_P.outputformatting_Gain[10] *
        LCITopology6x6Minimal_B.phimd;
      LCITopology6x6Minimal_B.outputformatting[11] =
        LCITopology6x6Minimal_P.outputformatting_Gain[11] *
        LCITopology6x6Minimal_B.Switch_i[1];
      LCITopology6x6Minimal_B.outputformatting[12] =
        LCITopology6x6Minimal_P.outputformatting_Gain[12] *
        LCITopology6x6Minimal_B.Switch_i[0];
      LCITopology6x6Minimal_B.outputformatting[13] =
        LCITopology6x6Minimal_P.outputformatting_Gain[13] *
        LCITopology6x6Minimal_B.Switch2[0];
      LCITopology6x6Minimal_B.outputformatting[14] =
        LCITopology6x6Minimal_P.outputformatting_Gain[14] *
        LCITopology6x6Minimal_B.Switch2[1];
      LCITopology6x6Minimal_B.outputformatting[15] =
        LCITopology6x6Minimal_P.outputformatting_Gain[15] *
        LCITopology6x6Minimal_B.Gain;
      LCITopology6x6Minimal_B.outputformatting[16] =
        LCITopology6x6Minimal_P.outputformatting_Gain[16] *
        LCITopology6x6Minimal_B.Gain_d;
      LCITopology6x6Minimal_B.outputformatting[17] =
        LCITopology6x6Minimal_P.outputformatting_Gain[17] *
        LCITopology6x6Minimal_B.Gain1_j;

      /* Outport: '<Root>/machine_flux_d' */
      LCITopology6x6Minimal_Y.machine_flux_d =
        LCITopology6x6Minimal_B.outputformatting[10];

      /* Outport: '<Root>/machine_flux_q' */
      LCITopology6x6Minimal_Y.machine_flux_q =
        LCITopology6x6Minimal_B.outputformatting[9];

      /* Outport: '<Root>/machine_stator_current_d' */
      LCITopology6x6Minimal_Y.machine_stator_current_d =
        LCITopology6x6Minimal_B.outputformatting[4];

      /* Outport: '<Root>/machine_stator_current_q' */
      LCITopology6x6Minimal_Y.machine_stator_current_q =
        LCITopology6x6Minimal_B.outputformatting[3];

      /* Outport: '<Root>/machine_field_current_d' */
      LCITopology6x6Minimal_Y.machine_field_current_d =
        LCITopology6x6Minimal_B.outputformatting[5];

      /* Outport: '<Root>/machine_damper_current_q1' */
      LCITopology6x6Minimal_Y.machine_damper_current_q1 =
        LCITopology6x6Minimal_B.outputformatting[6];

      /* Outport: '<Root>/machine_damper_current_q2' */
      LCITopology6x6Minimal_Y.machine_damper_current_q2 =
        LCITopology6x6Minimal_B.outputformatting[7];

      /* Outport: '<Root>/machine_damper_current_d' */
      LCITopology6x6Minimal_Y.machine_damper_current_d =
        LCITopology6x6Minimal_B.outputformatting[8];

      /* Outport: '<Root>/machine_stator_voltage_q' */
      LCITopology6x6Minimal_Y.machine_stator_voltage_q =
        LCITopology6x6Minimal_B.outputformatting[11];

      /* Outport: '<Root>/machine_stator_voltage_d' */
      LCITopology6x6Minimal_Y.machine_stator_voltage_d =
        LCITopology6x6Minimal_B.outputformatting[12];

      /* Outport: '<Root>/machine_phase_current_a' */
      LCITopology6x6Minimal_Y.machine_phase_current_a =
        LCITopology6x6Minimal_B.outputformatting[0];

      /* Outport: '<Root>/machine_phase_current_b' */
      LCITopology6x6Minimal_Y.machine_phase_current_b =
        LCITopology6x6Minimal_B.outputformatting[1];

      /* Outport: '<Root>/machine_phase_current_c' */
      LCITopology6x6Minimal_Y.machine_phase_current_c =
        LCITopology6x6Minimal_B.outputformatting[2];

      /* Gain: '<S71>/N' incorporates:
       *  Inport: '<Root>/voltage_excitation'
       */
      LCITopology6x6Minimal_B.Vfd = LCITopology6x6Minimal_P.N_Gain *
        LCITopology6x6Minimal_U.voltage_excitation;
      for (i = 0; i < 5; i++) {
        /* UnitDelay: '<S86>/voltages' */
        LCITopology6x6Minimal_B.voltages[i] =
          LCITopology6x6Minimal_DW.voltages_DSTATE[i];
      }

      /* DigitalClock: '<S86>/Digital Clock' */
      LCITopology6x6Minimal_B.DigitalClock_g = ((((&LCITopology6x6Minimal_M)
        ->Timing.clockTick1+(&LCITopology6x6Minimal_M)->Timing.clockTickH1*
        4294967296.0)) * 1.25E-5);

      /* Switch: '<S86>/IC' incorporates:
       *  Constant: '<S75>/Constant2'
       *  Switch: '<S75>/Switch1'
       */
      if (LCITopology6x6Minimal_B.DigitalClock_g >=
          LCITopology6x6Minimal_P.IC_Threshold) {
        /* Sum: '<S86>/sum' incorporates:
         *  Constant: '<S71>/[ Vkd =0 Vkq1=0  Vkq2=0 ]'
         */
        LCITopology6x6Minimal_B.ukuk1[0] = LCITopology6x6Minimal_B.Switch_i[1] +
          LCITopology6x6Minimal_B.voltages[0];
        LCITopology6x6Minimal_B.ukuk1[1] = LCITopology6x6Minimal_B.Switch_i[0] +
          LCITopology6x6Minimal_B.voltages[1];
        LCITopology6x6Minimal_B.ukuk1[2] = LCITopology6x6Minimal_B.Vfd +
          LCITopology6x6Minimal_B.voltages[2];
        LCITopology6x6Minimal_B.ukuk1[3] =
          LCITopology6x6Minimal_P.Vkd0Vkq10Vkq20_Value[0] +
          LCITopology6x6Minimal_B.voltages[3];
        LCITopology6x6Minimal_B.ukuk1[4] =
          LCITopology6x6Minimal_P.Vkd0Vkq10Vkq20_Value[1] +
          LCITopology6x6Minimal_B.voltages[4];
        if (LCITopology6x6Minimal_P.Constant2_Value_i >=
            LCITopology6x6Minimal_P.Switch1_Threshold) {
          /* Switch: '<S75>/Switch1' incorporates:
           *  Product: '<S85>/Product1'
           */
          std::memcpy(&LCITopology6x6Minimal_B.Switch1[0],
                      &LCITopology6x6Minimal_B.RLinv[0], 25U * sizeof(real_T));
        } else {
          /* Switch: '<S75>/Switch1' incorporates:
           *  Constant: '<S75>/Constant6'
           */
          std::memcpy(&LCITopology6x6Minimal_B.Switch1[0],
                      &LCITopology6x6Minimal_P.Constant6_Value[0], 25U * sizeof
                      (real_T));
        }

        for (i = 0; i < 25; i++) {
          /* Assignment: '<S83>/W(1,2)=wr' incorporates:
           *  Constant: '<S83>/u1'
           */
          LCITopology6x6Minimal_B.W12wr[i] =
            LCITopology6x6Minimal_P.u1_Value_p[i];
        }

        /* Assignment: '<S83>/W(1,2)=wr' incorporates:
         *  Constant: '<S75>/Constant2'
         *  Switch: '<S75>/Switch1'
         */
        LCITopology6x6Minimal_B.W12wr[5] = LCITopology6x6Minimal_B.Switch_e;

        /* Gain: '<S83>/Gain1' */
        LCITopology6x6Minimal_B.Gain1_e = LCITopology6x6Minimal_P.Gain1_Gain_g *
          LCITopology6x6Minimal_B.Switch_e;

        /* Assignment: '<S83>/W(2,1)=-wr' incorporates:
         *  Assignment: '<S83>/W(1,2)=wr'
         */
        std::memcpy(&LCITopology6x6Minimal_B.W21wr[0],
                    &LCITopology6x6Minimal_B.W12wr[0], 25U * sizeof(real_T));

        /* Assignment: '<S83>/W(2,1)=-wr' */
        LCITopology6x6Minimal_B.W21wr[1] = LCITopology6x6Minimal_B.Gain1_e;
        for (i = 0; i < 25; i++) {
          /* Sum: '<S75>/Sum1' incorporates:
           *  Assignment: '<S83>/W(2,1)=-wr'
           *  Switch: '<S75>/Switch1'
           */
          LCITopology6x6Minimal_B.A_m[i] = (0.0 -
            LCITopology6x6Minimal_B.W21wr[i]) -
            LCITopology6x6Minimal_B.Switch1[i];

          /* Gain: '<S94>/wbase*Ts//2' incorporates:
           *  Sum: '<S75>/Sum1'
           */
          LCITopology6x6Minimal_B.wbaseTs2[i] =
            LCITopology6x6Minimal_P.wbaseTs2_Gain *
            LCITopology6x6Minimal_B.A_m[i];

          /* Sum: '<S94>/Sum1' incorporates:
           *  Constant: '<S94>/u5'
           *  Gain: '<S94>/wbase*Ts//2'
           */
          LCITopology6x6Minimal_B.Sum1_b[i] =
            LCITopology6x6Minimal_P.u5_Value_o[i] -
            LCITopology6x6Minimal_B.wbaseTs2[i];
        }

        /* Product: '<S94>/inversion' incorporates:
         *  Sum: '<S94>/Sum1'
         */
        rt_invd5x5_snf(LCITopology6x6Minimal_B.Sum1_b,
                       LCITopology6x6Minimal_B.inversion);
        for (i = 0; i < 25; i++) {
          /* Gain: '<S94>/wbase*Ts//2 ' incorporates:
           *  Product: '<S94>/inversion'
           */
          LCITopology6x6Minimal_B.wbaseTs2_c[i] =
            LCITopology6x6Minimal_P.wbaseTs2_Gain_g *
            LCITopology6x6Minimal_B.inversion[i];

          /* Product: '<S86>/Product1' incorporates:
           *  Gain: '<S94>/wbase*Ts//2 '
           */
          tmp_1[i] = LCITopology6x6Minimal_B.wbaseTs2_c[i];
        }

        /* Product: '<S86>/Product1' */
        for (i = 0; i < 5; i++) {
          tmp[i] = LCITopology6x6Minimal_B.ukuk1[i];
        }

        for (i_0 = 0; i_0 < 5; i_0++) {
          tmp_0[i_0] = 0.0;
          for (i = 0; i < 5; i++) {
            tmpForInput_idx_1 = tmp_0[i_0];
            tmpForInput_idx_1 += tmp_1[5 * i + i_0] * tmp[i];
            tmp_0[i_0] = tmpForInput_idx_1;
          }

          /* Product: '<S86>/Product1' */
          LCITopology6x6Minimal_B.Product1_b[i_0] = tmp_0[i_0];
        }

        for (i = 0; i < 25; i++) {
          /* Sum: '<S94>/Sum5' incorporates:
           *  Constant: '<S94>/u5'
           *  Gain: '<S94>/wbase*Ts//2'
           */
          LCITopology6x6Minimal_B.Sum5[i] = LCITopology6x6Minimal_P.u5_Value_o[i]
            + LCITopology6x6Minimal_B.wbaseTs2[i];

          /* Product: '<S94>/Product4' incorporates:
           *  Product: '<S94>/inversion'
           *  Sum: '<S94>/Sum5'
           */
          tmp_1[i] = LCITopology6x6Minimal_B.inversion[i];
          tmp_2[i] = LCITopology6x6Minimal_B.Sum5[i];
        }

        for (i_0 = 0; i_0 < 5; i_0++) {
          for (i = 0; i < 5; i++) {
            /* Product: '<S94>/Product4' */
            LCITopology6x6Minimal_B.Product4_e[i + 5 * i_0] = 0.0;
            for (i_1 = 0; i_1 < 5; i_1++) {
              LCITopology6x6Minimal_B.Product4_e[i + 5 * i_0] += tmp_1[5 * i_1 +
                i] * tmp_2[5 * i_0 + i_1];
            }
          }
        }

        /* Product: '<S86>/Product2' incorporates:
         *  Product: '<S94>/Product4'
         */
        std::memcpy(&tmp_1[0], &LCITopology6x6Minimal_B.Product4_e[0], 25U *
                    sizeof(real_T));
        for (i = 0; i < 5; i++) {
          tmp[i] = LCITopology6x6Minimal_B.xk1[i];
        }

        for (i = 0; i < 5; i++) {
          tmp_0[i] = 0.0;
          for (i_0 = 0; i_0 < 5; i_0++) {
            tmpForInput_idx_1 = tmp_0[i];
            tmpForInput_idx_1 += tmp_1[5 * i_0 + i] * tmp[i_0];
            tmp_0[i] = tmpForInput_idx_1;
          }

          /* Product: '<S86>/Product2' */
          LCITopology6x6Minimal_B.Product2_a[i] = tmp_0[i];

          /* Sum: '<S86>/Ad*x(k-1) + Bd*( u(k-1) + u(k))' */
          LCITopology6x6Minimal_B.xk[i] = LCITopology6x6Minimal_B.Product2_a[i]
            + LCITopology6x6Minimal_B.Product1_b[i];

          /* Switch: '<S86>/IC' */
          LCITopology6x6Minimal_B.IC[i] = LCITopology6x6Minimal_B.xk[i];
        }

        /* End of Product: '<S86>/Product2' */
      } else {
        /* Switch: '<S86>/IC' */
        for (i = 0; i < 5; i++) {
          LCITopology6x6Minimal_B.IC[i] = LCITopology6x6Minimal_B.xk1[i];
        }
      }

      /* End of Switch: '<S86>/IC' */

      /* Gain: '<S73>/1_Pb' incorporates:
       *  Constant: '<Root>/Constant6'
       */
      LCITopology6x6Minimal_B.u_Pb = LCITopology6x6Minimal_P.u_Pb_Gain *
        LCITopology6x6Minimal_P.Constant6_Value_m;

      /* Fcn: '<S73>/div' */
      LCITopology6x6Minimal_B.div = LCITopology6x6Minimal_B.u_Pb /
        LCITopology6x6Minimal_B.Switch_e;

      /* Gain: '<S73>/F' */
      LCITopology6x6Minimal_B.F = LCITopology6x6Minimal_P.F_Gain *
        LCITopology6x6Minimal_B.n;

      /* Gain: '<S73>/1_Pb1' incorporates:
       *  Outport: '<Root>/machine_load'
       */
      LCITopology6x6Minimal_B.u_Pb1 = LCITopology6x6Minimal_P.u_Pb1_Gain *
        LCITopology6x6Minimal_Y.machine_load;

      /* Sum: '<S73>/Sum2' */
      LCITopology6x6Minimal_B.Sum2_o = ((LCITopology6x6Minimal_B.div -
        LCITopology6x6Minimal_B.Sum2) - LCITopology6x6Minimal_B.F) -
        LCITopology6x6Minimal_B.u_Pb1;

      /* Gain: '<S73>/1 ----- 2H' */
      LCITopology6x6Minimal_B.u2H = LCITopology6x6Minimal_P.u2H_Gain *
        LCITopology6x6Minimal_B.Sum2_o;

      /* Sum: '<S73>/Sum1' incorporates:
       *  Constant: '<S73>/nominal speed'
       */
      LCITopology6x6Minimal_B.Sum1_p = LCITopology6x6Minimal_B.Switch_e -
        LCITopology6x6Minimal_P.nominalspeed_Value;

      /* DiscreteIntegrator: '<S109>/Rotor spee<d deviation (dw)' incorporates:
       *  Inport: '<Root>/enable_forced_speed'
       */
      if (LCITopology6x6Minimal_DW.Rotorspeeddeviationdw_IC_LOADIN != 0) {
        LCITopology6x6Minimal_DW.Rotorspeeddeviationdw_DSTATE =
          LCITopology6x6Minimal_B.Sum1_p;

        /* DiscreteIntegrator: '<S109>/Rotor spee<d deviation (dw)' */
        LCITopology6x6Minimal_B.dw =
          LCITopology6x6Minimal_DW.Rotorspeeddeviationdw_DSTATE;
      } else if ((LCITopology6x6Minimal_U.enable_forced_speed != 0.0) ||
                 (LCITopology6x6Minimal_DW.Rotorspeeddeviationdw_PrevReset != 0))
      {
        LCITopology6x6Minimal_DW.Rotorspeeddeviationdw_DSTATE =
          LCITopology6x6Minimal_B.Sum1_p;

        /* DiscreteIntegrator: '<S109>/Rotor spee<d deviation (dw)' */
        LCITopology6x6Minimal_B.dw =
          LCITopology6x6Minimal_DW.Rotorspeeddeviationdw_DSTATE;
      } else {
        /* DiscreteIntegrator: '<S109>/Rotor spee<d deviation (dw)' */
        LCITopology6x6Minimal_B.dw =
          LCITopology6x6Minimal_P.Rotorspeeddeviationdw_gainval *
          LCITopology6x6Minimal_B.u2H +
          LCITopology6x6Minimal_DW.Rotorspeeddeviationdw_DSTATE;
      }

      /* End of DiscreteIntegrator: '<S109>/Rotor spee<d deviation (dw)' */

      /* Gain: '<S73>/we base' */
      LCITopology6x6Minimal_B.webase = LCITopology6x6Minimal_P.webase_Gain *
        LCITopology6x6Minimal_B.Sum1_h;

      /* Gain: '<S73>/web3' */
      LCITopology6x6Minimal_B.web3 = LCITopology6x6Minimal_P.web3_Gain *
        LCITopology6x6Minimal_B.Switch_e;

      /* Gain: '<S26>/do not delete this gain' */
      LCITopology6x6Minimal_B.donotdeletethisgain_mj =
        LCITopology6x6Minimal_P.donotdeletethisgain_Gain_on *
        LCITopology6x6Minimal_B.StateSpace_o1[25];

      /* Outport: '<Root>/Udc_machine' incorporates:
       *  Inport: '<Root>/grid_pt_ratio'
       *  Product: '<S18>/Divide'
       */
      LCITopology6x6Minimal_Y.Udc_machine =
        LCITopology6x6Minimal_B.donotdeletethisgain_mj /
        LCITopology6x6Minimal_U.grid_pt_ratio;

      /* Product: '<S2>/Product4' incorporates:
       *  Constant: '<S2>/Constant9'
       *  Inport: '<Root>/voltage_frequency'
       */
      LCITopology6x6Minimal_B.Product4_m =
        LCITopology6x6Minimal_P.Constant9_Value *
        LCITopology6x6Minimal_U.voltage_frequency;

      /* Logic: '<Root>/Logical Operator' incorporates:
       *  Inport: '<Root>/close_dc_isolator'
       */
      LCITopology6x6Minimal_B.LogicalOperator =
        !LCITopology6x6Minimal_U.close_dc_isolator;

      /* DataTypeConversion: '<S38>/Data Type Conversion' */
      LCITopology6x6Minimal_B.DataTypeConversion =
        LCITopology6x6Minimal_B.LogicalOperator;

      /* DataTypeConversion: '<S40>/Data Type Conversion' incorporates:
       *  Inport: '<Root>/close_dc_isolator'
       */
      LCITopology6x6Minimal_B.DataTypeConversion_m =
        LCITopology6x6Minimal_U.close_dc_isolator;
    }
  }

  if (rtmIsMajorTimeStep((&LCITopology6x6Minimal_M))) {
    int32_T i;
    if (rtmIsMajorTimeStep((&LCITopology6x6Minimal_M))) {
      /* Update for UnitDelay: '<S86>/fluxes' */
      for (i = 0; i < 5; i++) {
        LCITopology6x6Minimal_DW.fluxes_DSTATE[i] = LCITopology6x6Minimal_B.IC[i];
      }

      /* End of Update for UnitDelay: '<S86>/fluxes' */

      /* Update for DiscreteIntegrator: '<S73>/Rotor angle dthetae' */
      LCITopology6x6Minimal_DW.Rotorangledthetae_DSTATE +=
        LCITopology6x6Minimal_P.Rotorangledthetae_gainval *
        LCITopology6x6Minimal_B.webase;

      /* Update for DiscreteIntegrator: '<S36>/Integrator' */
      LCITopology6x6Minimal_DW.Integrator_DSTATE +=
        LCITopology6x6Minimal_P.Integrator_gainval *
        LCITopology6x6Minimal_B.Product4_m;
      if (LCITopology6x6Minimal_DW.Integrator_DSTATE >=
          LCITopology6x6Minimal_P.Integrator_UpperSat) {
        LCITopology6x6Minimal_DW.Integrator_DSTATE =
          LCITopology6x6Minimal_P.Integrator_UpperSat;
      } else {
        if (LCITopology6x6Minimal_DW.Integrator_DSTATE <=
            LCITopology6x6Minimal_P.Integrator_LowerSat) {
          LCITopology6x6Minimal_DW.Integrator_DSTATE =
            LCITopology6x6Minimal_P.Integrator_LowerSat;
        }
      }

      if (LCITopology6x6Minimal_B.Constant4 > 0.0) {
        LCITopology6x6Minimal_DW.Integrator_PrevResetState = 1;
      } else if (LCITopology6x6Minimal_B.Constant4 < 0.0) {
        LCITopology6x6Minimal_DW.Integrator_PrevResetState = -1;
      } else if (LCITopology6x6Minimal_B.Constant4 == 0.0) {
        LCITopology6x6Minimal_DW.Integrator_PrevResetState = 0;
      } else {
        LCITopology6x6Minimal_DW.Integrator_PrevResetState = 2;
      }

      /* End of Update for DiscreteIntegrator: '<S36>/Integrator' */

      /* Update for S-Function (sfun_spssw_discc): '<S164>/State-Space' incorporates:
       *  Constant: '<S166>/SwitchCurrents'
       *  Inport: '<Root>/pulses_in_grid'
       *  Inport: '<Root>/pulses_in_motor'
       */

      /* S-Function block: <S164>/State-Space */
      {
        const real_T *As = (real_T*)LCITopology6x6Minimal_DW.StateSpace_PWORK.AS;
        const real_T *Bs = (real_T*)LCITopology6x6Minimal_DW.StateSpace_PWORK.BS;
        real_T *xtmp = (real_T*)LCITopology6x6Minimal_DW.StateSpace_PWORK.XTMP;
        real_T accum;

        /* Calculate new states... */
        {
          int_T i1;
          real_T *xd = &LCITopology6x6Minimal_DW.StateSpace_DSTATE[0];
          for (i1=0; i1 < 21; i1++) {
            accum = 0.0;

            {
              int_T i2;
              real_T *xd = &LCITopology6x6Minimal_DW.StateSpace_DSTATE[0];
              for (i2=0; i2 < 21; i2++) {
                accum += *(As++) * xd[i2];
              }
            }

            {
              int_T i2;
              const real_T *u0 = LCITopology6x6Minimal_P.SwitchCurrents_Value;
              for (i2=0; i2 < 14; i2++) {
                accum += *(Bs++) * u0[i2];
              }

              accum += *(Bs++) * LCITopology6x6Minimal_B.iabc[0];
              accum += *(Bs++) * LCITopology6x6Minimal_B.iabc[1];
              accum += *(Bs++) * LCITopology6x6Minimal_B.Product5;
              accum += *(Bs++) * LCITopology6x6Minimal_B.Product6;
              accum += *(Bs++) * LCITopology6x6Minimal_B.Product7;
            }

            xtmp[i1] = accum;
          }
        }

        {
          int_T i1;
          real_T *xd = &LCITopology6x6Minimal_DW.StateSpace_DSTATE[0];
          for (i1=0; i1 < 21; i1++) {
            xd[i1] = xtmp[i1];
          }
        }

        {
          int_T *gState = (int_T*)
            LCITopology6x6Minimal_DW.StateSpace_PWORK.G_STATE;

          /* Store switch gates values for next step */
          *(gState++) = (int_T) LCITopology6x6Minimal_B.DataTypeConversion;
          *(gState++) = (int_T) LCITopology6x6Minimal_B.DataTypeConversion_m;
          *(gState++) = (int_T) LCITopology6x6Minimal_U.pulses_in_grid[0];
          *(gState++) = (int_T) LCITopology6x6Minimal_U.pulses_in_grid[5];
          *(gState++) = (int_T) LCITopology6x6Minimal_U.pulses_in_grid[1];
          *(gState++) = (int_T) LCITopology6x6Minimal_U.pulses_in_grid[3];
          *(gState++) = (int_T) LCITopology6x6Minimal_U.pulses_in_grid[2];
          *(gState++) = (int_T) LCITopology6x6Minimal_U.pulses_in_grid[4];
          *(gState++) = (int_T) LCITopology6x6Minimal_U.pulses_in_motor[0];
          *(gState++) = (int_T) LCITopology6x6Minimal_U.pulses_in_motor[5];
          *(gState++) = (int_T) LCITopology6x6Minimal_U.pulses_in_motor[1];
          *(gState++) = (int_T) LCITopology6x6Minimal_U.pulses_in_motor[3];
          *(gState++) = (int_T) LCITopology6x6Minimal_U.pulses_in_motor[2];
          *(gState++) = (int_T) LCITopology6x6Minimal_U.pulses_in_motor[4];
        }
      }

      /* Update for Memory: '<S42>/Memory' */
      LCITopology6x6Minimal_DW.Memory_PreviousInput = LCITopology6x6Minimal_B.y;
    }

    /* Update for Integrator: '<S42>/Integrator' */
    LCITopology6x6Minimal_DW.Integrator_IWORK = 0;

    /* Update for Enabled SubSystem: '<S42>/Automatic Gain Control' incorporates:
     *  EnablePort: '<S43>/Enable'
     */
    if (LCITopology6x6Minimal_DW.AutomaticGainControl_MODE) {
      /* Update for VariableTransportDelay: '<S52>/Variable Transport Delay' */
      {
        real_T **uBuffer = (real_T**)
          &LCITopology6x6Minimal_DW.VariableTransportDelay_PWORK_g.TUbufferPtrs
          [0];
        real_T **tBuffer = (real_T**)
          &LCITopology6x6Minimal_DW.VariableTransportDelay_PWORK_g.TUbufferPtrs
          [1];
        real_T **xBuffer = (real_T**)
          &LCITopology6x6Minimal_DW.VariableTransportDelay_PWORK_g.TUbufferPtrs
          [2];
        real_T simTime = (&LCITopology6x6Minimal_M)->Timing.t[0];
        LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK_g.Head =
          ((LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK_g.Head <
            (LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK_g.CircularBufSize
             -1)) ?
           (LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK_g.Head+1) : 0);
        if (LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK_g.Head ==
            LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK_g.Tail) {
          LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK_g.Tail =
            ((LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK_g.Tail <
              (LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK_g.CircularBufSize
               -1)) ?
             (LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK_g.Tail+1) :
             0);
        }

        (*tBuffer)[LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK_g.Head]
          = simTime;
        (*uBuffer)[LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK_g.Head]
          = LCITopology6x6Minimal_B.integrator_d;
        (*xBuffer)[LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK_g.Head]
          = LCITopology6x6Minimal_X.VariableTransportDelay_CSTATE_c;

        /* when use fixed buffer, reset solver at when buffer is updated
         * to avoid output consistency fail.
         */
      }

      if (rtmIsMajorTimeStep((&LCITopology6x6Minimal_M))) {
        /* Update for Memory: '<S52>/Memory' */
        LCITopology6x6Minimal_DW.Memory_PreviousInput_d =
          LCITopology6x6Minimal_B.Switch_n;
      }

      /* Update for VariableTransportDelay: '<S53>/Variable Transport Delay' */
      {
        real_T **uBuffer = (real_T**)
          &LCITopology6x6Minimal_DW.VariableTransportDelay_PWORK_j.TUbufferPtrs
          [0];
        real_T **tBuffer = (real_T**)
          &LCITopology6x6Minimal_DW.VariableTransportDelay_PWORK_j.TUbufferPtrs
          [1];
        real_T **xBuffer = (real_T**)
          &LCITopology6x6Minimal_DW.VariableTransportDelay_PWORK_j.TUbufferPtrs
          [2];
        real_T simTime = (&LCITopology6x6Minimal_M)->Timing.t[0];
        LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK_a.Head =
          ((LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK_a.Head <
            (LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK_a.CircularBufSize
             -1)) ?
           (LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK_a.Head+1) : 0);
        if (LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK_a.Head ==
            LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK_a.Tail) {
          LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK_a.Tail =
            ((LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK_a.Tail <
              (LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK_a.CircularBufSize
               -1)) ?
             (LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK_a.Tail+1) :
             0);
        }

        (*tBuffer)[LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK_a.Head]
          = simTime;
        (*uBuffer)[LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK_a.Head]
          = LCITopology6x6Minimal_B.integrator_de;
        (*xBuffer)[LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK_a.Head]
          = LCITopology6x6Minimal_X.VariableTransportDelay_CSTATE_l;

        /* when use fixed buffer, reset solver at when buffer is updated
         * to avoid output consistency fail.
         */
      }

      if (rtmIsMajorTimeStep((&LCITopology6x6Minimal_M))) {
        /* Update for Memory: '<S53>/Memory' */
        LCITopology6x6Minimal_DW.Memory_PreviousInput_h =
          LCITopology6x6Minimal_B.Switch_m;
      }
    }

    /* End of Update for SubSystem: '<S42>/Automatic Gain Control' */

    /* Update for VariableTransportDelay: '<S64>/Variable Transport Delay' */
    {
      real_T **uBuffer = (real_T**)
        &LCITopology6x6Minimal_DW.VariableTransportDelay_PWORK.TUbufferPtrs[0];
      real_T **tBuffer = (real_T**)
        &LCITopology6x6Minimal_DW.VariableTransportDelay_PWORK.TUbufferPtrs[1];
      real_T **xBuffer = (real_T**)
        &LCITopology6x6Minimal_DW.VariableTransportDelay_PWORK.TUbufferPtrs[2];
      real_T simTime = (&LCITopology6x6Minimal_M)->Timing.t[0];
      LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK.Head =
        ((LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK.Head <
          (LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK.CircularBufSize
           -1)) ? (LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK.Head+1)
         : 0);
      if (LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK.Head ==
          LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK.Tail) {
        LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK.Tail =
          ((LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK.Tail <
            (LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK.CircularBufSize
             -1)) ? (LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK.Tail+
                     1) : 0);
      }

      (*tBuffer)[LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK.Head] =
        simTime;
      (*uBuffer)[LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK.Head] =
        LCITopology6x6Minimal_B.integrator;
      (*xBuffer)[LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK.Head] =
        LCITopology6x6Minimal_X.VariableTransportDelay_CSTATE;

      /* when use fixed buffer, reset solver at when buffer is updated
       * to avoid output consistency fail.
       */
    }

    if (rtmIsMajorTimeStep((&LCITopology6x6Minimal_M))) {
      /* Update for Memory: '<S64>/Memory' */
      LCITopology6x6Minimal_DW.Memory_PreviousInput_p =
        LCITopology6x6Minimal_B.Switch_g;

      /* Update for UnitDelay: '<S110>/dw_delay' */
      LCITopology6x6Minimal_DW.dw_delay_DSTATE = LCITopology6x6Minimal_B.dw;

      /* Update for UnitDelay: '<S110>/dw_predict' */
      LCITopology6x6Minimal_DW.dw_predict_DSTATE =
        LCITopology6x6Minimal_B.dw_delay;

      /* Update for DiscreteIntegrator: '<S73>/theta' */
      LCITopology6x6Minimal_DW.theta_DSTATE +=
        LCITopology6x6Minimal_P.theta_gainval * LCITopology6x6Minimal_B.web3;

      /* Update for UnitDelay: '<S86>/voltages' incorporates:
       *  Constant: '<S71>/[ Vkd =0 Vkq1=0  Vkq2=0 ]'
       */
      LCITopology6x6Minimal_DW.voltages_DSTATE[0] =
        LCITopology6x6Minimal_B.Switch_i[1];
      LCITopology6x6Minimal_DW.voltages_DSTATE[1] =
        LCITopology6x6Minimal_B.Switch_i[0];
      LCITopology6x6Minimal_DW.voltages_DSTATE[2] = LCITopology6x6Minimal_B.Vfd;
      LCITopology6x6Minimal_DW.voltages_DSTATE[3] =
        LCITopology6x6Minimal_P.Vkd0Vkq10Vkq20_Value[0];
      LCITopology6x6Minimal_DW.voltages_DSTATE[4] =
        LCITopology6x6Minimal_P.Vkd0Vkq10Vkq20_Value[1];

      /* Update for DiscreteIntegrator: '<S109>/Rotor spee<d deviation (dw)' incorporates:
       *  Inport: '<Root>/enable_forced_speed'
       */
      LCITopology6x6Minimal_DW.Rotorspeeddeviationdw_IC_LOADIN = 0U;
      LCITopology6x6Minimal_DW.Rotorspeeddeviationdw_DSTATE =
        LCITopology6x6Minimal_P.Rotorspeeddeviationdw_gainval *
        LCITopology6x6Minimal_B.u2H + LCITopology6x6Minimal_B.dw;
      if (LCITopology6x6Minimal_U.enable_forced_speed > 0.0) {
        LCITopology6x6Minimal_DW.Rotorspeeddeviationdw_PrevReset = 1;
      } else if (LCITopology6x6Minimal_U.enable_forced_speed < 0.0) {
        LCITopology6x6Minimal_DW.Rotorspeeddeviationdw_PrevReset = -1;
      } else if (LCITopology6x6Minimal_U.enable_forced_speed == 0.0) {
        LCITopology6x6Minimal_DW.Rotorspeeddeviationdw_PrevReset = 0;
      } else {
        LCITopology6x6Minimal_DW.Rotorspeeddeviationdw_PrevReset = 2;
      }

      /* End of Update for DiscreteIntegrator: '<S109>/Rotor spee<d deviation (dw)' */
    }

    /* Update for RateLimiter: '<S42>/Rate Limiter' */
    LCITopology6x6Minimal_DW.PrevY = LCITopology6x6Minimal_B.RateLimiter;
    LCITopology6x6Minimal_DW.LastMajorTime = (&LCITopology6x6Minimal_M)
      ->Timing.t[0];

    /* ContTimeOutputInconsistentWithStateAtMajorOutputFlag is set, need to run a minor output */
    if (rtmIsMajorTimeStep((&LCITopology6x6Minimal_M))) {
      if (rtsiGetContTimeOutputInconsistentWithStateAtMajorStep
          (&(&LCITopology6x6Minimal_M)->solverInfo)) {
        rtsiSetSimTimeStep(&(&LCITopology6x6Minimal_M)->solverInfo,
                           MINOR_TIME_STEP);
        rtsiSetContTimeOutputInconsistentWithStateAtMajorStep
          (&(&LCITopology6x6Minimal_M)->solverInfo, false);
        LCITopology6x6MinimalModelClass::step();
        rtsiSetSimTimeStep(&(&LCITopology6x6Minimal_M)->solverInfo,
                           MAJOR_TIME_STEP);
      }
    }
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep((&LCITopology6x6Minimal_M))) {
    rt_ertODEUpdateContinuousStates(&(&LCITopology6x6Minimal_M)->solverInfo);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick0 and the high bits
     * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
     */
    if (!(++(&LCITopology6x6Minimal_M)->Timing.clockTick0)) {
      ++(&LCITopology6x6Minimal_M)->Timing.clockTickH0;
    }

    (&LCITopology6x6Minimal_M)->Timing.t[0] = rtsiGetSolverStopTime
      (&(&LCITopology6x6Minimal_M)->solverInfo);

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
      (&LCITopology6x6Minimal_M)->Timing.clockTick1++;
      if (!(&LCITopology6x6Minimal_M)->Timing.clockTick1) {
        (&LCITopology6x6Minimal_M)->Timing.clockTickH1++;
      }
    }
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void LCITopology6x6MinimalModelClass::LCITopology6x6Minimal_derivatives()
{
  XDot_LCITopology6x6Minimal_T *_rtXdot;
  boolean_T lsat;
  boolean_T usat;
  _rtXdot = ((XDot_LCITopology6x6Minimal_T *) (&LCITopology6x6Minimal_M)->derivs);

  /* Derivatives for Integrator: '<S42>/Integrator' */
  lsat = (LCITopology6x6Minimal_X.Integrator_CSTATE <=
          LCITopology6x6Minimal_P.Integrator_LowerSat_n);
  usat = (LCITopology6x6Minimal_X.Integrator_CSTATE >=
          LCITopology6x6Minimal_P.Integrator_UpperSat_l);
  if (((!lsat) && (!usat)) || (lsat && (LCITopology6x6Minimal_B.Saturation2 >
        0.0)) || (usat && (LCITopology6x6Minimal_B.Saturation2 < 0.0))) {
    _rtXdot->Integrator_CSTATE = LCITopology6x6Minimal_B.Saturation2;
  } else {
    /* in saturation */
    _rtXdot->Integrator_CSTATE = 0.0;
  }

  /* End of Derivatives for Integrator: '<S42>/Integrator' */

  /* Derivatives for Enabled SubSystem: '<S42>/Automatic Gain Control' */
  if (LCITopology6x6Minimal_DW.AutomaticGainControl_MODE) {
    /* Derivatives for VariableTransportDelay: '<S52>/Variable Transport Delay' */
    {
      real_T instantDelay;
      instantDelay = LCITopology6x6Minimal_B.period_i;
      if (instantDelay >
          (LCITopology6x6Minimal_P.VariableTransportDelay_MaxDelay)) {
        instantDelay = (LCITopology6x6Minimal_P.VariableTransportDelay_MaxDelay);
      }

      if (instantDelay < 0.0) {
        ((XDot_LCITopology6x6Minimal_T *) (&LCITopology6x6Minimal_M)->derivs)
          ->VariableTransportDelay_CSTATE_c = 0;
      } else {
        ((XDot_LCITopology6x6Minimal_T *) (&LCITopology6x6Minimal_M)->derivs)
          ->VariableTransportDelay_CSTATE_c = 1.0/instantDelay;
      }
    }

    /* Derivatives for Integrator: '<S52>/integrator' */
    _rtXdot->integrator_CSTATE_g = LCITopology6x6Minimal_B.Switch_dq[0];

    /* Derivatives for VariableTransportDelay: '<S53>/Variable Transport Delay' */
    {
      real_T instantDelay;
      instantDelay = LCITopology6x6Minimal_B.period_a;
      if (instantDelay >
          (LCITopology6x6Minimal_P.VariableTransportDelay_MaxDel_i)) {
        instantDelay = (LCITopology6x6Minimal_P.VariableTransportDelay_MaxDel_i);
      }

      if (instantDelay < 0.0) {
        ((XDot_LCITopology6x6Minimal_T *) (&LCITopology6x6Minimal_M)->derivs)
          ->VariableTransportDelay_CSTATE_l = 0;
      } else {
        ((XDot_LCITopology6x6Minimal_T *) (&LCITopology6x6Minimal_M)->derivs)
          ->VariableTransportDelay_CSTATE_l = 1.0/instantDelay;
      }
    }

    /* Derivatives for Integrator: '<S53>/integrator' */
    _rtXdot->integrator_CSTATE_b = LCITopology6x6Minimal_B.Switch_dq[1];
  } else {
    {
      real_T *dx;
      int_T i;
      dx = &(((XDot_LCITopology6x6Minimal_T *) (&LCITopology6x6Minimal_M)
              ->derivs)->VariableTransportDelay_CSTATE_c);
      for (i=0; i < 4; i++) {
        dx[i] = 0.0;
      }
    }
  }

  /* End of Derivatives for SubSystem: '<S42>/Automatic Gain Control' */

  /* Derivatives for Integrator: '<S44>/Integrator' */
  lsat = (LCITopology6x6Minimal_X.Integrator_CSTATE_i <=
          LCITopology6x6Minimal_P.Integrator_LowerSat_c);
  usat = (LCITopology6x6Minimal_X.Integrator_CSTATE_i >=
          LCITopology6x6Minimal_P.Integrator_UpperSat_j);
  if (((!lsat) && (!usat)) || (lsat && (LCITopology6x6Minimal_B.Kp5 > 0.0)) ||
      (usat && (LCITopology6x6Minimal_B.Kp5 < 0.0))) {
    _rtXdot->Integrator_CSTATE_i = LCITopology6x6Minimal_B.Kp5;
  } else {
    /* in saturation */
    _rtXdot->Integrator_CSTATE_i = 0.0;
  }

  /* End of Derivatives for Integrator: '<S44>/Integrator' */

  /* Derivatives for VariableTransportDelay: '<S64>/Variable Transport Delay' */
  {
    real_T instantDelay;
    instantDelay = LCITopology6x6Minimal_B.period;
    if (instantDelay > (LCITopology6x6Minimal_P.VariableTransportDelay_MaxDel_l))
    {
      instantDelay = (LCITopology6x6Minimal_P.VariableTransportDelay_MaxDel_l);
    }

    if (instantDelay < 0.0) {
      ((XDot_LCITopology6x6Minimal_T *) (&LCITopology6x6Minimal_M)->derivs)
        ->VariableTransportDelay_CSTATE = 0;
    } else {
      ((XDot_LCITopology6x6Minimal_T *) (&LCITopology6x6Minimal_M)->derivs)
        ->VariableTransportDelay_CSTATE = 1.0/instantDelay;
    }
  }

  /* Derivatives for Integrator: '<S64>/integrator' */
  _rtXdot->integrator_CSTATE = LCITopology6x6Minimal_B.Switch_a[1];

  /* Derivatives for TransferFcn: '<S44>/Transfer Fcn' */
  _rtXdot->TransferFcn_CSTATE = 0.0;
  _rtXdot->TransferFcn_CSTATE += LCITopology6x6Minimal_P.TransferFcn_A *
    LCITopology6x6Minimal_X.TransferFcn_CSTATE;
  _rtXdot->TransferFcn_CSTATE += LCITopology6x6Minimal_B.Kp6;

  /* Derivatives for Integrator: '<S60>/Integrator_x1' */
  _rtXdot->Integrator_x1_CSTATE = LCITopology6x6Minimal_B.x1_i;

  /* Derivatives for Integrator: '<S60>/Integrator_x2' */
  _rtXdot->Integrator_x2_CSTATE = LCITopology6x6Minimal_B.x2_m;
}

/* Model initialize function */
void LCITopology6x6MinimalModelClass::initialize()
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* non-finite (run-time) assignments */
  LCITopology6x6Minimal_P.Saturation_UpperSat = rtInf;
  LCITopology6x6Minimal_P.Integrator_UpperSat = rtInf;
  LCITopology6x6Minimal_P.Integrator_LowerSat = rtMinusInf;
  LCITopology6x6Minimal_P.Saturation_UpperSat_e = rtInf;
  LCITopology6x6Minimal_P.Saturation_LowerSat_f = rtMinusInf;
  LCITopology6x6Minimal_P.Integrator_UpperSat_l = rtInf;
  LCITopology6x6Minimal_P.Integrator_LowerSat_n = rtMinusInf;
  LCITopology6x6Minimal_P.Integrator_UpperSat_j = rtInf;
  LCITopology6x6Minimal_P.Saturation2_UpperSat = rtInf;

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&(&LCITopology6x6Minimal_M)->solverInfo,
                          &(&LCITopology6x6Minimal_M)->Timing.simTimeStep);
    rtsiSetTPtr(&(&LCITopology6x6Minimal_M)->solverInfo, &rtmGetTPtr
                ((&LCITopology6x6Minimal_M)));
    rtsiSetStepSizePtr(&(&LCITopology6x6Minimal_M)->solverInfo,
                       &(&LCITopology6x6Minimal_M)->Timing.stepSize0);
    rtsiSetdXPtr(&(&LCITopology6x6Minimal_M)->solverInfo,
                 &(&LCITopology6x6Minimal_M)->derivs);
    rtsiSetContStatesPtr(&(&LCITopology6x6Minimal_M)->solverInfo, (real_T **) &(
      &LCITopology6x6Minimal_M)->contStates);
    rtsiSetNumContStatesPtr(&(&LCITopology6x6Minimal_M)->solverInfo,
      &(&LCITopology6x6Minimal_M)->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&(&LCITopology6x6Minimal_M)->solverInfo,
      &(&LCITopology6x6Minimal_M)->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&(&LCITopology6x6Minimal_M)->solverInfo,
      &(&LCITopology6x6Minimal_M)->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&(&LCITopology6x6Minimal_M)->solverInfo,
      &(&LCITopology6x6Minimal_M)->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&(&LCITopology6x6Minimal_M)->solverInfo,
                          (&rtmGetErrorStatus((&LCITopology6x6Minimal_M))));
    rtsiSetRTModelPtr(&(&LCITopology6x6Minimal_M)->solverInfo,
                      (&LCITopology6x6Minimal_M));
  }

  rtsiSetSimTimeStep(&(&LCITopology6x6Minimal_M)->solverInfo, MAJOR_TIME_STEP);
  (&LCITopology6x6Minimal_M)->intgData.f[0] = (&LCITopology6x6Minimal_M)->odeF[0];
  (&LCITopology6x6Minimal_M)->contStates = ((X_LCITopology6x6Minimal_T *)
    &LCITopology6x6Minimal_X);
  rtsiSetSolverData(&(&LCITopology6x6Minimal_M)->solverInfo, static_cast<void *>
                    (&(&LCITopology6x6Minimal_M)->intgData));
  rtsiSetSolverName(&(&LCITopology6x6Minimal_M)->solverInfo,"ode1");
  rtmSetTPtr((&LCITopology6x6Minimal_M), &(&LCITopology6x6Minimal_M)
             ->Timing.tArray[0]);
  (&LCITopology6x6Minimal_M)->Timing.stepSize0 = 1.25E-5;
  rtmSetFirstInitCond((&LCITopology6x6Minimal_M), 1);

  /* Start for Constant: '<S2>/Constant4' */
  LCITopology6x6Minimal_B.Constant4 = LCITopology6x6Minimal_P.Constant4_Value_d;

  /* Start for S-Function (sfun_spssw_discc): '<S164>/State-Space' incorporates:
   *  Constant: '<S166>/SwitchCurrents'
   *  Inport: '<Root>/pulses_in_grid'
   *  Inport: '<Root>/pulses_in_motor'
   */

  /* S-Function block: <S164>/State-Space */
  {
    LCITopology6x6Minimal_DW.StateSpace_PWORK.AS = (real_T*)calloc(21 * 21,
      sizeof(real_T));
    LCITopology6x6Minimal_DW.StateSpace_PWORK.BS = (real_T*)calloc(21 * 19,
      sizeof(real_T));
    LCITopology6x6Minimal_DW.StateSpace_PWORK.CS = (real_T*)calloc(37 * 21,
      sizeof(real_T));
    LCITopology6x6Minimal_DW.StateSpace_PWORK.DS = (real_T*)calloc(37 * 19,
      sizeof(real_T));
    LCITopology6x6Minimal_DW.StateSpace_PWORK.DX_COL = (real_T*)calloc(37,
      sizeof(real_T));
    LCITopology6x6Minimal_DW.StateSpace_PWORK.TMP2 = (real_T*)calloc(19, sizeof
      (real_T));
    LCITopology6x6Minimal_DW.StateSpace_PWORK.BD_COL = (real_T*)calloc(21,
      sizeof(real_T));
    LCITopology6x6Minimal_DW.StateSpace_PWORK.TMP1 = (real_T*)calloc(21, sizeof
      (real_T));
    LCITopology6x6Minimal_DW.StateSpace_PWORK.XTMP = (real_T*)calloc(21, sizeof
      (real_T));
    LCITopology6x6Minimal_DW.StateSpace_PWORK.SWITCH_STATUS = (int_T*)calloc(14,
      sizeof(int_T));
    LCITopology6x6Minimal_DW.StateSpace_PWORK.SW_CHG = (int_T*)calloc(14, sizeof
      (int_T));
    LCITopology6x6Minimal_DW.StateSpace_PWORK.G_STATE = (int_T*)calloc(14,
      sizeof(int_T));
    LCITopology6x6Minimal_DW.StateSpace_PWORK.Y_SWITCH = (real_T*)calloc(14,
      sizeof(real_T));
    LCITopology6x6Minimal_DW.StateSpace_PWORK.SWITCH_TYPES = (int_T*)calloc(14,
      sizeof(int_T));
    LCITopology6x6Minimal_DW.StateSpace_PWORK.IDX_OUT_SW = (int_T*)calloc(14,
      sizeof(int_T));
    LCITopology6x6Minimal_DW.StateSpace_PWORK.SWITCH_STATUS_INIT = (int_T*)
      calloc(14, sizeof(int_T));
    LCITopology6x6Minimal_DW.StateSpace_PWORK.USWLAST = (real_T*)calloc(14,
      sizeof(real_T));
  }

  /* Start for InitialCondition: '<S42>/Initial' */
  LCITopology6x6Minimal_B.Initial = LCITopology6x6Minimal_P.Initial_Value;

  /* Start for InitialCondition: '<S42>/Initial' */
  LCITopology6x6Minimal_DW.Initial_FirstOutputTime = (rtMinusInf);

  /* Start for VariableTransportDelay: '<S64>/Variable Transport Delay' */
  {
    real_T *pBuffer =
      &LCITopology6x6Minimal_DW.VariableTransportDelay_RWORK.TUbufferArea[0];
    int_T j;
    LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK.Tail = 0;
    LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK.Head = 0;
    LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK.Last = 0;
    LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK.CircularBufSize = 8192;
    for (j=0; j < 8192; j++) {
      pBuffer[j] = LCITopology6x6Minimal_P.VariableTransportDelay_InitOu_b;
      pBuffer[8192 + j] = (&LCITopology6x6Minimal_M)->Timing.t[0];
    }

    pBuffer[2*8192] = 0.0;
    LCITopology6x6Minimal_DW.VariableTransportDelay_PWORK.TUbufferPtrs[0] =
      (void *) &pBuffer[0];
    LCITopology6x6Minimal_DW.VariableTransportDelay_PWORK.TUbufferPtrs[1] =
      (void *) &pBuffer[8192];
    LCITopology6x6Minimal_DW.VariableTransportDelay_PWORK.TUbufferPtrs[2] =
      (void *) &pBuffer[2*8192];
  }

  LCITopology6x6Minimal_PrevZCX.Integrator_Reset_ZCE = UNINITIALIZED_ZCSIG;

  {
    int32_T i;

    /* InitializeConditions for UnitDelay: '<S86>/fluxes' */
    for (i = 0; i < 5; i++) {
      LCITopology6x6Minimal_DW.fluxes_DSTATE[i] =
        LCITopology6x6Minimal_P.fluxes_InitialCondition[i];
    }

    /* End of InitializeConditions for UnitDelay: '<S86>/fluxes' */

    /* InitializeConditions for DiscreteIntegrator: '<S73>/Rotor angle dthetae' */
    LCITopology6x6Minimal_DW.Rotorangledthetae_DSTATE =
      LCITopology6x6Minimal_P.Rotorangledthetae_IC;

    /* InitializeConditions for DiscreteIntegrator: '<S36>/Integrator' */
    LCITopology6x6Minimal_DW.Integrator_DSTATE =
      LCITopology6x6Minimal_B.Constant4;
    if (LCITopology6x6Minimal_DW.Integrator_DSTATE >=
        LCITopology6x6Minimal_P.Integrator_UpperSat) {
      LCITopology6x6Minimal_DW.Integrator_DSTATE =
        LCITopology6x6Minimal_P.Integrator_UpperSat;
    } else {
      if (LCITopology6x6Minimal_DW.Integrator_DSTATE <=
          LCITopology6x6Minimal_P.Integrator_LowerSat) {
        LCITopology6x6Minimal_DW.Integrator_DSTATE =
          LCITopology6x6Minimal_P.Integrator_LowerSat;
      }
    }

    /* End of InitializeConditions for DiscreteIntegrator: '<S36>/Integrator' */

    /* InitializeConditions for S-Function (sfun_spssw_discc): '<S164>/State-Space' incorporates:
     *  Constant: '<S166>/SwitchCurrents'
     *  Inport: '<Root>/pulses_in_grid'
     *  Inport: '<Root>/pulses_in_motor'
     */
    {
      int32_T i, j;
      real_T *As = (real_T*)LCITopology6x6Minimal_DW.StateSpace_PWORK.AS;
      real_T *Bs = (real_T*)LCITopology6x6Minimal_DW.StateSpace_PWORK.BS;
      real_T *Cs = (real_T*)LCITopology6x6Minimal_DW.StateSpace_PWORK.CS;
      real_T *Ds = (real_T*)LCITopology6x6Minimal_DW.StateSpace_PWORK.DS;
      real_T *X0 = (real_T*)&LCITopology6x6Minimal_DW.StateSpace_DSTATE[0];
      for (i = 0; i < 21; i++) {
        X0[i] = (LCITopology6x6Minimal_P.StateSpace_X0_param[i]);
      }

      /* Copy and transpose A and B */
      for (i=0; i<21; i++) {
        for (j=0; j<21; j++)
          As[i*21 + j] = (LCITopology6x6Minimal_P.StateSpace_AS_param[i + j*21]);
        for (j=0; j<19; j++)
          Bs[i*19 + j] = (LCITopology6x6Minimal_P.StateSpace_BS_param[i + j*21]);
      }

      /* Copy and transpose C */
      for (i=0; i<37; i++) {
        for (j=0; j<21; j++)
          Cs[i*21 + j] = (LCITopology6x6Minimal_P.StateSpace_CS_param[i + j*37]);
      }

      /* Copy and transpose D */
      for (i=0; i<37; i++) {
        for (j=0; j<19; j++)
          Ds[i*19 + j] = (LCITopology6x6Minimal_P.StateSpace_DS_param[i + j*37]);
      }

      {
        /* Switches work vectors */
        int_T *switch_status = (int_T*)
          LCITopology6x6Minimal_DW.StateSpace_PWORK.SWITCH_STATUS;
        int_T *gState = (int_T*)
          LCITopology6x6Minimal_DW.StateSpace_PWORK.G_STATE;
        real_T *yswitch = (real_T*)
          LCITopology6x6Minimal_DW.StateSpace_PWORK.Y_SWITCH;
        int_T *switchTypes = (int_T*)
          LCITopology6x6Minimal_DW.StateSpace_PWORK.SWITCH_TYPES;
        int_T *idxOutSw = (int_T*)
          LCITopology6x6Minimal_DW.StateSpace_PWORK.IDX_OUT_SW;
        int_T *switch_status_init = (int_T*)
          LCITopology6x6Minimal_DW.StateSpace_PWORK.SWITCH_STATUS_INIT;

        /* Initialize work vectors */
        switch_status[0] = 0;
        switch_status_init[0] = 0;
        gState[0] = (int_T) 0.0;
        yswitch[0] = 1/0.001;
        switchTypes[0] = (int_T)1.0;
        idxOutSw[0] = ((int_T)0.0) - 1;
        switch_status[1] = 0;
        switch_status_init[1] = 0;
        gState[1] = (int_T) 0.0;
        yswitch[1] = 1/0.001;
        switchTypes[1] = (int_T)1.0;
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
        switch_status[12] = 0;
        switch_status_init[12] = 0;
        gState[12] = (int_T) 0.0;
        yswitch[12] = 1/1.0E-6;
        switchTypes[12] = (int_T)4.0;
        idxOutSw[12] = ((int_T)0.0) - 1;
        switch_status[13] = 0;
        switch_status_init[13] = 0;
        gState[13] = (int_T) 0.0;
        yswitch[13] = 1/1.0E-6;
        switchTypes[13] = (int_T)4.0;
        idxOutSw[13] = ((int_T)0.0) - 1;
      }
    }

    /* InitializeConditions for Integrator: '<S42>/Integrator' */
    if (rtmIsFirstInitCond((&LCITopology6x6Minimal_M))) {
      LCITopology6x6Minimal_X.Integrator_CSTATE = 0.0;
    }

    LCITopology6x6Minimal_DW.Integrator_IWORK = 1;

    /* End of InitializeConditions for Integrator: '<S42>/Integrator' */

    /* InitializeConditions for Memory: '<S42>/Memory' */
    LCITopology6x6Minimal_DW.Memory_PreviousInput =
      LCITopology6x6Minimal_P.Memory_InitialCondition_a;

    /* InitializeConditions for Integrator: '<S44>/Integrator' */
    LCITopology6x6Minimal_X.Integrator_CSTATE_i =
      LCITopology6x6Minimal_P.Continuous_Init;

    /* InitializeConditions for VariableTransportDelay: '<S64>/Variable Transport Delay' */
    LCITopology6x6Minimal_X.VariableTransportDelay_CSTATE = 0.0;

    /* InitializeConditions for Integrator: '<S64>/integrator' */
    LCITopology6x6Minimal_X.integrator_CSTATE =
      LCITopology6x6Minimal_P.integrator_IC_n;

    /* InitializeConditions for Memory: '<S64>/Memory' */
    LCITopology6x6Minimal_DW.Memory_PreviousInput_p =
      LCITopology6x6Minimal_P.Memory_InitialCondition_b;

    /* InitializeConditions for TransferFcn: '<S44>/Transfer Fcn' */
    LCITopology6x6Minimal_X.TransferFcn_CSTATE = 0.0;

    /* InitializeConditions for RateLimiter: '<S42>/Rate Limiter' */
    LCITopology6x6Minimal_DW.LastMajorTime = (rtInf);

    /* InitializeConditions for Integrator: '<S60>/Integrator_x1' */
    LCITopology6x6Minimal_X.Integrator_x1_CSTATE =
      LCITopology6x6Minimal_P.Integrator_x1_IC;

    /* InitializeConditions for Integrator: '<S60>/Integrator_x2' */
    LCITopology6x6Minimal_X.Integrator_x2_CSTATE =
      LCITopology6x6Minimal_P.Integrator_x2_IC;

    /* InitializeConditions for UnitDelay: '<S110>/dw_delay' */
    LCITopology6x6Minimal_DW.dw_delay_DSTATE =
      LCITopology6x6Minimal_P.dw_delay_InitialCondition;

    /* InitializeConditions for UnitDelay: '<S110>/dw_predict' */
    LCITopology6x6Minimal_DW.dw_predict_DSTATE =
      LCITopology6x6Minimal_P.dw_predict_InitialCondition;

    /* InitializeConditions for DiscreteIntegrator: '<S73>/theta' */
    LCITopology6x6Minimal_DW.theta_DSTATE = LCITopology6x6Minimal_P.theta_IC;

    /* InitializeConditions for UnitDelay: '<S86>/voltages' */
    for (i = 0; i < 5; i++) {
      LCITopology6x6Minimal_DW.voltages_DSTATE[i] =
        LCITopology6x6Minimal_P.voltages_InitialCondition;
    }

    /* End of InitializeConditions for UnitDelay: '<S86>/voltages' */

    /* InitializeConditions for DiscreteIntegrator: '<S109>/Rotor spee<d deviation (dw)' */
    LCITopology6x6Minimal_DW.Rotorspeeddeviationdw_IC_LOADIN = 1U;

    /* SystemInitialize for Enabled SubSystem: '<S75>/Saturation' */
    /* InitializeConditions for UnitDelay: '<S87>/Lmd_sat' */
    LCITopology6x6Minimal_DW.Lmd_sat_DSTATE =
      LCITopology6x6Minimal_P.Lmd_sat_InitialCondition;

    /* SystemInitialize for Enabled SubSystem: '<S85>/Lmq_sat' */
    /* InitializeConditions for UnitDelay: '<S88>/Lmq_sat' */
    LCITopology6x6Minimal_DW.Lmq_sat_DSTATE =
      LCITopology6x6Minimal_P.Lmq_sat_InitialCondition;

    /* SystemInitialize for Gain: '<S88>/Lmq' incorporates:
     *  Outport: '<S88>/Lmq_sat '
     */
    LCITopology6x6Minimal_B.Lmsatq = LCITopology6x6Minimal_P.Lmq_sat_Y0;

    /* SystemInitialize for Math: '<S92>/Math Function1' incorporates:
     *  Outport: '<S88>/Laq_'
     *
     * About '<S92>/Math Function1':
     *  Operator: reciprocal
     */
    LCITopology6x6Minimal_B.MathFunction1 = LCITopology6x6Minimal_P.Laq_Y0;

    /* End of SystemInitialize for SubSystem: '<S85>/Lmq_sat' */
    for (i = 0; i < 25; i++) {
      /* SystemInitialize for Product: '<S85>/inversion' incorporates:
       *  Outport: '<S85>/Linv'
       */
      LCITopology6x6Minimal_B.Linv_d[i] = LCITopology6x6Minimal_P.Linv_Y0;

      /* SystemInitialize for Product: '<S85>/Product1' incorporates:
       *  Outport: '<S85>/R*Linv'
       */
      LCITopology6x6Minimal_B.RLinv[i] = LCITopology6x6Minimal_P.RLinv_Y0;
    }

    /* SystemInitialize for Switch: '<S85>/Switch1' incorporates:
     *  Outport: '<S85>/Lmqd'
     */
    LCITopology6x6Minimal_B.Switch1_j = LCITopology6x6Minimal_P.Lmqd_Y0;

    /* SystemInitialize for Gain: '<S87>/Lmd' incorporates:
     *  Outport: '<S85>/Lmqd'
     */
    LCITopology6x6Minimal_B.Lmsatd = LCITopology6x6Minimal_P.Lmqd_Y0;

    /* SystemInitialize for Switch: '<S85>/Switch2' incorporates:
     *  Outport: '<S85>/Laqd'
     */
    LCITopology6x6Minimal_B.Switch2_a = LCITopology6x6Minimal_P.Laqd_Y0;

    /* SystemInitialize for Math: '<S90>/Math Function3' incorporates:
     *  Outport: '<S85>/Laqd'
     *
     * About '<S90>/Math Function3':
     *  Operator: reciprocal
     */
    LCITopology6x6Minimal_B.MathFunction3 = LCITopology6x6Minimal_P.Laqd_Y0;

    /* End of SystemInitialize for SubSystem: '<S75>/Saturation' */

    /* SystemInitialize for Enabled SubSystem: '<S104>/Subsystem1' */
    /* SystemInitialize for Fcn: '<S108>/Fcn' incorporates:
     *  Outport: '<S108>/alpha_beta'
     */
    LCITopology6x6Minimal_B.Fcn_m = LCITopology6x6Minimal_P.alpha_beta_Y0_l[0];

    /* SystemInitialize for Fcn: '<S108>/Fcn1' incorporates:
     *  Outport: '<S108>/alpha_beta'
     */
    LCITopology6x6Minimal_B.Fcn1 = LCITopology6x6Minimal_P.alpha_beta_Y0_l[1];

    /* End of SystemInitialize for SubSystem: '<S104>/Subsystem1' */

    /* SystemInitialize for Enabled SubSystem: '<S104>/Subsystem - pi//2 delay' */
    /* SystemInitialize for Fcn: '<S107>/Fcn' incorporates:
     *  Outport: '<S107>/alpha_beta'
     */
    LCITopology6x6Minimal_B.Fcn_p = LCITopology6x6Minimal_P.alpha_beta_Y0[0];

    /* SystemInitialize for Fcn: '<S107>/Fcn1' incorporates:
     *  Outport: '<S107>/alpha_beta'
     */
    LCITopology6x6Minimal_B.Fcn1_h = LCITopology6x6Minimal_P.alpha_beta_Y0[1];

    /* End of SystemInitialize for SubSystem: '<S104>/Subsystem - pi//2 delay' */

    /* SystemInitialize for Enabled SubSystem: '<S42>/Automatic Gain Control' */
    /* Start for VariableTransportDelay: '<S52>/Variable Transport Delay' */
    {
      real_T *pBuffer =
        &LCITopology6x6Minimal_DW.VariableTransportDelay_RWORK_b.TUbufferArea[0];
      int_T j;
      LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK_g.Tail = 0;
      LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK_g.Head = 0;
      LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK_g.Last = 0;
      LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK_g.CircularBufSize =
        8192;
      for (j=0; j < 8192; j++) {
        pBuffer[j] = LCITopology6x6Minimal_P.VariableTransportDelay_InitOutp;
        pBuffer[8192 + j] = (&LCITopology6x6Minimal_M)->Timing.t[0];
      }

      pBuffer[2*8192] = 0.0;
      LCITopology6x6Minimal_DW.VariableTransportDelay_PWORK_g.TUbufferPtrs[0] =
        (void *) &pBuffer[0];
      LCITopology6x6Minimal_DW.VariableTransportDelay_PWORK_g.TUbufferPtrs[1] =
        (void *) &pBuffer[8192];
      LCITopology6x6Minimal_DW.VariableTransportDelay_PWORK_g.TUbufferPtrs[2] =
        (void *) &pBuffer[2*8192];
    }

    /* Start for VariableTransportDelay: '<S53>/Variable Transport Delay' */
    {
      real_T *pBuffer =
        &LCITopology6x6Minimal_DW.VariableTransportDelay_RWORK_p.TUbufferArea[0];
      int_T j;
      LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK_a.Tail = 0;
      LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK_a.Head = 0;
      LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK_a.Last = 0;
      LCITopology6x6Minimal_DW.VariableTransportDelay_IWORK_a.CircularBufSize =
        8192;
      for (j=0; j < 8192; j++) {
        pBuffer[j] = LCITopology6x6Minimal_P.VariableTransportDelay_InitOu_g;
        pBuffer[8192 + j] = (&LCITopology6x6Minimal_M)->Timing.t[0];
      }

      pBuffer[2*8192] = 0.0;
      LCITopology6x6Minimal_DW.VariableTransportDelay_PWORK_j.TUbufferPtrs[0] =
        (void *) &pBuffer[0];
      LCITopology6x6Minimal_DW.VariableTransportDelay_PWORK_j.TUbufferPtrs[1] =
        (void *) &pBuffer[8192];
      LCITopology6x6Minimal_DW.VariableTransportDelay_PWORK_j.TUbufferPtrs[2] =
        (void *) &pBuffer[2*8192];
    }

    /* InitializeConditions for VariableTransportDelay: '<S52>/Variable Transport Delay' */
    LCITopology6x6Minimal_X.VariableTransportDelay_CSTATE_c = 0.0;

    /* InitializeConditions for Integrator: '<S52>/integrator' */
    LCITopology6x6Minimal_X.integrator_CSTATE_g =
      LCITopology6x6Minimal_P.integrator_IC;

    /* InitializeConditions for Memory: '<S52>/Memory' */
    LCITopology6x6Minimal_DW.Memory_PreviousInput_d =
      LCITopology6x6Minimal_P.Memory_InitialCondition;

    /* InitializeConditions for VariableTransportDelay: '<S53>/Variable Transport Delay' */
    LCITopology6x6Minimal_X.VariableTransportDelay_CSTATE_l = 0.0;

    /* InitializeConditions for Integrator: '<S53>/integrator' */
    LCITopology6x6Minimal_X.integrator_CSTATE_b =
      LCITopology6x6Minimal_P.integrator_IC_m;

    /* InitializeConditions for Memory: '<S53>/Memory' */
    LCITopology6x6Minimal_DW.Memory_PreviousInput_h =
      LCITopology6x6Minimal_P.Memory_InitialCondition_i;

    /* SystemInitialize for Enabled SubSystem: '<S54>/Subsystem - pi//2 delay' */
    LCITopol_Subsystempi2delay_Init(&LCITopology6x6Minimal_B.Subsystempi2delay_k,
      &LCITopology6x6Minimal_P.Subsystempi2delay_k);

    /* End of SystemInitialize for SubSystem: '<S54>/Subsystem - pi//2 delay' */

    /* SystemInitialize for Enabled SubSystem: '<S54>/Subsystem1' */
    LCITopology6x6M_Subsystem1_Init(&LCITopology6x6Minimal_B.Subsystem1_ka,
      &LCITopology6x6Minimal_P.Subsystem1_ka);

    /* End of SystemInitialize for SubSystem: '<S54>/Subsystem1' */

    /* SystemInitialize for Math: '<S43>/Math Function' incorporates:
     *  Outport: '<S43>/Gain'
     *
     * About '<S43>/Math Function':
     *  Operator: reciprocal
     */
    LCITopology6x6Minimal_B.MathFunction_c = LCITopology6x6Minimal_P.Gain_Y0;

    /* End of SystemInitialize for SubSystem: '<S42>/Automatic Gain Control' */

    /* SystemInitialize for Enabled SubSystem: '<S65>/Subsystem - pi//2 delay' */
    LCITopol_Subsystempi2delay_Init(&LCITopology6x6Minimal_B.Subsystempi2delay,
      &LCITopology6x6Minimal_P.Subsystempi2delay);

    /* End of SystemInitialize for SubSystem: '<S65>/Subsystem - pi//2 delay' */

    /* SystemInitialize for Enabled SubSystem: '<S65>/Subsystem1' */
    LCITopology6x6M_Subsystem1_Init(&LCITopology6x6Minimal_B.Subsystem1,
      &LCITopology6x6Minimal_P.Subsystem1);

    /* End of SystemInitialize for SubSystem: '<S65>/Subsystem1' */

    /* SystemInitialize for Enabled SubSystem: '<S96>/Subsystem1' */
    /* SystemInitialize for Fcn: '<S101>/Fcn' incorporates:
     *  Outport: '<S101>/dq'
     */
    LCITopology6x6Minimal_B.Fcn_b = LCITopology6x6Minimal_P.dq_Y0_m[0];

    /* SystemInitialize for Fcn: '<S101>/Fcn1' incorporates:
     *  Outport: '<S101>/dq'
     */
    LCITopology6x6Minimal_B.Fcn1_j = LCITopology6x6Minimal_P.dq_Y0_m[1];

    /* End of SystemInitialize for SubSystem: '<S96>/Subsystem1' */

    /* SystemInitialize for Enabled SubSystem: '<S96>/Subsystem - pi//2 delay' */
    /* SystemInitialize for Fcn: '<S100>/Fcn' incorporates:
     *  Outport: '<S100>/dq'
     */
    LCITopology6x6Minimal_B.Fcn_bn = LCITopology6x6Minimal_P.dq_Y0[0];

    /* SystemInitialize for Fcn: '<S100>/Fcn1' incorporates:
     *  Outport: '<S100>/dq'
     */
    LCITopology6x6Minimal_B.Fcn1_p = LCITopology6x6Minimal_P.dq_Y0[1];

    /* End of SystemInitialize for SubSystem: '<S96>/Subsystem - pi//2 delay' */

    /* set "at time zero" to false */
    if (rtmIsFirstInitCond((&LCITopology6x6Minimal_M))) {
      rtmSetFirstInitCond((&LCITopology6x6Minimal_M), 0);
    }
  }
}

/* Model terminate function */
void LCITopology6x6MinimalModelClass::terminate()
{
  /* Terminate for S-Function (sfun_spssw_discc): '<S164>/State-Space' incorporates:
   *  Constant: '<S166>/SwitchCurrents'
   *  Inport: '<Root>/pulses_in_grid'
   *  Inport: '<Root>/pulses_in_motor'
   */

  /* S-Function block: <S164>/State-Space */
  {
    /* Free memory */
    free(LCITopology6x6Minimal_DW.StateSpace_PWORK.AS);
    free(LCITopology6x6Minimal_DW.StateSpace_PWORK.BS);
    free(LCITopology6x6Minimal_DW.StateSpace_PWORK.CS);
    free(LCITopology6x6Minimal_DW.StateSpace_PWORK.DS);
    free(LCITopology6x6Minimal_DW.StateSpace_PWORK.DX_COL);
    free(LCITopology6x6Minimal_DW.StateSpace_PWORK.TMP2);
    free(LCITopology6x6Minimal_DW.StateSpace_PWORK.BD_COL);
    free(LCITopology6x6Minimal_DW.StateSpace_PWORK.TMP1);
    free(LCITopology6x6Minimal_DW.StateSpace_PWORK.XTMP);

    /*
     * Circuit has switches*/
    free(LCITopology6x6Minimal_DW.StateSpace_PWORK.G_STATE);
    free(LCITopology6x6Minimal_DW.StateSpace_PWORK.SWITCH_STATUS);
    free(LCITopology6x6Minimal_DW.StateSpace_PWORK.SW_CHG);
    free(LCITopology6x6Minimal_DW.StateSpace_PWORK.SWITCH_STATUS_INIT);
  }
}

/* Constructor */
LCITopology6x6MinimalModelClass::LCITopology6x6MinimalModelClass() :
  LCITopology6x6Minimal_U(),
  LCITopology6x6Minimal_Y(),
  LCITopology6x6Minimal_B(),
  LCITopology6x6Minimal_DW(),
  LCITopology6x6Minimal_X(),
  LCITopology6x6Minimal_PrevZCX(),
  LCITopology6x6Minimal_M()
{
  /* Currently there is no constructor body generated.*/
}

/* Destructor */
LCITopology6x6MinimalModelClass::~LCITopology6x6MinimalModelClass()
{
  /* Currently there is no destructor body generated.*/
}

/* Real-Time Model get method */
RT_MODEL_LCITopology6x6Minima_T * LCITopology6x6MinimalModelClass::getRTM()
{
  return (&LCITopology6x6Minimal_M);
}
