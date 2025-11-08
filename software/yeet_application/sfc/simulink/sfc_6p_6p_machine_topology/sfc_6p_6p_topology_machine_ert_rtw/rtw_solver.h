/*
 * rtw_solver.h
 *
 * Minimal stub for Simulink Coder solver support
 */

#ifndef RTW_SOLVER_H
#define RTW_SOLVER_H

#include "rtwtypes.h"

/* Solver time step macros */
#define MAJOR_TIME_STEP 0
#define MINOR_TIME_STEP 1

/* Minimal type stubs */
typedef enum { SOLVER_MODE_AUTO, SOLVER_MODE_SINGLETASKING, SOLVER_MODE_MULTITASKING } SolverMode;

typedef enum { MINOR_TIME_STEP_ENUM = MINOR_TIME_STEP, MAJOR_TIME_STEP_ENUM = MAJOR_TIME_STEP } SimTimeStep;

/* Minimal RTWSolverInfo struct stub */
typedef struct {
    void* unused;  // Placeholder
} RTWSolverInfo;

#endif /* RTW_SOLVER_H */
