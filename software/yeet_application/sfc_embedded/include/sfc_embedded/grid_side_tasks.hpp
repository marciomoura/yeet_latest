#ifndef SFC_EMBEDDED_GRID_SIDE_TASKS_HPP
#define SFC_EMBEDDED_GRID_SIDE_TASKS_HPP

#include "sfc_embedded/task_io.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initialization function (called from main)
void yeet_embedded_init(void);

// Task declarations (called from STM32 ISRs)
void yeet_task_200us(const task1_inputs_t* inputs, task1_outputs_t* outputs);
void yeet_task_1000us(void);
void yeet_task_10000us(void);

#ifdef __cplusplus
}
#endif

#endif  // SFC_EMBEDDED_GRID_SIDE_TASKS_HPP