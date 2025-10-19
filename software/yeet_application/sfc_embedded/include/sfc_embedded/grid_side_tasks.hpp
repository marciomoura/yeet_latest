#ifndef SFC_EMBEDDED_GRID_SIDE_TASKS_HPP
#define SFC_EMBEDDED_GRID_SIDE_TASKS_HPP

#include "sfc_embedded/task_io.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initialization function (called from main)
void yeet_embedded_init(void);

// Real-time task declarations (called from STM32 ISRs)
void yeet_task_200us(const task_inputs_t* inputs, task_outputs_t* outputs);
void yeet_task_1000us(const task_inputs_t* inputs, task_outputs_t* outputs);
void yeet_task_10000us(const task_inputs_t* inputs, task_outputs_t* outputs);

// Non-real-time network communication task (called from main loop)
void yeet_network(const network_inputs_t* inputs, network_outputs_t* outputs);

#ifdef __cplusplus
}
#endif

#endif  // SFC_EMBEDDED_GRID_SIDE_TASKS_HPP