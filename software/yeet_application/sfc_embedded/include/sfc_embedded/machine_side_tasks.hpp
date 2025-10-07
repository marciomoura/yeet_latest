#ifndef SFC_EMBEDDED_MACHINE_SIDE_TASKS_HPP
#define SFC_EMBEDDED_MACHINE_SIDE_TASKS_HPP

#ifdef __cplusplus
extern "C" {
#endif

// Initialization function (called from main)
void yeet_embedded_init(void);

// Task declarations (called from STM32 ISRs)
void yeet_task_1(void);
void yeet_task_2(void);
void yeet_task_3(void);

#ifdef __cplusplus
}
#endif

#endif  // SFC_EMBEDDED_MACHINE_SIDE_TASKS_HPP