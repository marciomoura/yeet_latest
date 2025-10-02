#ifndef YEET_SFC_EMBEDDED_TASKS_HPP
#define YEET_SFC_EMBEDDED_TASKS_HPP

#ifdef __cplusplus
extern "C" {
#endif

// Task declarations (called from STM32 ISRs)
void yeet_task_1(void);
void yeet_task_2(void);
void yeet_task_3(void);

// Initialization function (called from main)
void yeet_embedded_init(void);

#ifdef __cplusplus
}
#endif

#endif  // YEET_SFC_EMBEDDED_TASKS_HPP