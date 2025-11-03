#ifndef SFC_EMBEDDED_MACHINE_SIDE_TASKS_HPP
#define SFC_EMBEDDED_MACHINE_SIDE_TASKS_HPP

#include "sfc_embedded/task_io.h"

#ifdef __cplusplus
#include "sfc_embedded/machine_side_network_telemetry_builder.hpp"
#include "sfc_integration/sfc_machine_side_control.hpp"
#include "yeet/model_registry.hpp"
#include "yeet/network_handler.hpp"
#include "yeet/network_telemetry_manager.hpp"
#include "yeet/parameter_handler.hpp"

// Network Telemetry/Parameter Infrastructure
// Instantiated per control instance to support both embedded (static) and host (dynamic) usage
struct machine_side_network_infrastructure {
    yeet::model_registry registry;
    yeet::application::sfc::telemetry::machine_side_telemetry_builder telemetry_builder;
    yeet::telemetry::telemetry_manager telemetry_manager;
    yeet::telemetry::parameter_handler parameter_handler;
    yeet::telemetry::network_handler network_handler;

    // Two-phase initialization: first register datamodel, then construct handlers
    // This ensures parameter_handler has access to populated registry
    machine_side_network_infrastructure(yeet::application::sfc::integration::sfc_machine_side_control& control);
};

extern "C" {
#endif

// ==================== Machine-Side Specific Functions ====================
// These functions are prefixed with 'machine_' to allow co-existence with
// grid-side functions in a dual-controller build.

// Initialization function (called from main)
// Pass nullptr to use internal static instance (embedded), or pass a specific instance (host simulation)
void machine_embedded_init(
#ifdef __cplusplus
    yeet::application::sfc::integration::sfc_machine_side_control* control
#else
    void* control
#endif
);

// Real-time task declarations (called from STM32 ISRs)
void machine_task_200us(
#ifdef __cplusplus
    yeet::application::sfc::integration::sfc_machine_side_control* control,
#else
    void* control,
#endif
    const task_inputs_t* inputs,
    task_outputs_t* outputs);

void machine_task_1000us(
#ifdef __cplusplus
    yeet::application::sfc::integration::sfc_machine_side_control* control,
#else
    void* control,
#endif
    const task_inputs_t* inputs,
    task_outputs_t* outputs);

void machine_task_10000us(
#ifdef __cplusplus
    yeet::application::sfc::integration::sfc_machine_side_control* control,
#else
    void* control,
#endif
    const task_inputs_t* inputs,
    task_outputs_t* outputs);

// Non-real-time network communication task (called from main loop)
void machine_network(
#ifdef __cplusplus
    machine_side_network_infrastructure* infrastructure,
#else
    void* infrastructure,
#endif
    const network_inputs_t* inputs,
    network_outputs_t* outputs);

// ==================== Backward Compatibility Wrappers ====================
// For embedded builds with ONLY machine-side (no dual-controller), provide
// the original function names as wrappers to maintain compatibility with
// existing embedded main.c code. These wrappers use the static instance.
#if defined(YEET_EMBEDDED_MACHINE_SIDE) && !defined(YEET_BUILD_DUAL_CONTROLLER)
static inline void yeet_embedded_init(void)
{
#ifdef __cplusplus
    extern yeet::application::sfc::integration::sfc_machine_side_control& get_machine_side_control();
    machine_embedded_init(&get_machine_side_control());
#else
    machine_embedded_init(0);
#endif
}
static inline void yeet_task_200us(const task_inputs_t* inputs, task_outputs_t* outputs)
{
#ifdef __cplusplus
    extern yeet::application::sfc::integration::sfc_machine_side_control& get_machine_side_control();
    machine_task_200us(&get_machine_side_control(), inputs, outputs);
#else
    machine_task_200us(0, inputs, outputs);
#endif
}
static inline void yeet_task_1000us(const task_inputs_t* inputs, task_outputs_t* outputs)
{
#ifdef __cplusplus
    extern yeet::application::sfc::integration::sfc_machine_side_control& get_machine_side_control();
    machine_task_1000us(&get_machine_side_control(), inputs, outputs);
#else
    machine_task_1000us(0, inputs, outputs);
#endif
}
static inline void yeet_task_10000us(const task_inputs_t* inputs, task_outputs_t* outputs)
{
#ifdef __cplusplus
    extern yeet::application::sfc::integration::sfc_machine_side_control& get_machine_side_control();
    machine_task_10000us(&get_machine_side_control(), inputs, outputs);
#else
    machine_task_10000us(0, inputs, outputs);
#endif
}
static inline void yeet_network(const network_inputs_t* inputs, network_outputs_t* outputs)
{
#ifdef __cplusplus
    // For embedded, we need a static infrastructure instance
    extern machine_side_network_infrastructure& get_machine_side_network_infrastructure();
    machine_network(&get_machine_side_network_infrastructure(), inputs, outputs);
#else
    machine_network(0, inputs, outputs);
#endif
}
#endif

#ifdef __cplusplus
}
#endif

#endif  // SFC_EMBEDDED_MACHINE_SIDE_TASKS_HPP