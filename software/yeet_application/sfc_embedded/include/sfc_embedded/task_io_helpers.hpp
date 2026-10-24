#pragma once

#include <cstdint>

#include "sfc_integration/sfc_io_types.hpp"
// Common I/O helper functions for embedded tasks
// Used by both grid_side_tasks.cpp and machine_side_tasks.cpp

namespace yeet::embedded::io {

inline bool is_valid_di_index(int idx) noexcept { return idx >= 0 && idx < TASK_IO_NUM_DIGITAL_INPUTS; }

inline bool is_valid_do_index(int idx) noexcept { return idx >= 0 && idx < TASK_IO_NUM_DIGITAL_OUTPUTS; }

inline bool is_valid_ao_index(int idx) noexcept { return idx >= 0 && idx < TASK_IO_NUM_ANALOG_OUTPUTS; }

inline bool read_di(const task_inputs_t* inputs, int idx) noexcept
{
    if (inputs == nullptr || !inputs->digital_inputs.valid) {
        return false;
    }
    if (!is_valid_di_index(idx)) {
        return false;
    }
    return inputs->digital_inputs.signals[idx];
}

inline bool read_di(
    const task_inputs_t* inputs, int idx, yeet::application::sfc::integration::io_polarity_t polarity) noexcept
{
    if (inputs == nullptr || !inputs->digital_inputs.valid) {
        return false;
    }
    if (!is_valid_di_index(idx)) {
        return false;
    }

    if (polarity == yeet::application::sfc::integration::io_polarity_t::ACTIVE_LOW) {
        return !inputs->digital_inputs.signals[idx];
    }
    else {
        return inputs->digital_inputs.signals[idx];
    }
}

inline void write_do(task_outputs_t* outputs, int idx, bool value) noexcept
{
    if (outputs == nullptr) {
        return;
    }
    if (!is_valid_do_index(idx)) {
        return;
    }
    outputs->digital_outputs.signals[idx] = value;
}

inline void write_ao(task_outputs_t* outputs, int idx, float value, float gain, float offset) noexcept
{
    if (outputs == nullptr) {
        return;
    }
    if (!is_valid_ao_index(idx)) {
        return;
    }
    outputs->analog_outputs.channels[idx] = value * gain + offset;
    outputs->analog_outputs.valid = true;
}

inline void clear_analog_outputs(task_outputs_t* outputs) noexcept
{
    if (outputs == nullptr) {
        return;
    }
    for (auto& c : outputs->analog_outputs.channels) {
        c = 0.0f;
    }
    outputs->analog_outputs.valid = false;
}

}  // namespace yeet::embedded::io
