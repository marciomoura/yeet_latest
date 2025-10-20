/**
 * @file host_timing.cpp
 * @brief Implementation of deterministic timing control for host platform
 */

#ifdef HOST_BUILD

#include "sfc_embedded/host_platform/host_timing.hpp"

#include <algorithm>
#include <limits>

namespace yeet::application::sfc::host {

host_timing_loop::host_timing_loop(float frequency_hz)
    : frequency_hz_(frequency_hz),
      cycle_period_ms_(1000.0f / frequency_hz),
      period_(static_cast<std::uint64_t>(1'000'000.0 / frequency_hz)),
      running_(false),
      cycle_number_(0),
      total_cycle_time_ms_(0.0f),
      max_cycle_time_ms_(0.0f),
      min_cycle_time_ms_(std::numeric_limits<float>::max()),
      overrun_count_(0)
{
}

host_timing_loop::~host_timing_loop() { stop(); }

void host_timing_loop::start()
{
    if (running_.load()) {
        return;  // Already running
    }

    // Reset state
    cycle_number_ = 0;
    reset_statistics();

    // Initialize timing
    start_time_ = clock::now();
    next_cycle_time_ = start_time_;

    running_.store(true);
}

void host_timing_loop::stop() { running_.store(false); }

bool host_timing_loop::is_running() const { return running_.load(); }

bool host_timing_loop::wait_for_next_cycle()
{
    if (!running_.load()) {
        return false;
    }

    // Calculate next cycle time
    next_cycle_time_ += period_;

    // Sleep until next cycle
    std::this_thread::sleep_until(next_cycle_time_);

    // Mark cycle start
    cycle_start_time_ = clock::now();
    cycle_number_++;

    return running_.load();
}

void host_timing_loop::mark_cycle_end()
{
    if (!running_.load()) {
        return;
    }

    // Measure actual cycle time
    auto cycle_end_time = clock::now();
    duration cycle_duration = cycle_end_time - cycle_start_time_;
    float cycle_time_ms = static_cast<float>(cycle_duration.count() * 1000.0);

    // Check for overrun
    if (cycle_time_ms > cycle_period_ms_) {
        overrun_count_++;
    }

    // Update statistics
    update_statistics(cycle_time_ms);
}

timing_statistics host_timing_loop::get_statistics() const
{
    timing_statistics stats;

    if (cycle_number_ > 0) {
        stats.average_cycle_time_ms = total_cycle_time_ms_ / cycle_number_;
    }
    else {
        stats.average_cycle_time_ms = 0.0f;
    }

    stats.max_cycle_time_ms = max_cycle_time_ms_;
    stats.min_cycle_time_ms = (min_cycle_time_ms_ == std::numeric_limits<float>::max()) ? 0.0f : min_cycle_time_ms_;
    stats.overrun_count = overrun_count_;
    stats.total_cycles = cycle_number_;

    return stats;
}

void host_timing_loop::reset_statistics()
{
    total_cycle_time_ms_ = 0.0f;
    max_cycle_time_ms_ = 0.0f;
    min_cycle_time_ms_ = std::numeric_limits<float>::max();
    overrun_count_ = 0;
}

std::uint32_t host_timing_loop::get_cycle_number() const { return cycle_number_; }

float host_timing_loop::get_elapsed_time_ms() const
{
    if (!running_.load() && cycle_number_ == 0) {
        return 0.0f;
    }

    auto now = clock::now();
    duration elapsed = now - start_time_;
    return static_cast<float>(elapsed.count() * 1000.0);
}

float host_timing_loop::get_cycle_period_ms() const { return cycle_period_ms_; }

void host_timing_loop::update_statistics(float cycle_time_ms)
{
    total_cycle_time_ms_ += cycle_time_ms;
    max_cycle_time_ms_ = std::max(max_cycle_time_ms_, cycle_time_ms);
    min_cycle_time_ms_ = std::min(min_cycle_time_ms_, cycle_time_ms);
}

}  // namespace yeet::application::sfc::host

#endif  // HOST_BUILD
