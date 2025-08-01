#pragma once

#include <gtest/gtest.h>

#include "simulator.hpp"
#include "testing_info_interface.hpp"

namespace yeet::testing {

inline simulator make_gtest_simulator(double data_sampler_sampling_rate = 100e-6) {
    auto s = simulator{std::make_shared<gtest_testing_info>(::testing::UnitTest::GetInstance())};

#ifndef YEET_ENABLE_DATA_LOGGING
    s.configure_sampler_sampling_rate(data_sampler_sampling_rate);  // Default 100us sampling rate
#else
    // When data logging is disabled, don't configure sampling to avoid overhead
    (void)data_sampler_sampling_rate;  // Suppress unused parameter warning
#endif

    return s;
}

}  // namespace yeet::testing