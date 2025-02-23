#pragma once

#include <gtest/gtest.h>

#include "simulator.hpp"
#include "testing_info_interface.hpp"

namespace yeet::testing {

inline simulator make_gtest_simulator() {
    return simulator(std::make_shared<gtest_testing_info>(::testing::UnitTest::GetInstance()));
}

}  // namespace yeet::testing