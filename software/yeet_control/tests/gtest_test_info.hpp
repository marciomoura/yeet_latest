#ifndef YEET_TESTING_GTEST_TEST_INFO_HPP
#define YEET_TESTING_GTEST_TEST_INFO_HPP

#include <gtest/gtest.h>

#include "yeet_testing/testing_info_interface.hpp"

namespace yeet::testing {

class gtest_testing_info : public testing_info_interface {
public:
    gtest_testing_info(const ::testing::UnitTest* unit_test) : _unit_test(unit_test) {}

    std::string get_test_suite_name() const override { return _unit_test->current_test_info()->test_suite_name(); }
    std::string get_test_case_name() const override { return _unit_test->current_test_info()->name(); }

private:
    const ::testing::UnitTest* _unit_test{};
};

}  // namespace yeet::testing

#endif