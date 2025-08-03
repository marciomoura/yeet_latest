#ifndef YEET_TESTING_TESTING_INFO_INTERFACE_HPP
#define YEET_TESTING_TESTING_INFO_INTERFACE_HPP

#include <gtest/gtest.h>

#include <string>

namespace yeet {

class testing_info_interface {
public:
    virtual std::string get_test_suite_name() const = 0;

    virtual std::string get_test_case_name() const = 0;

    virtual bool has_failed() const = 0;
};

class gtest_testing_info : public testing_info_interface {
public:
    gtest_testing_info(const ::testing::UnitTest* unit_test) : _unit_test(unit_test) {}

    std::string get_test_suite_name() const override { return _unit_test->current_test_info()->test_suite_name(); }

    std::string get_test_case_name() const override { return _unit_test->current_test_info()->name(); }

    bool has_failed() const override { return _unit_test->Failed(); }

private:
    const ::testing::UnitTest* _unit_test{};
};

}  // namespace yeet

#endif  // YEET_TESTING_TESTING_INFO_INTERFACE_HPP