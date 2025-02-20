#ifndef YEET_TESTING_TESTING_INFO_INTERFACE_HPP
#define YEET_TESTING_TESTING_INFO_INTERFACE_HPP

#include <string>

class testing_info_interface {
public:
    virtual std::string get_test_suite_name() const = 0;
    virtual std::string get_test_case_name() const = 0;
};

#endif  // YEET_TESTING_TESTING_INFO_INTERFACE_HPP