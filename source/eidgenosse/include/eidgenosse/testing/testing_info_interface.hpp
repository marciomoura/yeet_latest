#ifndef EIDGENOSSE_TESTING_TESTING_INFO_INTERFACE_HPP
#define EIDGENOSSE_TESTING_TESTING_INFO_INTERFACE_HPP

#include <string>

class testing_info_interface {
public:
    virtual std::string get_test_suite_name() const = 0;
    virtual std::string get_test_case_name() const = 0;
};

#endif  // EIDGENOSSE_TESTING_TESTING_INFO_INTERFACE_HPP