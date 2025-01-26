#include "yeet/parameter.hpp"

#include <gtest/gtest.h>

#include <cstdlib>  // For std::rand
#include <limits>
#include <type_traits>

// Define a test fixture template
template <typename T>
class parameter_test : public ::testing::Test {
public:
    double get_rand_bounded_to_int8() {
        return static_cast<double>(std::rand() * std::numeric_limits<std::int8_t>::max()) / RAND_MAX;
    }

    T get_testing_value() {
        if constexpr (std::is_same_v<T, bool>) {
            return (static_cast<std::uint32_t>(get_rand_bounded_to_int8()) % 2) ? true : false;
        } else {
            return static_cast<T>(get_rand_bounded_to_int8());
        }
    }

    parameter<T> param;
};

using fundamental_types = ::testing::Types<std::int8_t, std::int16_t, std::int32_t, std::int64_t, std::uint8_t,
                                           std::uint16_t, std::uint32_t, std::uint64_t, float, double>;

// Instantiate the test case with the types
TYPED_TEST_SUITE(parameter_test, fundamental_types);

TYPED_TEST(parameter_test, read_write) {
    TypeParam test_value = this->get_testing_value();
    this->param.write(test_value);

    if (std::is_floating_point_v<TypeParam>) {
        EXPECT_FLOAT_EQ(this->param.read(), test_value);
    } else {
        EXPECT_EQ(this->param.read(), test_value);
    }

    TypeParam new_test_value = this->get_testing_value();
    this->param.write(new_test_value);

    if (std::is_floating_point_v<TypeParam>) {
        EXPECT_FLOAT_EQ(this->param.read(), new_test_value);
    } else {
        EXPECT_EQ(this->param.read(), new_test_value);
    }
}

TYPED_TEST(parameter_test, assignment_operator) {
    TypeParam test_value = this->get_testing_value();
    this->param = test_value;

    if (std::is_floating_point_v<TypeParam>) {
        EXPECT_FLOAT_EQ(this->param.read(), test_value);
    } else {
        EXPECT_EQ(this->param.read(), test_value);
    }

    TypeParam new_test_value = this->get_testing_value();
    this->param = new_test_value;

    if (std::is_floating_point_v<TypeParam>) {
        EXPECT_FLOAT_EQ(this->param.read(), new_test_value);
    } else {
        EXPECT_EQ(this->param.read(), new_test_value);
    }
}

TYPED_TEST(parameter_test, conversion_operator) {
    TypeParam test_value = this->get_testing_value();
    this->param.write(test_value);
    TypeParam read_value = this->param;

    if (std::is_floating_point_v<TypeParam>) {
        EXPECT_FLOAT_EQ(read_value, test_value);
    } else {
        EXPECT_EQ(read_value, test_value);
    }
}

TYPED_TEST(parameter_test, llaccess) {
    TypeParam test_value = this->get_testing_value();
    this->param.write(test_value);

    low_level_access llaccess = this->param.get_low_level_access();

    if (std::is_floating_point_v<TypeParam>) {
        EXPECT_FLOAT_EQ(llaccess.read<TypeParam>(), test_value);
    } else {
        EXPECT_EQ(llaccess.read<TypeParam>(), test_value);
    }

    TypeParam new_test_value = this->get_testing_value();
    llaccess.write(new_test_value);

    if (std::is_floating_point_v<TypeParam>) {
        EXPECT_FLOAT_EQ(llaccess.read<TypeParam>(), new_test_value);
    } else {
        EXPECT_EQ(llaccess.read<TypeParam>(), new_test_value);
    }
}