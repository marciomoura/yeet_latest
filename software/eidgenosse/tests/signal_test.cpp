
#include "eidgenosse/signal.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <cstdlib>  // For std::rand
#include <limits>
#include <type_traits>

template <typename T>
class signal_input_test : public ::testing::Test {
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

    signal_input<T> signal_input;

    signal_output<T> signal_output;
};

// Instantiate the test case with the types
using fundamental_types = ::testing::Types<std::int8_t, std::int16_t, std::int32_t, std::int64_t, std::uint8_t,
                                           std::uint16_t, std::uint32_t, std::uint64_t, float, double>;

TYPED_TEST_SUITE(signal_input_test, fundamental_types);

TYPED_TEST(signal_input_test, connect_write_and_read) {
    // evaluate that signal input is empty
    EXPECT_TRUE(this->signal_input.empty());

    // connect signal output to signal input
    signals_connector::connect_output_to_input(this->signal_output, this->signal_input);

    // evaluate that signal input is not empty
    EXPECT_FALSE(this->signal_input.empty());

    // evaluate read/write functionality
    for (std::size_t i = 0; i < 100; ++i) {
        // Test with a random value
        TypeParam test_value = this->get_testing_value();
        this->signal_output.write(test_value);

        if (std::is_floating_point_v<TypeParam>) {
            EXPECT_FLOAT_EQ(this->signal_output.read(), test_value);
            EXPECT_FLOAT_EQ(this->signal_input.read(), test_value);
        } else {
            EXPECT_EQ(this->signal_output.read(), test_value);
            EXPECT_EQ(this->signal_input.read(), test_value);
        }
    }
}

TYPED_TEST(signal_input_test, assignment_operator) {
    const TypeParam test_value = this->get_testing_value();

    this->signal_output = test_value;

    if (std::is_floating_point_v<TypeParam>) {
        EXPECT_FLOAT_EQ(this->signal_output.read(), test_value);
    } else {
        EXPECT_EQ(this->signal_output.read(), test_value);
    }
}

TYPED_TEST(signal_input_test, conversion_operator) {
    const TypeParam test_value = this->get_testing_value();

    this->signal_output.write(test_value);

    if (std::is_floating_point_v<TypeParam>) {
        EXPECT_FLOAT_EQ(static_cast<TypeParam>(this->signal_output), test_value);
    } else {
        EXPECT_EQ(static_cast<TypeParam>(this->signal_output), test_value);
    }
}

TYPED_TEST(signal_input_test, llaccess) {
    // connect signal output to signal input
    signals_connector::connect_output_to_input(this->signal_output, this->signal_input);

    const low_level_access llaccess = this->signal_output.get_low_level_access();

    EXPECT_FALSE(llaccess.empty());

    for (std::size_t i = 0; i < 10; ++i) {
        const TypeParam value = this->get_testing_value();
        this->signal_output.write(value);

        const TypeParam value_llaccess = llaccess.read<TypeParam>();

        if (std::is_floating_point_v<TypeParam>) {
            EXPECT_FLOAT_EQ(value, value_llaccess);
        } else {
            EXPECT_EQ(value, value_llaccess);
        }
    }
}