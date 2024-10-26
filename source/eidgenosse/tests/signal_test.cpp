
#include "eidgenosse/signal.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <cstdlib>  // For std::rand
#include <limits>
#include <type_traits>


using fundamental_types = ::testing::Types<std::int8_t, std::int16_t, std::int32_t, std::int64_t, std::uint8_t,
                                           std::uint16_t, std::uint32_t, std::uint64_t, float, double>;

template <typename T>
class signal_input_test : public ::testing::Test {
public:
    double get_rand_bounded_to_int8() {
        return static_cast<double>(std::rand() * std::numeric_limits<std::int8_t>::max()) / RAND_MAX;
    }

    T get_testing_value() {
        if constexpr (std::is_same_v<T, bool>) {
            return true;
        } else {
            return static_cast<T>(get_rand_bounded_to_int8());
        }
    }

    signal_input<T> signal_input;
    signal_output<T> signal_output;
};

// Instantiate the test case with the types
TYPED_TEST_SUITE(signal_input_test, fundamental_types);

TYPED_TEST(signal_input_test, connect_write_and_read) {
    signals_connector::connect_output_to_input(this->signal_output, this->signal_input);

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
