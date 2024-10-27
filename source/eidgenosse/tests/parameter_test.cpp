#include "eidgenosse/parameter.hpp"

#include <gtest/gtest.h>

TEST(parameter, write_read) {
    parameter<float> float_parameter;

    float_parameter.write(10.0f);

    EXPECT_FLOAT_EQ(float_parameter.read(), 10.0f);
}

// TEST(parameter, write_read_low_level_access) {
//     parameter<float> float_parameter;

//     float_parameter.write(10.0f);

//     low_level_access low_level_access = float_parameter.get_low_level_access();

//     EXPECT_FLOAT_EQ(low_level_access.read<float>(), 10.0f);
// }
