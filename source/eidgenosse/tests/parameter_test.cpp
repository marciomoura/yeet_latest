#include "eidgenosse/parameter.hpp"

#include <gtest/gtest.h>

TEST(Parametertest, print) {
    testing::internal::CaptureStdout();
    print();
    std::string output = testing::internal::GetCapturedStdout();
    EXPECT_EQ(output, "Hello, World!\n");
}