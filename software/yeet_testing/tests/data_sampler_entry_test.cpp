#include "yeet_testing/data_sampler_entry.hpp"

#include <gtest/gtest.h>

#include <array>
#include <vector>

namespace {

class DataSamplerEntryTest : public ::testing::Test {
protected:
    static constexpr float kTolerance = 1e-6f;
};

// Default constructor tests
TEST_F(DataSamplerEntryTest, DefaultConstructorInitializesEmptyEntry)
{
    yeet::data_sampler_entry entry;

    EXPECT_TRUE(entry.get_name().empty());
    EXPECT_EQ(entry.get_dimension(), 1);
    EXPECT_TRUE(entry.sample().empty());
}

// Scalar function constructor tests
TEST_F(DataSamplerEntryTest, ScalarFunctionConstructorWorks)
{
    float value = 42.0f;
    auto entry = yeet::data_sampler_entry("test_scalar", [&value]() { return value; });

    EXPECT_EQ(entry.get_name(), "test_scalar");
    EXPECT_EQ(entry.get_dimension(), 1);

    auto samples = entry.sample();
    ASSERT_EQ(samples.size(), 1);
    EXPECT_NEAR(samples[0], value, kTolerance);
}

// Fundamental type constructor tests
TEST_F(DataSamplerEntryTest, FundamentalTypeConstructorWithInt)
{
    int value = 42;
    auto entry = yeet::data_sampler_entry("test_int", value);

    EXPECT_EQ(entry.get_name(), "test_int");
    EXPECT_EQ(entry.get_dimension(), 1);

    auto samples = entry.sample();
    ASSERT_EQ(samples.size(), 1);
    EXPECT_NEAR(samples[0], static_cast<float>(value), kTolerance);
}

TEST_F(DataSamplerEntryTest, FundamentalTypeConstructorWithDouble)
{
    double value = 3.14159;
    auto entry = yeet::data_sampler_entry("test_double", value);

    auto samples = entry.sample();
    ASSERT_EQ(samples.size(), 1);
    EXPECT_NEAR(samples[0], static_cast<float>(value), kTolerance);
}

// Container type constructor tests
TEST_F(DataSamplerEntryTest, ContainerTypeConstructorWithVector)
{
    std::vector<double> values{1.0, 2.0, 3.0};
    auto entry = yeet::data_sampler_entry("test_vector", values);

    EXPECT_EQ(entry.get_name(), "test_vector");
    EXPECT_EQ(entry.get_dimension(), 3);

    auto samples = entry.sample();
    ASSERT_EQ(samples.size(), 3);
    for (size_t i = 0; i < values.size(); ++i) {
        EXPECT_NEAR(samples[i], static_cast<float>(values[i]), kTolerance);
    }
}

TEST_F(DataSamplerEntryTest, ContainerTypeConstructorWithArray)
{
    std::array<double, 2> values{4.0, 5.0};
    auto entry = yeet::data_sampler_entry("test_array", values);

    EXPECT_EQ(entry.get_name(), "test_array");
    EXPECT_EQ(entry.get_dimension(), 2);

    auto samples = entry.sample();
    ASSERT_EQ(samples.size(), 2);
    for (size_t i = 0; i < values.size(); ++i) {
        EXPECT_NEAR(samples[i], static_cast<float>(values[i]), kTolerance);
    }
}

// Edge cases and error conditions
TEST_F(DataSamplerEntryTest, EmptyContainerHandling)
{
    std::vector<double> empty_values;
    auto entry = yeet::data_sampler_entry("test_empty", empty_values);

    EXPECT_EQ(entry.get_dimension(), 0);
    EXPECT_TRUE(entry.sample().empty());
}

TEST_F(DataSamplerEntryTest, ReferenceCapture)
{
    double value = 1.0;
    auto entry = yeet::data_sampler_entry("test_ref", value);

    auto initial_samples = entry.sample();
    ASSERT_EQ(initial_samples.size(), 1);
    EXPECT_NEAR(initial_samples[0], 1.0f, kTolerance);

    value = 2.0;
    auto updated_samples = entry.sample();
    ASSERT_EQ(updated_samples.size(), 1);
    EXPECT_NEAR(updated_samples[0], 2.0f, kTolerance);
}

TEST_F(DataSamplerEntryTest, ReferenceIntCapture)
{
    int value = 1;
    auto entry = yeet::data_sampler_entry("test_int_ref", value);

    auto initial_samples = entry.sample();
    ASSERT_EQ(initial_samples.size(), 1);
    EXPECT_NEAR(initial_samples[0], 1.0f, kTolerance);

    value = 2;
    auto updated_samples = entry.sample();
    ASSERT_EQ(updated_samples.size(), 1);
    EXPECT_NEAR(updated_samples[0], 2.0f, kTolerance);
}

TEST_F(DataSamplerEntryTest, ContainerReferenceCapture)
{
    std::vector<double> values{1.0, 2.0};
    auto entry = yeet::data_sampler_entry("test_container_ref", values);

    auto initial_samples = entry.sample();
    ASSERT_EQ(initial_samples.size(), 2);

    values[0] = 3.0;
    auto updated_samples = entry.sample();
    EXPECT_NEAR(updated_samples[0], 3.0f, kTolerance);
}

TEST_F(DataSamplerEntryTest, IntContainerReferenceCapture)
{
    std::vector<int> values{1, 2};
    auto entry = yeet::data_sampler_entry("test_int_container_ref", values);

    auto initial_samples = entry.sample();
    ASSERT_EQ(initial_samples.size(), 2);

    values[0] = 3;
    auto updated_samples = entry.sample();
    EXPECT_NEAR(updated_samples[0], 3.0f, kTolerance);
}

}  // namespace