#include "yeet_testing/data_sampler.hpp"

#include <gtest/gtest.h>

#include <array>
#include <memory>

#include "yeet_testing/data_sampler_entry.hpp"

namespace {

// Mock post-processor for validation
class TestPostProcessor : public yeet::data_sample_post_processing_interface {
public:
    void dump(std::string test_suite,
        std::string test_name,
        std::map<yeet::data_sampler_entry*, std::vector<std::vector<float>>> const& data) override
    {
        last_test_suite = test_suite;
        last_test_name = test_name;
        last_data = data;
    }

    std::string last_test_suite;
    std::string last_test_name;
    std::map<yeet::data_sampler_entry*, std::vector<std::vector<float>>> last_data;
};

class DataSamplerTest : public ::testing::Test {
protected:
    static constexpr float kTolerance = 1e-6f;
    static constexpr std::size_t kNumSamples = 10;

    void SetUp() override
    {
        sampler = std::make_unique<yeet::data_sampler>();
        post_processor = std::make_unique<TestPostProcessor>();
    }

    std::unique_ptr<yeet::data_sampler> sampler{};
    std::unique_ptr<TestPostProcessor> post_processor{};
};

TEST_F(DataSamplerTest, RegisterAndSampleAllTypes)
{
    // Test data
    int scalar_int = 42;
    double scalar_double = 3.14159;
    std::vector<double> vector_data = {1.0, 2.0, 3.0};
    std::array<float, 2> array_data = {4.0f, 5.0f};
    double changing_value = 0.0;

    // Register different types of samplers
    sampler->register_entry(yeet::make_reference_sampler("scalar_int", scalar_int));
    sampler->register_entry(yeet::make_reference_sampler("scalar_double", scalar_double));
    sampler->register_entry(yeet::make_array_reference_sampler("vector_data", vector_data));
    sampler->register_entry(yeet::make_array_reference_sampler("array_data", array_data));
    sampler->register_entry(
        yeet::make_lambda_sampler<double>("lambda_data", [&changing_value]() -> double { return changing_value; }));

    // Allocate space for samples
    sampler->allocate(kNumSamples);

    // Take samples with changing value
    for (std::size_t i = 0; i < kNumSamples; ++i) {
        changing_value = static_cast<double>(i);
        sampler->sample();
    }

    // Dump data to post-processor
    sampler->dump("test_suite", "all_types_test", *post_processor);

    // Verify post-processor received correct data
    EXPECT_EQ(post_processor->last_test_suite, "test_suite");
    EXPECT_EQ(post_processor->last_test_name, "all_types_test");

    // Verify data was captured correctly
    const auto& captured_data = post_processor->last_data;
    EXPECT_EQ(captured_data.size(), 5);  // Five registered entries

    // Verify each entry's data
    for (const auto& [entry, data] : captured_data) {
        EXPECT_EQ(data.size(), kNumSamples);

        // Check specific values based on entry name
        std::string name = entry->get_name();
        if (name == "scalar_int") {
            for (const auto& sample : data) {
                EXPECT_EQ(sample.size(), 1);
                EXPECT_NEAR(sample[0], 42.0f, kTolerance);
            }
        }
        else if (name == "scalar_double") {
            for (const auto& sample : data) {
                EXPECT_EQ(sample.size(), 1);
                EXPECT_NEAR(sample[0], 3.14159f, kTolerance);
            }
        }
        else if (name == "vector_data") {
            for (const auto& sample : data) {
                EXPECT_EQ(sample.size(), 3);
                EXPECT_NEAR(sample[0], 1.0f, kTolerance);
                EXPECT_NEAR(sample[1], 2.0f, kTolerance);
                EXPECT_NEAR(sample[2], 3.0f, kTolerance);
            }
        }
        else if (name == "array_data") {
            for (const auto& sample : data) {
                EXPECT_EQ(sample.size(), 2);
                EXPECT_NEAR(sample[0], 4.0f, kTolerance);
                EXPECT_NEAR(sample[1], 5.0f, kTolerance);
            }
        }
        else if (name == "lambda_data") {
            for (size_t i = 0; i < data.size(); ++i) {
                EXPECT_EQ(data[i].size(), 1);
                EXPECT_NEAR(data[i][0], static_cast<float>(i), kTolerance);
            }
        }
    }
}

// TEST_F(DataSamplerTest, ExceedingSampleSize) {
//     double value = 1.0;
//     sampler->register_entry(yeet::make_sampler_entry("test", value));
//     sampler->allocate(1);  // Only one sample

//     testing::internal::CaptureStdout();

//     sampler->sample();  // First sample (ok)
//     sampler->sample();  // Second sample (should warn)

//     std::string output = testing::internal::GetCapturedStdout();
//     EXPECT_TRUE(output.find("Sample position exceeds data size") != std::string::npos);
// }

TEST_F(DataSamplerTest, EmptyDataSampler)
{
    sampler->dump("empty_suite", "empty_test", *post_processor);
    EXPECT_TRUE(post_processor->last_data.empty());
}

}  // namespace