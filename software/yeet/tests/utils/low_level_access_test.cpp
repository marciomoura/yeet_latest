
#include "yeet/utils/low_level_access.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <memory>

#include "yeet/parameter.hpp"
#include "yeet/utils/parameter_accessors.hpp"
#include "yeet_control/angle_wrapped.hpp"
#include "yeet_control/units.hpp"

TEST(low_level_access, empty)
{
    low_level_access llaccess;
    EXPECT_TRUE(llaccess.empty());
}

TEST(low_level_access, set_storage_type)
{
    std::int32_t data = 0;
    storage_type<std::int32_t> storage = storage_type<std::int32_t>(&data);
    std::int32_t* internal_storage = storage.get_internal_storage();

    low_level_access llaccess{storage};

    EXPECT_FALSE(llaccess.empty());

    EXPECT_EQ(llaccess.read<std::int32_t>(), *internal_storage);
    EXPECT_EQ(llaccess.read<std::int32_t>(), 0);

    llaccess.write(10);
    EXPECT_EQ(llaccess.read<std::int32_t>(), *internal_storage);
    EXPECT_EQ(llaccess.read<std::int32_t>(), 10);

    *internal_storage = 20;
    EXPECT_EQ(llaccess.read<std::int32_t>(), *internal_storage);
    EXPECT_EQ(llaccess.read<std::int32_t>(), 20);

    // test other data types
    EXPECT_EQ(llaccess.read<std::uint32_t>(), 20);
    EXPECT_EQ(llaccess.read<std::uint16_t>(), 20);
    EXPECT_EQ(llaccess.read<std::uint8_t>(), 20);
    EXPECT_EQ(llaccess.read<std::int32_t>(), 20);
    EXPECT_EQ(llaccess.read<std::int16_t>(), 20);
    EXPECT_EQ(llaccess.read<std::int8_t>(), 20);
}

TEST(low_level_access, function_pointer_constructor)
{
    float test_value = 42.0f;

    auto read_fn = [](void* ctx) -> float { return *static_cast<float*>(ctx); };
    auto write_fn = [](void* ctx, float value) { *static_cast<float*>(ctx) = value; };

    low_level_access llaccess(+read_fn, +write_fn, &test_value);

    EXPECT_FLOAT_EQ(llaccess.read<float>(), 42.0f);

    llaccess.write(100.0f);
    EXPECT_FLOAT_EQ(test_value, 100.0f);
    EXPECT_FLOAT_EQ(llaccess.read<float>(), 100.0f);
}

TEST(low_level_access, sampler_accessor)
{
    float test_value = 3.14f;

    auto read_fn = [](void* ctx) -> float { return *static_cast<float*>(ctx); };
    auto write_fn = [](void* ctx, float value) { *static_cast<float*>(ctx) = value; };

    low_level_access llaccess(+read_fn, +write_fn, &test_value);

    auto sampler = llaccess.get_sampler_accessor();
    EXPECT_FLOAT_EQ(sampler(), 3.14f);

    test_value = 2.71f;
    EXPECT_FLOAT_EQ(sampler(), 2.71f);
}

// Test parameter accessor helpers

TEST(parameter_accessors, make_units_accessor_frequency)
{
    yeet::parameter<yeet::frequency_t> freq_param;
    freq_param.write(yeet::frequency_t{50.0f});

    auto accessor = yeet::utils::make_units_accessor(freq_param);

    EXPECT_FLOAT_EQ(accessor.read<float>(), 50.0f);

    accessor.write(60.0f);
    EXPECT_FLOAT_EQ(freq_param.read(), 60.0f);
}

TEST(parameter_accessors, make_units_accessor_voltage)
{
    yeet::parameter<yeet::voltage_pu_t> voltage_param;
    voltage_param.write(yeet::voltage_pu_t{1.0f});

    auto accessor = yeet::utils::make_units_accessor(voltage_param);

    EXPECT_FLOAT_EQ(accessor.read<float>(), 1.0f);

    accessor.write(1.2f);
    EXPECT_FLOAT_EQ(voltage_param.read(), 1.2f);
}

TEST(parameter_accessors, make_units_accessor_current)
{
    yeet::parameter<yeet::current_pu_t> current_param;
    current_param.write(yeet::current_pu_t{0.8f});

    auto accessor = yeet::utils::make_units_accessor(current_param);

    EXPECT_FLOAT_EQ(accessor.read<float>(), 0.8f);

    accessor.write(1.5f);
    EXPECT_FLOAT_EQ(current_param.read(), 1.5f);
}

TEST(parameter_accessors, make_bool_accessor)
{
    yeet::parameter<bool> bool_param;
    bool_param.write(false);

    auto accessor = yeet::utils::make_bool_accessor(bool_param);

    EXPECT_FLOAT_EQ(accessor.read<float>(), 0.0f);

    accessor.write(1.0f);
    EXPECT_TRUE(bool_param.read());
    EXPECT_FLOAT_EQ(accessor.read<float>(), 1.0f);

    accessor.write(0.0f);
    EXPECT_FALSE(bool_param.read());
    EXPECT_FLOAT_EQ(accessor.read<float>(), 0.0f);

    // Test that any positive value becomes true
    accessor.write(0.5f);
    EXPECT_TRUE(bool_param.read());

    accessor.write(100.0f);
    EXPECT_TRUE(bool_param.read());

    // Test that negative and zero become false
    accessor.write(-1.0f);
    EXPECT_FALSE(bool_param.read());
}

TEST(parameter_accessors, make_enum_accessor)
{
    enum class test_mode : int { mode_a = 0, mode_b = 1, mode_c = 2 };

    yeet::parameter<test_mode> enum_param;
    enum_param.write(test_mode::mode_a);

    auto accessor = yeet::utils::make_enum_accessor(enum_param);

    EXPECT_FLOAT_EQ(accessor.read<float>(), 0.0f);

    accessor.write(1.0f);
    EXPECT_EQ(enum_param.read(), test_mode::mode_b);
    EXPECT_FLOAT_EQ(accessor.read<float>(), 1.0f);

    accessor.write(2.0f);
    EXPECT_EQ(enum_param.read(), test_mode::mode_c);
    EXPECT_FLOAT_EQ(accessor.read<float>(), 2.0f);
}

TEST(parameter_accessors, make_angle_wrapped_accessor_radians)
{
    yeet::parameter<yeet::angle_wrapped> angle_param;
    angle_param.write(yeet::angle_wrapped::from_radians(yeet::angle_t{1.57f}));  // ~90 degrees

    auto accessor = yeet::utils::make_angle_wrapped_accessor(angle_param);

    EXPECT_NEAR(accessor.read<float>(), 1.57f, 0.01f);

    accessor.write(3.14f);  // ~180 degrees
    EXPECT_NEAR(angle_param.read().get_radians(), 3.14f, 0.01f);
}

TEST(parameter_accessors, make_degrees_angle_wrapped_accessor)
{
    yeet::parameter<yeet::angle_wrapped> angle_param;
    angle_param.write(yeet::angle_wrapped::from_degrees(90.0f));

    auto accessor = yeet::utils::make_degrees_angle_wrapped_accessor(angle_param);

    EXPECT_NEAR(accessor.read<float>(), 90.0f, 0.01f);

    accessor.write(180.0f);
    EXPECT_NEAR(angle_param.read().get_degrees(), 180.0f, 0.01f);

    accessor.write(45.0f);
    EXPECT_NEAR(angle_param.read().get_degrees(), 45.0f, 0.01f);
}

TEST(parameter_accessors, make_array_element_accessor)
{
    yeet::parameter<std::array<yeet::voltage_pu_t, 5>> array_param;
    std::array<yeet::voltage_pu_t, 5> initial_values = {yeet::voltage_pu_t{1.0f}, yeet::voltage_pu_t{2.0f},
        yeet::voltage_pu_t{3.0f}, yeet::voltage_pu_t{4.0f}, yeet::voltage_pu_t{5.0f}};
    array_param.write(initial_values);

    // Create accessors for each element
    auto accessor_0 = yeet::utils::make_array_element_accessor(array_param, 0);
    auto accessor_2 = yeet::utils::make_array_element_accessor(array_param, 2);
    auto accessor_4 = yeet::utils::make_array_element_accessor(array_param, 4);

    // Test reading
    EXPECT_FLOAT_EQ(accessor_0.read<float>(), 1.0f);
    EXPECT_FLOAT_EQ(accessor_2.read<float>(), 3.0f);
    EXPECT_FLOAT_EQ(accessor_4.read<float>(), 5.0f);

    // Test writing via accessor
    accessor_2.write(10.0f);
    EXPECT_FLOAT_EQ(array_param.read()[2], 10.0f);
    EXPECT_FLOAT_EQ(accessor_2.read<float>(), 10.0f);

    // Verify other elements unchanged
    EXPECT_FLOAT_EQ(accessor_0.read<float>(), 1.0f);
    EXPECT_FLOAT_EQ(accessor_4.read<float>(), 5.0f);

    // Test writing multiple elements
    accessor_0.write(100.0f);
    accessor_4.write(500.0f);

    EXPECT_FLOAT_EQ(array_param.read()[0], 100.0f);
    EXPECT_FLOAT_EQ(array_param.read()[2], 10.0f);
    EXPECT_FLOAT_EQ(array_param.read()[4], 500.0f);
}

TEST(parameter_accessors, make_array_element_accessor_multiple_arrays)
{
    // Test that multiple array parameters can coexist
    yeet::parameter<std::array<yeet::current_pu_t, 3>> current_array;
    yeet::parameter<std::array<yeet::voltage_pu_t, 3>> voltage_array;

    current_array.write({yeet::current_pu_t{1.0f}, yeet::current_pu_t{2.0f}, yeet::current_pu_t{3.0f}});
    voltage_array.write({yeet::voltage_pu_t{10.0f}, yeet::voltage_pu_t{20.0f}, yeet::voltage_pu_t{30.0f}});

    auto current_accessor_0 = yeet::utils::make_array_element_accessor(current_array, 0);
    auto voltage_accessor_0 = yeet::utils::make_array_element_accessor(voltage_array, 0);

    EXPECT_FLOAT_EQ(current_accessor_0.read<float>(), 1.0f);
    EXPECT_FLOAT_EQ(voltage_accessor_0.read<float>(), 10.0f);

    current_accessor_0.write(5.0f);
    voltage_accessor_0.write(50.0f);

    EXPECT_FLOAT_EQ(current_array.read()[0], 5.0f);
    EXPECT_FLOAT_EQ(voltage_array.read()[0], 50.0f);
}

// Test edge cases

TEST(parameter_accessors, bool_accessor_edge_cases)
{
    yeet::parameter<bool> bool_param;

    auto accessor = yeet::utils::make_bool_accessor(bool_param);

    // Test boundary values
    accessor.write(0.0f);
    EXPECT_FALSE(bool_param.read());

    accessor.write(0.001f);
    EXPECT_TRUE(bool_param.read());

    accessor.write(-0.001f);
    EXPECT_FALSE(bool_param.read());
}

TEST(parameter_accessors, enum_accessor_rounding)
{
    enum class test_mode : int { mode_0 = 0, mode_1 = 1, mode_2 = 2 };

    yeet::parameter<test_mode> enum_param;
    auto accessor = yeet::utils::make_enum_accessor(enum_param);

    // Test that float values get truncated to int
    accessor.write(1.7f);
    EXPECT_EQ(enum_param.read(), test_mode::mode_1);

    accessor.write(2.3f);
    EXPECT_EQ(enum_param.read(), test_mode::mode_2);
}

TEST(parameter_accessors, angle_wrapping)
{
    yeet::parameter<yeet::angle_wrapped> angle_param;
    auto accessor_rad = yeet::utils::make_angle_wrapped_accessor(angle_param);
    auto accessor_deg = yeet::utils::make_degrees_angle_wrapped_accessor(angle_param);

    // Test basic radians read/write through accessor
    accessor_rad.write(1.57f);  // ~pi/2 radians
    EXPECT_NEAR(angle_param.read().get_radians(), 1.57f, 0.01f);

    // Test basic degrees read/write through accessor
    accessor_deg.write(90.0f);
    EXPECT_NEAR(angle_param.read().get_degrees(), 90.0f, 0.1f);

    // Test conversion between radians and degrees
    accessor_rad.write(3.14159f);  // pi radians
    EXPECT_NEAR(accessor_deg.read<float>(), 180.0f, 0.1f);

    accessor_deg.write(45.0f);                               // 45 degrees
    EXPECT_NEAR(accessor_rad.read<float>(), 0.785f, 0.01f);  // ~pi/4 radians
}

// Test memory efficiency - verify size

TEST(low_level_access, memory_footprint)
{
    // Verify that low_level_access is 32 bytes (3 pointers + padding)
    // Function pointer implementation: read_fn (8 bytes) + write_fn (8 bytes) + context (8 bytes) + padding (8 bytes)
    EXPECT_EQ(sizeof(low_level_access), 32);
}
