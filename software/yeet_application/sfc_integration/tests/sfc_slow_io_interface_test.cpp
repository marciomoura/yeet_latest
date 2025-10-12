// Copyright 2025 Techperts
// Unit tests for SFC Slow IO Interface

#include "sfc_integration/sfc_slow_io_interface.hpp"

#include <gtest/gtest.h>

using namespace yeet::sfc;

// =============================================================================
// Memory Footprint Tests
// =============================================================================

TEST(SfcSlowIoInterface, MemoryFootprint)
{
    // Verify total memory usage is 28 bytes as designed
    EXPECT_EQ(sizeof(sfc_digital_input_raw), 8);
    EXPECT_EQ(sizeof(sfc_digital_output_raw), 4);
    EXPECT_EQ(sizeof(sfc_analog_input_raw), 8);
    EXPECT_EQ(sizeof(sfc_analog_output_raw), 8);

    constexpr size_t total_bytes = sizeof(sfc_digital_input_raw) + sizeof(sfc_digital_output_raw) +
                                   sizeof(sfc_analog_input_raw) + sizeof(sfc_analog_output_raw);
    EXPECT_EQ(total_bytes, 28);
}

// =============================================================================
// Digital Input Tests
// =============================================================================

class SfcDigitalInputsTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        // Zero-initialize raw struct
        raw = sfc_digital_input_raw{};
    }

    sfc_digital_input_raw raw{};
};

TEST_F(SfcDigitalInputsTest, TcsCommands)
{
    raw.operation_on_cmd = 1;
    raw.operation_off_cmd = 0;

    sfc_digital_inputs inputs(raw);

    EXPECT_TRUE(inputs.get_start_command());
    EXPECT_FALSE(inputs.get_stop_command());
}

TEST_F(SfcDigitalInputsTest, SafetyInterlocks)
{
    raw.front_door_open = 1;
    raw.lv_supervision = 1;
    raw.supervision = 1;
    raw.mcb_open_alarm = 0;

    sfc_digital_inputs inputs(raw);

    EXPECT_TRUE(inputs.get_front_door_open());
    EXPECT_TRUE(inputs.get_lv_supervision_ok());
    EXPECT_TRUE(inputs.get_supervision_ok());
    EXPECT_FALSE(inputs.get_mcb_open_alarm());
}

TEST_F(SfcDigitalInputsTest, ExcitationStatus)
{
    raw.exciter_run_stop_status = 1;
    raw.exciter_active = 1;

    sfc_digital_inputs inputs(raw);

    EXPECT_TRUE(inputs.get_exciter_is_running());
    EXPECT_TRUE(inputs.get_exciter_voltage_applied());
}

TEST_F(SfcDigitalInputsTest, CircuitBreakerStatus)
{
    raw.icb_closed_fb = 1;
    raw.icb_open_fb = 0;
    raw.icb_tripping = 0;

    sfc_digital_inputs inputs(raw);

    EXPECT_TRUE(inputs.get_icb_is_closed());
    EXPECT_FALSE(inputs.get_icb_is_open());
    EXPECT_FALSE(inputs.get_icb_is_tripping());
}

TEST_F(SfcDigitalInputsTest, ProtectionSignals)
{
    raw.transformer_pressure_alarm = 1;
    raw.overheat_alarm = 1;
    raw.overheat_trip = 0;
    raw.emergency_off = 0;
    raw.common_hw_trip = 0;
    raw.external_fault = 0;

    sfc_digital_inputs inputs(raw);

    EXPECT_TRUE(inputs.get_transformer_pressure_alarm());
    EXPECT_TRUE(inputs.get_overheat_alarm());
    EXPECT_FALSE(inputs.get_overheat_trip());
    EXPECT_FALSE(inputs.get_emergency_stop());
    EXPECT_FALSE(inputs.get_common_hardware_trip());
    EXPECT_FALSE(inputs.get_external_fault());
}

TEST_F(SfcDigitalInputsTest, CompositeHealthCheck_AllHealthy)
{
    // Set all required signals for healthy operation
    raw.supervision = 1;
    raw.lv_supervision = 1;
    raw.fan_stop_alarm = 0;
    raw.mcb_open_alarm = 0;
    raw.front_door_open = 0;
    raw.common_hw_trip = 0;

    sfc_digital_inputs inputs(raw);

    EXPECT_TRUE(inputs.get_auxiliaries_are_healthy());
}

TEST_F(SfcDigitalInputsTest, CompositeHealthCheck_FanFault)
{
    raw.supervision = 1;
    raw.lv_supervision = 1;
    raw.fan_stop_alarm = 1;  // FAULT
    raw.mcb_open_alarm = 0;
    raw.front_door_open = 0;
    raw.common_hw_trip = 0;

    sfc_digital_inputs inputs(raw);

    EXPECT_FALSE(inputs.get_auxiliaries_are_healthy());
}

TEST_F(SfcDigitalInputsTest, CompositeHealthCheck_PowerLoss)
{
    raw.supervision = 0;  // FAULT
    raw.lv_supervision = 1;
    raw.fan_stop_alarm = 0;
    raw.mcb_open_alarm = 0;
    raw.front_door_open = 0;
    raw.common_hw_trip = 0;

    sfc_digital_inputs inputs(raw);

    EXPECT_FALSE(inputs.get_auxiliaries_are_healthy());
}

TEST_F(SfcDigitalInputsTest, CompositeHealthCheck_HardwareTrip)
{
    raw.supervision = 1;
    raw.lv_supervision = 1;
    raw.fan_stop_alarm = 0;
    raw.mcb_open_alarm = 0;
    raw.front_door_open = 0;
    raw.common_hw_trip = 1;  // FAULT

    sfc_digital_inputs inputs(raw);

    EXPECT_FALSE(inputs.get_auxiliaries_are_healthy());
}

TEST_F(SfcDigitalInputsTest, BreakersReady_Nominal)
{
    raw.icb_closed_fb = 1;
    raw.icb_tripping = 0;

    sfc_digital_inputs inputs(raw);

    EXPECT_TRUE(inputs.get_breakers_are_ready());
}

TEST_F(SfcDigitalInputsTest, BreakersReady_NotClosed)
{
    raw.icb_closed_fb = 0;
    raw.icb_tripping = 0;

    sfc_digital_inputs inputs(raw);

    EXPECT_FALSE(inputs.get_breakers_are_ready());
}

TEST_F(SfcDigitalInputsTest, BreakersReady_Tripping)
{
    raw.icb_closed_fb = 1;
    raw.icb_tripping = 1;

    sfc_digital_inputs inputs(raw);

    EXPECT_FALSE(inputs.get_breakers_are_ready());
}

TEST_F(SfcDigitalInputsTest, ShutdownRequested_EmergencyStop)
{
    raw.emergency_off = 1;
    raw.common_hw_trip = 0;
    raw.external_fault = 0;
    raw.overheat_trip = 0;
    raw.transformer_heavy_gas_trip = 0;
    raw.reactor_temp_trip = 0;

    sfc_digital_inputs inputs(raw);

    EXPECT_TRUE(inputs.get_shutdown_requested());
}

TEST_F(SfcDigitalInputsTest, ShutdownRequested_HardwareTrip)
{
    raw.emergency_off = 0;
    raw.common_hw_trip = 1;
    raw.external_fault = 0;
    raw.overheat_trip = 0;
    raw.transformer_heavy_gas_trip = 0;
    raw.reactor_temp_trip = 0;

    sfc_digital_inputs inputs(raw);

    EXPECT_TRUE(inputs.get_shutdown_requested());
}

TEST_F(SfcDigitalInputsTest, ShutdownRequested_MultipleTrips)
{
    raw.emergency_off = 1;
    raw.common_hw_trip = 1;
    raw.external_fault = 1;
    raw.overheat_trip = 1;
    raw.transformer_heavy_gas_trip = 1;
    raw.reactor_temp_trip = 1;

    sfc_digital_inputs inputs(raw);

    EXPECT_TRUE(inputs.get_shutdown_requested());
}

TEST_F(SfcDigitalInputsTest, ShutdownRequested_NoTrips)
{
    raw.emergency_off = 0;
    raw.common_hw_trip = 0;
    raw.external_fault = 0;
    raw.overheat_trip = 0;
    raw.transformer_heavy_gas_trip = 0;
    raw.reactor_temp_trip = 0;

    sfc_digital_inputs inputs(raw);

    EXPECT_FALSE(inputs.get_shutdown_requested());
}

// =============================================================================
// Digital Output Tests
// =============================================================================

class SfcDigitalOutputsTest : public ::testing::Test {
protected:
    void SetUp() override { raw = sfc_digital_output_raw{}; }

    sfc_digital_output_raw raw{};
};

TEST_F(SfcDigitalOutputsTest, FanControl_Individual)
{
    sfc_digital_outputs outputs(raw);

    outputs.set_fan1_on(true);
    outputs.set_fan2_on(false);
    outputs.set_fan3_on(true);
    outputs.set_fan4_on(false);

    EXPECT_EQ(raw.fan1_on, 1);
    EXPECT_EQ(raw.fan2_on, 0);
    EXPECT_EQ(raw.fan3_on, 1);
    EXPECT_EQ(raw.fan4_on, 0);
}

TEST_F(SfcDigitalOutputsTest, FanControl_AllOn)
{
    sfc_digital_outputs outputs(raw);

    outputs.set_all_fans(true);

    EXPECT_EQ(raw.fan1_on, 1);
    EXPECT_EQ(raw.fan2_on, 1);
    EXPECT_EQ(raw.fan3_on, 1);
    EXPECT_EQ(raw.fan4_on, 1);
}

TEST_F(SfcDigitalOutputsTest, FanControl_AllOff)
{
    // Pre-set all fans on
    raw.fan1_on = 1;
    raw.fan2_on = 1;
    raw.fan3_on = 1;
    raw.fan4_on = 1;

    sfc_digital_outputs outputs(raw);
    outputs.set_all_fans(false);

    EXPECT_EQ(raw.fan1_on, 0);
    EXPECT_EQ(raw.fan2_on, 0);
    EXPECT_EQ(raw.fan3_on, 0);
    EXPECT_EQ(raw.fan4_on, 0);
}

TEST_F(SfcDigitalOutputsTest, BreakerCommands)
{
    sfc_digital_outputs outputs(raw);

    outputs.set_icb_close_command(true);
    outputs.set_icb_open_command(false);
    outputs.set_icb_trip_command(false);

    EXPECT_EQ(raw.icb_close, 1);
    EXPECT_EQ(raw.icb_open, 0);
    EXPECT_EQ(raw.icb_trip, 0);
}

TEST_F(SfcDigitalOutputsTest, TcsFeedback)
{
    sfc_digital_outputs outputs(raw);

    // System running
    outputs.set_operation_off_feedback(false);
    outputs.set_operation_on_feedback(true);

    EXPECT_EQ(raw.operation_off_fb, 0);
    EXPECT_EQ(raw.operation_on_fb, 1);

    // System stopped
    outputs.set_operation_off_feedback(true);
    outputs.set_operation_on_feedback(false);

    EXPECT_EQ(raw.operation_off_fb, 1);
    EXPECT_EQ(raw.operation_on_fb, 0);
}

TEST_F(SfcDigitalOutputsTest, ExcitationCommands)
{
    sfc_digital_outputs outputs(raw);

    outputs.set_exciter_on_command(true);
    outputs.set_exciter_off_command(false);
    outputs.set_exciter_control_mode(true);

    EXPECT_EQ(raw.exciter_on_cmd, 1);
    EXPECT_EQ(raw.exciter_off_cmd, 0);
    EXPECT_EQ(raw.exciter_control_mode, 1);
}

TEST_F(SfcDigitalOutputsTest, ResetAll)
{
    // Pre-set all outputs
    raw.fan1_on = 1;
    raw.fan2_on = 1;
    raw.icb_close = 1;
    raw.exciter_on_cmd = 1;
    raw.operation_on_fb = 1;

    sfc_digital_outputs outputs(raw);
    outputs.reset_all();

    // Verify all bits are zero
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&raw);
    for (size_t i = 0; i < sizeof(raw); ++i) {
        EXPECT_EQ(bytes[i], 0);
    }
}

// =============================================================================
// Analog Input Tests
// =============================================================================

class SfcAnalogInputsTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        raw.excitation_current_a = 0.0f;
        raw.excitation_voltage_v = 0.0f;
    }

    sfc_analog_input_raw raw{};
};

TEST_F(SfcAnalogInputsTest, RawValues)
{
    raw.excitation_current_a = 123.45f;
    raw.excitation_voltage_v = 678.90f;

    sfc_analog_inputs inputs(raw);

    EXPECT_FLOAT_EQ(inputs.get_excitation_current_a(), 123.45f);
    EXPECT_FLOAT_EQ(inputs.get_excitation_voltage_v(), 678.90f);
}

TEST_F(SfcAnalogInputsTest, PerUnitConversion_Current)
{
    raw.excitation_current_a = 50.0f;
    const float base_current_a = 100.0f;  // Rated field current

    sfc_analog_inputs inputs(raw);

    auto current_pu = inputs.get_excitation_current_pu(base_current_a);
    EXPECT_FLOAT_EQ(current_pu, 0.5f);  // 50/100 = 0.5 pu
}

TEST_F(SfcAnalogInputsTest, PerUnitConversion_Voltage)
{
    raw.excitation_voltage_v = 240.0f;
    const float base_voltage_v = 300.0f;  // Rated field voltage

    sfc_analog_inputs inputs(raw);

    auto voltage_pu = inputs.get_excitation_voltage_pu(base_voltage_v);
    EXPECT_FLOAT_EQ(voltage_pu, 0.8f);  // 240/300 = 0.8 pu
}

TEST_F(SfcAnalogInputsTest, PerUnitConversion_ZeroValues)
{
    raw.excitation_current_a = 0.0f;
    raw.excitation_voltage_v = 0.0f;

    sfc_analog_inputs inputs(raw);

    EXPECT_FLOAT_EQ(inputs.get_excitation_current_pu(100.0f), 0.0f);
    EXPECT_FLOAT_EQ(inputs.get_excitation_voltage_pu(300.0f), 0.0f);
}

TEST_F(SfcAnalogInputsTest, PerUnitConversion_OverRated)
{
    raw.excitation_current_a = 150.0f;
    raw.excitation_voltage_v = 450.0f;

    sfc_analog_inputs inputs(raw);

    EXPECT_FLOAT_EQ(inputs.get_excitation_current_pu(100.0f), 1.5f);  // Overexcited
    EXPECT_FLOAT_EQ(inputs.get_excitation_voltage_pu(300.0f), 1.5f);
}

// =============================================================================
// Analog Output Tests
// =============================================================================

class SfcAnalogOutputsTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        raw.excitation_voltage_ref_v = 0.0f;
        raw.excitation_current_ref_a = 0.0f;
    }

    sfc_analog_output_raw raw{};
};

TEST_F(SfcAnalogOutputsTest, RawSetpoints)
{
    sfc_analog_outputs outputs(raw);

    outputs.set_excitation_voltage_ref_v(250.0f);
    outputs.set_excitation_current_ref_a(75.5f);

    EXPECT_FLOAT_EQ(raw.excitation_voltage_ref_v, 250.0f);
    EXPECT_FLOAT_EQ(raw.excitation_current_ref_a, 75.5f);
}

TEST_F(SfcAnalogOutputsTest, PerUnitConversion_Voltage)
{
    const float base_voltage_v = 300.0f;
    const yeet::real_t voltage_pu(0.8);  // 80% of rated

    sfc_analog_outputs outputs(raw);
    outputs.set_excitation_voltage_ref_pu(voltage_pu, base_voltage_v);

    EXPECT_FLOAT_EQ(raw.excitation_voltage_ref_v, 240.0f);  // 0.8 * 300
}

TEST_F(SfcAnalogOutputsTest, PerUnitConversion_Current)
{
    const float base_current_a = 100.0f;
    const yeet::real_t current_pu(0.5);  // 50% of rated

    sfc_analog_outputs outputs(raw);
    outputs.set_excitation_current_ref_pu(current_pu, base_current_a);

    EXPECT_FLOAT_EQ(raw.excitation_current_ref_a, 50.0f);  // 0.5 * 100
}

TEST_F(SfcAnalogOutputsTest, PerUnitConversion_ZeroSetpoint)
{
    const yeet::real_t zero_pu(0.0);

    sfc_analog_outputs outputs(raw);
    outputs.set_excitation_voltage_ref_pu(zero_pu, 300.0f);
    outputs.set_excitation_current_ref_pu(zero_pu, 100.0f);

    EXPECT_FLOAT_EQ(raw.excitation_voltage_ref_v, 0.0f);
    EXPECT_FLOAT_EQ(raw.excitation_current_ref_a, 0.0f);
}

TEST_F(SfcAnalogOutputsTest, PerUnitConversion_MaxSetpoint)
{
    const yeet::real_t max_pu(1.0);
    const float base_voltage_v = 300.0f;
    const float base_current_a = 100.0f;

    sfc_analog_outputs outputs(raw);
    outputs.set_excitation_voltage_ref_pu(max_pu, base_voltage_v);
    outputs.set_excitation_current_ref_pu(max_pu, base_current_a);

    EXPECT_FLOAT_EQ(raw.excitation_voltage_ref_v, 300.0f);
    EXPECT_FLOAT_EQ(raw.excitation_current_ref_a, 100.0f);
}

TEST_F(SfcAnalogOutputsTest, ResetAll)
{
    raw.excitation_voltage_ref_v = 250.0f;
    raw.excitation_current_ref_a = 75.0f;

    sfc_analog_outputs outputs(raw);
    outputs.reset_all();

    EXPECT_FLOAT_EQ(raw.excitation_voltage_ref_v, 0.0f);
    EXPECT_FLOAT_EQ(raw.excitation_current_ref_a, 0.0f);
}

// =============================================================================
// Integration Tests - Realistic Scenarios
// =============================================================================

TEST(SfcSlowIoInterface, Scenario_SystemStartup)
{
    // Simulate a complete startup sequence
    sfc_digital_input_raw di_raw{};
    sfc_digital_output_raw do_raw{};
    sfc_analog_output_raw ao_raw{};

    // Step 1: Initial state - auxiliaries healthy, breakers open
    di_raw.supervision = 1;
    di_raw.lv_supervision = 1;
    di_raw.icb_open_fb = 1;
    di_raw.icb_closed_fb = 0;

    sfc_digital_inputs inputs(di_raw);
    sfc_digital_outputs outputs(do_raw);
    sfc_analog_outputs analog_out(ao_raw);

    EXPECT_TRUE(inputs.get_auxiliaries_are_healthy());
    EXPECT_FALSE(inputs.get_breakers_are_ready());

    // Step 2: Turn on fans
    outputs.set_all_fans(true);
    EXPECT_EQ(do_raw.fan1_on, 1);

    // Step 3: Close breaker
    outputs.set_icb_close_command(true);
    di_raw.icb_closed_fb = 1;
    di_raw.icb_open_fb = 0;

    EXPECT_TRUE(inputs.get_breakers_are_ready());

    // Step 4: Start command received
    di_raw.operation_on_cmd = 1;
    EXPECT_TRUE(inputs.get_start_command());

    // Step 5: Enable excitation
    outputs.set_exciter_on_command(true);
    analog_out.set_excitation_voltage_ref_pu(yeet::real_t(0.8), 300.0f);

    EXPECT_EQ(do_raw.exciter_on_cmd, 1);
    EXPECT_FLOAT_EQ(ao_raw.excitation_voltage_ref_v, 240.0f);

    // Step 6: System running feedback
    outputs.set_operation_on_feedback(true);
    EXPECT_EQ(do_raw.operation_on_fb, 1);
}

TEST(SfcSlowIoInterface, Scenario_EmergencyShutdown)
{
    sfc_digital_input_raw di_raw{};
    sfc_digital_output_raw do_raw{};

    // System running normally
    di_raw.supervision = 1;
    di_raw.lv_supervision = 1;
    di_raw.icb_closed_fb = 1;
    di_raw.operation_on_cmd = 1;

    do_raw.operation_on_fb = 1;
    do_raw.exciter_on_cmd = 1;

    sfc_digital_inputs inputs(di_raw);
    sfc_digital_outputs outputs(do_raw);

    EXPECT_TRUE(inputs.get_auxiliaries_are_healthy());
    EXPECT_FALSE(inputs.get_shutdown_requested());

    // Emergency stop pressed
    di_raw.emergency_off = 1;

    EXPECT_TRUE(inputs.get_shutdown_requested());

    // Controller responds
    outputs.set_exciter_off_command(true);
    outputs.set_operation_on_feedback(false);
    outputs.set_operation_off_feedback(true);

    EXPECT_EQ(do_raw.exciter_off_cmd, 1);
    EXPECT_EQ(do_raw.operation_on_fb, 0);
    EXPECT_EQ(do_raw.operation_off_fb, 1);
}

TEST(SfcSlowIoInterface, Scenario_FieldEstablishmentCheck)
{
    // Test the analog input thresholds for excitation_is_ready event
    sfc_analog_input_raw ai_raw{};

    // Initial state: no field
    ai_raw.excitation_current_a = 0.0f;
    ai_raw.excitation_voltage_v = 0.0f;

    sfc_analog_inputs inputs(ai_raw);

    const float rated_current = 100.0f;
    const float rated_voltage = 300.0f;
    const float current_threshold = 0.10f;  // 10%
    const float voltage_threshold = 0.80f;  // 80%

    // Field not ready
    auto current_pu = inputs.get_excitation_current_pu(rated_current);
    auto voltage_pu = inputs.get_excitation_voltage_pu(rated_voltage);
    bool field_ready = (current_pu > current_threshold) && (voltage_pu > voltage_threshold);
    EXPECT_FALSE(field_ready);

    // Pre-fluxing in progress: voltage building, low current
    ai_raw.excitation_current_a = 5.0f;    // 5% (below threshold)
    ai_raw.excitation_voltage_v = 250.0f;  // 83% (above threshold)

    current_pu = inputs.get_excitation_current_pu(rated_current);
    voltage_pu = inputs.get_excitation_voltage_pu(rated_voltage);
    field_ready = (current_pu > current_threshold) && (voltage_pu > voltage_threshold);
    EXPECT_FALSE(field_ready);  // Current still too low

    // Field established: both voltage and current nominal
    ai_raw.excitation_current_a = 15.0f;   // 15% (above threshold)
    ai_raw.excitation_voltage_v = 260.0f;  // 87% (above threshold)

    current_pu = inputs.get_excitation_current_pu(rated_current);
    voltage_pu = inputs.get_excitation_voltage_pu(rated_voltage);
    field_ready = (current_pu > current_threshold) && (voltage_pu > voltage_threshold);
    EXPECT_TRUE(field_ready);
}

