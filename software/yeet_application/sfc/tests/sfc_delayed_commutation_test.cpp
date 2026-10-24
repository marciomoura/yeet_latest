#include <gtest/gtest.h>

#include <deque>

#include "sfc/sfc_grid_side_modulator.hpp"
#include "sfc/sfc_machine_side_modulator.hpp"
#include "yeet_control/constants.hpp"

using namespace yeet::application::sfc;
using namespace yeet;

namespace {

/**
 * @brief Test fixture for combined modulators with communication delays.
 */
class SfcDelayedCommutationTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        grid_modulator = std::make_unique<sfc_grid_side_modulator>(sampling_time);
        machine_modulator = std::make_unique<sfc_machine_side_modulator>(sampling_time);
    }

    std::unique_ptr<sfc_grid_side_modulator> grid_modulator;
    std::unique_ptr<sfc_machine_side_modulator> machine_modulator;

    static constexpr duration_t sampling_time{1.0e-3};        // 1ms sampling (matching fiber optics task)
    static constexpr angle_t test_pi_angle = angle_t(0.785);  // 45 deg

    // Signal delay buffers
    std::deque<bool> grid_to_machine_delay;  // Acknowledge signal
    std::deque<bool> machine_to_grid_delay;  // Force request signal

    void simulate_cycle_with_delay(
        bool enable, sfc_operating_mode_t mode, angle_t rotor_pos, bool is_current_zero, int delay_ms)
    {
        // 1. Get current outputs
        bool raw_force_request = machine_modulator->get_request_force_commutation();
        bool raw_force_ack = grid_modulator->get_force_commutation_acknowledged();

        // 2. Buffer them
        machine_to_grid_delay.push_back(raw_force_request);
        grid_to_machine_delay.push_back(raw_force_ack);

        // 3. Extract delayed signals
        bool delayed_force_request = false;
        bool delayed_force_ack = false;

        if (machine_to_grid_delay.size() > (size_t)delay_ms) {
            delayed_force_request = machine_to_grid_delay.front();
            machine_to_grid_delay.pop_front();
        }

        if (grid_to_machine_delay.size() > (size_t)delay_ms) {
            delayed_force_ack = grid_to_machine_delay.front();
            grid_to_machine_delay.pop_front();
        }

        // 4. Update modulators with delayed cross-signals
        machine_modulator->update(enable, mode, delayed_force_ack, angle_wrapped::from_radians(angle_t(rotor_pos)),
            angle_wrapped::from_radians(angle_t(test_pi_angle)));
        grid_modulator->update(enable, mode, angle_wrapped::from_radians(angle_t(0.0)), delayed_force_request,
            is_current_zero, angle_wrapped::from_radians(angle_t(test_pi_angle)), false);
    }
};

TEST_F(SfcDelayedCommutationTest, StartupWith3msDelayAndReset)
{
    // Simulate startup sequence in forced commutation with 3ms fiber optic delay
    constexpr int delay_ms = 3;
    bool enable = true;
    sfc_operating_mode_t mode = sfc_operating_mode_t::forced_commutation;
    angle_t rotor_pos = angle_t(0.0);
    bool current_zero = true;

    // Run for several cycles
    for (int i = 0; i < 10; ++i) {
        simulate_cycle_with_delay(enable, mode, rotor_pos, current_zero, delay_ms);

        // During zero current startup, even with delays, the grid modulator MUST:
        // 1. Clamp its firing angle to safe value (fix)
        // 2. Request main PI reset (fix)

        EXPECT_GE(grid_modulator->get_firing_angle_rad().value(), 1.0);
        EXPECT_TRUE(grid_modulator->get_is_requesting_main_pi_reset());
    }
}

}  // namespace
