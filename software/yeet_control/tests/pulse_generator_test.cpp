#include "yeet_control/pulse_generator.hpp"

#include <gtest/gtest.h>

#include "yeet_control/constants.hpp"

using namespace yeet;
using namespace yeet::control;

class PulseGeneratorTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Default pulse generator with 1° width starting at 0°
        default_gen = std::make_unique<pulse_generator>(angle<zero_to_two_pi>(deg_to_rad(0.0)),  // firing_offset = 0°
                                                        angle<zero_to_two_pi>(deg_to_rad(1.0))   // pulse_width = 1°
        );
    }

    std::unique_ptr<pulse_generator> default_gen;

    // Small epsilon to avoid floating-point precision issues at boundaries
    static constexpr double eps = 1e-6;  // Small angle in radians (~0.00006°)
};

TEST_F(PulseGeneratorTest, PulseIsOffWhenDisabled) {
    // Pulse should be off regardless of angles when disabled
    EXPECT_FALSE(default_gen->update(angle<zero_to_two_pi>(deg_to_rad(0.0)),  // reference_angle
                                     angle<zero_to_two_pi>(deg_to_rad(0.0)),  // firing_delay
                                     false                                    // enable = false
                                     ));

    EXPECT_FALSE(default_gen->update(angle<zero_to_two_pi>(deg_to_rad(0.5)),  // reference_angle in normal pulse range
                                     angle<zero_to_two_pi>(deg_to_rad(0.0)),  // firing_delay
                                     false                                    // enable = false
                                     ));
}

TEST_F(PulseGeneratorTest, BasicPulseOperation) {
    // Test pulse with firing_offset=0°, pulse_width=1°, no firing delay

    // Should be ON at 0° reference angle
    EXPECT_TRUE(default_gen->update(angle<zero_to_two_pi>(deg_to_rad(0.0)),  // reference_angle = 0°
                                    angle<zero_to_two_pi>(deg_to_rad(0.0)),  // firing_delay = 0°
                                    true                                     // enable = true
                                    ));

    // Should be ON at 0.5° reference angle
    EXPECT_TRUE(default_gen->update(angle<zero_to_two_pi>(deg_to_rad(0.5)),  // reference_angle = 0.5°
                                    angle<zero_to_two_pi>(deg_to_rad(0.0)),  // firing_delay = 0°
                                    true                                     // enable = true
                                    ));

    // Should be ON just before 1° (still inside window)
    EXPECT_TRUE(default_gen->update(angle<zero_to_two_pi>(deg_to_rad(1.0) - eps),  // reference_angle = 1° - eps
                                    angle<zero_to_two_pi>(deg_to_rad(0.0)),        // firing_delay = 0°
                                    true                                           // enable = true
                                    ));

    // Should be OFF just after 1° reference angle (avoiding edge case precision issues)
    EXPECT_FALSE(default_gen->update(angle<zero_to_two_pi>(deg_to_rad(1.0) + eps),  // reference_angle = 1° + eps
                                     angle<zero_to_two_pi>(deg_to_rad(0.0)),        // firing_delay = 0°
                                     true                                           // enable = true
                                     ));

    // Should be OFF at 2° reference angle
    EXPECT_FALSE(default_gen->update(angle<zero_to_two_pi>(deg_to_rad(2.0)),  // reference_angle = 2°
                                     angle<zero_to_two_pi>(deg_to_rad(0.0)),  // firing_delay = 0°
                                     true                                     // enable = true
                                     ));
}

TEST_F(PulseGeneratorTest, FiringDelayEffect) {
    // Test how firing delay shifts the pulse window
    // With firing_delay=30°, the pulse window shifts from [0°,1°) to [30°,31°)

    // Should be OFF at 0° reference with 30° delay
    EXPECT_FALSE(default_gen->update(angle<zero_to_two_pi>(deg_to_rad(0.0)),   // reference_angle = 0°
                                     angle<zero_to_two_pi>(deg_to_rad(30.0)),  // firing_delay = 30°
                                     true                                      // enable = true
                                     ));

    // Should be ON at 30° reference with 30° delay
    EXPECT_TRUE(default_gen->update(angle<zero_to_two_pi>(deg_to_rad(30.0)),  // reference_angle = 30°
                                    angle<zero_to_two_pi>(deg_to_rad(30.0)),  // firing_delay = 30°
                                    true                                      // enable = true
                                    ));

    // Should be ON at 30.5° reference with 30° delay
    EXPECT_TRUE(default_gen->update(angle<zero_to_two_pi>(deg_to_rad(30.5)),  // reference_angle = 30.5°
                                    angle<zero_to_two_pi>(deg_to_rad(30.0)),  // firing_delay = 30°
                                    true                                      // enable = true
                                    ));

    // Should be ON just before 31° with 30° delay (still inside window)
    EXPECT_TRUE(default_gen->update(angle<zero_to_two_pi>(deg_to_rad(31.0) - eps),  // reference_angle = 31° - eps
                                    angle<zero_to_two_pi>(deg_to_rad(30.0)),        // firing_delay = 30°
                                    true                                            // enable = true
                                    ));

    // Should be OFF just after 31° reference with 30° delay (avoiding edge case precision issues)
    EXPECT_FALSE(default_gen->update(angle<zero_to_two_pi>(deg_to_rad(31.0) + eps),  // reference_angle = 31° + eps
                                     angle<zero_to_two_pi>(deg_to_rad(30.0)),        // firing_delay = 30°
                                     true                                            // enable = true
                                     ));
}

TEST_F(PulseGeneratorTest, AngleWrappingBehavior) {
    // Test pulse behavior across the 360°/0° boundary
    // Create a pulse generator with firing_offset=358° and width=5°
    // This should create a window [358°, 363°) which wraps to [358°,360°) ∪ [0°,3°)

    pulse_generator wrap_gen(angle<zero_to_two_pi>(deg_to_rad(358.0)),  // firing_offset = 358°
                             angle<zero_to_two_pi>(deg_to_rad(5.0))     // pulse_width = 5°
    );

    // Should be ON at 358° (start of window)
    EXPECT_TRUE(wrap_gen.update(angle<zero_to_two_pi>(deg_to_rad(358.0)),  // reference_angle = 358°
                                angle<zero_to_two_pi>(deg_to_rad(0.0)),    // firing_delay = 0°
                                true                                       // enable = true
                                ));

    // Should be ON at 359°
    EXPECT_TRUE(wrap_gen.update(angle<zero_to_two_pi>(deg_to_rad(359.0)),  // reference_angle = 359°
                                angle<zero_to_two_pi>(deg_to_rad(0.0)),    // firing_delay = 0°
                                true                                       // enable = true
                                ));

    // Should be ON at 0° (wrapped part of window)
    EXPECT_TRUE(wrap_gen.update(angle<zero_to_two_pi>(deg_to_rad(0.0)),  // reference_angle = 0°
                                angle<zero_to_two_pi>(deg_to_rad(0.0)),  // firing_delay = 0°
                                true                                     // enable = true
                                ));

    // Should be ON at 2°
    EXPECT_TRUE(wrap_gen.update(angle<zero_to_two_pi>(deg_to_rad(2.0)),  // reference_angle = 2°
                                angle<zero_to_two_pi>(deg_to_rad(0.0)),  // firing_delay = 0°
                                true                                     // enable = true
                                ));

    // Should be ON just before 3° (still inside window)
    EXPECT_TRUE(wrap_gen.update(angle<zero_to_two_pi>(deg_to_rad(3.0) - eps),  // reference_angle = 3° - eps
                                angle<zero_to_two_pi>(deg_to_rad(0.0)),        // firing_delay = 0°
                                true                                           // enable = true
                                ));

    // Should be OFF just after 3° (end of window, avoiding edge case precision issues)
    EXPECT_FALSE(wrap_gen.update(angle<zero_to_two_pi>(deg_to_rad(3.0) + eps),  // reference_angle = 3° + eps
                                 angle<zero_to_two_pi>(deg_to_rad(0.0)),        // firing_delay = 0°
                                 true                                           // enable = true
                                 ));

    // Should be OFF at 357° (before window)
    EXPECT_FALSE(wrap_gen.update(angle<zero_to_two_pi>(deg_to_rad(357.0)),  // reference_angle = 357°
                                 angle<zero_to_two_pi>(deg_to_rad(0.0)),    // firing_delay = 0°
                                 true                                       // enable = true
                                 ));
}

TEST_F(PulseGeneratorTest, ConfigurationMethods) {
    // Test setter and getter methods

    // Test firing offset
    default_gen->set_firing_offset(angle<zero_to_two_pi>(deg_to_rad(45.0)));
    EXPECT_NEAR(default_gen->get_firing_offset().get_degrees(), 45.0, 1e-10);

    // Test pulse width
    default_gen->set_pulse_width(angle<zero_to_two_pi>(deg_to_rad(10.0)));
    EXPECT_NEAR(default_gen->get_pulse_width().get_degrees(), 10.0, 1e-10);

    // Verify the new configuration works
    // Should be ON at 45° with new firing offset
    EXPECT_TRUE(default_gen->update(angle<zero_to_two_pi>(deg_to_rad(50.0)),  // reference_angle = 50°
                                    angle<zero_to_two_pi>(deg_to_rad(0.0)),   // firing_delay = 0°
                                    true                                      // enable = true
                                    ));
}
