#pragma once
#include <array>
#include <cstdint>

#include "pulse_generator.hpp"
#include "yeet_control/angle_wrapped.hpp"
#include "yeet_control/constants.hpp"
#include "yeet_control/units.hpp"

namespace yeet::control {

/**
 * @brief Generates firing pulses for a 6-pulse thyristor converter.
 *
 * This class takes a reference angle (typically from a PLL synchronized with the
 * grid voltage) and a firing angle command (alpha) to generate the six pulse
 * signals for a standard 6-pulse bridge.
 *
 * The generator supports three operating modes:
 * 1. **Normal mode**: External reference angle with firing angle control
 * 2. **Virtual reference mode**: Internal angle integration based on configured frequency
 * 3. **Pulse masking**: Individual pulses can be enabled/disabled for testing
 *
 * Example usage (normal mode):
 * @code
 * six_pulse_generator gen(six_pulse_generator::reference_type::phase);
 * auto pulses = gen.update(pll_angle, firing_angle, enable);
 * @endcode
 *
 * Example usage (virtual reference mode):
 * @code
 * six_pulse_generator gen(six_pulse_generator::reference_type::phase);
 * gen.configure_virtual_reference(frequency_t{50.0}, us(100.0));
 * gen.enable_virtual_reference(true);
 * auto pulses = gen.update(firing_angle, enable);  // Uses internal angle
 * @endcode
 *
 * Example usage (pulse masking):
 * @code
 * six_pulse_generator gen(six_pulse_generator::reference_type::phase);
 * gen.configure_pulse_mask({true, false, true, false, true, false});  // Only P1, P3, P5
 * auto pulses = gen.update(reference_angle, firing_angle, enable);
 * @endcode
 */
class six_pulse_generator {
public:
    using angle_t = angle_wrapped;

    /**
     * @brief Defines the type of AC voltage reference used for synchronization.
     */
    enum class reference_type {
        line,  ///< Reference angle is from a line-to-line voltage (e.g., Vab). No offset needed.
        phase  ///< Reference angle is from a phase-to-neutral voltage (e.g., Van). A 30-degree offset is applied.
    };

    enum class pulse_sector_t : std::uint32_t {
        none = 0,
        sector_1 = 1,
        sector_2 = 2,
        sector_3 = 3,
        sector_4 = 4,
        sector_5 = 5,
        sector_6 = 6
    };

    /**
     * @brief Structure to hold the state of the six output pulses.
     */
    struct pulses {
        bool p1{false}, p2{false}, p3{false}, p4{false}, p5{false}, p6{false};

        std::array<bool, 6> to_array() const { return {p1, p2, p3, p4, p5, p6}; }

        // Conversional operator
        operator std::array<bool, 6>() const { return to_array(); }

        constexpr size_t size() const { return 6; }

        auto begin() { return &p1; }
        auto end() { return &p6 + 1; }
    };

    /**
     * @brief Structure to configure which pulses are enabled (mask).
     *
     * When a pulse mask bit is false, the corresponding pulse output will always
     * be false, regardless of the angle-based pulse generation logic.
     * This is useful for commissioning and testing individual thyristors.
     */
    struct pulse_mask {
        bool p1{true}, p2{true}, p3{true}, p4{true}, p5{true}, p6{true};

        std::array<bool, 6> to_array() const { return {p1, p2, p3, p4, p5, p6}; }

        /**
         * @brief Creates a mask with all pulses enabled
         */
        static pulse_mask all_enabled() { return {true, true, true, true, true, true}; }

        /**
         * @brief Creates a mask with all pulses disabled
         */
        static pulse_mask all_disabled() { return {false, false, false, false, false, false}; }

        /**
         * @brief Creates a mask with only odd pulses enabled (P1, P3, P5)
         */
        static pulse_mask odd_only() { return {true, false, true, false, true, false}; }

        /**
         * @brief Creates a mask with only even pulses enabled (P2, P4, P6)
         */
        static pulse_mask even_only() { return {false, true, false, true, false, true}; }
    };

    /**
     * @brief Constructs a six_pulse_generator.
     *
     * @param ref_type Specifies whether the reference angle is from a line or phase voltage.
     * @param pulse_width The desired width of the firing pulses.
     */
    explicit six_pulse_generator(
        reference_type ref_type = reference_type::phase, angle_t pulse_width = angle_t(yeet::deg_to_rad(30.0)));

    /**
     * @brief Updates the pulse generator and returns the new pulse states.
     *
     * The firing sequence is assumed to be 1-2-3-4-5-6, with each pulse
     * separated by 60 degrees. The firing angle `alpha` is measured from the
     * natural commutation point, which is assumed to be aligned with the
     * provided `reference_angle`.
     *
     * @param reference_angle The reference angle of the AC system.
     * @param firing_angle The desired firing angle (alpha).
     * @param enable Gating enable signal. If false, all pulses are off.
     * @return The generated pulses for the six thyristors.
     */
    pulses update(angle_t reference_angle, angle_t firing_angle, bool enable);

    /**
     * @brief Updates the pulse generator using internal virtual reference angle.
     *
     * This overload uses the internally integrated virtual reference angle instead
     * of an external reference. The virtual reference must be configured and enabled
     * via configure_virtual_reference() and enable_virtual_reference().
     *
     * If virtual reference is not enabled, this method returns all-off pulses.
     *
     * @param firing_angle The desired firing angle (alpha).
     * @param enable Gating enable signal. If false, all pulses are off.
     * @return The generated pulses for the six thyristors.
     */
    pulses update(angle_t firing_angle, bool enable);

    /**
     * @brief Configures the virtual reference generator parameters.
     *
     * The virtual reference generates an internally integrated angle based on the
     * specified frequency and sampling time. This is useful for open-loop testing
     * and commissioning when no external PLL reference is available.
     *
     * @param frequency The desired reference frequency (e.g., 50 Hz for grid frequency).
     * @param sampling_time The sampling period at which update() will be called.
     */
    void configure_virtual_reference(frequency_t frequency, duration_t sampling_time);

    /**
     * @brief Enables or disables the virtual reference mode.
     *
     * When enabled, the update(firing_angle, enable) overload uses the internally
     * generated angle. When disabled, that overload returns all-off pulses.
     *
     * @param enable True to enable virtual reference mode, false to disable.
     */
    void enable_virtual_reference(bool enable);

    /**
     * @brief Resets the virtual reference angle to zero.
     */
    void reset_virtual_reference();

    /**
     * @brief Resets the virtual reference angle to a specified initial value.
     *
     * @param initial_angle The angle to reset to.
     */
    void reset_virtual_reference(angle_t initial_angle);

    /**
     * @brief Configures the pulse mask for individual pulse control.
     *
     * Pulses with their mask bit set to false will always be off, regardless
     * of the angle-based generation logic. This is useful for testing individual
     * thyristors during commissioning.
     *
     * @param mask The pulse mask configuration.
     */
    void configure_pulse_mask(const pulse_mask& mask);

    /**
     * @brief Enables or disables pulse masking.
     *
     * When enabled, the configured pulse mask is applied to the generated pulses.
     * When disabled, all pulses are generated normally (equivalent to all_enabled mask).
     *
     * @param enable True to enable pulse masking, false to disable.
     */
    void enable_pulse_mask(bool enable);

    /**
     * @brief Sets the pulse width for all six pulse generators.
     *
     * @param pulse_width The new pulse width.
     */
    void set_pulse_width(angle_t pulse_width);

    /**
     * @brief Gets the current pulse width.
     *
     * @return The pulse width.
     */
    angle_t get_pulse_width() const;

    /**
     * @brief Gets the current pulses.
     *
     * @return The current state of the six pulses.
     */
    pulses get_pulses() const;

    /**
     * @brief Gets the current operational sector.
     *
     * The sector is an enum from `pulse_sector_t`, corresponding to each 60-degree
     * interval of the reference angle (e.g., Sector 1 is [0°, 60°)).
     *
     * @return The current sector.
     */
    pulse_sector_t get_sector() const noexcept;

    /**
     * @brief Returns true for one cycle if the operational sector has changed.
     *
     * This flag is updated during the `update` call. It will be true if the
     * calculated sector in the current `update` call is different from the
     * sector in the previous call, and false otherwise.
     *
     * @return true if the sector has just changed, false otherwise.
     */
    bool get_sector_changed() const noexcept;

    /**
     * @brief Gets the current virtual reference angle.
     *
     * @return The internally integrated virtual reference angle.
     */
    angle_t get_virtual_reference_angle() const noexcept;

    /**
     * @brief Checks if virtual reference mode is enabled.
     *
     * @return True if virtual reference mode is enabled, false otherwise.
     */
    bool is_virtual_reference_enabled() const noexcept;

    /**
     * @brief Gets the configured virtual reference frequency.
     *
     * @return The virtual reference frequency in Hz.
     */
    frequency_t get_virtual_reference_frequency() const noexcept;

    /**
     * @brief Gets the current pulse mask configuration.
     *
     * @return The current pulse mask.
     */
    pulse_mask get_pulse_mask() const noexcept;

    /**
     * @brief Checks if pulse masking is enabled.
     *
     * @return True if pulse masking is enabled, false otherwise.
     */
    bool is_pulse_mask_enabled() const noexcept;

private:
    // Core pulse generation
    pulses generate_pulses(angle_t reference_angle, angle_t firing_angle, bool enable);
    void update_sector(angle_t reference_angle);
    void update_virtual_reference();

    // The type of reference angle provided to the update method.
    reference_type _ref_type;

    // Array of six individual pulse generators, one for each thyristor
    std::array<pulse_generator, 6> _pulse_generators;

    pulses _pulses;
    pulse_sector_t _current_sector{pulse_sector_t::none};
    bool _sector_has_changed{false};

    // Virtual reference generator state
    angle_t _virtual_reference_angle{0.0};
    frequency_t _virtual_reference_frequency{50.0};
    duration_t _virtual_reference_sampling_time{us(100.0)};
    bool _virtual_reference_enabled{false};

    // Pulse masking state
    pulse_mask _pulse_mask{pulse_mask::all_enabled()};
    bool _pulse_mask_enabled{false};

    // Default firing offsets for the 6-pulse bridge topology
    // These are spaced 60 degrees apart following the standard firing sequence
    static constexpr std::array<double, 6> firing_offsets_deg = {
        0.0,    // P1: 0°
        60.0,   // P2: 60°
        120.0,  // P3: 120°
        180.0,  // P4: 180°
        240.0,  // P5: 240°
        300.0   // P6: 300°
    };
};

}  // namespace yeet::control
