#pragma once

namespace yeet::control {

// implement an edge detection class to detect rising edge of a boolean input signal
class edge_detector {
public:
    /**
     * @brief Constructor for the edge detector.
     */
    edge_detector() = default;

    /**
     * @brief Updates the edge detector with the latest input value.
     *
     * @param input The current boolean input value.
     * @return true if a rising edge was detected, false otherwise.
     */
    bool update(bool input);

    /**
     * @brief Checks if a rising edge was detected in the last update.
     * @return true if a rising edge was detected, false otherwise.
     */
    bool is_rising_edge() const { return _rising_edge; }

    /**
     * @brief Conversion operator to bool.
     * @return true if a rising edge was detected, false otherwise.
     */
    operator bool() const { return _rising_edge; }

private:
    bool _rising_edge{false};     ///< Indicates if a rising edge was detected.
    bool _previous_input{false};  ///< Stores the previous input value to detect edges.
};

}  // namespace yeet::control