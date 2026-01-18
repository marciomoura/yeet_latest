#include "yeet_control/edge_detector.hpp"

namespace yeet::control {

bool edge_detector::update(bool input)
{
    _rising_edge = !_previous_input && input;
    _falling_edge = _previous_input && !input;
    _previous_input = input;
    return _rising_edge;
}

void edge_detector::reset()
{
    _rising_edge = false;
    _falling_edge = false;
    _previous_input = false;
}

}  // namespace yeet::control