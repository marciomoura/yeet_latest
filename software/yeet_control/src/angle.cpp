#include "yeet_control/angle.hpp"

namespace yeet::control {

// Explicit instantiations
template class angle<minus_pi_to_pi>;
template class angle<zero_to_two_pi>;

}  // namespace yeet::control