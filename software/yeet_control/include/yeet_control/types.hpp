#pragma once

namespace yeet {

//==============================================================================
// Compile-Time Precision Control
//==============================================================================

/**
 * @brief Defines the default floating-point type for quantities.
 *
 * To use single precision (float), define YEET_PRECISION_FLOAT before including this header.
 * Otherwise, it defaults to double precision.
 *
 * Example for single precision:
 * #define YEET_PRECISION_FLOAT
 * #include "yeet_control/units.hpp"
 */
#if defined(YEET_PRECISION_DOUBLE)
using real_t = double;
#else
using real_t = float;
#endif

}  // namespace yeet