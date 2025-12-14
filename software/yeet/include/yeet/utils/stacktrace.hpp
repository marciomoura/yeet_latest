#pragma once

#include <string>

// NOTE: YEET_BUILD_EMBEDDED is a CMake option, not inherently a preprocessor macro.
// Embedded targets in this repo define YEET_EMBEDDED_TARGET; we treat either as embedded.
#if !defined(YEET_EMBEDDED_TARGET) && !defined(YEET_BUILD_EMBEDDED)
#if defined(__has_include)
#if __has_include(<cpptrace/cpptrace.hpp>)
#define YEET_HAS_CPPTRACE 1
#include <cpptrace/cpptrace.hpp>
#else
#define YEET_HAS_CPPTRACE 0
#endif
#else
#define YEET_HAS_CPPTRACE 0
#endif
#else
#define YEET_HAS_CPPTRACE 0
#endif

namespace yeet::utils {

#if YEET_HAS_CPPTRACE

inline std::string get_stacktrace() { return cpptrace::generate_trace().to_string(); }

inline void print_stacktrace() { cpptrace::generate_trace().print(); }

inline void print_stacktrace_on_signal() { cpptrace::register_terminate_handler(); }

#else

inline std::string get_stacktrace() { return "[stacktrace not available on embedded]"; }

inline void print_stacktrace() {}

inline void print_stacktrace_on_signal() {}

#endif

}  // namespace yeet::utils
