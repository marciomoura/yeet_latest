#pragma once

#include <string>

#ifndef YEET_BUILD_EMBEDDED
#include <cpptrace/cpptrace.hpp>
#endif

namespace yeet::utils {

#ifndef YEET_BUILD_EMBEDDED

inline std::string get_stacktrace() { return cpptrace::generate_trace().to_string(); }

inline void print_stacktrace() { cpptrace::generate_trace().print(); }

inline void print_stacktrace_on_signal() { cpptrace::register_terminate_handler(); }

#else

inline std::string get_stacktrace() { return "[stacktrace not available on embedded]"; }

inline void print_stacktrace() {}

inline void print_stacktrace_on_signal() {}

#endif

}  // namespace yeet::utils
