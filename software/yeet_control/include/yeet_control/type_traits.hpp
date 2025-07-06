#pragma once

#include <type_traits>

namespace yeet::control {

// Type trait to get the fundamental type
template <typename T>
struct fundamental_type {
    using type = T;
};

// Forward declare abc
template <typename T>
class abc;

// Forward declare dq
template <typename T>
class dq;

// Forward declare alphabeta
template <typename T>
class alphabeta;

// Defined specialized fundamental types
template <typename T>
struct fundamental_type<abc<T>> {
    using type = T;
};

template <typename T>
struct fundamental_type<dq<T>> {
    using type = T;
};

template <typename T>
struct fundamental_type<alphabeta<T>> {
    using type = T;
};

// Helper alias
template <typename T>
using fundamental_type_t = typename fundamental_type<T>::type;

// Type trait to check if a type is a coordinate frame (abc, dq, or alphabeta)
template <typename T>
struct is_coordinate_frame : std::false_type {};

template <typename T>
struct is_coordinate_frame<abc<T>> : std::true_type {};

template <typename T>
struct is_coordinate_frame<dq<T>> : std::true_type {};

template <typename T>
struct is_coordinate_frame<alphabeta<T>> : std::true_type {};

// Helper variable template
template <typename T>
inline constexpr bool is_coordinate_frame_v = is_coordinate_frame<T>::value;

}  // namespace yeet::control