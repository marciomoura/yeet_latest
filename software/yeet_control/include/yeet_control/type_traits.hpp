#pragma once

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

}  // namespace yeet::control