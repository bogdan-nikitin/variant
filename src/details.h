#pragma once

#include "variant_utility.h"

#include <cstddef>
#include <memory>
#include <type_traits>
#include <utility>

namespace details {

template <typename T>
using array_t = T[1];

template <typename From, typename To>
concept narrowing = requires(From&& arg) { array_t<To>{std::forward<From>(arg)}; };

template <size_t N>
using index_t = std::integral_constant<size_t, N>;

template <typename T>
concept copy_constructible = std::is_copy_constructible_v<T>;

template <typename T>
concept copy_assignable = std::is_copy_assignable_v<T>;

template <typename T>
concept move_assignable = std::is_move_assignable_v<T>;

template <typename T>
concept move_constructible = std::is_move_constructible_v<T>;

template <typename T>
concept trivially_copy_constructible = std::is_trivially_copy_constructible_v<T>;

template <typename T>
concept trivially_copy_assignable = std::is_trivially_copy_assignable_v<T>;

template <typename T>
concept trivially_move_constructible = std::is_trivially_move_constructible_v<T>;

template <typename T>
concept trivially_move_assignable = std::is_trivially_move_assignable_v<T>;

template <typename T>
concept trivially_destructible = std::is_trivially_destructible_v<T>;

struct no_type {};

template <size_t, typename...>
struct nth_type {
  using type = no_type;
};

template <size_t N, typename T, typename... Types>
struct nth_type<N, T, Types...> {
  using type = nth_type<N - 1, Types...>::type;
};

template <typename T, typename... Types>
struct nth_type<0, T, Types...> {
  using type = T;
};

template <size_t N, typename... Types>
using nth_type_t = nth_type<N, Types...>::type;

template <typename T, typename... Ts>
inline constexpr size_t type_index_v = 1;

template <typename T, typename U, typename... Ts>
inline constexpr size_t type_index_v<T, U, Ts...> = 1 + type_index_v<T, Ts...>;

template <typename T, typename... Ts>
inline constexpr size_t type_index_v<T, T, Ts...> = 0;

template <typename T, typename... Ts>
inline constexpr bool contains_type_t = type_index_v<T, Ts...> < sizeof...(Ts);

template <typename T, typename... Ts>
inline constexpr bool unique_type_t = true;

template <typename T, typename... Ts>
inline constexpr bool unique_type_t<T, T, Ts...> = !contains_type_t<T, Ts...>;

template <typename T, typename U, typename... Ts>
inline constexpr bool unique_type_t<T, U, Ts...> = unique_type_t<T, Ts...>;

template <typename T, typename... Ts>
inline constexpr bool type_once_v = contains_type_t<T, Ts...> && unique_type_t<T, Ts...>;

template <typename...>
union storage {};

template <typename T, typename... Types>
union storage<T, Types...> {
  storage<Types...> stor;
  T value;

  constexpr storage() {}

  constexpr ~storage() {}

  constexpr ~storage()
    requires(std::is_trivially_destructible_v<T> && (std::is_trivially_destructible_v<Types> && ...))
  = default;
};

template <size_t N, typename... Types>
constexpr nth_type_t<N, Types...>& get(storage<Types...>& stor) {
  if constexpr (N == 0) {
    return stor.value;
  } else {
    return get<N - 1>(stor.stor);
  }
}

template <size_t N, typename... Types>
constexpr const nth_type_t<N, Types...>& get(const storage<Types...>& stor) {
  if constexpr (N == 0) {
    return stor.value;
  } else {
    return get<N - 1>(stor.stor);
  }
}

template <size_t N, typename... Types, typename... Args>
constexpr decltype(auto) construct(storage<Types...>& stor, Args&&... args) {
  if constexpr (N == 0) {
    return std::construct_at(std::addressof(stor.value), std::forward<Args>(args)...);
  } else {
    std::construct_at(std::addressof(stor.stor));
    return construct<N - 1>(stor.stor, std::forward<Args>(args)...);
  }
}

template <typename From, typename To, size_t I>
struct overload_resolver_base {
  constexpr index_t<I> operator()(To) const
    requires(narrowing<From, To>)
  {
    return {};
  }
};

template <typename From, typename... Ts>
struct overload_resolver : overload_resolver_base<From, Ts, type_index_v<Ts, Ts...>>... {
  using overload_resolver_base<From, Ts, type_index_v<Ts, Ts...>>::operator()...;
};

template <typename>
inline constexpr bool is_in_place_type = false;

template <typename T>
inline constexpr bool is_in_place_type<in_place_type_t<T>> = true;

template <typename>
inline constexpr bool is_in_place_index = false;

template <std::size_t I>
inline constexpr bool is_in_place_index<in_place_index_t<I>> = true;

template <typename Variant, typename T>
concept is_converting = !std::is_same_v<Variant, T> && !is_in_place_type<T> && !is_in_place_index<T>;

template <template <typename> typename Compare, typename T>
concept comparable_using = requires(const T& a, const T& b) { Compare<T>{}(a, b); };

} // namespace details
