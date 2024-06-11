#pragma once

#include "details.h"
#include "variant_utility.h"

#include <compare>
#include <concepts>
#include <exception>
#include <functional>
#include <string>
#include <type_traits>
#include <utility>


struct monostate {};

constexpr bool operator==(monostate, monostate) noexcept {
  return true;
}

constexpr std::strong_ordering operator<=>(monostate, monostate) noexcept {
  return std::strong_ordering::equal;
}

inline constexpr std::size_t variant_npos = -1;

template <typename... Types>
class variant;

class bad_variant_access : public std::exception {
  template <typename... Types>
  friend class variant;

  template <class R, class Visitor, class... Variants>
  friend constexpr R visit(Visitor&& vis, Variants&&... vars);

private:
  const char* message = "";

  explicit bad_variant_access(const char* message) noexcept : message(message) {}

public:
  using std::exception::exception;

  const char* what() const noexcept override {
    return message;
  }
};

namespace details {
template <class... Variants, class... Indices, class Visitor>
constexpr decltype(auto) indexed_visit(const Visitor& vis, Indices... indices);
} // namespace details

template <std::size_t I, class T>
struct variant_alternative;

template <std::size_t I, class... Types>
struct variant_alternative<I, variant<Types...>> {
  using type = details::nth_type_t<I, Types...>;
};

template <std::size_t I, class T>
struct variant_alternative<I, const T> {
  using type = std::add_const_t<typename variant_alternative<I, T>::type>;
};

template <std::size_t I, class T>
using variant_alternative_t = typename variant_alternative<I, T>::type;

template <class T>
struct variant_size;

template <class... Types>
struct variant_size<variant<Types...>> : std::integral_constant<std::size_t, sizeof...(Types)> {};

template <class T>
struct variant_size<const T> : std::integral_constant<std::size_t, variant_size<T>::value> {};

template <class T>
inline constexpr std::size_t variant_size_v = variant_size<T>::value;

template <class Visitor, class... Variants>
constexpr decltype(auto) visit(Visitor&& vis, Variants&&... vars);

template <typename... Types>
class variant {
private:
  details::storage<Types...> _storage;
  std::size_t _index{};
  using T_0 = details::nth_type_t<0, Types...>;

  template <typename Visitor, typename T>
  static decltype(auto) invoke_visitor(Visitor&& vis, T&& arg) {
    return std::invoke(std::forward<Visitor>(vis), std::forward<T>(arg));
  }

  template <std::size_t I>
  constexpr details::nth_type_t<I, Types...>& get_impl() {
    if (I != index()) {
      throw bad_variant_access{"get: wrong index"};
    }
    return details::get<I>(_storage);
  }

  template <std::size_t I>
  constexpr const details::nth_type_t<I, Types...>& get_impl() const {
    if (I != index()) {
      throw bad_variant_access{"get: wrong index"};
    }
    return details::get<I>(_storage);
  }

  template <typename T>
  constexpr T& get_impl() {
    return get_impl<details::type_index_v<T, Types...>>();
  }

  template <typename T>
  constexpr const T& get_impl() const {
    return get_impl<details::type_index_v<T, Types...>>();
  }

  template <typename T>
  using converting_constructor_t = details::nth_type_t<
      decltype(details::overload_resolver<T, Types...>{}(std::forward<T>(std::declval<T>())))::value, Types...>;

  constexpr void become_valueless() noexcept {
    if (!valueless_by_exception()) {
      details::indexed_visit<variant>(
          [&](auto index) {
            constexpr std::size_t idx = decltype(index)::value;
            std::destroy_at(std::addressof(get<idx>(_storage)));
          },
          index());
      _index = variant_npos;
    }
  }

public:
  constexpr variant() noexcept(std::is_nothrow_default_constructible_v<T_0>)
    requires(std::is_default_constructible_v<T_0>)
  {
    details::construct<0>(_storage);
  }

  constexpr variant(const variant& other)
    requires((std::is_copy_constructible_v<Types> && ...) && !(std::is_trivially_copy_constructible_v<Types> && ...))
      : _index(other.index()) {
    if (!other.valueless_by_exception()) {
      details::indexed_visit<variant>(
          [&](auto index) {
            constexpr std::size_t idx = decltype(index)::value;
            std::construct_at(std::addressof(details::get<idx>(_storage)), details::get<idx>(other._storage));
          },
          other.index());
    }
  }

  constexpr variant(const variant& other)
    requires((std::is_copy_constructible_v<Types> && ...) && (std::is_trivially_copy_constructible_v<Types> && ...))
  = default;

  constexpr variant(variant&& other) noexcept((std::is_nothrow_move_constructible_v<Types> && ...))
    requires((std::is_move_constructible_v<Types> && ...) && !(std::is_trivially_move_constructible_v<Types> && ...))
      : _index(other.index()) {
    if (!other.valueless_by_exception()) {
      details::indexed_visit<variant>(
          [&](auto index) {
            constexpr std::size_t idx = decltype(index)::value;
            std::construct_at(std::addressof(details::get<idx>(_storage)),
                              std::move(details::get<idx>(other._storage)));
          },
          other.index());
    }
  }

  constexpr variant(variant&& other) noexcept((std::is_nothrow_move_constructible_v<Types> && ...))
    requires((std::is_move_constructible_v<Types> && ...) && (std::is_trivially_move_constructible_v<Types> && ...))
  = default;

  template <class T>
  constexpr variant(T&& t) noexcept(std::is_nothrow_constructible_v<converting_constructor_t<T>, T>)
    requires(details::is_converting<variant, std::remove_cvref_t<T>> &&
             std::is_constructible_v<converting_constructor_t<T>, T>)
      : _index(details::type_index_v<converting_constructor_t<T>, Types...>) {
    details::construct<details::type_index_v<converting_constructor_t<T>, Types...>>(_storage, std::forward<T>(t));
  }

  template <class T, class... Args>
  constexpr explicit variant(in_place_type_t<T>, Args&&... args)
    requires(details::type_once_v<T, Types...> && std::is_constructible_v<T, Args...>)
      : variant(in_place_index<details::type_index_v<T, Types...>>, std::forward<Args>(args)...) {}

  template <std::size_t I, class... Args>
  constexpr explicit variant(in_place_index_t<I>, Args&&... args)
    requires(I < sizeof...(Types) && std::is_constructible_v<details::nth_type_t<I, Types...>, Args...>)
      : _index(variant_npos) {
    emplace<I>(std::forward<Args>(args)...);
  }

  constexpr variant& operator=(const variant& rhs)
    requires((std::is_copy_constructible_v<Types> && std::is_copy_assignable_v<Types>) && ... &&
             !((std::is_trivially_copy_constructible_v<Types> && std::is_trivially_copy_assignable_v<Types> &&
                std::is_trivially_destructible_v<Types>) &&
               ...))
  {
    if (rhs.valueless_by_exception()) {
      become_valueless();
    } else if (rhs.index() == index()) {
      details::indexed_visit<variant>(
          [&](auto index) {
            constexpr std::size_t idx = decltype(index)::value;
            get<idx>(_storage) = get<idx>(rhs._storage);
          },
          rhs.index());
    } else if (!details::indexed_visit<variant>(
                   [&](auto index) {
                     constexpr std::size_t idx = decltype(index)::value;
                     using T = details::nth_type_t<idx, Types...>;
                     if constexpr (std::is_nothrow_copy_constructible_v<T> ||
                                   !std::is_nothrow_move_constructible_v<T>) {
                       this->emplace<idx>(*get_if<idx>(std::addressof(rhs)));
                       return true;
                     }
                     return false;
                   },
                   rhs.index())) {
      this->operator=(variant(rhs));
    }
    return *this;
  }

  constexpr variant& operator=(const variant& rhs)
    requires((std::is_copy_constructible_v<Types> && std::is_copy_assignable_v<Types>) && ... &&
             ((std::is_trivially_copy_constructible_v<Types> && std::is_trivially_copy_assignable_v<Types> &&
               std::is_trivially_destructible_v<Types>) &&
              ...))
  = default;

  constexpr variant& operator=(variant&& rhs) noexcept(
      ((std::is_nothrow_move_constructible_v<Types> && std::is_nothrow_move_assignable_v<Types>)&&...))
    requires(((std::is_move_constructible_v<Types> && std::is_move_assignable_v<Types>) && ...) &&
             !((std::is_trivially_move_constructible_v<Types> && std::is_trivially_move_assignable_v<Types> &&
                std::is_trivially_destructible_v<Types>) &&
               ...))
  {
    if (rhs.valueless_by_exception()) {
      become_valueless();
    } else if (rhs.index() == index()) {
      details::indexed_visit<variant>(
          [&](auto index) {
            constexpr std::size_t idx = decltype(index)::value;
            get<idx>(_storage) = std::move(get<idx>(rhs._storage));
          },
          rhs.index());
    } else {
      details::indexed_visit<variant>(
          [&](auto index) {
            constexpr std::size_t idx = decltype(index)::value;
            this->emplace<idx>(std::move(*get_if<idx>(std::addressof(rhs))));
          },
          rhs.index());
    }
    return *this;
  }

  constexpr variant& operator=(variant&& rhs) noexcept(
      ((std::is_nothrow_move_constructible_v<Types> && std::is_nothrow_move_assignable_v<Types>)&&...))
    requires(((std::is_move_constructible_v<Types> && std::is_move_assignable_v<Types>) && ...) &&
             ((std::is_trivially_move_constructible_v<Types> && std::is_trivially_move_assignable_v<Types> &&
               std::is_trivially_destructible_v<Types>) &&
              ...))
  = default;

  template <class T>
  constexpr variant& operator=(T&& t) noexcept(std::is_nothrow_assignable_v<converting_constructor_t<T>&, T> &&
                                               std::is_nothrow_constructible_v<converting_constructor_t<T>, T>)
    requires(!std::is_same_v<variant, std::remove_cvref_t<T>> &&
             std::is_assignable_v<converting_constructor_t<T>&, T> &&
             std::is_constructible_v<converting_constructor_t<T>, T>)
  {
    using T_j = converting_constructor_t<T>;
    if (holds_alternative<T_j>(*this)) {
      get<T_j>(*this) = std::forward<T>(t);
    } else if (std::is_nothrow_constructible_v<T_j, T> || !std::is_nothrow_move_constructible_v<T_j>) {
      this->emplace<T_j>(std::forward<T>(t));
    } else {
      this->emplace<T_j>(T_j(std::forward<T>(t)));
    }
    return *this;
  }

  constexpr void swap(variant& rhs) noexcept(
      ((std::is_nothrow_move_constructible_v<Types> && std::is_nothrow_swappable_v<Types>)&&...)) {
    if (rhs.valueless_by_exception() && valueless_by_exception()) {
      return;
    } else if (rhs.index() == index()) {
      details::indexed_visit<variant>(
          [&](auto index) {
            constexpr std::size_t idx = decltype(index)::value;
            using std::swap;
            swap(*get_if<idx>(this), *get_if<idx>(std::addressof(rhs)));
          },
          index());
    } else {
      auto tmp = std::move(rhs);
      rhs = std::move(*this);
      *this = std::move(tmp);
    }
  }

  constexpr std::size_t index() const noexcept {
    return _index;
  }

  constexpr bool valueless_by_exception() const noexcept {
    return index() == variant_npos;
  }

  template <std::size_t I, class... Ts>
  friend constexpr variant_alternative_t<I, variant<Ts...>>& get(variant<Ts...>& v);

  template <std::size_t I, class... Ts>
  friend constexpr variant_alternative_t<I, variant<Ts...>>&& get(variant<Ts...>&& v);

  template <std::size_t I, class... Ts>
  friend constexpr const variant_alternative_t<I, variant<Ts...>>& get(const variant<Ts...>& v);

  template <std::size_t I, class... Ts>
  friend constexpr const variant_alternative_t<I, variant<Ts...>>&& get(const variant<Ts...>&& v);

  template <class T, class... Ts>
  friend constexpr T& get(variant<Ts...>& v)
    requires(details::type_once_v<T, Ts...>);

  template <class T, class... Ts>
  friend constexpr T&& get(variant<Ts...>&& v)
    requires(details::type_once_v<T, Ts...>);

  template <class T, class... Ts>
  friend constexpr const T& get(const variant<Ts...>& v)
    requires(details::type_once_v<T, Ts...>);

  template <class T, class... Ts>
  friend constexpr const T&& get(const variant<Ts...>&& v)
    requires(details::type_once_v<T, Ts...>);

  template <std::size_t I, class... Ts>
  friend constexpr std::add_pointer_t<variant_alternative_t<I, variant<Ts...>>> get_if(variant<Ts...>* pv) noexcept;

  template <std::size_t I, class... Ts>
  friend constexpr std::add_pointer_t<const variant_alternative_t<I, variant<Ts...>>> get_if(
      const variant<Ts...>* pv) noexcept;

  template <class T, class... Ts>
  friend constexpr std::add_pointer_t<T> get_if(variant<Ts...>* pv) noexcept
    requires(details::type_once_v<T, Ts...>);

  template <class T, class... Ts>
  friend constexpr std::add_pointer_t<const T> get_if(const variant<Ts...>* pv) noexcept
    requires(details::type_once_v<T, Ts...>);

  constexpr ~variant()
    requires(details::trivially_destructible<Types> && ...)
  = default;

  constexpr ~variant() {
    become_valueless();
  }

  template <class T, class... Args>
  constexpr T& emplace(Args&&... args)
    requires(details::type_once_v<T, Types...>)
  {
    return emplace<details::type_index_v<T, Types...>>(std::forward<Args>(args)...);
  }

  template <std::size_t I, class... Args>
  constexpr variant_alternative_t<I, variant>& emplace(Args&&... args) {
    become_valueless();
    auto& result = *details::construct<I>(_storage, std::forward<Args>(args)...);
    _index = I;
    return result;
  }
};

template <std::size_t I, class... Ts>
constexpr variant_alternative_t<I, variant<Ts...>>& get(variant<Ts...>& v) {
  return v.template get_impl<I>();
}

template <std::size_t I, class... Ts>
constexpr variant_alternative_t<I, variant<Ts...>>&& get(variant<Ts...>&& v) {
  return std::move(v.template get_impl<I>());
}

template <std::size_t I, class... Ts>
constexpr const variant_alternative_t<I, variant<Ts...>>& get(const variant<Ts...>& v) {
  return v.template get_impl<I>();
}

template <std::size_t I, class... Ts>
constexpr const variant_alternative_t<I, variant<Ts...>>&& get(const variant<Ts...>&& v) {
  return std::move(v.template get_impl<I>());
}

template <class T, class... Ts>
constexpr T& get(variant<Ts...>& v)
  requires(details::type_once_v<T, Ts...>)
{
  return v.template get_impl<T>();
}

template <class T, class... Ts>
constexpr T&& get(variant<Ts...>&& v)
  requires(details::type_once_v<T, Ts...>)
{
  return std::move(v.template get_impl<T>());
}

template <class T, class... Ts>
constexpr const T& get(const variant<Ts...>& v)
  requires(details::type_once_v<T, Ts...>)
{
  return v.template get_impl<T>();
}

template <class T, class... Ts>
constexpr const T&& get(const variant<Ts...>&& v)
  requires(details::type_once_v<T, Ts...>)
{
  return std::move(v.template get_impl<T>());
}

template <std::size_t I, class... Ts>
constexpr std::add_pointer_t<variant_alternative_t<I, variant<Ts...>>> get_if(variant<Ts...>* pv) noexcept {
  if (pv == nullptr || I != pv->index()) {
    return nullptr;
  }
  return std::addressof(details::get<I>(pv->_storage));
}

template <std::size_t I, class... Ts>
constexpr std::add_pointer_t<const variant_alternative_t<I, variant<Ts...>>> get_if(const variant<Ts...>* pv) noexcept {
  if (pv == nullptr || I != pv->index()) {
    return nullptr;
  }
  return std::addressof(details::get<I>(pv->_storage));
}

template <class T, class... Ts>
constexpr std::add_pointer_t<T> get_if(variant<Ts...>* pv) noexcept
  requires(details::type_once_v<T, Ts...>)
{
  return get_if<details::type_index_v<T, Ts...>>(pv);
}

template <class T, class... Ts>
constexpr std::add_pointer_t<const T> get_if(const variant<Ts...>* pv) noexcept {
  return get_if<details::type_index_v<T, Ts...>>(pv);
}

namespace details {

template <typename R, typename Visitor, std::size_t... Dims>
struct visit_context {
  static constexpr std::size_t dimensions = sizeof...(Dims);

  using function_t = R (*)(const Visitor&);

  template <std::size_t... Is>
  static constexpr R invoke(const Visitor& visitor) {
    return visitor(index_t<Is>{}...);
  }

  template <std::size_t...>
  struct visitor_table {
    function_t f = nullptr;

    constexpr function_t get() const {
      return f;
    }

    constexpr void set(function_t f) {
      this->f = f;
    }

    static constexpr std::size_t size(std::size_t) {
      return 0;
    }
  };

  template <std::size_t I, std::size_t... Is>
  struct visitor_table<I, Is...> {
    using inner = visitor_table<Is...>;

    std::array<inner, I> arr;

    template <typename... Indices>
    constexpr function_t get(std::size_t index, Indices... indices) const {
      return arr[index].get(indices...);
    }

    template <typename... Indices>
    constexpr void set(function_t f, std::size_t index, Indices... indices) {
      return arr[index].set(f, indices...);
    }

    static constexpr std::size_t size(std::size_t n) {
      return n == 1 ? I : inner::size(n - 1);
    }
  };

  using table_t = visitor_table<Dims...>;

  template <std::size_t I = 0, std::size_t D = dimensions, std::size_t... Is>
  static constexpr void fill_table(table_t& table) {
    if constexpr (D == 0) {
      table.set(invoke<Is...>, Is...);
    } else {
      fill_table<I, D - 1, Is..., I>(table);
      if constexpr (I + 1 < table_t::size(D)) {
        fill_table<I + 1, D, Is...>(table);
      }
    }
  }

  static constexpr table_t create_table() {
    table_t table;
    fill_table(table);
    return table;
  }

  static constexpr auto table = create_table();
};

template <class R, class... Variants, class... Indices, class Visitor>
constexpr decltype(auto) visit_impl(const Visitor& vis, Indices... indices) {
  return visit_context<R, Visitor, variant_size_v<std::remove_cvref_t<Variants>>...>::table.get(indices...)(vis);
}

template <class... Variants, class... Indices, class Visitor>
constexpr decltype(auto) indexed_visit(const Visitor& vis, Indices... indices) {
  using return_t =
      decltype(std::declval<const Visitor&>()(index_t<variant_size_v<std::remove_cvref_t<Variants>> - 1>{}...));
  return visit_impl<return_t, Variants...>(vis, indices...);
}

template <template <typename> typename Compare, typename... Types>
constexpr bool compare_impl(const variant<Types...>& v, const variant<Types...>& w) {
  if (v.valueless_by_exception() || w.valueless_by_exception()) {
    return Compare<bool>{}(!v.valueless_by_exception(), !w.valueless_by_exception());
  }
  if (v.index() != w.index()) {
    return Compare<bool>{}(v.index(), w.index());
  }
  return details::indexed_visit<variant<Types...>>(
      [&](auto index) {
        constexpr std::size_t idx = decltype(index)::value;
        using T = details::nth_type_t<idx, Types...>;
        return Compare<T>{}(*get_if<idx>(std::addressof(v)), *get_if<idx>(std::addressof(w)));
      },
      v.index());
}

} // namespace details

template <class R, class Visitor, class... Variants>
constexpr R visit(Visitor&& vis, Variants&&... vars) {
  if ((vars.valueless_by_exception() || ...)) {
    throw bad_variant_access{"visit: valueless variant"};
  }
  return details::visit_impl<R, Variants...>(
      [&](auto... indices) -> R {
        if constexpr (std::is_void_v<R>) {
          std::forward<Visitor>(vis)(get<decltype(indices)::value>(std::forward<Variants>(vars))...);
        } else {
          return std::forward<Visitor>(vis)(get<decltype(indices)::value>(std::forward<Variants>(vars))...);
        }
      },
      vars.index()...);
}

template <class Visitor, class... Variants>
constexpr decltype(auto) visit(Visitor&& vis, Variants&&... vars) {
  using return_t = std::invoke_result_t<Visitor, decltype(get<0>(std::forward<Variants>(vars)))...>;
  return visit<return_t>(std::forward<Visitor>(vis), std::forward<Variants>(vars)...);
}

template <class T, class... Types>
constexpr bool holds_alternative(const variant<Types...>& v) noexcept
  requires(details::type_once_v<T, Types...>)
{
  return v.index() == details::type_index_v<T, Types...>;
}

template <class... Types>
constexpr bool operator==(const variant<Types...>& v, const variant<Types...>& w)
  requires(details::comparable_using<std::equal_to, Types> && ...)
{
  return details::compare_impl<std::equal_to>(v, w);
}

template <class... Types>
constexpr bool operator!=(const variant<Types...>& v, const variant<Types...>& w)
  requires(details::comparable_using<std::not_equal_to, Types> && ...)
{
  return details::compare_impl<std::not_equal_to>(v, w);
}

template <class... Types>
constexpr bool operator<(const variant<Types...>& v, const variant<Types...>& w)
  requires(details::comparable_using<std::less, Types> && ...)
{
  return details::compare_impl<std::less>(v, w);
}

template <class... Types>
constexpr bool operator>(const variant<Types...>& v, const variant<Types...>& w)
  requires(details::comparable_using<std::greater, Types> && ...)
{
  return details::compare_impl<std::greater>(v, w);
}

template <class... Types>
constexpr bool operator<=(const variant<Types...>& v, const variant<Types...>& w)
  requires(details::comparable_using<std::less_equal, Types> && ...)
{
  return details::compare_impl<std::less_equal>(v, w);
}

template <class... Types>
constexpr bool operator>=(const variant<Types...>& v, const variant<Types...>& w)
  requires(details::comparable_using<std::greater_equal, Types> && ...)
{
  return details::compare_impl<std::greater_equal>(v, w);
}

template <class... Types>
constexpr std::common_comparison_category_t<std::compare_three_way_result_t<Types>...> operator<=>(
    const variant<Types...>& v, const variant<Types...>& w) {
  if (v.valueless_by_exception() && w.valueless_by_exception()) {
    return std::strong_ordering::equal;
  }
  if (v.valueless_by_exception()) {
    return std::strong_ordering::less;
  }
  if (v.index() != w.index()) {
    return v.index() <=> w.index();
  }
  return details::indexed_visit<variant<Types...>>(
      [&](auto index) {
        constexpr std::size_t idx = decltype(index)::value;
        return *get_if<idx>(std::addressof(v)) <=> *get_if<idx>(std::addressof(w));
      },
      v.index());
}
