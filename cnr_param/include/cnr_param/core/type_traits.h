#ifndef CNR_PARAM__INCLUDE__CNR_PARAM__UTILS__TYPE_TRAITS__H
#define CNR_PARAM__INCLUDE__CNR_PARAM__UTILS__TYPE_TRAITS__H

#include <cstddef>
#include <stdexcept>
#include <Eigen/Core>
#include <boost/type_index.hpp>
#include <type_traits>
#include <vector>
#include <string>
#include <array>
#include <variant>
#include <optional>
#include <cstdint>
#include <Eigen/Core>

#if !defined(__PRETTY_FUNCTION__) && !defined(__GNUC__)
#define __PRETTY_FUNCTION__ __FUNCSIG__
#endif

#if !defined(UNUSED)
#define UNUSED(expr)                                                                                                   \
  do                                                                                                                   \
  {                                                                                                                    \
    (void)(expr);                                                                                                      \
  } while (0)
#endif

namespace cnr
{
namespace param
{
// vector -------------------------------------------------------------------------
template <typename C>
struct is_std_vector : std::false_type
{
  using base = std::decay_t<C>;
};

template <typename C, typename A>
struct is_std_vector<std::vector<C, A>> : std::true_type
{
  using base = std::decay_t<C>;
};

template <typename Derived>
struct is_eigen_matrix : std::is_base_of<Eigen::MatrixBase<std::decay_t<Derived>>, std::decay_t<Derived>>
{
  using base = std::decay_t<Derived>;
};

template <typename D, typename T = void>
struct is_eigen_vector
{
  static const bool value = false;
  using base = std::decay_t<D>;
};

template <typename D>
struct is_eigen_vector<D, typename std::enable_if<is_eigen_matrix<D>::value>::type>
{
  static const bool value = std::decay_t<D>::IsVectorAtCompileTime;
  using base = std::decay_t<D>;
};

template <typename D, typename T = void>
struct extract_base_type_from_eigen_vector
{
  using type = std::decay_t<D>;
};

template <typename D>
struct extract_base_type_from_eigen_vector<D, typename std::enable_if<is_eigen_matrix<D>::value>::type>
{
  using type = std::decay_t<D>::value_type;
};
// --------------------------------------------------------------------------------

// array --------------------------------------------------------------------------
template <typename C>
struct is_array : std::false_type
{
  using base = std::decay_t<C>;
};
template <typename C, int A>
struct is_array<std::array<C, A>> : std::true_type
{
  using base = std::decay_t<C>;
};
// --------------------------------------------------------------------------------

// bool ---------------------------------------------------------------------------
template <typename T>
struct is_bool : std::false_type
{
};
template <>
struct is_bool<bool> : std::true_type
{
};
template <class T>
struct is_bool_v
{
  using B = typename is_std_vector<T>::base;
  static constexpr bool value = is_std_vector<T>::value && is_bool<B>::value;
};
// --------------------------------------------------------------------------------

// byte ---------------------------------------------------------------------------
template <typename T>
struct is_byte : std::false_type
{
};
template <>
struct is_byte<uint8_t> : std::true_type
{
};
template <>
struct is_byte<uint16_t> : std::true_type
{
};
template <>
struct is_byte<uint32_t> : std::true_type
{
};
template <>
struct is_byte<uint64_t> : std::true_type
{
};
template <class T>
struct is_byte_v
{
  using B = typename is_std_vector<T>::base;
  static constexpr bool value = is_std_vector<T>::value && is_byte<B>::value;
};
// --------------------------------------------------------------------------------

// double -------------------------------------------------------------------------
template <typename T>
struct is_double
{
  static constexpr bool value = ((!is_eigen_matrix<T>::value && std::is_floating_point<T>::value &&
                                  !is_std_vector<T>::value && !is_array<T>::value) ||
                                 (is_eigen_matrix<T>::value && std::is_floating_point<T>::value &&
                                  !is_std_vector<T>::value && !is_array<T>::value));
};

template <class T>
struct is_double_v
{
  using B = typename std::conditional<is_eigen_matrix<T>::value, typename is_eigen_matrix<T>::base::value_type,
                                      typename is_std_vector<T>::base>::type;
  static constexpr bool value = is_std_vector<T>::value && is_double<B>::value;
};

template <class T>
struct is_double_std_v
{
  using B = typename is_std_vector<T>::base;
  static constexpr bool value = is_std_vector<T>::value && is_double<B>::value;
};

// int -------------------------------------------------------------------------
template <typename T>
struct is_integer
{
  static constexpr bool value = std::is_integral<T>::value && std::is_signed<T>::value &&
                                !std::is_floating_point<T>::value && !is_std_vector<T>::value && !is_array<T>::value;
};
template <class T>
struct is_integer_v
{
  using B = typename is_std_vector<T>::base;
  static constexpr bool value = is_std_vector<T>::value && is_integer<B>::value;
};

// unsigned int -------------------------------------------------------------------------
template <typename T>
struct is_unsigned_integer
{
  static constexpr bool value = std::is_integral<T>::value && !std::is_signed<T>::value && !is_bool<T>::value &&
                                !is_byte<T>::value && !is_std_vector<T>::value && !is_array<T>::value;
};
template <class T>
struct is_unsigned_integer_v
{
  using B = typename is_std_vector<T>::base;
  static constexpr bool value = is_std_vector<T>::value && is_unsigned_integer<B>::value;
};

// char -------------------------------------------------------------------------
template <typename T>
struct is_char
{
  static constexpr bool value = std::is_same<T, char>::value;
};
template <class T>
struct is_char_v
{
  using B = typename is_std_vector<T>::base;
  static constexpr bool value = is_std_vector<T>::value && is_char<B>::value;
};

// char -------------------------------------------------------------------------
template <typename T>
struct is_unsigned_char
{
  static constexpr bool value = std::is_same<T, unsigned char>::value;
};
template <class T>
struct is_unsigned_char_v
{
  using B = typename is_std_vector<T>::base;
  static constexpr bool value = is_std_vector<T>::value && is_unsigned_char<B>::value;
};
// string -------------------------------------------------------------------------
template <typename T>
struct is_string : std::false_type
{
};
template <>
struct is_string<std::string> : std::true_type
{
};
template <>
struct is_string<const char*> : std::true_type
{
};
template <class T>
struct is_string_v
{
  using B = typename is_std_vector<T>::base;
  static constexpr bool value = is_std_vector<T>::value && is_string<B>::value;
};
//---------------------------------------------------------------------------------

// scalar -------------------------------------------------------------------------
template <typename T>
struct is_scalar
{
  using type = T;
  static constexpr bool value = is_bool<T>::value || is_byte<T>::value || is_double<T>::value || is_string<T>::value ||
                                is_integer<T>::value || is_unsigned_integer<T>::value;
};

// sequence -------------------------------------------------------------------------
template <typename T>
struct is_vector
{
  static constexpr bool value = is_std_vector<T>::value || is_eigen_vector<T>::value;
};

// map
template <typename T, typename U = void>
struct is_map : std::false_type
{
};

template <typename T>
struct is_map<T, std::void_t<typename T::key_type, typename T::mapped_type,
                             decltype(std::declval<T&>()[std::declval<const typename T::key_type&>()])>>
  : std::true_type
{
  using key_type = typename T::key_type;
  using mapped_type = typename T::mapped_type;
};

// map
template <typename T, typename = void>
struct is_string_to_scalar_map
{
  static constexpr bool value = false;
};
// map
template <typename T>
struct is_string_to_scalar_map<T, typename std::enable_if<is_map<T>::value, bool>::type>
{
  static constexpr bool value =
      is_string<typename is_map<T>::key_type>::value && is_scalar<typename is_map<T>::mapped_type>::value;
};

// ============================================
template <class... Ts>
inline std::type_info const& variant_held_type(std::variant<Ts...> const& v, std::optional<std::size_t> idx = {})
{
  if (!idx)
    idx = v.index();
  if (*idx == std::variant_npos)
    return typeid(void);
  const std::array<std::type_info const*, sizeof...(Ts)> infos[] = { &typeid(Ts)... };
  return *(infos[*idx]);
}

// Main lookup logic of looking up a type in a list.
template <typename T, typename... ALL_T>
struct is_one_of : public std::false_type
{
};

template <typename T, typename FRONT_T, typename... REST_T>
struct is_one_of<T, FRONT_T, REST_T...>
  : public std::conditional<std::is_same<T, FRONT_T>::value, std::true_type, is_one_of<T, REST_T...>>::type
{
};

// Convenience wrapper for std::variant<>.
template <typename T, typename VARIANT_T>
struct is_variant_member : public std::false_type
{
};

template <typename T, typename... ALL_T>
struct is_variant_member<T, std::variant<ALL_T...>> : public is_one_of<T, ALL_T...>
{
};

// helper
template <typename To, typename From>
struct are_compatible_std_vectors
{
  static constexpr bool value =
      (is_std_vector<std::decay_t<To>>::value && is_std_vector<std::decay_t<From>>::value &&
       std::is_convertible<typename is_std_vector<std::decay_t<To>>::base, typename is_std_vector<std::decay_t<From>>::base>::value);
};

template <typename To, typename From>
struct are_compatible_vectors
{
  static constexpr bool value =
      (are_compatible_std_vectors<To, From>::value) ||
      (is_eigen_vector<To>::value && is_std_vector<From>::value &&
       std::is_convertible<typename extract_base_type_from_eigen_vector<To>::type,
                           typename is_std_vector<From>::base>::value) ||
      (is_eigen_vector<From>::value && is_std_vector<To>::value &&
       std::is_convertible<typename extract_base_type_from_eigen_vector<To>::type, typename is_std_vector<To>::base>::value);
};

template <typename To, typename From, std::enable_if<are_compatible_std_vectors<std::decay_t<To>, std::decay_t<From>>::value, int>::type* = nullptr>
void container_cast(To& to, const From& from)
{
  to.clear();
  for(const auto & f : from)
    to.push_back(f);
}

template <typename To, typename From, std::enable_if<!are_compatible_std_vectors<std::decay_t<To>, std::decay_t<From>>::value && are_compatible_vectors<std::decay_t<To>, std::decay_t<From>>::value, bool>::type* = nullptr>
void container_cast(To& to, const From& from)
{
  if constexpr (is_std_vector<To>::value)
  {
    to = To(from.data(), from.data() + from.rows() * from.cols());
  }
  else
  {
    to = is_eigen_vector<To>::base::Map(&from[0], from.size());
  }
}

// helper type for the visitor #4
template <class... Ts>
struct overloaded : Ts...
{
  using Ts::operator()...;
};
// explicit deduction guide (not needed as of C++20)
template <class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

// =======================================================================================================
template <typename F, typename T>
inline typename std::enable_if<
    (std::is_same<F, T>::value || std::is_convertible<F, T>::value) && !is_eigen_matrix<T>::value, T>::type
implicit_cast(const F& rhs)
{
  return rhs;
}

template <typename F, typename T>
inline typename std::enable_if<is_eigen_matrix<T>::value && is_double_std_v<F>::value, T>::type
implicit_cast(const F& rhs)
{
  auto _rhs = rhs;
  return Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(_rhs.data(), _rhs.size());
}

template <typename F, typename T>
inline typename std::enable_if<
    !std::is_same<F, T>::value && is_std_vector<F>::value && is_std_vector<T>::value &&
        std::is_convertible<typename is_std_vector<F>::base, typename is_std_vector<T>::base>::value,
    T>::type
implicit_cast(const F& rhs)
{
  T lhs;
  for (const auto& v : rhs)
  {
    lhs.push_back(v);
  }
  return lhs;
}

template <typename F, typename T>
inline typename std::enable_if<
    !std::is_same<F, T>::value && !std::is_convertible<F, T>::value &&
        !(is_eigen_matrix<T>::value && is_double_std_v<F>::value) &&
        !(is_std_vector<F>::value && is_std_vector<T>::value &&
          std::is_convertible<typename is_std_vector<F>::base, typename is_std_vector<T>::base>::value),
    T>::type
implicit_cast(const F&)
{
  std::string err = __PRETTY_FUNCTION__ + std::string(":") + std::to_string(__LINE__) + ": " +
                    "Umatched implicit cast from '" +
                    boost::typeindex::type_id_with_cvr<decltype(F())>().pretty_name() + "' to '" +
                    boost::typeindex::type_id_with_cvr<decltype(T())>().pretty_name() + "'";
  throw std::runtime_error(err.c_str());
  return T();
}

}  // namespace param
}  // namespace cnr

#endif  // CNR_PARAM__INCLUDE__CNR_PARAM__UTILS__TYPE_TRAITS__H
