#ifndef CNR_PARAM__INCLUDE__CNR_PARAM__UTILS__TYPE_TRAITS__H
#define CNR_PARAM__INCLUDE__CNR_PARAM__UTILS__TYPE_TRAITS__H

#include <stdexcept>
#include <boost/type_index.hpp>
#include <type_traits>
#include <vector>
#include <string>
#include <array>
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
struct is_vector : std::false_type
{
  using base = C;
};
template <typename C, typename A>
struct is_vector<std::vector<C, A> > : std::true_type
{
  using base = C;
};
template <typename Derived>
struct is_matrix_expression : std::is_base_of<Eigen::MatrixBase<std::decay_t<Derived> >, std::decay_t<Derived> >
{
  using base = Derived;
};
// --------------------------------------------------------------------------------

// array --------------------------------------------------------------------------
template <typename C>
struct is_array : std::false_type
{
  using base = C;
};
template <typename C, int A>
struct is_array<std::array<C, A> > : std::true_type
{
  using base = C;
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
  using B = typename is_vector<T>::base;
  static constexpr bool value = is_vector<T>::value && is_bool<B>::value;
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
template <class T>
struct is_byte_v
{
  using B = typename is_vector<T>::base;
  static constexpr bool value = is_vector<T>::value && is_byte<B>::value;
};
// --------------------------------------------------------------------------------

// double -------------------------------------------------------------------------
template <typename T>
struct is_double
{
  static constexpr bool value = std::is_floating_point<T>::value && !is_vector<T>::value && !is_array<T>::value;
};
template <class T>
struct is_double_v
{
  using B = typename std::conditional<is_matrix_expression<T>::value, typename is_matrix_expression<T>::base,
                                      typename is_vector<T>::base>::type;
  static constexpr bool value = is_vector<T>::value && is_double<B>::value;
};

template <class T>
struct is_double_std_v
{
  using B = typename is_vector<T>::base;
  static constexpr bool value = is_vector<T>::value && is_double<B>::value;
};

// int -------------------------------------------------------------------------
template <typename T>
struct is_integer
{
  static constexpr bool value = std::is_integral<T>::value && std::is_signed<T>::value &&
                                !std::is_floating_point<T>::value && !is_vector<T>::value && !is_array<T>::value;
};
template <class T>
struct is_integer_v
{
  using B = typename is_vector<T>::base;
  static constexpr bool value = is_vector<T>::value && is_integer<B>::value;
};

// unsigned int -------------------------------------------------------------------------
template <typename T>
struct is_unsigned_integer
{
  static constexpr bool value = std::is_integral<T>::value && !std::is_signed<T>::value && !is_bool<T>::value &&
                                !is_byte<T>::value && !is_vector<T>::value && !is_array<T>::value;
};
template <class T>
struct is_unsigned_integer_v
{
  using B = typename is_vector<T>::base;
  static constexpr bool value = is_vector<T>::value && is_unsigned_integer<B>::value;
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
  using B = typename is_vector<T>::base;
  static constexpr bool value = is_vector<T>::value && is_string<B>::value;
};

// =======================================================================================================
template <typename F, typename T>
inline typename std::enable_if<std::is_same<F, T>::value || std::is_convertible<F,T>::value, T>::type implicit_cast(const F& rhs)
{
  return rhs;
}

template <typename F, typename T>
inline typename std::enable_if<is_matrix_expression<T>::value && is_double_std_v<F>::value, T>::type
implicit_cast(const F& rhs)
{
  return Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(rhs.data(), rhs.size());
}

template <typename F, typename T>
inline typename std::enable_if<!std::is_same<F, T>::value && is_vector<F>::value && is_vector<T>::value &&
                                   std::is_convertible<typename is_vector<F>::base, typename is_vector<T>::base>::value,
                               T>::type
implicit_cast(const F& rhs)
{
  T lhs;
  ;
  for (const auto& v : rhs)
    lhs.push_back(v);
  return lhs;
}

template <typename F, typename T>
inline
    typename std::enable_if<!std::is_same<F, T>::value && ! std::is_convertible<F,T>::value && 
                                !(is_matrix_expression<T>::value && is_double_std_v<F>::value) &&
                                !(is_vector<F>::value && is_vector<T>::value &&
                                  std::is_convertible<typename is_vector<F>::base, typename is_vector<T>::base>::value),
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