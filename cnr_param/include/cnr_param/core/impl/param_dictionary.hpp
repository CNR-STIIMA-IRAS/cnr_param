#ifndef CNR_PARAM__INCLUDE__CNR_PARAM__CORE__IMPL__PARAM_DICTIONARY__HPP
#define CNR_PARAM__INCLUDE__CNR_PARAM__CORE__IMPL__PARAM_DICTIONARY__HPP

#include <iostream>

#include <stdexcept>
#include <boost/algorithm/string.hpp>

#include <cnr_param/core/type_traits.h>
#include <cnr_param/core/param_dictionary.h>

namespace cnr
{
namespace param
{
namespace yaml
{
struct Value
{
  enum Type : uint8_t
  {
    FLOATING,
    INTEGER,
    UNSIGNED_INTEGER,
    CHAR,
    UNSIGNED_CHAR,
    STRING,
    BOOL,
    BYTES
  };
};

using double_param =
    ::cnr::param::core::type_holder<::cnr::param::yaml::Value::Type, ::cnr::param::yaml::Value::Type::FLOATING, double>;
using int_param =
    ::cnr::param::core::type_holder<::cnr::param::yaml::Value::Type, ::cnr::param::yaml::Value::Type::INTEGER, int>;
using uint_param = ::cnr::param::core::type_holder<::cnr::param::yaml::Value::Type,
                                                   ::cnr::param::yaml::Value::Type::UNSIGNED_INTEGER, uint64_t>;
using string_param = ::cnr::param::core::type_holder<::cnr::param::yaml::Value::Type,
                                                     ::cnr::param::yaml::Value::Type::STRING, std::string>;
using bool_param =
    ::cnr::param::core::type_holder<::cnr::param::yaml::Value::Type, ::cnr::param::yaml::Value::Type::BOOL, bool>;
using bytes_param = ::cnr::param::core::type_holder<::cnr::param::yaml::Value::Type,
                                                    ::cnr::param::yaml::Value::Type::BYTES, std::vector<char>>;

using AllowedParamType = std::variant<double_param::c_type, int_param::c_type, uint_param::c_type, string_param::c_type,
                                      bool_param::c_type, bytes_param::c_type>;
}  // namespace yaml
}  // namespace param
}  // namespace cnr

namespace cnr
{
namespace param
{
namespace core
{
// Alternative type for implicit conversion from XmlRpcValue::TypeDouble
template <>
struct type_variant_holder<cnr::param::yaml::double_param::c_type>
{
  using base = cnr::param::yaml::double_param::c_type;
  using variant = std::variant<double, long double, float, int32_t, int64_t, int16_t, int8_t>;
};

template <>
struct type_variant_holder<cnr::param::yaml::int_param::c_type>
{
  using base = cnr::param::yaml::int_param::c_type;
  using variant = std::variant<double, long double, float, int32_t, int64_t, int16_t, int8_t>;
};

template <>
struct type_variant_holder<cnr::param::yaml::uint_param::c_type>
{
  using base = cnr::param::yaml::uint_param::c_type;
  using variant = std::variant<uint32_t, uint64_t, uint16_t, uint8_t>;
};

template <>
struct type_variant_holder<cnr::param::yaml::string_param::c_type>
{
  using base = cnr::param::yaml::string_param::c_type;
  using variant = std::variant<std::string>;
};

template <>
struct type_variant_holder<cnr::param::yaml::bool_param::c_type>
{
  using base = cnr::param::yaml::bool_param::c_type;
  using variant = std::variant<bool, char, uint8_t, uint16_t, uint32_t, uint64_t>;
};

template <>
struct type_variant_holder<cnr::param::yaml::bytes_param::c_type>
{
  using base = cnr::param::yaml::bool_param::c_type;
  using variant = std::variant<std::vector<bool>, std::vector<char>, std::vector<uint8_t>, std::vector<uint16_t>,
                               std::vector<uint32_t>, std::vector<uint64_t>>;
};

/**
 * @brief Retrieves a value from a variant.
 *
 * This function attempts to retrieve a value of type T from a variant object. It supports implicit casting if
 * specified, and provides error messages if the requested type is not found or if no conversion is possible.
 *
 * @tparam T The type of the value to retrieve.
 * @tparam F The first type in the variant.
 * @tparam Ts The remaining types in the variant.
 * @param ret [out] The retrieved value of type T.
 * @param rhs The variant object to retrieve the value from.
 * @param implicit_cast_if_possible Flag indicating whether implicit casting is allowed.
 * @param what [out] Error message describing the reason for failure, if any.
 * @return True if the value was successfully retrieved, false otherwise.
 */
template <typename T, typename F, typename... Ts>
inline bool get_from_variant(T& ret, const std::variant<F, Ts...>& rhs, const bool& implicit_cast_if_possible,
                             std::string& what)
{
  if (std::holds_alternative<F>(rhs))
  {
    if constexpr (is_variant_member<T, decltype(rhs)>::value)
    {
      return std::get<T>(rhs);
    }
    else
    {
      if (!implicit_cast_if_possible)
      {
        what = "The variant contains an object type '" +
               boost::typeindex::type_id_with_cvr<decltype(F())>().pretty_name() + "', while the requested type is '" +
               boost::typeindex::type_id_with_cvr<decltype(T())>().pretty_name() + "'";
        return false;
      }
      if constexpr (is_forward_implicit_conversion_allowed<c_type_to_param_type<F>::value, T>::value)
      {
        ret = static_cast<T>(std::get<F>(rhs));
        return true;
      }
      else
      {
        if constexpr (sizeof...(Ts) > 0)
        {
          std::variant<Ts...> _rhs;
          std::visit(overloaded{ [&_rhs](auto arg) { _rhs = arg; }, [](F) {} }, rhs);
          return get_from_variant<T, Ts...>(ret, _rhs, implicit_cast_if_possible, what);
        }
        else
        {
          what = "The variant contains an object type '" +
                 boost::typeindex::type_id_with_cvr<decltype(F())>().pretty_name() +
                 "', while the requested type is '" +
                 boost::typeindex::type_id_with_cvr<decltype(T())>().pretty_name() + "'. No conversion is possible";
          return false;
        }
      }
    }
  }

  if constexpr (sizeof...(Ts) > 0)
  {
    std::variant<Ts...> _rhs;
    std::visit(overloaded{ [&_rhs](auto arg) { _rhs = arg; }, [](F) {} }, rhs);
    return get_from_variant(ret, _rhs, implicit_cast_if_possible, what);
  }
}

template <typename A, typename P>
inline A as_generic(const P&)
{
  std::string err = __PRETTY_FUNCTION__ + std::string(":") + std::to_string(__LINE__) + ": " +
                    "To be specialized to your specific case";
  throw std::runtime_error(err.c_str());
  return A();
}

template <typename P>
inline ParamDictionary<P>::ParamDictionary(const std::string& key) : param_({ key, ParamDictionary<P>::EmptyParam() })
{
}

template <typename P>
inline const std::string& ParamDictionary<P>::name() const
{
  return param_.first;
}

template <typename P>
inline const ParamDictionary<P>::AllowedParam& ParamDictionary<P>::value() const
{
  return param_.second;
}

template <typename P>
inline ParamDictionary<P>& ParamDictionary<P>::operator=(const P& value)
{
  param_.second = value;
  return *this;
}

template <typename P>
inline ParamDictionary<P>& ParamDictionary<P>::operator[](const std::string& key)
{
  const std::string& _n = param_.first;
  ParamDictionary<P>::AllowedParam& _p = param_.second;
  if (std::holds_alternative<P>(_p))
  {
    if (key != param_.first)
    {
      throw std::runtime_error((__PRETTY_FUNCTION__ + (":" + std::to_string(__LINE__) + ":") +
                                " The Dictionary store a param under the name '" + _n +
                                "', while the required value name is '" + key + "'")
                                   .c_str());
    }
    return *this;
  }
  else if (std::holds_alternative<typename ParamDictionary<P>::NestedParams>(_p))
  {
    auto& _m = std::get<typename ParamDictionary<P>::NestedParams>(_p);
    if ((_m.find(key) == _m.end()))
    {
      _m.insert({ key, ParamDictionary(key) });
    }
    return _m.at(key);
  }
  else if (std::holds_alternative<ParamDictionary<P>::EmptyParam>(_p))
  {
    if (key != param_.first)
    {
      _p = NestedParams({ { key, ParamDictionary(key) } });
    }
    return std::get<NestedParams>(_p).at(key);
  }
  throw std::runtime_error(
      (__PRETTY_FUNCTION__ + (":" + std::to_string(__LINE__) + ":") + " Weird Error! Debug ....").c_str());
}

template <typename P>
inline const ParamDictionary<P>& ParamDictionary<P>::operator[](const std::string& key) const
{
  const std::string& _n = param_.first;
  const ParamDictionary<P>::AllowedParam& _p = param_.second;
  if (std::holds_alternative<P>(_p))
  {
    if (key != param_.first)
    {
      throw std::runtime_error((__PRETTY_FUNCTION__ + (":" + std::to_string(__LINE__) + ":") +
                                " The Dictionary store a param under the name '" + _n +
                                "', while the required value name is '" + key + "'")
                                   .c_str());
    }
    return *this;
  }
  else if (std::holds_alternative<typename ParamDictionary<P>::NestedParams>(_p))
  {
    auto& _m = std::get<typename ParamDictionary<P>::NestedParams>(_p);
    if ((_m.find(key) == _m.end()))
    {
      throw std::runtime_error((__PRETTY_FUNCTION__ + (":" + std::to_string(__LINE__) + ":") +
                                " The Dictionary store a map of parameters under the name '" + _n +
                                "'. The required '" + key + "' is not in the map")
                                   .c_str());
    }
    return _m.at(key);
  }

  throw std::runtime_error(
      (__PRETTY_FUNCTION__ + (":" + std::to_string(__LINE__) + ":") + " The Dictionary is unitialized").c_str());
}

template <typename P>
inline bool ParamDictionary<P>::initialized() const
{
  return std::holds_alternative<P>(param_.second) ||
         std::holds_alternative<typename ParamDictionary<P>::NestedParams>(param_.second);
}

template <typename P>
inline bool ParamDictionary<P>::is_scalar() const
{
  std::string err = __PRETTY_FUNCTION__ + std::string(":") + std::to_string(__LINE__) + ": " +
                    "To be specialized to your specific case";
  throw std::runtime_error(err.c_str());
  return false;
}

template <typename P>
inline bool ParamDictionary<P>::is_sequence() const
{
  std::string err = __PRETTY_FUNCTION__ + std::string(":") + std::to_string(__LINE__) + ": " +
                    "To be specialized to your specific case";
  throw std::runtime_error(err.c_str());
  return false;
}

template <typename P>
inline bool ParamDictionary<P>::is_map() const
{
  return std::holds_alternative<typename ParamDictionary<P>::NestedParams>(param_.second);
}

template <typename P>
inline const ParamDictionary<P>* find(const ParamDictionary<P>& tree, const std::string key,
                                      const std::string& parameter_separator_string, std::string& what)
{
  std::size_t ll = __LINE__;
  try
  {
    ll = __LINE__;
    if (std::holds_alternative<P>(tree.value()))
    {
      ll = __LINE__;
      if (key != tree.name())
      {
        what = "The Dictionary store a single param under the param name '" + tree.name() +
               "', but you are searching for the key '" + key + "'";
        return nullptr;
      }
      return &tree;
    }
    else if (std::holds_alternative<typename ParamDictionary<P>::NestedParams>(tree.value()))
    {
      ll = __LINE__;
      std::vector<std::string> split;
      boost::split(split, key, boost::is_any_of(parameter_separator_string), boost::token_compress_on);
      auto& value = std::get<typename ParamDictionary<P>::NestedParams>(tree.value());

      auto it = value.begin();
      auto ft = value.begin();
      auto et = value.end();
      ll = __LINE__;
      for (size_t i = 0; i < split.size(); i++)
      {
        ll = __LINE__;
        std::string _key = split.at(i);
        ll = __LINE__;
        ft = std::find_if(it, et,
                          [&_key](const std::pair<std::string, ParamDictionary<P>>& v) { return v.first == _key; });
        if (ft == et)
        {
          what = "The Dictionary store a map of params param under the root param name '" + tree.name() +
                 "'. You are searching for the key '" + key + "' (subkey: " + _key.at(i) +
                 ") that is not in the dictionary: " + tree.to_string("/");
          return nullptr;
        }
        ll = __LINE__;
        if (std::holds_alternative<typename ParamDictionary<P>::NestedParams>(ft->second.value()))
        {
          auto& value = std::get<typename ParamDictionary<P>::NestedParams>(ft->second.value());
          it = value.begin();
          et = value.end();
          continue;
        }
        ll = __LINE__;
        if (std::holds_alternative<P>(ft->second.value()) && (i < split.size() - 1))
        {
          what = "The Dictionary store a map of params param under the root param name '" + tree.name() +
                 "'. The key '" + _key + "' is  not in the dictionary: " + it->second.to_string("/");
          return nullptr;
        }
      }
      ll = __LINE__;
      return &(ft->second);
    }
  }
  catch (std::exception& e)
  {
    what = std::string(e.what()) + " at line " + std::to_string(ll) + " in " + __FILE__;
  }
  catch (...)
  {
    what = "Unknown exception at line " + std::to_string(ll) + " in " + __FILE__;
  }

  return nullptr;
}

template <typename P>
inline bool to_yaml(const ParamDictionary<P>&, YAML::Node&, std::string&)
{
  std::string err = __PRETTY_FUNCTION__ + std::string(":") + std::to_string(__LINE__) + ": " +
                    "To be specialized to your specific case";
  throw std::runtime_error(err.c_str());
  return false;
}

}  // namespace core
}  // namespace param
}  // namespace cnr

#endif  // CNR_PARAM__INCLUDE__CNR_PARAM__CORE__IMPL__PARAM_DICTIONARY__HPP