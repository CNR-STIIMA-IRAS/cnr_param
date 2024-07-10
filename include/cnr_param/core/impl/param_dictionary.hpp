#ifndef CNR_PARAM__INCLUDE__CNR_PARAM__CORE__IMPL__PARAM_DICTIONARY__HPP
#define CNR_PARAM__INCLUDE__CNR_PARAM__CORE__IMPL__PARAM_DICTIONARY__HPP

#include <iostream>

#include <stdexcept>
#include <boost/algorithm/string.hpp>

#include <cnr_yaml/type_traits.h>
#include <cnr_param/core/param_dictionary.h>

namespace cnr
{
namespace param
{
namespace core
{

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
inline ParamDictionary<P>& ParamDictionary<P>::operator=(const NestedParams& value)
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