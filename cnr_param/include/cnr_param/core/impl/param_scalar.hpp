#ifndef CNR_PARAM__INCLUDE__CNR_PARAM__UTILS__IMPL__PARAM_SCALAR__HPP
#define CNR_PARAM__INCLUDE__CNR_PARAM__UTILS__IMPL__PARAM_SCALAR__HPP

#include <iostream>
#include <type_traits>
#include <boost/type_index.hpp>
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/node/convert.h>

#include <cnr_param/core/string.h>
#include <cnr_param/core/type_traits.h>
#include <cnr_param/core/yaml.h>

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
namespace core
{

template <typename T>
inline typename std::enable_if<cnr::param::is_scalar<T>::value, bool>::type _get_scalar(const YAML::Node& node, T& ret,
                                                                                       std::string& what, const bool& implicit_cast_if_possible)
{
  YAML::Node config(node);
  if (!config.IsScalar())
  {
    std::stringstream _node;
    _node << node;
    what = "Tried to extract a '" + boost::typeindex::type_id_with_cvr<decltype(T())>().pretty_name() +
           "' but the node is not a scalar\n Input Node: " + _node.str();
    return false;
  }

  bool ok = YAML::convert<T>::decode(node, ret); 
  if(!ok && implicit_cast_if_possible)
  {
    if constexpr(is_integer<T>::value) // maybe a double is stored
    {
      double _ret;
      ok = YAML::convert<double>::decode(node, _ret); 
      ret = _ret;
    }
    if constexpr(std::is_floating_point<T>::value) // maybe an integer is stored
    {
      int _ret;
      ok = YAML::convert<int>::decode(node, _ret); 
      ret = _ret;
    }
  }

  if(!ok)
  {
    what = "Error in getting the type '"                                      
        + boost::typeindex::type_id_with_cvr<decltype(ret)>().pretty_name() + "' from the node:\n" 
        + std::to_string(node);
  }

  return ok;
}

template <typename T>
inline typename std::enable_if<!cnr::param::is_scalar<T>::value, bool>::type _get_scalar(const YAML::Node& node, T& ret,
                                                                                        std::string& what, const bool&)
{
  UNUSED(node);
  UNUSED(ret);
  std::stringstream _node;
  _node << node;
  what = "Tried to extract a '" + boost::typeindex::type_id_with_cvr<decltype(T())>().pretty_name() +
         "' but the node is not a scalar\n Input Node: " + _node.str();
  return false;
}

template <typename T>
inline bool get_scalar(const YAML::Node& node, T& ret, std::string& what, const bool& implicit_cast_if_possible)
{
  return _get_scalar(node,ret,what,implicit_cast_if_possible);
}

}  // namespace utils
}  // namespace param
}  // namespace cnr

#endif  // CNR_PARAM__INCLUDE__CNR_PARAM__UTILS__IMPL__PARAM_SCALAR__HPP