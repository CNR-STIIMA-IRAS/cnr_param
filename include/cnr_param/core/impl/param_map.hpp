#ifndef CNR_PARAM__INCLUDE__CNR_PARAM__UTILS__IMPL__PARAM_MAP__HPP
#define CNR_PARAM__INCLUDE__CNR_PARAM__UTILS__IMPL__PARAM_MAP__HPP

#include <boost/type_index.hpp>
#include <yaml-cpp/yaml.h>
#include <cnr_param/core/string.h>
#include <cnr_param/core/type_traits.h>
#include <cnr_param/core/yaml.h>

#if !defined(UNUSED)
#define UNUSED(expr) do { (void)(expr); } while (0)
#endif

namespace cnr
{
namespace param
{
namespace core
{

// =============================================================================================
// MAP
// =============================================================================================
template<typename T>
inline bool get_map(const YAML::Node& node, T& ret, std::string& what)
{
  UNUSED(ret);
  std::stringstream _node;
  _node << node;
  what = "The type ' "
        + boost::typeindex::type_id_with_cvr<decltype(T())>().pretty_name() 
          + "' is not supported. You must specilized your own 'get_map' template function\n Input Node: " + _node.str();
  return false;
}

template<>
inline bool get_map(const YAML::Node& node, YAML::Node& ret, std::string& what)
{
  UNUSED(what);
  ret = node;
  return true;
}
// =============================================================================================
// END MAP
// =============================================================================================



}   // namespace core
}   // namespace param
}   // namespace cnr

#endif  // CNR_PARAM__INCLUDE__CNR_PARAM__UTILS__IMPL__PARAM_MAP__HPP