#ifndef CNR_PARAM__INCLUDE__CNR_PARAM__CORE__IMPL__PARAM_EXTRACT_HPP
#define CNR_PARAM__INCLUDE__CNR_PARAM__CORE__IMPL__PARAM_EXTRACT_HPP

#include <string>
#include <boost/type_index.hpp>
#include <yaml-cpp/yaml.h>
#include <sstream>
#include <cnr_param/core/yaml.h>
#include <cnr_param/core/eigen.h>
#include <cnr_param/core/type_traits.h>

namespace cnr
{
namespace param
{
namespace core
{

/**
 * @brief Get the templated object from the node. 
 * 
 * @tparam T 
 * @param node 
 * @return T 
 */
template<typename T>
inline T extract(const YAML::Node& node, const std::string& key, const std::string& error_heading_msgs)
{
  T ret;
  bool ok = false;
  std::string what;
  YAML::Node leaf;
  if(key.length())
  {
    if(node[key])
    {
      leaf = node[key];
      ok = true;
    }
    else
    {
      std::stringstream _node;
      _node << node;
      what = (error_heading_msgs.length() ? error_heading_msgs  : "" ) 
            + "Tried to extract a '" + boost::typeindex::type_id_with_cvr<decltype(T())>().pretty_name() 
                + "' but the key '" + key + "' is not in the node's key set.\n Input Node: \n" + _node.str();
    }
  }
  else
  {
    leaf = node;
    ok = true;
  }

  if(ok)
  {
    if(leaf.IsScalar())
    {
      ok = get_scalar<T>(leaf, ret, what);
    }
    else if(leaf.IsSequence())
    {
      ok = get_sequence<T>(leaf, ret, what);
    } 
    else if(leaf.IsMap())
    {
      ok = get_map<T>(leaf, ret, what);
    }
    else
    {
      std::stringstream _node;
      _node << node;
      what = (error_heading_msgs.length() ? error_heading_msgs  : "" ) 
            + "Tried to extract a '" + boost::typeindex::type_id_with_cvr<decltype(T())>().pretty_name() 
                + "' but the node type is undefined\n Input Node: \n" + _node.str();
    }
  }

  if(!ok)
  {
    throw std::runtime_error(what);
  }
  return ret;
}

template<>
inline YAML::Node extract(const YAML::Node& node, const std::string& key, const std::string& error_heading_msgs)
{
  UNUSED(key);
  UNUSED(error_heading_msgs);
  YAML::Node ret(node);
  return ret;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



}
}
}
#endif  // CNR_PARAM__INCLUDE__CNR_PARAM__CORE__IMPL__PARAM_EXTRACT_HPP
