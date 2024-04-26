#ifndef CNR_PARAM__INCLUDE__CNR_PARAM__ROS2__IMPL__PARAM__HPP
#define CNR_PARAM__INCLUDE__CNR_PARAM__ROS2__IMPL__PARAM__HPP

#include <string>
#include <Eigen/Core>

#include <boost/type_index.hpp>

#include <rclcpp/node.hpp>

#include <cnr_param/utils/string.h>
#include <cnr_param/utils/eigen.h>

#include <cnr_param/ros2/param.h>
#include <cnr_param/ros2/param_retriever.h>

#include <cnr_param/ros2/impl/scalar.hpp>
#include <cnr_param/ros2/impl/sequence.hpp>
#include <cnr_param/ros2/impl/map.hpp>

using namespace std::chrono_literals;

namespace cnr 
{
namespace param
{
namespace ros2
{

std::shared_ptr<rclcpp::Node>& background_node();
std::shared_ptr<cnr::param::ros2::ParamRetriever>& param_retriever();

/**
 * @brief 
 * 
 * @tparam T 
 * @param key 
 * @param ret 
 * @param what 
 * @param default_val 
 * @return true 
 * @return false 
 */
template<typename T>
inline bool get(const std::string& key, T& ret, std::string& what)
{
  if (!background_node())
  {
    what = "The node is not initialized. Remeber to call the macro CNR_PARAM_INIT_NODE(<your shared_ptr to node>)";
    return false;
  }

  std::string resolved_node_name, resolved_key;
  if(!param_retriever()->resolve_names(key, resolved_node_name, resolved_key, what))
  {
    return false;
  }
  std::cerr << "Key               :" << key << std::endl;
  std::cerr << "Resolved Node Name:" << resolved_node_name << std::endl;
  std::cerr << "Resolved Key      :" << resolved_key << std::endl;

  ParamDictionary param(resolved_node_name);
  if(!param_retriever()->get_parameter(resolved_node_name, resolved_key, param, what))
  {
    return false;
  }
  bool ok = false;
  if(param.is_scalar())
  {
    ok = ::cnr::param::ros2::get_scalar<T>(param, ret, what);
    
  }
  else if(param.is_sequence())
  {
    ok = ::cnr::param::ros2::get_sequence<T>(param, ret, what);
  } 
  else if(param.is_map())
  {
    ok = ::cnr::param::ros2::get_map<T>(param, ret, what);
  } 
  else
  {
    what = "Failure in getting '"+key+"' (Resolved Node Name: '"+resolved_node_name+"', resolved key: '"+resolved_key+"'. Required type: '"
           + boost::typeindex::type_id_with_cvr<decltype(ret)>().pretty_name() 
              + "'. Extracted param: '" + std::to_string(param) + "')";
  }

  return ok;
}

template<typename T>
bool set(const std::string&, const T&, std::string&)
{
  return true;
}

/**
 * @brief 
 * 
 * @tparam T 
 * @param key 
 * @param ret 
 * @param what 
 * @param default_val 
 * @return true 
 * @return false 
 */
template<typename T>
inline bool get(const std::string& key, T& ret, std::string& what, const T& default_val)
{
  T param;
  if (!get(key, param, what))
  {
    what = (what.size() ? (what + "\n") : std::string("") ) + "Try to superimpose default value...";
    if (!utils::resize(ret, default_val))
    {
      what += " Error!";
      return false;
    }
    what += " OK!";
    ret = default_val;
    return true;
  }

  return true;
}




/**
 * @brief 
 * 
 * @tparam T 
 * @param node 
 * @param key 
 * @param value 
 */
template<typename T>
void insert(ParamDictionary& node, const std::string& key, const T& value)
{
  assert(0);
}

}  // namespace ros2
}  // namespace param
}  // namespace cnr

#endif  /* CNR_PARAM__INCLUDE__CNR_PARAM__ROS2__IMPL__PARAM__HPP */