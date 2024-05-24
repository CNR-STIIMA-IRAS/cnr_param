#ifndef CNR_PARAM__INCLUDE__CNR_PARAM__ROS__IMPL__PARAM__HPP
#define CNR_PARAM__INCLUDE__CNR_PARAM__ROS__IMPL__PARAM__HPP

#include <string>
#include <Eigen/Core>

#include <boost/type_index.hpp>
#include <boost/algorithm/string/split.hpp>           // Include for boost::split
#include <boost/algorithm/string/classification.hpp>  // Include boost::for is_any_of

#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <ros/exceptions.h>

#include <xmlrpcpp/XmlRpcValue.h>

#include <cnr_param/ros/param.h>

#include <cnr_param/core/type_traits.h>
#include <cnr_param/core/string.h>
#include <cnr_param/core/eigen.h>
#include <cnr_param/core/yaml.h>

#include <cnr_param/ros/impl/param_retriever.hpp>
#include <cnr_param/ros/impl/scalar.hpp>
#include <cnr_param/ros/impl/sequence.hpp>
#include <cnr_param/ros/impl/map.hpp>


namespace cnr 
{
namespace param
{
namespace ros
{

/**
 * @brief 
 * 
 * @return std::shared_ptr<rclcpp::Node> 
 */
std::shared_ptr<::ros::NodeHandle>& background_node(std::shared_ptr<::ros::NodeHandle> node = nullptr);

/**
 * @brief 
 * 
 * @return std::shared_ptr<cnr::param::ros2::ParamRetriever> 
 */
std::shared_ptr<::cnr::param::ParamRetriever< ::ros::NodeHandle, XmlRpc::XmlRpcValue> >& param_retriever();

/**
 * @brief 
 * 
 * @param node_name 
 * @param key 
 * @param resolved_key 
 * @param what 
 * @return true 
 * @return false 
 */
inline bool resolve_autogenerated_names(const std::string& node_name, const std::string& key, std::string& , std::string& what)
{
  what = "Asked for parameter key '"+key+"', but the node '"+node_name+"' does not have it. The parmeters are: ";

  return false;
}

//=====================================================================================================
//
//
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
inline bool get(const std::string& key, T& ret, std::string& what, const bool& implicit_cast_if_possible)
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

  // std::cout << "=================================="  << std::endl;
  // std::cout << "Key: " << key << std::endl;
  // std::cout << "Resolved Node Name: " << resolved_node_name << std::endl;
  // std::cout << "Resolved Key: " << resolved_key << std::endl;

  bool ok = false;
  ParamDictionary<XmlRpc::XmlRpcValue> param(key);
  if(!param_retriever()->get_parameter(resolved_node_name, resolved_key, param, what))
  {
    std::cout << "get_parameter failed!"<< std::endl;
    return false;
  }
  else 
  {
    if(param.is_scalar())
    {
      ok = get_scalar<T>(param, ret, what, implicit_cast_if_possible);
    }
    else if(param.is_sequence())
    {
      ok = get_sequence<T>(param, ret, what);
    } 
    else if(param.is_map())
    {
      ok = get_map<T>(param, ret, what);
    } 
    else
    {
      what = "Failure in getting '"+key+"' (Resolved Node Name: '"+resolved_node_name+"', resolved key: '"+resolved_key+"'. Required type: '"
            + boost::typeindex::type_id_with_cvr<decltype(ret)>().pretty_name() 
                + "'. Extracted param: '" + std::to_string(param) + "')";
    }
  }
  return ok;
}

template<typename T>
bool set(const std::string&, const T&, std::string&)
{
  return false;
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
void insert(ParamDictionary<XmlRpc::XmlRpcValue>& node, const std::string& key, const T& value)
{
  assert(0);
}

}  // namespace ros2
}  // namespace param
}  // namespace cnr

#endif  /* CNR_PARAM__INCLUDE__CNR_PARAM__ROS__IMPL__PARAM__HPP */