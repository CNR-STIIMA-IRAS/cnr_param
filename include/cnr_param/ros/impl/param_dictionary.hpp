#ifndef CNR_PARAM__INCLUDE__CNR_PARAM__ROS__IMPL__PARAM_DICTIONARY__HPP
#define CNR_PARAM__INCLUDE__CNR_PARAM__ROS__IMPL__PARAM_DICTIONARY__HPP

#include <string>
#include <type_traits>
#include <vector>
#include <variant>

#include <yaml-cpp/yaml.h>


#include <ros/node_handle.h>
#include <ros/master.h>
#include <ros/names.h>
#include <ros/param.h>
#include <xmlrpcpp/XmlRpcValue.h>


#include <cnr_param/core/string.h>
#include <cnr_param/ros/param_dictionary.h>

namespace cnr
{
namespace param
{
namespace core
{

template<>
inline bool ParamDictionary<XmlRpc::XmlRpcValue>::is_scalar() const
{
  if (std::holds_alternative<XmlRpc::XmlRpcValue>(param_.second))
  {
    const auto& p = std::get<XmlRpc::XmlRpcValue>(param_.second);
    return p.getType() == XmlRpc::XmlRpcValue::TypeBoolean ||
           p.getType() == XmlRpc::XmlRpcValue::TypeInt ||
           p.getType() == XmlRpc::XmlRpcValue::TypeDouble ||
           p.getType() == XmlRpc::XmlRpcValue::TypeString;
  }
  return false;
}

template<>
inline bool ParamDictionary<XmlRpc::XmlRpcValue>::is_sequence() const
{
  if (std::holds_alternative<XmlRpc::XmlRpcValue>(param_.second))
  {
    const auto& p = std::get<XmlRpc::XmlRpcValue>(param_.second);
    return p.getType() == XmlRpc::XmlRpcValue::TypeArray && p.size();
  }
  return false;
}

template <>
inline std::string ParamDictionary<XmlRpc::XmlRpcValue>::to_string(const std::string& prefix) const
{
  std::string ret;
  ret = prefix + param_.first;
  if (std::holds_alternative<XmlRpc::XmlRpcValue>(param_.second))
  {
    ret += ": [type: XmlRpcc] " + std::get<XmlRpc::XmlRpcValue>(param_.second).toXml() + "\n";
  }
  else if (std::holds_alternative<typename ParamDictionary<XmlRpc::XmlRpcValue>::NestedParams>(param_.second))
  {
    ret += "[type: ParamDictionary<P>::NestedParams(" +
           std::to_string(std::get<typename ParamDictionary<XmlRpc::XmlRpcValue>::NestedParams>(param_.second).size()) + ")]\n";
    for (const auto& p : std::get<typename ParamDictionary<XmlRpc::XmlRpcValue>::NestedParams>(param_.second))
    {
      ret += p.second.to_string(prefix + param_.first + "/");
    }
  }
  return ret;
}

/**
 * @brief 
 * 
 * @tparam  
 * @param tree 
 * @param node 
 * @param what 
 * @return true 
 * @return false 
 */
template<>
inline bool to_yaml(const ParamDictionary<XmlRpc::XmlRpcValue>& tree, YAML::Node& node, std::string& what)
{
  return cnr::param::ros::to_yaml(tree,node,what);
}


}  // namespace core
}  // namespace param
}  // namespace cnr



#endif  // CNR_PARAM__INCLUDE__CNR_PARAM__ROS__IMPL__PARAM_DICTIONARY__HPP