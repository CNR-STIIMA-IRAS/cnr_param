#ifndef CNR_PARAM__INCLUDE__CNR_PARAM__ROS__PARAM_DICTIONARY__H
#define CNR_PARAM__INCLUDE__CNR_PARAM__ROS__PARAM_DICTIONARY__H

#include <cstdint>
#include <string>
#include <vector>
#include <variant>
#include <Eigen/Core>

#include <yaml-cpp/yaml.h>

#include <xmlrpcpp/XmlRpc.h>
#include <ros/node_handle.h>

#include <cnr_param/core/type_traits.h>
#include <cnr_param/core/param_dictionary.h>

namespace cnr
{
namespace param
{
namespace ros
{

using ParamDictionary = ::cnr::param::core::ParamDictionary<XmlRpc::XmlRpcValue>;

using double_param =
    ::cnr::param::core::type_holder<XmlRpc::XmlRpcValue::Type, XmlRpc::XmlRpcValue::TypeDouble, double>;
using int_param =
    ::cnr::param::core::type_holder<XmlRpc::XmlRpcValue::Type, XmlRpc::XmlRpcValue::TypeInt, int64_t>;
using string_param =
    ::cnr::param::core::type_holder<XmlRpc::XmlRpcValue::Type, XmlRpc::XmlRpcValue::TypeString, std::string>;
using bool_param = ::cnr::param::core::type_holder<XmlRpc::XmlRpcValue::Type, XmlRpc::XmlRpcValue::TypeBoolean, bool>;
using bytes_param = ::cnr::param::core::type_holder<XmlRpc::XmlRpcValue::Type, XmlRpc::XmlRpcValue::TypeBase64, std::vector<char>>;

using AllowedParamType = std::variant<ros::double_param::c_type, ros::int_param::c_type, ros::string_param::c_type,
                                      ros::bool_param::c_type, ros::bytes_param::c_type>;

bool to_yaml(const std::string& key, const XmlRpc::XmlRpcValue& tree, YAML::Node& node, std::string& what);
bool to_yaml(const ::cnr::param::core::ParamDictionary<XmlRpc::XmlRpcValue>& tree, YAML::Node& node, std::string& what);

}  // namespace ros2
}  // namespace param
}  // namespace cnr

namespace std
{

/**
 * @brief
 *
 * @param val
 * @return std::string
 */
std::string to_string(const XmlRpc::XmlRpcValue& val);

/**
 * @brief
 *
 * @param val
 * @return std::string
 */
std::string to_string(const std::shared_ptr<XmlRpc::XmlRpcValue>& val);

/**
 * @brief
 *
 * @param val
 * @return std::string
 */
std::string to_string(const cnr::param::ros::ParamDictionary& val);

}  // namespace std

#include <cnr_param/ros/impl/param_dictionary.hpp>

#endif  // CNR_PARAM__INCLUDE__CNR_PARAM__ROS__PARAM_DICTIONARY__H