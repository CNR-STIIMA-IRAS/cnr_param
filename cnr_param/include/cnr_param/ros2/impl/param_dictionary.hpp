#ifndef CNR_PARAM__INCLUDE__CNR_PARAM__ROS2__IMPL__PARAM_DICTIONARY__HPP
#define CNR_PARAM__INCLUDE__CNR_PARAM__ROS2__IMPL__PARAM_DICTIONARY__HPP

#include <cstdint>
#include <string>

#include <variant>
#include <Eigen/Core>

#include <yaml-cpp/yaml.h>

#include <rclcpp/node.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/parameter_client.hpp>

#include <cnr_param/core/param_dictionary.h>
#include <cnr_param/ros2/param_dictionary.h>

namespace cnr
{
namespace param
{
namespace core
{

template <>
inline bool ParamDictionary<rclcpp::Parameter>::is_scalar() const
{
  if (std::holds_alternative<rclcpp::Parameter>(param_.second))
  {
    const auto& p = std::get<rclcpp::Parameter>(param_.second);
    return p.get_type() == rclcpp::ParameterType::PARAMETER_BOOL ||
           p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER ||
           p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE ||
           p.get_type() == rclcpp::ParameterType::PARAMETER_STRING;
  }
  return false;
}

template <>
inline bool ParamDictionary<rclcpp::Parameter>::is_sequence() const
{
  if (std::holds_alternative<rclcpp::Parameter>(param_.second))
  {
    const auto& p = std::get<rclcpp::Parameter>(param_.second);
    return p.get_type() == rclcpp::ParameterType::PARAMETER_BOOL_ARRAY ||
           p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY ||
           p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY ||
           p.get_type() == rclcpp::ParameterType::PARAMETER_BYTE_ARRAY ||
           p.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY;
  }
  return false;
}

template <>
inline std::string ParamDictionary<rclcpp::Parameter>::to_string(const std::string& prefix) const
{
  std::string ret;
  ret = prefix + param_.first;
  if (std::holds_alternative<rclcpp::Parameter>(param_.second))
  {
    ret += ": " + std::get<rclcpp::Parameter>(param_.second).value_to_string() + "\n";
  }
  else if (std::holds_alternative<typename ParamDictionary::NestedParams>(param_.second))
  {
    ret += "[type: ParamDictionary<P>::NestedParams(" +
           std::to_string(std::get<typename ParamDictionary::NestedParams>(param_.second).size()) + ")]\n";
    for (const auto& p : std::get<typename ParamDictionary::NestedParams>(param_.second))
    {
      ret += p.second.to_string(prefix + param_.first + "/");
    }
  }
  return ret;
}

template <>
inline bool to_yaml(const ParamDictionary<rclcpp::Parameter>& tree, YAML::Node& node, std::string& what)
{
  return cnr::param::ros2::to_yaml(tree, node, what);
}

}  // namespace core
}  // namespace param
}  // namespace cnr

namespace cnr
{
namespace param
{
namespace ros2
{

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
inline std::string to_string(const rclcpp::Parameter& val)
{
  return val.value_to_string();
}

/**
 * @brief
 *
 * @param val
 * @return std::string
 */
inline std::string to_string(const std::shared_ptr<rclcpp::Parameter>& val)
{
  return val->value_to_string();
}

/**
 * @brief
 *
 * @param val
 * @return std::string
 */
inline std::string to_string(const cnr::param::ros2::ParamDictionary& val)
{
  YAML::Node n;
  std::string what;
  if (!cnr::param::ros2::to_yaml(val, n, what))
  {
    return "!!!Error in create the string for the parameter '" + val.name() + "'. Error: " + what;
  }
  std::stringstream ss;
  ss << n;
  return ss.str();
}

}  // namespace std

#endif  // CNR_PARAM__INCLUDE__CNR_PARAM__ROS2__IMPL__PARAM_DICTIONARY__HPP