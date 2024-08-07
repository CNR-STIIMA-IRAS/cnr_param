#ifndef CNR_PARAM__INCLUDE__CNR_PARAM__ROS2__PARAM_DICTIONARY__H
#define CNR_PARAM__INCLUDE__CNR_PARAM__ROS2__PARAM_DICTIONARY__H

#include <cstdint>
#include <string>
#include <vector>
#include <variant>
#include <Eigen/Core>

#include <yaml-cpp/yaml.h>

#include <rclcpp/node.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/parameter_client.hpp>

#include <cnr_yaml/type_traits.h>
#include <cnr_param/core/param_dictionary.h>

namespace cnr
{
namespace param
{
namespace ros2
{

using ParamDictionary = ::cnr::param::core::ParamDictionary<rclcpp::Parameter>;

bool to_yaml(const ::cnr::param::core::ParamDictionary<rclcpp::Parameter>& tree, YAML::Node& node, std::string& what);
bool from_yaml(const YAML::Node& node, rclcpp::ParameterValue& tree, std::string& what);

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
std::string to_string(const rclcpp::Parameter& val);

/**
 * @brief
 *
 * @param val
 * @return std::string
 */
std::string to_string(const std::shared_ptr<rclcpp::Parameter>& val);

/**
 * @brief
 *
 * @param val
 * @return std::string
 */
inline std::string to_string(const cnr::param::ros2::ParamDictionary& val);

}  // namespace std

#include <cnr_param/ros2/impl/param_dictionary.hpp>

#endif  // CNR_PARAM__INCLUDE__CNR_PARAM__ROS2__PARAM_DICTIONARY__H