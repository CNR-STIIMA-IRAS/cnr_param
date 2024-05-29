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

#include <cnr_param/core/type_traits.h>
#include <cnr_param/core/param_dictionary.h>

namespace cnr
{
namespace param
{
namespace ros2
{

using ParamDictionary = ::cnr::param::core::ParamDictionary<rclcpp::Parameter>;

using double_param =
    ::cnr::param::core::type_holder<rclcpp::ParameterType, rclcpp::ParameterType::PARAMETER_DOUBLE, double>;
using int_param =
    ::cnr::param::core::type_holder<rclcpp::ParameterType, rclcpp::ParameterType::PARAMETER_INTEGER, int64_t>;
using string_param =
    ::cnr::param::core::type_holder<rclcpp::ParameterType, rclcpp::ParameterType::PARAMETER_STRING, std::string>;
using bool_param = ::cnr::param::core::type_holder<rclcpp::ParameterType, rclcpp::ParameterType::PARAMETER_BOOL, bool>;
using bytes_param = ::cnr::param::core::type_holder<rclcpp::ParameterType, rclcpp::ParameterType::PARAMETER_BYTE_ARRAY,
                                                    std::vector<uint8_t>>;

using AllowedParamType = std::variant<ros2::double_param::c_type, ros2::int_param::c_type, ros2::string_param::c_type,
                                      ros2::bool_param::c_type, ros2::bytes_param::c_type>;

bool to_yaml(const ::cnr::param::core::ParamDictionary<rclcpp::Parameter>& tree, YAML::Node& node, std::string& what);
// bool to_yaml(const rclcpp::Parameter& tree, YAML::Node& node, std::string& what);

AllowedParamType as_generic(const rclcpp::Parameter& param);

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

#endif  // CNR_PARAM__INCLUDE__CNR_PARAM__ros2::_IMPL__PARAM_DICTIONARY__HPP