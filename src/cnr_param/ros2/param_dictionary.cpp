#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>

#include <cnr_yaml/cnr_yaml.h>

#include <cnr_param/core/string.h>
#include <cnr_param/core/param_dictionary.h>
#include <cnr_param/ros2/param_dictionary.h>

#include <rclcpp/rclcpp.hpp>
#include "rclcpp/parameter_value.hpp"


namespace cnr
{
namespace param
{
namespace ros2
{

bool to_yaml(const cnr::param::core::ParamDictionary<rclcpp::Parameter>& tree, YAML::Node& node, std::string& what)
{
  if (std::holds_alternative<cnr::param::core::ParamDictionary<rclcpp::Parameter>::EmptyParam>(tree.value()))
  {
    what = "The param is empty";
    return false;
  }
  else if (std::holds_alternative<rclcpp::Parameter>(tree.value()))
  {
    auto& rclcpp_par = std::get<rclcpp::Parameter>(tree.value());
    std::string json_str = rclcpp::_to_json_dict_entry(rclcpp_par);
    YAML::Node yaml_node = YAML::Load(json_str);
    node[tree.name()] = YAML::Load(yaml_node[rclcpp_par.get_name()]["value"].as<std::string>());
  }
  else
  {
    auto& nested = std::get<cnr::param::core::ParamDictionary<rclcpp::Parameter>::NestedParams>(tree.value());
    YAML::Node nested_node(YAML::NodeType::Map);
    for (const auto& p : nested)
    {
      YAML::Node leaf;
      if (!to_yaml(p.second, leaf, what))
      {
        return false;
      }
      nested_node[p.second.name()] = leaf[p.second.name()];
    }
    node[tree.name()] = nested_node;
  }
  return true;
}

bool from_yaml(const YAML::Node& node, rclcpp::ParameterValue& tree, std::string& what)
{
  if (node.IsNull())
  {
    what = "The node is null";
    return false;
  }
  else if (node.IsScalar())
  {
    std::optional<bool> as_bool          = YAML::as_if<bool, std::optional<bool> >(node)();
    std::optional<int> as_int            = YAML::as_if<int, std::optional<int> >(node)();
    std::optional<double> as_double      = YAML::as_if<double, std::optional<double> >(node)();
    std::optional<std::string> as_string = YAML::as_if<std::string, std::optional<std::string> >(node)();
    if (as_bool)
    {
      tree = rclcpp::ParameterValue(as_bool.value());
    }
    else if (as_int)
    {
      tree= rclcpp::ParameterValue(as_int.value());
    }
    else if (as_double)
    {
      tree= rclcpp::ParameterValue(as_double.value());
    }
    else if (as_string)
    {
      tree= rclcpp::ParameterValue(as_string.value());
    }
    else
    {
      what = "The node is a scalar, but the conversion to rclcpp::Parameter failed";
      return false;
    }
  }
  else if (node.IsSequence())
  {
    std::optional<std::vector<bool>> as_bool          = YAML::as_if<std::vector<bool>, std::optional<std::vector<bool>>>(node)();
    std::optional<std::vector<int>> as_int            = YAML::as_if<std::vector<int>, std::optional<std::vector<int>>>(node)();
    std::optional<std::vector<double>> as_double      = YAML::as_if<std::vector<double>, std::optional<std::vector<double>>>(node)();
    std::optional<std::vector<uint8_t>> as_byte_array = YAML::as_if<std::vector<uint8_t>, std::optional<std::vector<uint8_t>>>(node)();
    std::optional<std::vector<std::string>> as_string = YAML::as_if<std::vector<std::string>, std::optional<std::vector<std::string>>>(node)();
    if (as_bool)
    {
      tree = rclcpp::ParameterValue(as_bool.value());
    }
    else if (as_int)
    {
      tree = rclcpp::ParameterValue(as_int.value());
    }
    else if (as_double)
    {
      tree = rclcpp::ParameterValue(as_double.value());
    }
    else if (as_byte_array)
    {
      tree = rclcpp::ParameterValue(as_byte_array.value());
    }
    else if (as_string)
    {
      tree = rclcpp::ParameterValue(as_string.value());
    }
    else
    {
      what = "The nodeis a sequence of item with unrecognized type, but the conversion to rclcpp::Parameter failed";
      return false;
    }
  }
  else if (node.IsMap())
  {
    what = "The node is a Map, that it not supported by rclcpp::ParameterValue";
    return false;
  }
  else
  {
    what = "The node type is Null or Undefined. Abort.";
    return false;
  }
  return true;
}

}  // namespace ros2
}  // namespace param
}  // namespace cnr
