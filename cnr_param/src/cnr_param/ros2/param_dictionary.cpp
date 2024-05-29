#include <string>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <cnr_param/core/string.h>
#include <cnr_param/core/param_dictionary.h>
#include <cnr_param/ros2/param_dictionary.h>

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

}  // namespace ros2
}  // namespace param
}  // namespace cnr
