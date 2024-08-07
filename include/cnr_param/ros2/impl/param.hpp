#ifndef CNR_PARAM__INCLUDE__CNR_PARAM__ROS2__IMPL__PARAM__HPP
#define CNR_PARAM__INCLUDE__CNR_PARAM__ROS2__IMPL__PARAM__HPP

#include <string>
#include <Eigen/Core>

#include <boost/type_index.hpp>
#include <boost/algorithm/string/split.hpp>  // Include for boost::split

#include <yaml-cpp/node/node.h>
#include <yaml-cpp/yaml.h>

#include <rclcpp/node.hpp>

#include <cnr_yaml/cnr_yaml.h>

#include <cnr_param/core/string.h>
#include <cnr_param/ros2/param.h>
#include <cnr_param/ros2/param_retriever.h>
#include <cnr_param/ros2/param_retriever.h>
#include <cnr_param/ros2/yaml_formatter.h>

#include <boost/algorithm/string/classification.hpp>  // Include boost::for is_any_of
#include "cnr_param/ros2/param_dictionary.h"
#include "rclcpp/parameter_value.hpp"

using namespace std::chrono_literals;

namespace cnr
{
namespace param
{
namespace ros2
{

std::shared_ptr<rclcpp::Node>& background_node();
std::shared_ptr<cnr::param::ros2::ParamRetriever>& param_retriever();

inline bool resolve_autogenerated_names(const std::string& node_name, const std::string& key, std::string& resolved_key,
                                        std::string& what)
{
  std::vector<std::string> parameter_names;
  if (!param_retriever()->list_parameters(node_name, {}, parameter_names, what))
  {
    return false;
  }

  for (const auto& p : parameter_names)
  {
    if (key == p)
    {
      resolved_key = p;
      return true;
    }
    else
    {
      std::vector<std::string> tokens = { "_autogenerated_from_sequence_of_Sequence",
                                          "_autogenerated_from_sequence_of_Map" };
      for (const auto& token : tokens)
      {
        auto it = p.find(key + token);
        if (it != std::string::npos)
        {
          resolved_key = p.substr(0, it + key.length() + token.length());
          return true;
        }
      }
    }
  }

  what = "Asked for parameter key '" + key + "', but the node '" + node_name +
         "' does not have it. The parmeters are: " + std::to_string(parameter_names);

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
template <typename T>
inline bool get(const std::string& key, T& ret, std::string& what, const bool& implicit_cast_if_possible)
{
  if (!background_node())
  {
    what = "The node is not initialized. Remeber to call the macro CNR_PARAM_INIT_NODE(<your shared_ptr to node>)";
    return false;
  }

  std::string resolved_node_name, resolved_key;
  if (!param_retriever()->resolve_names(key, resolved_node_name, resolved_key, what))
  {
    return false;
  }

  ParamDictionary param(resolved_node_name);
  if (!param_retriever()->get_parameter(resolved_node_name, resolved_key, param, what))
  {
    std::string autogenerated_key;
    if (!resolve_autogenerated_names(resolved_node_name, resolved_key, autogenerated_key, what))
    {
      return false;
    }
    if (!param_retriever()->get_parameter(resolved_node_name, autogenerated_key, param, what))
    {
      return false;
    }
  }

  // ========================================================================
  YAML::Node node(YAML::NodeType::Map);
  if (!cnr::param::ros2::to_yaml(param, node, what))
  {
    return false;
  }

  YAML::Node uncrustified_node;
  if (!cnr::param::ros2::ros2_yaml_decoder(node, uncrustified_node, what))
  {
    return false;
  }
  // ========================================================================

  YAML::Node uncrustified_value = uncrustified_node.begin()->second;
  return cnr::yaml::get(uncrustified_value, ret, what, implicit_cast_if_possible);
}

template <typename T>
bool set(const std::string& key, const T& value, std::string& what)
{
  if (!background_node())
  {
    what = "The node is not initialized. Remeber to call the macro CNR_PARAM_INIT_NODE(<your shared_ptr to node>)";
    return false;
  }

  std::string resolved_node_name, resolved_key;
  if (!param_retriever()->resolve_names(key, resolved_node_name, resolved_key, what))
  {
    return false;
  }

  YAML::Node leaf;
  if (!cnr::yaml::set(value, leaf, what))
  {
    return false;
  }
  YAML::Node node(YAML::NodeType::Map);
  node[resolved_key] = leaf;

  std::size_t ll = __LINE__;
  try
  {
    ll = __LINE__;
    YAML::Node crusted_node;
    if (!cnr::param::ros2::ros2_yaml_encoder(node, crusted_node, what))
    {
      return false;
    }
    ll = __LINE__;
    std::vector<std::pair<std::string, YAML::Node>> param_tree;
    ll = __LINE__;
    cnr::yaml::get_nodes_tree("/", crusted_node, param_tree);
    ll = __LINE__;
    for (const auto& param : param_tree)
    {
      ll = __LINE__;
      rclcpp::ParameterValue value;
      if (!cnr::param::ros2::from_yaml(param.second, value, what))
      {
        return false;
      }
      
      rclcpp::Parameter p(resolved_key, value);
      if (!param_retriever()->set_parameter(resolved_node_name, resolved_key, p, what))
      {
        return false;
      }
    }
    return true;
  }
  catch (YAML::Exception& e)
  {
    std::cerr << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": " << ll << ":"
              << "Node: " << std::endl
              << node << std::endl
              << "What: " << std::endl
              << e.what() << std::endl;
  }
  catch (std::exception& e)
  {
    std::cerr << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": " << ll << ":"
              << "Node: " << std::endl
              << node << std::endl
              << "What: " << std::endl
              << e.what() << std::endl;
  }

  return false;
}

}  // namespace ros2
}  // namespace param
}  // namespace cnr

#endif /* CNR_PARAM__INCLUDE__CNR_PARAM__ROS2__IMPL__PARAM__HPP */