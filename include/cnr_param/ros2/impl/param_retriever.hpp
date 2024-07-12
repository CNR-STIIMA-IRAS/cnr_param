#ifndef CNR_PARAM__INCLUDE__CNR_PARAM__ROS2__IMPL__PARAM_RETRIEVER__HPP
#define CNR_PARAM__INCLUDE__CNR_PARAM__ROS2__IMPL__PARAM_RETRIEVER__HPP

#include <string>
#include <vector>
#include "cnr_param/core/impl/param_retriever.hpp"
#include <yaml-cpp/yaml.h>

#include <rclcpp/node.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/parameter_client.hpp>

#include <cnr_yaml/string.h>

#include <cnr_param/ros2/param_retriever.h>
#include <cnr_param/ros2/impl/param_dictionary.hpp>

namespace cnr
{
namespace param
{
namespace core
{

template <>
inline std::string lintParamKey<rclcpp::Node>(const std::shared_ptr<rclcpp::Node>&, const std::string& param_key)
{
  std::string ret = param_key;
  if (ret.front() == '/' || ret.front() == '.')
  {
    ret.erase(0, 1);
  }
  std::replace(ret.begin(), ret.end(), '/', '.');
  return ret;
}

template <>
inline bool getNodeNames<rclcpp::Node>(const std::shared_ptr<rclcpp::Node>& node, std::vector<std::string>& names,
                                       std::string&)
{
  if (node)
  {
    names = node->get_node_graph_interface()->get_node_names();
    return names.size();
  }
  return false;
}

template <>
inline bool resolveParamName<rclcpp::Node>(const std::shared_ptr<rclcpp::Node>& node, const std::string& ns,
                                           std::string& node_name, std::string& resolved_key, std::string& what)
{
  std::string _nn = node->get_fully_qualified_name();
  resolved_key = ns;
  if (resolved_key.find(_nn) == 0)
  {
    node_name = _nn;
    resolved_key.erase(0, _nn.size());
    resolved_key = lintParamKey(node, resolved_key);
    return true;
  }

  std::vector<std::string> _nnn;
  if (!getNodeNames<rclcpp::Node>(node, _nnn, what))
  {
    return false;
  }

  for (const auto& nn : _nnn)
  {
    if (resolved_key.find(nn) == 0)
    {
      node_name = nn;
      resolved_key.erase(0, nn.size());
      resolved_key = lintParamKey(node, resolved_key);
      return true;
    }
  }
  what = "Error, param key scope not solvable. Key: " + ns + ", Node Names: " + std::to_string(_nnn);
  return false;
}

template <>
inline bool resolveNodeName<rclcpp::Node>(const std::shared_ptr<rclcpp::Node>& n, std::string& resolved_name,
                                          std::string&)
{
  resolved_name = n->get_fully_qualified_name();
  return true;
}

template <>
inline bool ParamRetriever<rclcpp::Node, rclcpp::Parameter>::list_parameters(const std::string& node_name,
                                                                             const std::vector<std::string>& keys,
                                                                             std::vector<std::string>& parameter_names,
                                                                             std::string& what)
{
  if (!cnr::param::ros2::init_async_params_client(node_name, node_))
  {
    what = "there has been a failure in the initialization of the param retriver for node '" + node_name + "'";
    return false;
  }

  if (node_name == node_->get_fully_qualified_name())  // The request is for a local parameter
  {
    auto list = node_->list_parameters(keys, 1000);
    parameter_names = list.names;
  }
  else
  {
    auto list = cnr::param::ros2::parameters_client()[node_name]->list_parameters(keys, 1000);
    std::chrono::milliseconds span(5000);
    if (list.wait_for(span) != std::future_status::ready)
    {
      printf("set_parameters service call failed. Exiting example.\n");
      return false;
    }
    parameter_names = list.get().names;
  }
  if (parameter_names.size() == 0)
  {
    what = "The remote node '" + node_name + "' does not store the requested parameter prefixes <";
    for (const auto& v : keys)
    {
      what += v + ", ";
    }
    what += ">";
    return false;
  }

  return true;
}

template <>
inline bool ParamRetriever<rclcpp::Node, rclcpp::Parameter>::get_existent_parameters(
    const std::string& node_name, const std::vector<std::string>& keys, std::vector<rclcpp::Parameter>& parameters,
    std::string& what)
{
  if (node_name == node_->get_effective_namespace())  // The request is for a local parameter
  {
    parameters = node_->get_parameters(keys);
  }
  else
  {
    auto result = cnr::param::ros2::parameters_client()[node_name]->get_parameters(keys);
    std::chrono::milliseconds span(5000);
    if (result.wait_for(span) != std::future_status::ready)
    {
      what = "The get_parameter from node '" + node_name + "' failed after 5s";
      return false;
    }
    if (result.get().size() == 0)
    {
      what = "The remote node '" + node_name + "' does not store the requested parameters <";
      for (const auto& v : keys)
      {
        what += v + ", ";
      }
      what += ">";
      return false;
    }
    parameters = result.get();
  }

  return true;
}

template <>
inline bool ParamRetriever<rclcpp::Node, rclcpp::Parameter>::retrieve_parameters(const std::string& resolved_node_name,
                                                                                 const std::string& resolved_key,
                                                                                 std::string& what, bool updated)
{
  if (!cnr::param::ros2::init_async_params_client(resolved_node_name, node_))
  {
    what = "there has been a failure in the initialization of the param retriver for node '" + resolved_node_name + "'";
    return false;
  }

  cnr::param::ros2::ParamDictionary& _node_dict = node_params(resolved_node_name);
  bool value_found = false;
  if (_node_dict.initialized())
  {
    value_found = (_node_dict.name() == resolved_key) ? true : bool(find(_node_dict, resolved_key, ".", what));
  }
  if (updated || !value_found)
  {
    std::vector<std::string> parameter_names;
    if (!list_parameters(resolved_node_name, { resolved_key }, parameter_names, what))
    {
      return false;
    }

    if (parameter_names.size())
    {
      value_found = true;

      std::sort(parameter_names.begin(), parameter_names.end());

      std::vector<rclcpp::Parameter> parameters;
      if (!get_existent_parameters(resolved_node_name, parameter_names, parameters, what))
      {
        return false;
      }

      for (size_t i = 0; i < parameters.size(); i++)
      {
        insert_dict(node_params(resolved_node_name), parameter_names.at(i), parameters.at(i));
      }
    }
  }
  return value_found;
}

template <>
inline bool ParamRetriever<rclcpp::Node, rclcpp::Parameter>::set_parameter(const std::string& resolved_node_name,
                                                                           const std::string&,
                                                                           const rclcpp::Parameter& param,
                                                                           std::string& what)
{
  if (!cnr::param::ros2::init_async_params_client(resolved_node_name, node_))
  {
    what = "there has been a failure in the initialization of the param retriver for node '" + resolved_node_name + "'";
    return false;
  }

  // if(resolved_node_name + "." + resolved_key != param.get_name() )
  // {
  //   std::cerr << "Resovled Node Name: "  << resolved_node_name << std::endl;
  //   std::cerr << "Resovled Key      : "  << resolved_key << std::endl;
  //   std::cerr << "Param Name        : "  << param.get_name() << std::endl;
  //   std::cerr << "Param Value       : "  << param.value_to_string() << std::endl;
  // }

  if (resolved_node_name == node_->get_fully_qualified_name())  // The request is for a local parameter
  {
    node_->set_parameter(param);
  }
  else
  {
    auto list = cnr::param::ros2::parameters_client()[resolved_node_name]->set_parameters({ param });
    std::chrono::milliseconds span(5000);
    if (list.wait_for(span) != std::future_status::ready)
    {
      what = "set_parameters service call failed. ";
      return false;
    }
  }
  return true;
}

}  // namespace core
}  // namespace param
}  // namespace cnr

/**
 * @brief
 *
 */
namespace cnr
{
namespace param
{
namespace ros2
{

}  // namespace ros2
}  // namespace param
}  // namespace cnr

#endif  // CNR_PARAM__INCLUDE__CNR_PARAM__ROS2__IMPL__PARAM_RETRIEVER__HPP