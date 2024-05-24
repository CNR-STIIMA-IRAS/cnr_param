#if ROS2_MODULE

#include <string>
#include <chrono>

#include <boost/algorithm/string/classification.hpp>  // Include boost::for is_any_of
#include <boost/algorithm/string/split.hpp>           // Include for boost::split

using namespace std::chrono_literals;

#include <cnr_param/ros2/param_retriever.h>

namespace cnr
{
namespace param
{
namespace ros2
{
AllowedParamType as_generic(const rclcpp::Parameter& param)
{
  AllowedParamType ret;
  switch (param.get_type())
  {
    case rcl_interfaces::msg::ParameterType::PARAMETER_BOOL:
      ret = param.as_bool();
      break;
    case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER:
      ret = param.as_int();
      break;
    case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE:
      ret = param.as_double();
      break;
    case rcl_interfaces::msg::ParameterType::PARAMETER_STRING:
      ret = param.as_string();
      break;
    case rcl_interfaces::msg::ParameterType::PARAMETER_BYTE_ARRAY:
      ret = param.as_byte_array();
      break;
    case rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY:
      ret = param.as_bool_array();
      break;
    case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY:
      ret = param.as_integer_array();
      break;
    case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY:
      ret = param.as_double_array();
      break;
    case rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY:
      ret = param.as_string_array();
      break;
    default:
      return nullptr;
  }
  return ret;
}

template<>
inline std::string linParamKey(const std::shared_ptr<rclcpp::Node>&, const std::string& param_key)
{
  std::string ret = param_key;
  if (ret.front() == '/')
  {
    ret.erase(0, 1);
  }
  std::replace(ret.begin(), ret.end(), '/', '.');
  return ret;
}

ParamDictionary::ParamDictionary(const std::string& key) : param_({ key, ParamDictionary::EmptyParam() }){};

const std::string& ParamDictionary::name() const
{
  return param_.first;
}
const ParamDictionary::AllowedParam& ParamDictionary::value() const
{
  return param_.second;
}
ParamDictionary& ParamDictionary::operator=(const rclcpp::Parameter& value)
{
  param_.second = value;
  return *this;
}

ParamDictionary& ParamDictionary::operator[](const std::string& key)
{
  const std::string& _n = param_.first;
  ParamDictionary::AllowedParam& _p = param_.second;
  if (std::holds_alternative<rclcpp::Parameter>(_p))
  {
    if (key != param_.first)
    {
      throw std::runtime_error((__PRETTY_FUNCTION__ + (":" + std::to_string(__LINE__) + ":") +
                                " The Dictionary store a rclcpp::Parameter under the name '" + _n +
                                "', while the required value name is '" + key + "'")
                                   .c_str());
    }
    return *this;
  }
  else if (std::holds_alternative<ParamDictionary::NestedParams>(_p))
  {
    auto& _m = std::get<ParamDictionary::NestedParams>(_p);
    if ((_m.find(key) == _m.end()))
    {
      _m.insert({ key, ParamDictionary(key) });
    }
    return _m.at(key);
  }
  else if (std::holds_alternative<ParamDictionary::EmptyParam>(_p))
  {
    if (key != param_.first)
    {
      _p = NestedParams({ { key, ParamDictionary(key) } });
    }
    return std::get<NestedParams>(_p).at(key);
  }
  throw std::runtime_error(
      (__PRETTY_FUNCTION__ + (":" + std::to_string(__LINE__) + ":") + " Weird Error! Debug ....").c_str());
}

const ParamDictionary& ParamDictionary::operator[](const std::string& key) const
{
  const std::string& _n = param_.first;
  const ParamDictionary::AllowedParam& _p = param_.second;
  if (std::holds_alternative<rclcpp::Parameter>(_p))
  {
    if (key != param_.first)
    {
      throw std::runtime_error((__PRETTY_FUNCTION__ + (":" + std::to_string(__LINE__) + ":") +
                                " The Dictionary store a rclcpp::Parameter under the name '" + _n +
                                "', while the required value name is '" + key + "'")
                                   .c_str());
    }
    return *this;
  }
  else if (std::holds_alternative<ParamDictionary::NestedParams>(_p))
  {
    auto& _m = std::get<ParamDictionary::NestedParams>(_p);
    if ((_m.find(key) == _m.end()))
    {
      throw std::runtime_error((__PRETTY_FUNCTION__ + (":" + std::to_string(__LINE__) + ":") +
                                " The Dictionary store a map of parameters under the name '" + _n +
                                "'. The required '" + key + "' is not in the map")
                                   .c_str());
    }
    return _m.at(key);
  }

  throw std::runtime_error(
      (__PRETTY_FUNCTION__ + (":" + std::to_string(__LINE__) + ":") + " The Dictionary is unitialized").c_str());
}

bool ParamDictionary::initialized() const
{
  return std::holds_alternative<rclcpp::Parameter>(param_.second) ||
         std::holds_alternative<ParamDictionary::NestedParams>(param_.second);
}

std::string ParamDictionary::to_string(const std::string& prefix) const
{
  std::string ret;
  ret = prefix + param_.first;
  if (std::holds_alternative<rclcpp::Parameter>(param_.second))
  {
    ret += ": [type: rclcpp::Param] " + std::get<rclcpp::Parameter>(param_.second).value_to_string() + "\n";
  }
  else if (std::holds_alternative<ParamDictionary::NestedParams>(param_.second))
  {
    ret += "[type: ParamDictionary::NestedParams(" +
           std::to_string(std::get<ParamDictionary::NestedParams>(param_.second).size()) + ")]\n";
    for (const auto& p : std::get<ParamDictionary::NestedParams>(param_.second))
    {
      ret += p.second.to_string(prefix + param_.first + "/");
    }
  }
  return ret;
}

bool ParamDictionary::is_scalar() const
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
bool ParamDictionary::is_sequence() const
{
  if (std::holds_alternative<rclcpp::Parameter>(param_.second))
  {
    const auto& p = std::get<rclcpp::Parameter>(param_.second);
    return p.get_type() == rclcpp::ParameterType::PARAMETER_BYTE_ARRAY ||
           p.get_type() == rclcpp::ParameterType::PARAMETER_BOOL_ARRAY ||
           p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY ||
           p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY ||
           p.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY;
  }
  return false;
}
bool ParamDictionary::is_map() const
{
  return std::holds_alternative<ParamDictionary::NestedParams>(param_.second);
}

//========================================================
//  ParamRetriever
//
//
//========================================================
ParamRetriever::ParamRetriever(rclcpp::Node::SharedPtr& node) : node_(node)
{
}

bool ParamRetriever::init_async_params_client(const std::string& node_name)
{
  if (!node_)
  {
    return false;
  }

  if (node_name == node_->get_effective_namespace())
  {
    return true;
  }

  if (parameters_client_.find(node_name) == parameters_client_.end() || parameters_client_.at(node_name) == nullptr)
  {
    parameters_client_[node_name] = std::make_shared<rclcpp::AsyncParametersClient>(node_, node_name);
    while (!parameters_client_[node_name]->wait_for_service(1s))
    {
      if (!rclcpp::ok())
      {
        return false;
      }
    }
  }
  return true;
}

bool ParamRetriever::resolve_names(const std::string& ns, std::string& resolved_node_name, std::string& resolved_ns,
                                   std::string& what)
{
  if (ns.empty())
  {
    what = "The key '" + ns + "' is invalid.";
    return false;
  }
  if (!node_)
  {
    what = "Do you have called the CNR_PARAM_INIT_NODE macro?";
    return false;
  }

  if (ns.front() == '/')  // remote
  {
    if(!getNodeNames(node_, names, what)
    {
      return false;
    }
    std::string _ns;
    for (const auto& n : names)
    {
      if (ns.rfind(n, 0) == 0)
      {
        _ns = ns;
        _ns.erase(0, n.size());
        resolved_ns = lintParamKey(_ns);
        resolved_node_name = n;
        return true;
      }
    }
    what = "The ns '" + ns + "' mismatches the node list name <";
    for (const auto& n : names)
    {
      what += n + ",";
    }
    what += ">";
    return false;
  }
  else  // local
  {
    resolved_node_name = std::string(node_->get_effective_namespace()) + node_->get_name();
    resolved_ns = lintParamKey(ns);
  }
  return true;
}

ParamDictionary& ParamRetriever::node_params(const std::string& node_name)
{
  auto it = std::find_if(node_params_.begin(), node_params_.end(),
                         [&node_name](const ParamDictionary& v) { return v.name() == node_name; });
  if (it == node_params_.end())
  {
    node_params_.push_back(ParamDictionary(node_name));
    return node_params_.back();
  }
  return *it;
}

const ParamDictionary& ParamRetriever::node_params(const std::string& node_name) const
{
  auto it = std::find_if(node_params_.begin(), node_params_.end(),
                         [&node_name](const ParamDictionary& v) { return v.name() == node_name; });
  if (it == node_params_.end())
  {
    throw std::runtime_error(("The node '" + node_name + "' is still unmapped from the parameter retriver").c_str());
  }
  return *it;
}

bool ParamRetriever::list_parameters(const std::string& node_name, const std::vector<std::string>& keys,
                                     std::vector<std::string>& parameter_names, std::string& what)
{
  if (node_name == node_->get_effective_namespace() + std::string(node_->get_name()))  // The request is for a local
                                                                                       // parameter
  {
    auto list = node_->list_parameters(keys, 1000);
    parameter_names = list.names;
  }
  else
  {
    auto list = parameters_client_[node_name]->list_parameters(keys, 1000);
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

/**
 * @brief Get the parameters for a vector of keys that have already been checked
 *
 * @param node_name
 * @param keys
 * @param parameters
 * @param what
 * @return true
 * @return false
 */
bool ParamRetriever::get_existent_parameters(const std::string& node_name, const std::vector<std::string>& keys,
                                             std::vector<rclcpp::Parameter>& parameters, std::string& what)
{
  if (node_name == node_->get_effective_namespace())  // The request is for a local parameter
  {
    parameters = node_->get_parameters(keys);
  }
  else
  {
    auto result = parameters_client_[node_name]->get_parameters(keys);
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

void ParamRetriever::insert_dict(ParamDictionary& dictionary, const std::string& key, const rclcpp::Parameter& value)
{
  std::vector<std::string> split;
  boost::split(split, key, boost::is_any_of(PARAMETER_SEPARATOR_STRING), boost::token_compress_on);
  if (split.size() > 1)
  {
    std::string relative_key;
    for(size_t i=1;i<split.size();i++)
    {
      relative_key += split.at(i); 
      if(i<split.size()-1)
      {
        relative_key +=std::string(PARAMETER_SEPARATOR_CHAR);
      }
    }

    insert_dict(dictionary[split.front()], relative_key, value);
  }
  else
  {
    dictionary[key] = value;
  }
}

/**
 * @brief
 *
 * @param node_name
 * @param resolved_key is the name of the param, eventually nested. The namespace separator is '.' as in ROS 2
 * standard.
 * @param dictionary
 * @param updated
 * @return true
 * @return false
 */
bool ParamRetriever::retrieve_parameters(const std::string& resolved_node_name, const std::string& resolved_key,
                                         std::string& what, bool updated)
{
  if (!init_async_params_client(resolved_node_name))
  {
    what = "there has been a failure in the initialization of the param retriver for node '" + resolved_node_name + "'";
    return false;
  }

  ParamDictionary& _node_dict = node_params(resolved_node_name);
  bool value_found = false;
  if (_node_dict.initialized())
  {
    value_found = (_node_dict.name() == resolved_key) ? true : bool(find(_node_dict, resolved_key, what));
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

bool ParamRetriever::get_parameter(const std::string& resolved_node_name, const std::string& resolved_key,
                                   ParamDictionary& param, std::string& what, bool updated)
{
  if (!retrieve_parameters(resolved_node_name, resolved_key, what, updated))
  {
    return false;
  }

  auto it = find(node_params(resolved_node_name), resolved_key, what);
  if (!it)
  {
    what = "Extraction failed (resolved node name: '" + resolved_node_name + "', resolved key: '" + resolved_key +
           "'): " + what;
    return false;
  }
  param = *it;
  return true;
}

const ParamDictionary* find(const ParamDictionary& tree, const std::string key, std::string& what)
{
  if (std::holds_alternative<rclcpp::Parameter>(tree.value()))
  {
    if (key != tree.name())
    {
      what = "The Dictionary store a single param under the param name '" + tree.name() +
             "', but you are searching for the key '" + key + "'";
      return nullptr;
    }
    return &tree;
  }
  else if (std::holds_alternative<ParamDictionary::NestedParams>(tree.value()))
  {
    std::vector<std::string> split;
    boost::split(split, key, boost::is_any_of(PARAMETER_SEPARATOR_STRING), boost::token_compress_on);
    auto& value = std::get<ParamDictionary::NestedParams>(tree.value());
    
    auto it = value.begin();
    auto ft = value.begin();
    auto et = value.end();
    for (size_t i = 0; i < split.size(); i++)
    {
      std::string _key = split.at(i);
      ft = std::find_if(it, et, [&_key](const std::pair<std::string, ParamDictionary>& v) { return v.first == _key; });
      if (ft == et)
      {
        what = "The Dictionary store a map of params param under the root param name '" + tree.name() +
               "'. You are searching for the key '" + key +
               "' that is not in the dictionary: " + it->second.to_string("/");
        return nullptr;
      }
      if (std::holds_alternative<ParamDictionary::NestedParams>(ft->second.value()))
      {
        auto& value = std::get<ParamDictionary::NestedParams>(ft->second.value());
        it = value.begin();
        et = value.end();
        continue;
      }

      if(std::holds_alternative<rclcpp::Parameter>(ft->second.value()) && (i < split.size() - 1) )
      {
        what = "The Dictionary store a map of params param under the root param name '" + tree.name() +
               "'. The key '" + _key + "' is  not in the dictionary: " + it->second.to_string("/");
        return nullptr;
      }
    }
    return &(ft->second);
  }

  return nullptr;
}

bool _to_yaml(const ParamDictionary& tree, YAML::Node& node, std::string& what)
{
  if (std::holds_alternative<ParamDictionary::EmptyParam>(tree.value()))
  {
    what = "The param is empty";
    return false;
  }
  else if (std::holds_alternative<rclcpp::Parameter>(tree.value()))
  {
    auto & rclcpp_par = std::get<rclcpp::Parameter>(tree.value());
    std::string json_str = rclcpp::_to_json_dict_entry( rclcpp_par );
    YAML::Node yaml_node = YAML::Load(json_str);
    node[tree.name()] = YAML::Load(yaml_node[rclcpp_par.get_name()]["value"].as<std::string>());
  }
  else
  {
    auto & nested = std::get<ParamDictionary::NestedParams>(tree.value());
    for (const auto& p : nested)
    {
      if (!_to_yaml(p.second, node, what))
      {
        return false;
      }
    }
  }
  return true;
}


bool to_yaml(const ParamDictionary& tree, YAML::Node& node, std::string& what)
{
  YAML::Node yaml_node;
  if(!_to_yaml(tree, yaml_node, what))
  {
    return false;
  }

  node[tree.name()] = yaml_node;
  return true;
}

template<>
inline bool getNodeNames(const std::sahred_ptr<rclcpp::Node> node&, std::vector<std::string>& names, std::string& what)
{
  auto names = node->get_node_graph_interface()->get_node_names();
  return true;
}



template<>
inline bool resolveParamName(const std::sahred_ptr<rclcpp::Node> node, const std::string& name, std::string& resolved_name, std::string& what)
{
  resolved_name = name;
  return true;
}

}  // namespace ros2
}  // namespace param
}  // namespace cnr
}


}  // namespace ros2
}  // namespace param
}  // namespace cnr



namespace std
{

std::string to_string(const rclcpp::Parameter& val)
{
  return val.value_to_string();
}

std::string to_string(const std::shared_ptr<rclcpp::Parameter>& val)
{
  return val ? val->value_to_string() : "NULL";
}

std::string to_string(const cnr::param::ros2::ParamDictionary& val)
{
  return val.to_string("");
}
}  // namespace std

#endif