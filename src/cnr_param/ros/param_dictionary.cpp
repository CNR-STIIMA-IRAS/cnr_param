#include <string>
#include <xmlrpcpp/XmlRpc.h>
#include <yaml-cpp/yaml.h>

#include <cnr_yaml/string.h>
#include <cnr_param/core/string.h>
#include <cnr_param/core/param_dictionary.h>
#include <cnr_param/ros/param_dictionary.h>

namespace cnr
{
namespace param
{
namespace ros
{


/**
 * @brief 
 * 
 * @param key 
 * @param val 
 * @return YAML::Node 
 */
bool to_yaml(const std::string& key, const XmlRpc::XmlRpcValue& val, YAML::Node& n, std::string& what)
{
  switch (val.getType())
  {
    case XmlRpc::XmlRpcValue::TypeBoolean:
      n[key] = bool(val);
      break;
    case XmlRpc::XmlRpcValue::TypeInt:
      n[key] = int(val);
      break;
    case XmlRpc::XmlRpcValue::TypeDouble:
      n[key] = double(val);
      break;
    case XmlRpc::XmlRpcValue::TypeString:
      n[key] = std::string(val);
      break;
    case XmlRpc::XmlRpcValue::TypeBase64:
      {
        std::vector<char> vv = val;
        n[key] = vv;
      }
      break;
    case XmlRpc::XmlRpcValue::TypeArray:
      {
        YAML::Node nn(YAML::NodeType::Sequence);
        for(int i = 0; i < val.size(); i++)
        {
          YAML::Node element;
          if(!to_yaml("not_used", val[i], element, what))
          {
            return false;
          }
          nn.push_back(element["not_used"]);
        }
        n[key] = nn;
      }
      break;
    case XmlRpc::XmlRpcValue::TypeStruct:
      {
        YAML::Node nn(YAML::NodeType::Map);
        for(auto it = val.begin(); it != val.end(); it++)
        {
          YAML::Node element;
          if(!to_yaml(it->first, it->second, element, what))
          {
            return false;
          }
          nn.push_back(element);
        }
        n = nn;
      }
      break;
    default:
      break;
  }
  return n;
}


bool to_yaml(const cnr::param::core::ParamDictionary<XmlRpc::XmlRpcValue>& tree, YAML::Node& node, std::string& what)
{
  if (std::holds_alternative<cnr::param::core::ParamDictionary<XmlRpc::XmlRpcValue>::EmptyParam>(tree.value()))
  {
    what = "The param is empty";
    return false;
  }
  else if (std::holds_alternative<XmlRpc::XmlRpcValue>(tree.value()))
  {
    auto& rosparam = std::get<XmlRpc::XmlRpcValue>(tree.value());
    YAML::Node leaf;
    if(!to_yaml(tree.name(), rosparam, leaf, what))
    {
      return false;
    }
    node[tree.name()] = leaf[tree.name()];
  }
  else
  {
    auto& nested = std::get<cnr::param::core::ParamDictionary<XmlRpc::XmlRpcValue>::NestedParams>(tree.value());
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

}  // namespace ros
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
std::string to_string(const XmlRpc::XmlRpcValue& val)
{
  YAML::Node n;
  std::string what;
  cnr::param::ros::to_yaml("not_used", val, n, what);
  return std::to_string(n["not_used"]);
}

/**
 * @brief 
 * 
 * @param val 
 * @return std::string 
 */
std::string to_string(const std::shared_ptr<XmlRpc::XmlRpcValue>& val)
{
  return to_string(*val);
}

/**
 * @brief 
 * 
 * @param val 
 * @return std::string 
 */
std::string to_string(const cnr::param::ros::ParamDictionary& val)
{
  YAML::Node n;
  std::string what;
  if(!to_yaml(val, n, what))
  {
    return "!!!Error in create the string for the parameter '" + val.name() + "'. Error: " + what;
  }
  return std::to_string(n);
}

}  // namespace std
