#ifndef CNR_PARAM__INCLUDE__CNR_PARAM__ROS__PARAM_RETRIEVER__H
#define CNR_PARAM__INCLUDE__CNR_PARAM__ROS__PARAM_RETRIEVER__H

#include <string>
#include <type_traits>
#include <vector>
#include <variant>
#include <Eigen/Core>
#include "cnr_param/core/impl/param_retriever.hpp"
#include <yaml-cpp/yaml.h>


#include <ros/node_handle.h>
#include <ros/master.h>
#include <ros/names.h>
#include <ros/param.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <cnr_param/core/type_traits.h>
#include <cnr_param/core/param_retriever.h>

namespace cnr
{
namespace param
{

using double_param  = type_holder<XmlRpc::XmlRpcValue::Type, XmlRpc::XmlRpcValue::TypeDouble, double>;
using int_param     = type_holder<XmlRpc::XmlRpcValue::Type, XmlRpc::XmlRpcValue::TypeInt, int32_t>;
using string_param  = type_holder<XmlRpc::XmlRpcValue::Type, XmlRpc::XmlRpcValue::TypeString, std::string>;
using bool_param    = type_holder<XmlRpc::XmlRpcValue::Type, XmlRpc::XmlRpcValue::TypeBoolean, bool>;
using bytes_param   = type_holder<XmlRpc::XmlRpcValue::Type, XmlRpc::XmlRpcValue::TypeBase64, std::vector<char>>;

SPECILIZE_C_TYPE_TO_PARAM_TYPE(double_param);
SPECILIZE_C_TYPE_TO_PARAM_TYPE(int_param);
SPECILIZE_C_TYPE_TO_PARAM_TYPE(string_param);
SPECILIZE_C_TYPE_TO_PARAM_TYPE(bool_param);
SPECILIZE_C_TYPE_TO_PARAM_TYPE(bytes_param);

SPECILIZE_PARAM_TYPE_TO_C_TYPE(double_param);
SPECILIZE_PARAM_TYPE_TO_C_TYPE(int_param);
SPECILIZE_PARAM_TYPE_TO_C_TYPE(string_param);
SPECILIZE_PARAM_TYPE_TO_C_TYPE(bool_param);
SPECILIZE_PARAM_TYPE_TO_C_TYPE(bytes_param);


// Alternative type for implicit conversion from XmlRpcValue::TypeDouble
template <> struct is_forward_implicit_conversion_allowed<XmlRpc::XmlRpcValue::TypeDouble, double> : std::true_type {};
template <> struct is_forward_implicit_conversion_allowed<XmlRpc::XmlRpcValue::TypeDouble, long double> : std::true_type {};
template <> struct is_forward_implicit_conversion_allowed<XmlRpc::XmlRpcValue::TypeDouble, float> : std::true_type {};
template <> struct is_forward_implicit_conversion_allowed<XmlRpc::XmlRpcValue::TypeDouble, int32_t> : std::true_type {};
template <> struct is_forward_implicit_conversion_allowed<XmlRpc::XmlRpcValue::TypeDouble, int64_t> : std::true_type {};
template <> struct is_forward_implicit_conversion_allowed<XmlRpc::XmlRpcValue::TypeDouble, int16_t> : std::true_type {};
template <> struct is_forward_implicit_conversion_allowed<XmlRpc::XmlRpcValue::TypeDouble, int8_t> : std::true_type {};

template <> struct is_forward_implicit_conversion_allowed<XmlRpc::XmlRpcValue::TypeInt, int32_t> : std::true_type {};
template <> struct is_forward_implicit_conversion_allowed<XmlRpc::XmlRpcValue::TypeInt, int64_t> : std::true_type {};
template <> struct is_forward_implicit_conversion_allowed<XmlRpc::XmlRpcValue::TypeInt, int16_t> : std::true_type {};
template <> struct is_forward_implicit_conversion_allowed<XmlRpc::XmlRpcValue::TypeInt, int8_t> : std::true_type {};
template <> struct is_forward_implicit_conversion_allowed<XmlRpc::XmlRpcValue::TypeInt, double> : std::true_type {};
template <> struct is_forward_implicit_conversion_allowed<XmlRpc::XmlRpcValue::TypeInt, long double> : std::true_type {};
template <> struct is_forward_implicit_conversion_allowed<XmlRpc::XmlRpcValue::TypeInt, float> : std::true_type {};

template <> struct is_forward_implicit_conversion_allowed<XmlRpc::XmlRpcValue::TypeString, std::string> : std::true_type {};

template <> struct is_forward_implicit_conversion_allowed<XmlRpc::XmlRpcValue::TypeBoolean, bool> : std::true_type {};
template <> struct is_forward_implicit_conversion_allowed<XmlRpc::XmlRpcValue::TypeBoolean, uint8_t> : std::true_type {};
template <> struct is_forward_implicit_conversion_allowed<XmlRpc::XmlRpcValue::TypeBoolean, uint16_t> : std::true_type {};
template <> struct is_forward_implicit_conversion_allowed<XmlRpc::XmlRpcValue::TypeBoolean, uint32_t> : std::true_type {};
template <> struct is_forward_implicit_conversion_allowed<XmlRpc::XmlRpcValue::TypeBoolean, uint64_t> : std::true_type {};

template <> struct is_forward_implicit_conversion_allowed<XmlRpc::XmlRpcValue::TypeBase64, std::vector<char> > : std::true_type {};
template <> struct is_forward_implicit_conversion_allowed<XmlRpc::XmlRpcValue::TypeBase64, std::vector<uint8_t> > : std::true_type {};
template <> struct is_forward_implicit_conversion_allowed<XmlRpc::XmlRpcValue::TypeBase64, std::vector<uint16_t> > : std::true_type {};
template <> struct is_forward_implicit_conversion_allowed<XmlRpc::XmlRpcValue::TypeBase64, std::vector<uint32_t> > : std::true_type {};
template <> struct is_forward_implicit_conversion_allowed<XmlRpc::XmlRpcValue::TypeBase64, std::vector<uint64_t> > : std::true_type {};

/**
 * @brief 
 * 
 */
using RosAllowedParamType = std::variant<double_param::c_type, int_param::c_type, string_param::c_type, bool_param::c_type, bytes_param::c_type>;


/**
 * @brief 
 * 
 * @param key 
 * @param val 
 * @return YAML::Node 
 */
inline YAML::Node _to_yaml(const std::string& key, const XmlRpc::XmlRpcValue& val)
{
  YAML::Node n;
  std::string what;
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
          YAML::Node element = _to_yaml("not_used", val[i]);
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
          nn.push_back(_to_yaml(it->first, it->second));
        }
        n[key] = nn;
      }
      break;
    default:
      break;
  }
  return n;
}

template<>
inline RosAllowedParamType as_generic<RosAllowedParamType, XmlRpc::XmlRpcValue>(const XmlRpc::XmlRpcValue& param)
{
  RosAllowedParamType ret;
  switch (param.getType())
  {
    case XmlRpc::XmlRpcValue::TypeBoolean:
      ret = (bool_param::c_type)(param);
      break;
    case XmlRpc::XmlRpcValue::TypeInt:
      ret = (int_param::c_type)(int(param));
      break;
    case XmlRpc::XmlRpcValue::TypeDouble:
      ret = (double_param::c_type)(param);
      break;
    case XmlRpc::XmlRpcValue::TypeString:
      ret = (string_param::c_type)(param);
      break;
    case XmlRpc::XmlRpcValue::TypeBase64:
      {
        bytes_param::c_type vv = param;
        ret = vv;
      }
      break;
    default:
      return nullptr;
  }
  return ret;
}






template<>
inline bool ParamDictionary<XmlRpc::XmlRpcValue>::is_scalar() const
{
  if (std::holds_alternative<XmlRpc::XmlRpcValue>(param_.second))
  {
    const auto& p = std::get<XmlRpc::XmlRpcValue>(param_.second);
    return p.getType() == XmlRpc::XmlRpcValue::TypeBoolean ||
           p.getType() == XmlRpc::XmlRpcValue::TypeInt ||
           p.getType() == XmlRpc::XmlRpcValue::TypeDouble ||
           p.getType() == XmlRpc::XmlRpcValue::TypeString;
  }
  return false;
}

template<>
inline bool ParamDictionary<XmlRpc::XmlRpcValue>::is_sequence() const
{
  if (std::holds_alternative<XmlRpc::XmlRpcValue>(param_.second))
  {
    const auto& p = std::get<XmlRpc::XmlRpcValue>(param_.second);
    return p.getType() == XmlRpc::XmlRpcValue::TypeArray && p.size();
  }
  return false;
}


template <>
inline std::string ParamDictionary<XmlRpc::XmlRpcValue>::to_string(const std::string& prefix) const
{
  std::string ret;
  ret = prefix + param_.first;
  if (std::holds_alternative<XmlRpc::XmlRpcValue>(param_.second))
  {
    ret += ": [type: XmlRpcc] " + std::get<XmlRpc::XmlRpcValue>(param_.second).toXml() + "\n";
  }
  else if (std::holds_alternative<typename ParamDictionary<XmlRpc::XmlRpcValue>::NestedParams>(param_.second))
  {
    ret += "[type: ParamDictionary<P>::NestedParams(" +
           std::to_string(std::get<typename ParamDictionary<XmlRpc::XmlRpcValue>::NestedParams>(param_.second).size()) + ")]\n";
    for (const auto& p : std::get<typename ParamDictionary<XmlRpc::XmlRpcValue>::NestedParams>(param_.second))
    {
      ret += p.second.to_string(prefix + param_.first + "/");
    }
  }
  return ret;
}

template <>
inline bool ParamRetriever<::ros::NodeHandle,XmlRpc::XmlRpcValue>::retrieve_parameters(const std::string& resolved_node_name,
                                                      const std::string& resolved_key, std::string& what, bool updated)
{
  ParamDictionary<XmlRpc::XmlRpcValue>& _node_dict = node_params(resolved_node_name);
  bool value_found = false;
  if (_node_dict.initialized())
  {
    value_found = (_node_dict.name() == resolved_key) ? true : bool(find(_node_dict, resolved_key, what));
  }
  if (updated || !value_found)
  {
    try
    {
      what = "Error in retriving the parameter '"+resolved_key+"' from the rosparam server. ";
      what += (resolved_node_name != g_absolute_param_resolution) ? "The node name is '"+resolved_node_name+"'" : "";
      XmlRpc::XmlRpcValue p;
      std::string _ns = (resolved_node_name == g_absolute_param_resolution) ? resolved_key : resolved_node_name + "/" + resolved_key;
      value_found = ::ros::param::get( _ns, p);
      if(value_found)
      {
        what = "";
        insert_dict(node_params(resolved_node_name), resolved_key, p);
        return true;
      }
      what += "Value not found";
      return false;
    }
    catch(::ros::InvalidNameException& e)
    {
      what += "Error message: " + std::string(e.what());
    }
    catch(std::exception& e)
    {
      what += "Error message: " + std::string(e.what());
    }
    catch(...)
    {
      what += "Unhandled Exception";
    }
  }
  return value_found;
  
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
template<>
inline bool ParamRetriever<::ros::NodeHandle,XmlRpc::XmlRpcValue>::get_existent_parameters(const std::string& node_name, const std::vector<std::string>& keys, 
  std::vector<XmlRpc::XmlRpcValue>& parameters, std::string& what)
{
  what.clear();
  for(const auto& k : keys)
  {
    XmlRpc::XmlRpcValue p;
    if(!::ros::param::get( "/" + node_name + "/" + k, p))
    {
      what += "Impossible to get the parameter '" + k + "'. ";
      continue;
    }
    parameters.push_back(p);
  }
  return keys.size() == parameters.size();
}

/**
 * @brief 
 * 
 * @tparam  
 * @param tree 
 * @param node 
 * @param what 
 * @return true 
 * @return false 
 */
template<>
inline bool to_yaml(const ParamDictionary<XmlRpc::XmlRpcValue>& tree, YAML::Node& node, std::string& )
{
  auto & par = std::get<XmlRpc::XmlRpcValue>(tree.value());
  node[tree.name()] = _to_yaml("", par);
  return true;
}


template<>
inline bool getNodeNames(const std::shared_ptr<::ros::NodeHandle>&, std::vector<std::string>& names, std::string& what)
{
  if(!::ros::master::getNodes(names))
  {
    what = "Impossible to retrive the node names";
    return false;
  }
  return true;
}

template<>
inline bool resolveParamName(const std::shared_ptr<::ros::NodeHandle>&, const std::string& name, std::string& resolved_name, std::string& what)
{
  try
  {
    resolved_name = ::ros::names::resolve(name);

  }
  catch(::ros::InvalidNameException& e)
  {
    what = "Impossible to retrive the name '"+name+"'. Error: " + std::string(e.what());
    return false;
  }
  catch(std::exception& e)
  {
    what = "Impossible to retrive the name '"+name+"'. Error: " + std::string(e.what());
    return false;
  }
  catch(...)
  {
    what = "Impossible to retrive the name '"+name+"'. Error: Unhandled expcetion";
    return false;
  }
  return true;
}


template<>
inline bool resolveNodeName(const std::shared_ptr<::ros::NodeHandle>& n, std::string& resolved_name, std::string& what)
{
  if(!n)
  {
    what = "The node handle is not valid";
    return false;
  }

  resolved_name = n->getNamespace();
  return true;
}


template <>
inline std::string lintParamKey(const std::shared_ptr<::ros::NodeHandle>&, const std::string& param_key)
{
  std::string ret = param_key;
  std::replace(ret.begin(), ret.end(), '.', '/');
  return ret;
}


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
template<>
inline std::string to_string(const cnr::param::ParamDictionary<XmlRpc::XmlRpcValue>& val)
{
  YAML::Node n;
  std::string what;
  if(!to_yaml(val, n, what))
  {
    return "!!!Error in create the string for the parameter '" + val.name() + "'. Error: " + what;
  }
  std::stringstream ss;
  ss << n;
  return ss.str();
}

}  // namespace std


#endif  // CNR_PARAM__INCLUDE__CNR_PARAM__ROS__PARAM_RETRIEVER__H