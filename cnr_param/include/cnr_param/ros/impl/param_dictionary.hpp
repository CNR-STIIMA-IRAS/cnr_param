#ifndef CNR_PARAM__INCLUDE__CNR_PARAM__ROS__IMPL__PARAM_DICTIONARY__HPP
#define CNR_PARAM__INCLUDE__CNR_PARAM__ROS__IMPL__PARAM_DICTIONARY__HPP

#include <string>
#include <type_traits>
#include <vector>
#include <variant>

#include <yaml-cpp/yaml.h>


#include <ros/node_handle.h>
#include <ros/master.h>
#include <ros/names.h>
#include <ros/param.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <cnr_param/core/type_traits.h>
#include <cnr_param/core/string.h>
#include <cnr_param/ros/param_dictionary.h>

namespace cnr
{
namespace param
{
namespace core
{

SPECILIZE_C_TYPE_TO_PARAM_TYPE(cnr::param::ros::double_param);
SPECILIZE_C_TYPE_TO_PARAM_TYPE(cnr::param::ros::int_param);
SPECILIZE_C_TYPE_TO_PARAM_TYPE(cnr::param::ros::string_param);
SPECILIZE_C_TYPE_TO_PARAM_TYPE(cnr::param::ros::bool_param);
SPECILIZE_C_TYPE_TO_PARAM_TYPE(cnr::param::ros::bytes_param);

SPECILIZE_PARAM_TYPE_TO_C_TYPE(cnr::param::ros::double_param);
SPECILIZE_PARAM_TYPE_TO_C_TYPE(cnr::param::ros::int_param);
SPECILIZE_PARAM_TYPE_TO_C_TYPE(cnr::param::ros::string_param);
SPECILIZE_PARAM_TYPE_TO_C_TYPE(cnr::param::ros::bool_param);
SPECILIZE_PARAM_TYPE_TO_C_TYPE(cnr::param::ros::bytes_param);


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

template<>
inline cnr::param::ros::AllowedParamType as_generic<cnr::param::ros::AllowedParamType, XmlRpc::XmlRpcValue>(const XmlRpc::XmlRpcValue& param)
{
  cnr::param::ros::AllowedParamType ret;
  switch (param.getType())
  {
    case XmlRpc::XmlRpcValue::TypeBoolean:
      ret = (cnr::param::ros::bool_param::c_type)(param);
      break;
    case XmlRpc::XmlRpcValue::TypeInt:
      ret = (cnr::param::ros::int_param::c_type)(int(param));
      break;
    case XmlRpc::XmlRpcValue::TypeDouble:
      ret = (cnr::param::ros::double_param::c_type)(param);
      break;
    case XmlRpc::XmlRpcValue::TypeString:
      ret = (cnr::param::ros::string_param::c_type)(param);
      break;
    case XmlRpc::XmlRpcValue::TypeBase64:
      {
        cnr::param::ros::bytes_param::c_type vv = param;
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
inline bool to_yaml(const ParamDictionary<XmlRpc::XmlRpcValue>& tree, YAML::Node& node, std::string& what)
{
  return cnr::param::ros::to_yaml(tree,node,what);
}


}  // namespace core
}  // namespace param
}  // namespace cnr



#endif  // CNR_PARAM__INCLUDE__CNR_PARAM__ROS__IMPL__PARAM_DICTIONARY__HPP