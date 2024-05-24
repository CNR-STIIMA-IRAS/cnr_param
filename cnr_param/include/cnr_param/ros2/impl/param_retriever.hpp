#ifndef CNR_PARAM__INCLUDE__CNR_PARAM__ROS2__IMPL__PARAM_RETRIEVER__HPP
#define CNR_PARAM__INCLUDE__CNR_PARAM__ROS2__IMPL__PARAM_RETRIEVER__HPP

#include <cnr_param/ros2/param_retriever.h>

namespace cnr 
{
namespace param
{


template <typename T>
inline T get_from_variant<AllowedParamType,T>(const AllowedParamType& rhs, const bool& implicit_cast_if_possible)
{
  if (std::holds_alternative<bool>(rhs))
  {
    return ::cnr::param::implicit_cast<bool, T>(std::get<bool>(rhs));
  }
  if (std::holds_alternative<int64_t>(rhs))
  {
    return ::cnr::param::implicit_cast<int64_t, T>(std::get<int64_t>(rhs));
  }
  if (std::holds_alternative<double>(rhs))
  {
    return ::cnr::param::implicit_cast<double, T>(std::get<double>(rhs));
  }
  if (std::holds_alternative<std::string>(rhs))
  {
    return ::cnr::param::implicit_cast<std::string, T>(std::get<std::string>(rhs));
  }
  if (std::holds_alternative<std::vector<uint8_t>>(rhs))
  {
    return ::cnr::param::implicit_cast<std::vector<uint8_t>, T>(std::get<std::vector<uint8_t>>(rhs));
  }
  if (std::holds_alternative<std::vector<bool>>(rhs))
  {
    return ::cnr::param::implicit_cast<std::vector<bool>, T>(std::get<std::vector<bool>>(rhs));
  }
  if (std::holds_alternative<std::vector<int64_t>>(rhs))
  {
    return ::cnr::param::implicit_cast<std::vector<int64_t>, T>(std::get<std::vector<int64_t>>(rhs));
  }
  if (std::holds_alternative<std::vector<double>>(rhs))
  {
    return ::cnr::param::implicit_cast<std::vector<double>, T>(std::get<std::vector<double>>(rhs));
  }
  if (std::holds_alternative<std::vector<std::string>>(rhs))
  {
    return ::cnr::param::implicit_cast<std::vector<std::string>, T>(std::get<std::vector<std::string>>(rhs));
  }

  std::string err = __PRETTY_FUNCTION__ + std::string(":") + std::to_string(__LINE__) + ": " +
                    "Umatched implicit cast from 'void*' to '" +
                    boost::typeindex::type_id_with_cvr<decltype(T())>().pretty_name() + "'";
  throw std::runtime_error(err.c_str());

  return T();
}

template <>
inline bool getNodeNames(const std::shared_ptr<rclcpp::Node>& n, std::vector<std::string>& names, std::string& what)
{
  names = n->get_node_graph_interface()->get_node_names() ;
  return true;
}

template <>
inline bool resolveParamName(const std::shared_ptr<rclcpp::Node>&, const std::string& name, std::string& resolved_name, std::string&)
{
  name = resolved_name;
  return true;
}

template<>
inline bool resolveNodeName(const std::shared_ptr<rclcpp::Node>& n, std::string& resolved_name, std::string& what)
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
inline std::string lintParamKey(const std::shared_ptr<rclcpp::Node>&, const std::string& param_key)
{
  std::string ret = param_key;
  if (ret.front() == '/')
  {
    ret.erase(0, 1);
  }
  std::replace(ret.begin(), ret.end(), '/', '.');
  return ret;
}

}  // namespace param
}  // namespace cnr

#endif  // CNR_PARAM__INCLUDE__CNR_PARAM__ROS2__IMPL__PARAM_RETRIEVER__HPP