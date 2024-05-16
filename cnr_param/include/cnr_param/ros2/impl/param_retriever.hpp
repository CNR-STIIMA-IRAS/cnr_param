#ifndef CNR_PARAM__INCLUDE__CNR_PARAM__ROS2__IMPL__PARAM_RETRIEVER__HPP
#define CNR_PARAM__INCLUDE__CNR_PARAM__ROS2__IMPL__PARAM_RETRIEVER__HPP

#include <cnr_param/ros2/param_retriever.h>

namespace cnr 
{
namespace param
{
namespace ros2
{

template <typename T>
inline T implicit_cast(const AllowedParamType& rhs)
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


}  // namespace ros2
}  // namespace param
}  // namespace cnr

#endif  // CNR_PARAM__INCLUDE__CNR_PARAM__ROS2__IMPL__PARAM_RETRIEVER__HPP