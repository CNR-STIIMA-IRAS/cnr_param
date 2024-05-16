#ifndef CNR_PARAM__INCLUDE__CNR_PARAM__ROS2__IMPL__SCALAR__HPP
#define CNR_PARAM__INCLUDE__CNR_PARAM__ROS2__IMPL__SCALAR__HPP

#include "rclcpp/parameter_value.hpp"

#include <cnr_param/core/type_traits.h>
#include <cnr_param/ros2/param_retriever.h>

namespace cnr
{
namespace param
{
namespace ros2
{


// =============================================================================================
// SCALAR
// =============================================================================================
template<typename T>
inline bool get_scalar(const ParamDictionary& p, T& ret, std::string& what)
{
  what = "Dictionary:\n " + std::to_string(p) +"\n. "
    "Requested '" + boost::typeindex::type_id_with_cvr<decltype(ret)>().pretty_name() + "'";

  if (!p.initialized())
  {
    what = "Weird Error! The param is unitialized ... (Dictionary:\n " + std::to_string(p) +
            "\n. You tried to extract: '" + boost::typeindex::type_id_with_cvr<decltype(ret)>().pretty_name() + "'";
    return false;
  }
  if(!p.is_scalar())
  {
    what = "The param dictionary is not mapping a scalar value as requested";
    return false;
  }

  try
  {
    if constexpr(cnr::param::ros2::ParamType<T>::value == rclcpp::ParameterType::PARAMETER_NOT_SET)
    {
      what = "The parameter is an unknow type";
      return false;
    }
    auto ros2param =  std::get<rclcpp::Parameter>(p.value());
   
    auto v = as_generic(ros2param);
    ret = cnr::param::ros2::implicit_cast<T>(v);
    return true;
  }
  catch (rclcpp::ParameterTypeException & e)               
  {
    what = "Error!" + what + ":" + std::string(e.what());   
  }
  catch (std::exception & e)            
  {
    what = "Error!" + what + ":" + std::string(e.what());   
  }
  return false;
}

// =============================================================================================
// END SCALAR
// =============================================================================================

}  // namespace ros2
}  // namespace param
}  // namespace cnr

#endif  /* CNR_PARAM__INCLUDE__CNR_PARAM__ROS2__IMPL__SCALAR__HPP  */ 
