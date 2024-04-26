#ifndef CNR_PARAM__INCLUDE__CNR_PARAM__ROS2__IMPL__SEQUENCE__HPP
#define CNR_PARAM__INCLUDE__CNR_PARAM__ROS2__IMPL__SEQUENCE__HPP

#include <string>
#include <cnr_param/utils/type_traits.h>
#include <cnr_param/utils/eigen.h>

#include <cnr_param/ros2/param_retriever.h>

namespace cnr
{
namespace param
{
namespace ros2
{

// ===========================================================================================
// SEQUENCE
// =============================================================================================
template <typename T>
inline bool get_sequence(const ParamDictionary& p, T& ret, std::string& what)
{
  std::string err_msg = what = "Dictionary:\n " + std::to_string(p) + "\n. Requested '" +
                               boost::typeindex::type_id_with_cvr<decltype(ret)>().pretty_name() + "'";

  if (!p.initialized())
  {
    what = "Weird Error! The param is unitialized ... (Dictionary:\n " + std::to_string(p) +
           "\n. You tried to extract: '" + boost::typeindex::type_id_with_cvr<decltype(ret)>().pretty_name() + "'";
    return false;
  }
  if (!p.is_sequence())
  {
    what = "The param dictionary is not mapping a scalar value as requested";
    return false;
  }

  try
  {
    if constexpr (cnr::param::ros2::ParamType<T>::value == rclcpp::ParameterType::PARAMETER_NOT_SET)
    {
      what = "The parameter is an unknow type";
      return false;
    }
 
    auto ros2param =  std::get<rclcpp::Parameter>(p.value());
   
    auto v = as_generic(ros2param);
    ret = cnr::param::ros2::implicit_cast<T>(v);
    return true;
  }
  catch (rclcpp::ParameterTypeException& e)
  {
    what = "Error!" + err_msg + ". What: " + std::string(e.what());
  }
  catch (std::exception& e)
  {
    what = "Error!" + err_msg + ". What: " + std::string(e.what());
  }
  return false;
}
// =============================================================================================
// END SEQUENCE
// =============================================================================================

}  // namespace ros2
}  // namespace param
}  // namespace cnr

#endif  /* CNR_PARAM__INCLUDE__CNR_PARAM__ROS2__IMPL__SEQUENCE__HPP */