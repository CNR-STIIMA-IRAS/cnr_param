#ifndef CNR_PARAM__INCLUDE__CNR_PARAM__ROS2__IMPL__SEQUENCE__HPP
#define CNR_PARAM__INCLUDE__CNR_PARAM__ROS2__IMPL__SEQUENCE__HPP

#include <boost/mpl/equal_to.hpp>
#include <string>
#include <xmlrpcpp/XmlRpcValue.h>
#include <cnr_param/ros/impl/param_retriever.hpp>
#include <cnr_param/core/param.h>
#include <cnr_param/core/type_traits.h>
#include <cnr_param/core/eigen.h>

#include <cnr_param/ros/impl/param.hpp>

namespace cnr
{
namespace param
{
namespace ros
{
// ===========================================================================================
// SEQUENCE
// =============================================================================================
template <typename T>
inline bool get_sequence(const cnr::param::ParamDictionary<XmlRpc::XmlRpcValue>& p, T& ret, std::string& what)
{
  std::string err_msg = what = "[Get Sequence] Dictionary:\n " + std::to_string(p) + "\n. Requested '" +
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
    auto param =  std::get<XmlRpc::XmlRpcValue>(p.value());
    auto node = cnr::param::to_yaml("not_used", param);
    return cnr::param::core::get_sequence(node["not_used"], ret, what);
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