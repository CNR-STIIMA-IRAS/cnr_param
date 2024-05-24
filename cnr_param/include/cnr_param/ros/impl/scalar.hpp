#ifndef CNR_PARAM__INCLUDE__CNR_PARAM__ROS__IMPL__SCALAR__HPP
#define CNR_PARAM__INCLUDE__CNR_PARAM__ROS__IMPL__SCALAR__HPP

#include <ros/param.h>
#include <xmlrpcpp/XmlRpc.h>

#include <cnr_param/core/type_traits.h>
#include <xmlrpcpp/XmlRpcValue.h>
#include <cnr_param/ros/param.h>
#include "cnr_param/core/impl/param_retriever.hpp"

namespace std 
{ 
inline std::string to_string(const XmlRpc::XmlRpcValue& v)
{
  return v.toXml();
}
}


namespace cnr
{
namespace param
{
namespace ros
{

// =============================================================================================
// SCALAR
// =============================================================================================
template<typename T>
inline bool get_scalar(const ParamDictionary<XmlRpc::XmlRpcValue>& p, T& ret, std::string& what, const bool &implicit_cast_if_possible)
{
  what = "[Get Scalar] Dictionary:\n " + std::to_string(p) +"\n. "
    "Requested '" + boost::typeindex::type_id_with_cvr<decltype(ret)>().pretty_name() + "'";

  if(!p.is_scalar())
  {
    what = "The param dictionary is not mapping a scalar value as requested";
    return false;
  }

  try
  {
    if constexpr(!cnr::param::ros::is_ros_scalar_supported_param_type<T>::value)
    {
      what += "The parameter is an unknow type";
      return false;
    }
    else
    {
      auto rosparam =  std::get<XmlRpc::XmlRpcValue>(p.value());
    
      RosAllowedParamType v = as_generic<RosAllowedParamType, XmlRpc::XmlRpcValue>(rosparam);
      return get_from_variant(ret, v, implicit_cast_if_possible, what); 
    }
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

}  // namespace ros
}  // namespace param
}  // namespace cnr

#endif  /* CNR_PARAM__INCLUDE__CNR_PARAM__ROS__IMPL__SCALAR__HPP  */ 
