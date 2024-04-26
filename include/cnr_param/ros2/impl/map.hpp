#ifndef CNR_PARAM__INCLUDE__CNR_PARAM__ROS2__IMPL__MAP__HPP
#define CNR_PARAM__INCLUDE__CNR_PARAM__ROS2__IMPL__MAP__HPP

#include <cnr_param/ros2/param_retriever.h>

namespace cnr
{
namespace param
{
namespace ros2
{

/**
 * @brief Get the map object
 * 
 * @tparam T 
 * @param node 
 * @param ret 
 * @param what 
 * @return true 
 * @return false 
 */
template<typename T>
inline bool get_map(const ParamDictionary& node, T& ret, std::string& what)
{
  UNUSED(ret);
  what = "The type ' "
        + boost::typeindex::type_id_with_cvr<decltype(ret)>().pretty_name() 
          + "' is not supported. You must specilized your own 'get_map' template function"+ "' Node: \n" 
              + std::to_string(node) ;
  return false;
}


}  // namespace ros2
}  // namespace param
}  // namespace cnr

#endif  // CNR_PARAM__INCLUDE__CNR_PARAM__ROS2__IMPL__MAP__HPP