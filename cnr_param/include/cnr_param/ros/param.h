#ifndef CNR_PARAM__INCLUDE__CNR_PARAM__ROS__PARAM__H
#define CNR_PARAM__INCLUDE__CNR_PARAM__ROS__PARAM__H

#include <string>
#include <memory>

#include <ros/node_handle.h>
#include <cnr_param/core/type_traits.h>

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
 * @param what 
 * @return true 
 * @return false 
 */
bool has(const std::string& key, std::string& what);

/**
 * @brief 
 * 
 * @tparam T 
 * @param key 
 * @param ret 
 * @param what 
 * @param default_val 
 * @return true 
 * @return false 
 */
template<typename T>
bool get(const std::string& key, T& ret, std::string& what, const bool& implicit_cast_if_possible = true);

/**
 * @brief 
 * 
 * @tparam T 
 * @return true 
 * @return false 
 */
template<typename T>
bool set(const std::string&, const T&, std::string&);

/**
 * @brief 
 * 
 * @param node 
 */
void CNR_PARAM_INIT_ROS_MODULE(std::shared_ptr<::ros::NodeHandle> node);

void CNR_PARAM_CLEANUP_ROS_MODULE();

}  // namespace ros
}  // namespace param
}  // namespace cnr

#include <cnr_param/ros/impl/param.hpp>

#endif  /* CNR_PARAM__INCLUDE__CNR_PARAM__ROS__PARAM__H */

