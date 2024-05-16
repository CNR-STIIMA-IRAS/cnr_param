#ifndef CNR_PARAM__INCLUDE__CNR_PARAM__ROS2__PARAM__H
#define CNR_PARAM__INCLUDE__CNR_PARAM__ROS2__PARAM__H

#include <string>
#include <Eigen/Core>
#include <rclcpp/node.hpp>
#include <memory>

namespace cnr 
{
namespace param
{
namespace ros2
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
inline bool get(const std::string& key, T& ret, std::string& what);

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
 * @tparam T 
 * @param key 
 * @param ret 
 * @param what 
 * @param default_val 
 * @return true 
 * @return false 
 */
template<typename T>
bool get(const std::string& key, T& ret, std::string& what, const T& default_val);

/**
 * @brief 
 * 
 */
class ParamRetriever;

/**
 * @brief 
 * 
 * @param node 
 */
void CNR_PARAM_INIT_RO2_MODULE(std::shared_ptr<rclcpp::Node>& node);

void CNR_PARAM_CLEANUP_RO2_MODULE();

/**
 * @brief 
 * 
 * @return std::shared_ptr<rclcpp::Node> 
 */
std::shared_ptr<rclcpp::Node> node();

/**
 * @brief 
 * 
 * @return std::shared_ptr<cnr::param::ros2::ParamRetriever> 
 */
std::shared_ptr<ParamRetriever> pr();

}  // namespace ros2
}  // namespace param
}  // namespace cnr

#include <cnr_param/ros2/impl/param.hpp>

#endif  /* CNR_PARAM__INCLUDE__CNR_PARAM__ROS2__PARAM__H */

