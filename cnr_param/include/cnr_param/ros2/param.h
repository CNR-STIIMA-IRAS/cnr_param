#ifndef CNR_PARAM__INCLUDE__CNR_PARAM__ROS2__PARAM__H
#define CNR_PARAM__INCLUDE__CNR_PARAM__ROS2__PARAM__H

#include <string>
#include <Eigen/Core>
#include <memory>

#include <rclcpp/node.hpp>
#include <rclcpp/parameter.hpp>

#include <cnr_param/core/param.h>
#include <cnr_param/ros2/param_retriever.h>

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
inline bool get(const std::string& key, T& ret, std::string& what, const bool &implicit_cast_if_possible);

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
void CNR_PARAM_INIT_ROS2_MODULE(std::shared_ptr<rclcpp::Node>& node);

void CNR_PARAM_CLEANUP_ROS2_MODULE();

std::shared_ptr<rclcpp::Node>& background_node();
std::shared_ptr<ParamRetriever>& param_retriever();


template<typename T>
struct is_supported_param_type
{
  static constexpr bool value =  
    cnr::param::is_scalar<T>::value               || 
    cnr::param::is_double_std_v<T>::value         ||
    cnr::param::is_integer_v<T>::value            ||
    cnr::param::is_bool_v<T>::value               ||
    cnr::param::is_string_v<T>::value             ||
    std::is_same<rclcpp::Parameter, T>::value;
};

template<typename T>
struct is_scalar_supported_param_type
{
  static constexpr bool value =  
    cnr::param::is_scalar<T>::value               || 
    std::is_same<rclcpp::Parameter, T>::value;
};

template<typename T>
struct is_ros_vector_supported_param_type
{
  static constexpr bool value =  
    cnr::param::is_double_std_v<T>::value          ||
    cnr::param::is_integer_v<T>::value             ||
    cnr::param::is_unsigned_integer_v<T>::value    ||
    cnr::param::is_char_v<T>::value                ||
    cnr::param::is_unsigned_char_v<T>::value       ||
    cnr::param::is_bool_v<T>::value                ||
    cnr::param::is_string_v<T>::value;
};


}  // namespace ros2
}  // namespace param
}  // namespace cnr

#include <cnr_param/ros2/impl/param.hpp>

#endif  /* CNR_PARAM__INCLUDE__CNR_PARAM__ROS2__PARAM__H */

