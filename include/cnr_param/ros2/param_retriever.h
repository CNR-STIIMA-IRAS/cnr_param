#ifndef CNR_PARAM__INCLUDE__CNR_PARAM__ROS2__PARAM_RETRIEVER__H
#define CNR_PARAM__INCLUDE__CNR_PARAM__ROS2__PARAM_RETRIEVER__H

#include <string>
#include <vector>
#include <map>
#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

#include <rclcpp/node.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_client.hpp>

#include <cnr_yaml/type_traits.h>
#include <cnr_param/core/param_retriever.h>
#include <cnr_param/ros2/param_dictionary.h>

namespace cnr
{
namespace param
{
namespace ros2
{

std::map<std::string, rclcpp::AsyncParametersClient::SharedPtr>& parameters_client();
bool init_async_params_client(const std::string& node_name, const std::shared_ptr<rclcpp::Node>& node);

using ParamRetriever = cnr::param::core::ParamRetriever<rclcpp::Node, rclcpp::Parameter>;

}  // namespace ros2
}  // namespace param
}  // namespace cnr


#include <cnr_param/ros2/impl/param_retriever.hpp>

#endif  // CNR_PARAM__INCLUDE__CNR_PARAM__ROS2__PARAM_RETRIEVER__H