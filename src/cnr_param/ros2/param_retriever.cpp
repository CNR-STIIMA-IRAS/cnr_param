#include <string>
#include <chrono>

using namespace std::chrono_literals;

#include <cnr_param/ros2/impl/param_retriever.hpp>

namespace cnr
{
namespace param
{
namespace ros2
{

std::map<std::string, rclcpp::AsyncParametersClient::SharedPtr>& parameters_client()
{
  static std::map<std::string, rclcpp::AsyncParametersClient::SharedPtr> __pc;
  return __pc;
}

bool init_async_params_client(const std::string& node_name, const std::shared_ptr<rclcpp::Node>& node)
{
  if (!node)
  {
    return false;
  }

  if (node_name == node->get_fully_qualified_name())
  {
    return true;
  }

  if (parameters_client().find(node_name) == parameters_client().end() || parameters_client().at(node_name) == nullptr)
  {
    parameters_client()[node_name] = std::make_shared<rclcpp::AsyncParametersClient>(node, node_name);
    while (!parameters_client()[node_name]->wait_for_service(1s))
    {
      if (!rclcpp::ok())
      {
        return false;
      }
    }
  }
  return true;
}

}  // namespace ros2
}  // namespace param
}  // namespace cnr