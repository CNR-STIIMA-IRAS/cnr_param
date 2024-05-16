#include <chrono>
#include <functional>
#include <memory>

#if ROS2_MODULE

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class TestParams : public rclcpp::Node
{
public:
  TestParams()
    : Node(
          "ros2_parameters_server_node",
          rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true))
  {
    timer_ = this->create_wall_timer(500ms, std::bind(&TestParams::timer_callback, this));
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<std::string> parameters_name_;

  void timer_callback()
  {
    if (parameters_name_.empty())
    {
      auto list = this->list_parameters({}, 1000);
      parameters_name_ = list.names;
      for (const auto& p : list.names)
      {
        std::cout << "- " << this->get_fully_qualified_name() << "." << p << std::endl;
      }
    }
  }
};

// Code below is just to start the node
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestParams>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

#else
int main(int argc, char** argv)
{
  std::cerr << "This test is the server to a ROS2 node, but the ROS2_MODULE is not defined.\n" << std::endl;
  return 0;
}
#endif