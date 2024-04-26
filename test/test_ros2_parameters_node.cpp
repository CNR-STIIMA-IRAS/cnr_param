#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class TestParams : public rclcpp::Node
{
public:
  TestParams()
    : Node(
          "test_params_rclcpp",
          rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true))
  {
    timer_ = this->create_wall_timer(500ms, std::bind(&TestParams::timer_callback, this));
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  int count_ = 0;
  void timer_callback()
  {
    if(count_++==0)
    {
      RCLCPP_INFO(this->get_logger(), "Dump parameters");
      auto vv = this->list_parameters({}, 1000);
      for (const auto& v : vv.names)
      {
        RCLCPP_INFO(this->get_logger(), "- %s", v.c_str());
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