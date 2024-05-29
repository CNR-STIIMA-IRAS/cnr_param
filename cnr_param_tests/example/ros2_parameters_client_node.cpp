// Copyright 2015 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <thread>
#include <vector>
#include <iostream>

#if ROS2_MODULE

#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/utilities.hpp"

#include <cnr_param/cnr_param.h>

using namespace std::chrono_literals;

class TestParams : public rclcpp::Node
{
public:
  TestParams(const std::string& node_name)
    : Node(
          node_name,
          rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true))
  {
    timer_ = this->create_wall_timer(500ms, std::bind(&TestParams::timer_callback, this));
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  void timer_callback()
  {
  }
};

template<typename T>
bool call(const std::string& key, T& value)
{
  std::string what;
  if(!cnr::param::get(key, value, what))
  {
    std::cerr << "Key: "<< key << ", What: " << what << std::endl;
    return false;
  }
  std::cout << "Key: "<< key << ", Value: " << std::to_string(value) << std::endl;
  return true;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  const std::string client_node_name = "parameters_client_node";
  std::shared_ptr<rclcpp::Node> parameters_client_node = rclcpp::Node::make_shared(client_node_name);

  rclcpp::executors::MultiThreadedExecutor executor;
  std::thread executor_thread;
  executor.add_node(parameters_client_node);

  executor_thread = std::thread(std::bind(&rclcpp::executors::MultiThreadedExecutor::spin, &executor));

  std::this_thread::sleep_for(1s);

  // Initialize the module
  cnr::param::ros2::CNR_PARAM_INIT_ROS2_MODULE(parameters_client_node);

  // Get the data
  std::string what;
  std::vector<double> v_double;
  call("/ros2_parameters_server_node/double_array", v_double);
  call("/ros2_parameters_server_node/double_array_2", v_double);

  double val = 0.0;
  call("/ros2_parameters_server_node/double_value", val);
  
  std::vector<int> v_int;
  call("/ros2_parameters_server_node/int_array", v_int);
  call("/ros2_parameters_server_node/int_array_2", v_int);
  
  int val_int = 0;
  call("/ros2_parameters_server_node/int_value", val_int);
  
  std::vector<std::string> v_string;
  call("/ros2_parameters_server_node/string_array", v_string);
  
  std::string val_string;
  call("/ros2_parameters_server_node/string_value", val_string);
  
  std::vector<bool> v_bool;
  call("/ros2_parameters_server_node/bool_array", v_bool);
  
  std::vector<uint8_t> v_bytes;
  
  call("/ros2_parameters_server_node/nested_param/another_int", val_int);
  
  call("/ros2_parameters_server_node/nested_param.another_int", val_int);
  
  call("/ros2_parameters_server_node/nested_param/another_int", val_int);
  
  call("/ros2_parameters_server_node/nested_param/nested_param/another_int2", val_int);
  
  call("/ros2_parameters_server_node/nested_param.nested_param.another_int2", val_int);
  
  call("/ros2_parameters_server_node/nested_param.nested_param/another_int2", val_int);
  
  call("/ros2_parameters_server_node/nested_param/nested_param.another_int2", val_int);
  
  call("/ros2_parameters_server_node/n1/n3/v1", v_string);
  
  std::vector<std::vector<std::string>> vv_string;
  call("/ros2_parameters_server_node/n1/n4/vv1", vv_string);
  
  cnr::param::ros2::CNR_PARAM_CLEANUP_ROS2_MODULE();

  executor.cancel();
  executor_thread.join();
  parameters_client_node.reset();
  rclcpp::shutdown();
  return 0;
}

#else
int main(int argc, char** argv)
{
  std::cerr << "This test is the remote accessor to a ROS2 node, but the ROS2_MODULE is not defined.\n" << std::endl;
  return 0;
}
#endif