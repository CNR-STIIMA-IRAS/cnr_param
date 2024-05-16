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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/utilities.hpp"

#include <cnr_param/ros2/param.h>
#include <cnr_param/ros2/impl/param.hpp>

#include <boost/interprocess/detail/os_file_functions.hpp>

// ====================================================================================================================
// === GLOBAL VARIABLES ===============================================================================================
// ====================================================================================================================
const std::string client_node_name = "parameters_client_node";
std::shared_ptr<rclcpp::Node> parameters_client_node;

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

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  parameters_client_node = rclcpp::Node::make_shared(client_node_name);

  rclcpp::executors::MultiThreadedExecutor executor;
  std::thread executor_thread;
  executor.add_node(parameters_client_node);

  executor_thread = std::thread(std::bind(&rclcpp::executors::MultiThreadedExecutor::spin, &executor));

  std::this_thread::sleep_for(1s);

  cnr::param::ros2::CNR_PARAM_INIT_RO2_MODULE(parameters_client_node);

  std::string what;
  std::vector<double> v_double;
  if(cnr::param::ros2::get("/ros2_parameters_server_node/double_array", v_double, what))
  {
    std::cout << "/ros2_parameters_server_node/double_array" << std::to_string(v_double) << std::endl;
  }
  else
  {
    std::cerr << what << std::endl;
  }

  if(cnr::param::ros2::get("/ros2_parameters_server_node/double_array_2", v_double, what))
  {
    std::cout << "/ros2_parameters_server_node/double_array_2" << std::to_string(v_double) << std::endl;
  }
  else
  {
    std::cerr << what << std::endl;
  }

  double val = 0.0;
  if(cnr::param::ros2::get("/ros2_parameters_server_node/double_value", val, what))
  {
    std::cout << "/ros2_parameters_server_node/double_value" << std::to_string(val) << std::endl;
  }
  else
  {
    std::cerr << what << std::endl;
  }


  std::vector<int> v_int;
  if(cnr::param::ros2::get("/ros2_parameters_server_node/int_array", v_int, what))
  {
    std::cout << "/ros2_parameters_server_node/v_int" << std::to_string(v_int) << std::endl;
  }
  else
  {
    std::cerr << what << std::endl;
  }
  if(cnr::param::ros2::get("/ros2_parameters_server_node/int_array_2", v_int, what))
  {
    std::cout << "/ros2_parameters_server_node/int_array_2" << std::to_string(v_int) << std::endl;
  }
  else
  {
    std::cerr << what << std::endl;
  }

  int val_int = 0;
  if(cnr::param::ros2::get("/ros2_parameters_server_node/int_value", val_int, what))
  {
    std::cout << "/ros2_parameters_server_node/int_value" << std::to_string(val_int) << std::endl;
  }
  else
  {
    std::cerr << what << std::endl;
  }

  std::vector<std::string> v_string;
  if(cnr::param::ros2::get("/ros2_parameters_server_node/string_array", v_string, what))
  {
    std::cout << "/ros2_parameters_server_node/string_array" << std::to_string(v_string) << std::endl;
  }
  else
  {
    std::cerr << what << std::endl;
  }

  std::string val_string;
  if(cnr::param::ros2::get("/ros2_parameters_server_node/string_value", val_string, what))
  {
    std::cout << "/ros2_parameters_server_node/string_value" << std::to_string(val_string) << std::endl;
  }
  else
  {
    std::cerr << what << std::endl;
  }

  std::vector<bool> v_bool;
  if(cnr::param::ros2::get("/ros2_parameters_server_node/bool_array", v_bool, what))
  {
    std::cout << "/ros2_parameters_server_node/bool_array" << std::to_string(v_bool) << std::endl;
  }
  else
  {
    std::cerr << what << std::endl;
  }

  bool val_bool = false;
  if(cnr::param::ros2::get("/ros2_parameters_server_node/bool_value", val_bool, what))
  {
    std::cout << "/ros2_parameters_server_node/bool_value" << std::to_string(val_bool) << std::endl;
  }
  else
  {
    std::cerr << what << std::endl;
  }

  std::vector<uint8_t> v_bytes;
  if(cnr::param::ros2::get("/ros2_parameters_server_node/bytes_array", v_bytes, what))
  {
    std::cout << "/ros2_parameters_server_node/bytes_array" << std::to_string(v_bytes) << std::endl;
  }
  else
  {
    std::cerr << what << std::endl;
  }

  if(cnr::param::ros2::get("/ros2_parameters_server_node/nested_param/another_int", val_int, what))
  {
    std::cout << "/ros2_parameters_server_node/nested_param/another_int" << std::to_string(val_int) << std::endl;
  }
  else
  {
    std::cerr << what << std::endl;
  }

  if(cnr::param::ros2::get("/ros2_parameters_server_node/nested_param.another_int", val_int, what))
  {
    std::cout << "/ros2_parameters_server_node/nested_param.another_int" << std::to_string(val_int) << std::endl;
  }
  else
  {
    std::cerr << what << std::endl;
  }

if(cnr::param::ros2::get("/ros2_parameters_server_node/nested_param/another_int", val_int, what))
  {
    std::cout << "/ros2_parameters_server_node/nested_param/another_int" << std::to_string(val_int) << std::endl;
  }
  else
  {
    std::cerr << what << std::endl;
  }

  if(cnr::param::ros2::get("/ros2_parameters_server_node/nested_param/nested_param/another_int2", val_int, what))
  {
    std::cout << "/ros2_parameters_server_node/nested_param/nested_param/another_int2" << std::to_string(val_int) << std::endl;
  }
  else
  {
    std::cerr << what << std::endl;
  }

  if(cnr::param::ros2::get("/ros2_parameters_server_node/nested_param.nested_param.another_int2", val_int, what))
  {
    std::cout << "/ros2_parameters_server_node/nested_param.nested_param.another_int2" << std::to_string(val_int) << std::endl;
  }
  else
  {
    std::cerr << what << std::endl;
  }


  if(cnr::param::ros2::get("/ros2_parameters_server_node/nested_param.nested_param/another_int2", val_int, what))
  {
    std::cout << "/ros2_parameters_server_node/nested_param.nested_param/another_int2" << std::to_string(val_int) << std::endl;
  }
  else
  {
    std::cerr << what << std::endl;
  }

  if(cnr::param::ros2::get("/ros2_parameters_server_node/nested_param/nested_param.another_int2", val_int, what))
  {
    std::cout << "/ros2_parameters_server_node/nested_param/nested_param.another_int2" << std::to_string(val_int) << std::endl;
  }
  else
  {
    std::cerr << what << std::endl;
  }
  

  if(cnr::param::ros2::get("/ros2_parameters_server_node/n1/n3/v1", v_string, what))
  {
    std::cout << "/ros2_parameters_server_node/n1/n3/v1" << std::to_string(v_string) << std::endl;
  }
  else
  {
    std::cerr << what << std::endl;
  }

  std::vector<std::vector<std::string>> vv_string;
  if(cnr::param::ros2::get("/ros2_parameters_server_node/n1/n4/vv1", vv_string, what))
  {
    std::cout << "/ros2_parameters_server_node/n1/n4/vv1" << std::to_string(vv_string) << std::endl;
  }
  else
  {
    std::cerr << what << std::endl;
  }


  cnr::param::ros2::CNR_PARAM_CLEANUP_RO2_MODULE();

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