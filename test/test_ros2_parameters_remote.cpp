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

#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/utilities.hpp"

#include <cnr_param/ros2/param.h>
#include <cnr_param/ros2/impl/param.hpp>

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("get_remote_params");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread executor_thread(std::bind(&rclcpp::executors::MultiThreadedExecutor::spin, &executor));

  std::this_thread::sleep_for(1s);

  cnr::param::ros2::CNR_PARAM_INIT_RO2_MODULE(node);
  std::string what;
  std::vector<std::string> prefixes = {"/test_ros2_parameters_node/" };
  for (const auto& p : prefixes)
  {
    std::cout << "============================================================" << std::endl;
    {
      std::vector<bool> v_bool;
      if (cnr::param::ros2::get(p + "bool_array", v_bool, what))
      {
        std::cout << "OK! Value: " << std::to_string(v_bool) << std::endl;
      }
      else
      {
        std::cout << "Error: " << what << std::endl;
      }
    }
    #if 1
    std::cout << "============================================================" << std::endl;
    {
      bool val;
      if (cnr::param::ros2::get(p + "bool_value", val, what))
      {
        std::cout << "OK! Value: " << val << std::endl;
      }
      else
      {
        std::cout << "Error: " << what << std::endl;
      }
    }

    std::cout << "============================================================" << std::endl;
    {
      std::vector<double> val;
      if (cnr::param::ros2::get(p + "float_array", val, what))
      {
        std::cout << "OK! Value: " << std::to_string(val) << std::endl;
      }
      else
      {
        std::cout << "Error: " << what << std::endl;
      }
    }
        {
      std::vector<double> val;
      if (cnr::param::ros2::get(p + "float_array2", val, what))
      {
        std::cout << "OK! Value: " << std::to_string(val) << std::endl;
      }
      else
      {
        std::cout << "Error: " << what << std::endl;
      }
    }
    std::cout << "============================================================" << std::endl;
    {
      double val;
      if (cnr::param::ros2::get(p + "float_number", val, what))
      {
        std::cout << "OK! Value: " << val << std::endl;
      }
      else
      {
        std::cout << "Error: " << what << std::endl;
      }
    }
    std::cout << "============================================================" << std::endl;
    {
      std::vector<int> val;
      if (cnr::param::ros2::get(p + "int_array", val, what))
      {
        std::cout << "OK! Value: " << std::to_string(val) << std::endl;
      }
      else
      {
        std::cout << "Error: " << what << std::endl;
      }
    }
        {
      std::vector<int> val;
      if (cnr::param::ros2::get(p + "int_array2", val, what))
      {
        std::cout << "OK! Value: " << std::to_string(val) << std::endl;
      }
      else
      {
        std::cout << "Error: " << what << std::endl;
      }
    }
    std::cout << "============================================================" << std::endl;
    {
      int val;
      if (cnr::param::ros2::get(p + "int_number", val, what))
      {
        std::cout << "OK! Value: " << val << std::endl;
      }
      else
      {
        std::cout << "Error: " << what << std::endl;
      }
    }
    std::cout << "============================================================" << std::endl;
    {
      int val;
      if (cnr::param::ros2::get(p + "nested_param.another_int", val, what))
      {
        std::cout << "OK! Value: " << val << std::endl;
      }
      else
      {
        std::cout << "Error: " << what << std::endl;
      }
    }
    std::cout << "============================================================" << std::endl;
    {
      int val;
      if (cnr::param::ros2::get(p + "nested_param.another_int2", val, what))
      {
        std::cout << "OK! Value: " << val << std::endl;
      }
      else
      {
        std::cout << "Error: " << what << std::endl;
      }
    }    
    #endif
    std::cout << "============================================================" << std::endl;
    {
      int val;
      if (cnr::param::ros2::get(p + "nested_param.nested_param.another_int", val, what))
      {
        std::cout << "OK! Value: " << val << std::endl;
      }
      else
      {
        std::cout << "Error: " << what << std::endl;
      }
    }
    std::cout << "============================================================" << std::endl;
    {
      int val;
      if (cnr::param::ros2::get(p + "nested_param.nested_param.another_int2", val, what))
      {
        std::cout << "OK! Value: " << val << std::endl;
      }
      else
      {
        std::cout << "Error: " << what << std::endl;
      }
    }
    
    //#endif
  }

  sleep(5);
  rclcpp::shutdown();

  return 0;
}
