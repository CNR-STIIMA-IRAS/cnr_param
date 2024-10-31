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

#include <gtest/gtest.h>

#if ROS2_MODULE

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/utilities.hpp"

#include <cnr_param/ros2/param.h>
#include <cnr_param/ros2/impl/param.hpp>

#include <boost/interprocess/detail/os_file_functions.hpp>

namespace detail
{
struct unwrapper
{
  explicit unwrapper(std::exception_ptr pe) : pe_(pe)
  {
  }

  operator bool() const
  {
    return bool(pe_);
  }

  friend auto operator<<(std::ostream& os, unwrapper const& u) -> std::ostream&
  {
    try
    {
      std::rethrow_exception(u.pe_);
      return os << "no exception";
    }
    catch (std::runtime_error const& e)
    {
      return os << "runtime_error: " << e.what();
    }
    catch (std::logic_error const& e)
    {
      return os << "logic_error: " << e.what();
    }
    catch (std::exception const& e)
    {
      return os << "exception: " << e.what();
    }
    catch (...)
    {
      return os << "non-standard exception";
    }
  }
  std::exception_ptr pe_;
};
}  // namespace detail

auto unwrap(std::exception_ptr pe)
{
  return detail::unwrapper(pe);
}

template <class F>
::testing::AssertionResult does_not_throw(F&& f)
{
  try
  {
    f();
    return ::testing::AssertionSuccess();
  }
  catch (...)
  {
    return ::testing::AssertionFailure() << unwrap(std::current_exception());
  }
}

std::map<std::string, std::map<std::string, std::vector<double>>> statistics;

#define EXECUTION_TIME(hdr, ...)                                                                                            \
  {                                                                                                                    \
    struct timespec start, end;                                                                                        \
    clock_gettime(CLOCK_MONOTONIC, &start);                                                                            \
    __VA_ARGS__;                                                                                                       \
    clock_gettime(CLOCK_MONOTONIC, &end);                                                                              \
    double time_taken;                                                                                                 \
    time_taken = double(end.tv_sec - start.tv_sec) * 1e9;                                                              \
    time_taken = double(time_taken + (end.tv_nsec - start.tv_nsec)) * 1e-9;                                            \
    std::cout << hdr << ": Elapsed time [us]: " << time_taken * 1e6 << std::endl;                                               \
  }

// ====================================================================================================================
// === GLOBAL VARIABLES ===============================================================================================
// ====================================================================================================================
const std::string server_node_name = "parameters_server_node";
const std::string client_node_name = "parameters_client_node";
std::shared_ptr<rclcpp::Node> parameters_client_node;

TEST(ROS2Module, Initialization)
{
  EXPECT_TRUE(does_not_throw([&] { EXECUTION_TIME( "Init", cnr::param::ros2::CNR_PARAM_INIT_ROS2_MODULE(parameters_client_node); ) }));
};

template<typename T>
bool call(const std::string& key, T& value, bool implicit_cast = true)
{
  std::string what;
  bool ok = false;
  EXECUTION_TIME(key, ok = cnr::param::ros2::get("/"+server_node_name+"/"+key, value, what, implicit_cast); );
  if(!ok)
  {
    std::cerr << "What: " << what << std::endl;
    return false;
  }
  return ok;
}

template<typename T>
bool send(const std::string& key, T& value)
{
  std::string what;
  bool ok = false;
  EXECUTION_TIME(key, ok = cnr::param::ros2::set("/"+server_node_name+"/"+key, value, what))
  return ok;
}

namespace cnr { 
namespace yaml 
{
template <>
struct decoding_type_variant_holder<int>
{
  using base = int;
  using variant = std::variant<int32_t, int64_t, int16_t, int8_t, double, long double, float>;
};

template <>
struct decoding_type_variant_holder<double>
{
  using base = int;
  using variant = std::variant<double, long double, float, int32_t, int64_t, int16_t, int8_t>;
};

}
}

TEST(ROS2Module, ClientUsageBasicTypes)
{
  std::string what;
  std::vector<double> v_double;
  EXPECT_TRUE(call("double_array", v_double));
  EXPECT_TRUE(v_double.size() == 2 && v_double[0] == 7.5 && v_double[1] == 400.4);

  EXPECT_TRUE(call("double_array_2", v_double));
  EXPECT_TRUE(v_double.size() == 2 && v_double[0] == 7 && v_double[1] == 400);

  double val = 0.0;
  EXPECT_TRUE(call("double_value", val));
  EXPECT_TRUE(val == 3.14);

  std::vector<int> v_int;
  EXPECT_TRUE(call("int_array", v_int));
  EXPECT_TRUE(v_int.size() == 4 && v_int[0] == 10 && v_int[1] == 11 && v_int[2] == 12 && v_int[3] == 13);

  EXPECT_FALSE(call("int_array_2", v_int, false));
  EXPECT_TRUE (call("int_array_2", v_int, true));
  EXPECT_TRUE(v_int.size() == 4 && v_int[0] == 10 && v_int[1] == 11 && v_int[2] == 12 && v_int[3] == 13);

  int val_int = 0;
  EXPECT_TRUE(call("int_value", val_int));
  EXPECT_TRUE(val_int == 5);

  std::vector<std::string> v_string;
  EXPECT_TRUE(call("string_array", v_string));
  EXPECT_TRUE(v_string.size() == 3 && v_string[0] == "Nice" && v_string[1] == "more" && v_string[2] == "params");

  std::string val_string;
  EXPECT_TRUE(call("string_value", val_string));
  EXPECT_TRUE(val_string == "Hello Universe");

  std::vector<bool> v_bool;
  EXPECT_TRUE(call("bool_array", v_bool));
  EXPECT_TRUE(v_bool.size() == 3 && v_bool[0] && !v_bool[1] && v_bool[2]);

  bool val_bool = false;
  EXPECT_TRUE(call("bool_value", val_bool));
  EXPECT_TRUE(val_bool);

  std::vector<uint8_t> v_bytes;
  EXPECT_TRUE(call("bytes_array", v_bytes));
  EXPECT_TRUE(v_bytes.size() == 3 && v_bytes[0] == 0x01 && v_bytes[1] == 0xF1 && v_bytes[2] == 0xA2);

  EXPECT_TRUE(call("nested_param/another_int", val_int));
  EXPECT_TRUE(call("nested_param.another_int", val_int));
  EXPECT_TRUE(val_int == 7);

  EXPECT_TRUE(call("nested_param/another_int2", val_int));
  EXPECT_TRUE(call("nested_param.another_int2", val_int));
  EXPECT_TRUE(val_int == 7);

  EXPECT_TRUE(
      call("nested_param/nested_param/another_int2", val_int));
  EXPECT_TRUE(
      call("nested_param.nested_param.another_int2", val_int));
  EXPECT_TRUE(
      call("nested_param.nested_param/another_int2", val_int));
  EXPECT_TRUE(
      call("nested_param/nested_param.another_int2", val_int));
  EXPECT_TRUE(val_int == 7);
}

TEST(ROS2Module, ClientUsageSetBasicTypes)
{
  std::string what;
  std::vector<double> v_double{7.5, 400.4};
  EXPECT_TRUE(send("darray", v_double));
  EXPECT_TRUE(call("darray", v_double));
  EXPECT_TRUE(v_double.size() == 2 && v_double[0] == 7.5 && v_double[1] == 400.4);

  double val = 3.14;
  EXPECT_TRUE(send("dvalue", val));
  EXPECT_TRUE(call("dvalue", val));
  EXPECT_TRUE(val == 3.14);

  std::vector<int> v_int{10, 11, 12, 13};
  EXPECT_TRUE(send("iarray", v_int));
  EXPECT_TRUE(call("iarray", v_int));
  EXPECT_TRUE(v_int.size() == 4 && v_int[0] == 10 && v_int[1] == 11 && v_int[2] == 12 && v_int[3] == 13);

  int val_int = 5;
  EXPECT_TRUE(send("ivalue", val_int));
  EXPECT_TRUE(call("ivalue", val_int));
  EXPECT_TRUE(val_int == 5);

  std::vector<std::string> v_string{"Nice", "more", "params"};
  EXPECT_TRUE(send("sarray", v_string));
  EXPECT_TRUE(call("sarray", v_string));
  EXPECT_TRUE(v_string.size() == 3 && v_string[0] == "Nice" && v_string[1] == "more" && v_string[2] == "params");

  std::string val_string = "Hello Universe";
  EXPECT_TRUE(send("svalue", val_string));
  EXPECT_TRUE(send("svalue", val_string));
  EXPECT_TRUE(val_string == "Hello Universe");

  std::vector<bool> v_bool{true,false,true};
  EXPECT_TRUE(send("barray", v_bool));
  EXPECT_TRUE(v_bool.size() == 3 && v_bool[0] && !v_bool[1] && v_bool[2]);

  bool val_bool = true;
  EXPECT_TRUE(send("bvalue", val_bool));
  EXPECT_TRUE(send("bvalue", val_bool));
  EXPECT_TRUE(val_bool);

  std::vector<uint16_t> v_bytes{0x01, 0xF, 0x1A};
  EXPECT_TRUE(send("by_array", v_bytes));
  EXPECT_TRUE(call("by_array", v_bytes));
  EXPECT_TRUE(v_bytes.size() == 3 && v_bytes[0] == 0x01 && v_bytes[1] == 0xF && v_bytes[2] == 0x1A);

  EXPECT_TRUE(send("n_param/another_int", val_int));
  EXPECT_TRUE(call("n_param/another_int", val_int));
  EXPECT_TRUE(send("n_param.another_int", val_int));
  EXPECT_TRUE(call("n_param.another_int", val_int));
  EXPECT_TRUE(val_int == 5);

  EXPECT_TRUE(send("n_param/another_int2", val_int));
  EXPECT_TRUE(call("n_param/another_int2", val_int));
  EXPECT_TRUE(call("n_param.another_int2", val_int));
  EXPECT_TRUE(send("n_param.another_int2", val_int));
  EXPECT_TRUE(val_int == 5);

  EXPECT_TRUE(
      send("n_param/nested_param/another_int2", val_int));
  EXPECT_TRUE(
      send("n_param.nested_param.another_int2", val_int));
  EXPECT_TRUE(
      send("n_param.nested_param/another_int2", val_int));
  EXPECT_TRUE(
      send("n_param/nested_param.another_int2", val_int));
  EXPECT_TRUE(val_int == 5);
}

TEST(ROS2Module, ClientUsageSequenceOf)
{
  std::string what;
  std::vector<std::string> v_string;
  EXPECT_TRUE(call("n1/n3/v1", v_string));
  EXPECT_TRUE(v_string.size() == 3 && v_string[0] == "s1" && v_string[1] == "s2" && v_string[2] == "s3");

  std::vector<std::vector<std::string>> vv_string;
  EXPECT_TRUE(call("n1/n4/vv1", vv_string));
  EXPECT_TRUE(vv_string.size() == 3 && vv_string.front().size() == 3 && vv_string[0][0] == "s11" && vv_string[0][1] == "s12" && vv_string[0][2] == "s13");
  EXPECT_TRUE(vv_string.size() == 3 && vv_string.front().size() == 3 && vv_string[1][0] == "s21" && vv_string[1][1] == "s22" && vv_string[1][2] == "s23");
  EXPECT_TRUE(vv_string.size() == 3 && vv_string.front().size() == 3 && vv_string[2][0] == "s31" && vv_string[2][1] == "s32" && vv_string[2][2] == "s33");
}

using namespace std::chrono_literals;

class TestParams : public rclcpp::Node
{
public:
  explicit TestParams(const std::string& node_name)
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
  testing::InitGoogleTest(&argc, argv);

  rclcpp::init(argc, argv);

  #if !defined(TEST_DIR)
    #error "The TEST_DIR macro MUST be defined to run the program"
  #endif


  const std::string& yaml_file_root_path = TEST_DIR;
  std::vector<std::string> yaml_files{ yaml_file_root_path + "/config/ros2parameters.yaml",   yaml_file_root_path + "/config/par_autogenerated.yaml"};

  auto parameters_server_node = std::make_shared<TestParams>(server_node_name);
  parameters_client_node = rclcpp::Node::make_shared(client_node_name);

  rclcpp::executors::MultiThreadedExecutor executor;
  std::thread executor_thread;
  executor.add_node(parameters_server_node);
  executor.add_node(parameters_client_node);

  executor_thread = std::thread(std::bind(&rclcpp::executors::MultiThreadedExecutor::spin, &executor));

  std::this_thread::sleep_for(1s);
  auto parameters_client =  std::make_shared<rclcpp::AsyncParametersClient>(parameters_server_node);
  parameters_client->wait_for_service();
  
  auto it = yaml_files.begin();
  while (it != yaml_files.end())
  {
    std::cout << "Reading yaml parameter files located at" <<  *it << std::endl;
    auto params = parameters_client->load_parameters(*it);
    std::chrono::milliseconds span(10000);
    bool ok;
    EXPECT_TRUE(ok = (params.wait_for(span) == std::future_status::ready) );
    if (!ok)
    {
      std::cerr << "Timeout in reading yaml parameter files located at" <<  *it << ". Abort "<< std::endl;
      return 0;
    }

    for(const auto & p : params.get())
    {
      EXPECT_TRUE(p.successful );
      if(!p.successful)
      {
        std::cerr << "Timeout in reading yaml parameter files located at" <<  *it << ". Abort "<< std::endl;
        return 0;
      }
      
    }
    it = yaml_files.erase(it);
  }


  std::vector<std::string> parameters_name;
  auto list = parameters_server_node->list_parameters({}, 1000);
  for(const auto & p : list.names)
  {
    std::cout << "- " << parameters_server_node->get_fully_qualified_name() << "." << p << std::endl;
  }
  
  std::this_thread::sleep_for(1s);

  cnr::param::ros2::CNR_PARAM_INIT_ROS2_MODULE(parameters_client_node);
  
  auto ret = RUN_ALL_TESTS();

  cnr::param::ros2::CNR_PARAM_CLEANUP_ROS2_MODULE();

  executor.cancel();
  executor_thread.join();
  parameters_client_node.reset();
  parameters_server_node.reset();
  rclcpp::shutdown();
  return ret;
}

#else
int main(int argc, char** argv)
{
  std::cerr << "This test is the remote accessor to a ROS2 node, but the ROS2_MODULE is not defined.\n" << std::endl;
  return 0;
}
#endif