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
#include <filesystem>
#include <gtest/gtest.h>

#if ROS1_MODULE

#include <ros/node_handle.h>

#include <cnr_param/ros/param.h>



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

#define EXECUTION_TIME(...)                                                                                            \
  {                                                                                                                    \
    struct timespec start, end;                                                                                        \
    clock_gettime(CLOCK_MONOTONIC, &start);                                                                            \
    __VA_ARGS__;                                                                                                       \
    clock_gettime(CLOCK_MONOTONIC, &end);                                                                              \
    double time_taken;                                                                                                 \
    time_taken = double(end.tv_sec - start.tv_sec) * 1e9;                                                              \
    time_taken = double(time_taken + (end.tv_nsec - start.tv_nsec)) * 1e-9;                                            \
    std::cout << "Elapsed time [us]: " << time_taken * 1e6 << std::endl;                                               \
  }

// ====================================================================================================================
// === GLOBAL VARIABLES ===============================================================================================
// ====================================================================================================================
const std::string server_node_name = "parameters_server_node";
const std::string client_node_name = "parameters_client_node";
std::shared_ptr<ros::NodeHandle> parameters_client_node;

TEST(ROS1Module, BasicAssertions)
{
  EXPECT_STRNE("hello", "world");
  EXPECT_EQ(7 * 6, 42);
}

TEST(ROS1Module, Initialization)
{
  EXPECT_TRUE(does_not_throw([&] { cnr::param::ros::CNR_PARAM_INIT_ROS_MODULE(parameters_client_node); }));
}

template<typename T>
bool call(const std::string& key, T& value)
{
  std::string what;
  if(!cnr::param::ros::get("/"+server_node_name+"/"+key, value, what))
  {
    std::cerr << "What: " << what << std::endl;
    return false;
  }
  std::cout << "Value: " << std::to_string(value) << std::endl;
  return true;
}

TEST(ROS1Module, ClientUsageBasicTypes)
{
  std::string what;
  std::vector<double> v_double;
  EXPECT_TRUE(call("double_array", v_double));
  EXPECT_TRUE(v_double.size() == 2 && v_double[0] == 7.5 && v_double[1] == 400.4);

  std::vector<double> v_double_2;
  EXPECT_TRUE(call("double_array_2", v_double_2));
  EXPECT_TRUE(v_double_2.size() == 2 && v_double_2[0] == 7 && v_double_2[1] == 400);

  double val = 0.0;
  EXPECT_TRUE(call("double_value", val));
  EXPECT_TRUE(val == 3.14);

  std::vector<int> v_int;
  EXPECT_TRUE(call("int_array", v_int));
  EXPECT_TRUE(v_int.size() == 4 && v_int[0] == 10 && v_int[1] == 11 && v_int[2] == 12 && v_int[3] == 13);

  EXPECT_TRUE(call("int_array_2", v_int));
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

TEST(ROS1Module, ClientUsageSequenceOf)
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

int main(int argc, char** argv)
{

  std::string command = "roscore & ";
  int result = std::system(command.c_str());
  if(result != 0)
  {
    std::cerr << "Could not launch the roscore" << std::endl;
    return -1;
  }

  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "test_ros_module");

  #if !defined(TEST_DIR)
    #error "The TEST_DIR macro MUST be defined to run the program"
  #endif


  const std::string& yaml_file_root_path = TEST_DIR;
  std::vector<std::string> yaml_files{ yaml_file_root_path + "/config/rosparameters.yaml",   yaml_file_root_path + "/config/par.yaml"};
  
  parameters_client_node.reset(new ros::NodeHandle("parameters_client_node"));

  if(!yaml_files.empty())
  {
    auto it = yaml_files.begin();
    while (it != yaml_files.end())
    {
      std::cout << "Reading yaml parameter files located at" <<  *it << std::endl;

      if(!std::filesystem::exists(*it) ||
          !(std::filesystem::is_regular_file(*it) || std::filesystem::is_symlink(*it)))
      {
        std::cerr << "Could not open camera param file " << *it << std::endl;;
      } 
      else 
      {
        // load the YAML file using the external rosparam command
        std::string command = "rosparam load " + *it;
        int result = std::system(command.c_str());
        if(result != 0)
        {
          std::cerr << "Could not set config file " << *it << " to the parameter server.";
        }
        command = "rosparam load " + *it + " /" + server_node_name;
        result = std::system(command.c_str());
        if(result != 0)
        {
          std::cerr << "Could not set config file " << *it << " to the parameter server under the namespace /" + server_node_name;
        }
      }
      it = yaml_files.erase(it);
    }
  }
 
  auto ret = RUN_ALL_TESTS();

  cnr::param::ros::CNR_PARAM_CLEANUP_ROS_MODULE();

  return ret;
}

#else
int main(int argc, char** argv)
{
  std::cerr << "This test is the remote accessor to a ROS2 node, but the ROS2_MODULE is not defined.\n" << std::endl;
  return 0;
}
#endif