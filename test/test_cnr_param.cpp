/**
 * @file test_yaml_server.cpp
 * @brief This file contains the unit tests for the YAML server functionality.
 */
#include <cstdlib>
#include <ostream>
#include <iostream>
#include <iostream>

#include <boost/interprocess/detail/os_file_functions.hpp>
#include <Eigen/Core>
#include <gtest/gtest.h>

#if MAPPED_FILE_MODULE

#include <string>

#include <cnr_yaml/eigen.h>

#include <cnr_param/cnr_param.h>

#include <cnr_param/mapped_file/args_parser.h>
#include <cnr_param/mapped_file/yaml_parser.h>
#include <cnr_param/mapped_file/yaml_manager.h>

struct ComplexType
{
  std::string name;
  double value;
};

namespace std 
{
std::string to_string(const ComplexType& c)
{
  return c.name + " = " + std::to_string(c.value);
}
}


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

#if !defined(TEST_DIR)
#error "The TEST_DIR macro MUST be defined to run the program"
#endif

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

template<typename T>
bool call(const std::string& key, T& value)
{
  std::string what;
  if(!cnr::param::get(key, value, what))
  {
    std::cerr << "What: " << what << std::endl;
    return false;
  }
  std::cout << "Value: " << std::to_string(value) << std::endl;
  return true;
}


// ====================================================================================================================
// === GLOBAL VARIABLES ===============================================================================================
// ====================================================================================================================
std::string param_root_directory;
const std::string default_param_root_directory = boost::interprocess::ipcdetail::get_temporary_path();

// ====================================================================================================================
// === HelloTest, BasicAssertions =====================================================================================
// ====================================================================================================================
TEST(CnrParam, BasicAssertions)
{
  // Expect two strings not to be equal.
  EXPECT_STRNE("hello", "world");
  // Expect equality.
  EXPECT_EQ(7 * 6, 42);
}

// ====================================================================================================================
// === ServerTest, ServerUsage ========================================================================================
// ====================================================================================================================
TEST(CnrParam, MappedFileServerUsage)
{
  const std::string default_shmem_name = "param_server_default_shmem";
  cnr::param::mapped_file::ArgParser* args = nullptr;
  cnr::param::mapped_file::YAMLParser* yaml_parser = nullptr;
  cnr::param::mapped_file::YAMLStreamer* yaml_streamer = nullptr;

  std::cout << "TEST_DIR: " << TEST_DIR << std::endl;

  // Parsing of program inputs
  const int argc = 11;
  const char* const argv[] = { "test", 
    "--ns-and-path-to-file", "/,"       TEST_DIR "/config/mqtt_config.yaml",
    "--ns-and-path-to-file", "/,"       TEST_DIR "/config/mqtt_config.yaml",
    "--ns-and-path-to-file", "/ns1,"    TEST_DIR "/config/drape_cell_hw.yaml",
    "--ns-and-path-to-file", "/ns2,"    TEST_DIR "/config/drape_cell_hw.yaml",
    "--ns-and-path-to-file", "/ns1/ns2," TEST_DIR "/config/drape_cell_hw.yaml",
  };

  EXPECT_TRUE(does_not_throw([&] { args = new cnr::param::mapped_file::ArgParser(argc, argv, default_shmem_name); }));

  EXPECT_TRUE(args);

  // Parsing of the file contents, and storing in YAML::Node root

  EXPECT_TRUE(does_not_throw([&] { yaml_parser = new cnr::param::mapped_file::YAMLParser(args->getNamespacesMap()); }));

  EXPECT_TRUE(yaml_parser);

  // Streaming of all the files in mapping_files: shared memes mapped on files
  // The tree is build under the 'param_root_directory'

  EXPECT_TRUE(does_not_throw([&] { yaml_streamer = new cnr::param::mapped_file::YAMLStreamer(yaml_parser->root(), param_root_directory); }));

  EXPECT_TRUE(yaml_parser);

  EXPECT_NO_FATAL_FAILURE(delete yaml_parser);
  EXPECT_NO_FATAL_FAILURE(delete yaml_streamer);
  EXPECT_NO_FATAL_FAILURE(delete args);
}

// ====================================================================================================================
// === ClientTest, ClientUsage ========================================================================================
// ====================================================================================================================
TEST(CnrParam, ClientUsage)
{
  std::string what;
  std::string value;
  auto f1 = [&](const std::string& path, const std::string& expected) {
    bool ok = false;
    EXECUTION_TIME(ok = cnr::param::get(path, value, what);)
    if(!ok)
    {
      std::cerr << what << std::endl;
    }
    if(value != expected)
    {
      std::cerr << "Value:" << value <<", expected:" << expected << std::endl;
    }
    return ok && value == expected;
  };

  EXPECT_TRUE(f1("/ns1/ns2/plan_hw/feedback_joint_state_topic", "/joint_states"));
  value = value + "_CIAO";
  EXPECT_TRUE(cnr::param::set("/ns1/ns2/plan_hw/feedback_joint_state_topic", value, what));
  EXPECT_TRUE(f1("/ns1/ns2/plan_hw/feedback_joint_state_topic", "/joint_states_CIAO"));

  EXPECT_TRUE(cnr::param::set("/ns1/ns3/plan_hw_NEW_NOT_IN_FILE/feedback_joint_state_topic", value, what));
  EXPECT_TRUE(f1("/ns1/ns3/plan_hw_NEW_NOT_IN_FILE/feedback_joint_state_topic", "/joint_states_CIAO"));

  int val = 9, before, after;
  EXPECT_TRUE(cnr::param::set("/a", val, what));
  EXPECT_TRUE(cnr::param::get("/a", before, what));
  EXPECT_TRUE(cnr::param::set("/a", "0x0009", what));
  EXPECT_TRUE(cnr::param::get("/a", after, what));
  EXPECT_TRUE(before - after == 0);

  EXPECT_FALSE(cnr::param::get("a", before, what));
}

// ====================================================================================================================
// === ClientErrorTest, ClientNonExistentParam ========================================================================
// ====================================================================================================================
TEST(CnrParam, ClientNonExistentParam)
{
  std::string defval = "DEFAULT";
  auto f1 = [](const std::string& path) {
    std::string topic;
    std::string what;
    bool ok = false;
    EXECUTION_TIME(ok = cnr::param::get(path, topic, what);)
    return ok;
  };

  auto f2 = [](const std::string& path, const std::string& defval) {
    std::string topic;
    std::string what;
    bool ok = cnr::param::get(path, topic, what, defval);
    return ok;
  };
  EXPECT_FALSE(f1("/ns1/ns2/plan_hw/feedback_joint_state_topic__NOT_EXIST"));
  EXPECT_TRUE(f2("/ns1/ns2/plan_hw/feedback_joint_state_topic__NOT_EXIST", defval));
}

// ====================================================================================================================
// === DeveloperTest, DeveloperFunctions ==============================================================================
// ====================================================================================================================
TEST(CnrParam, DeveloperFunctions)
{
  std::string defval = "DEFAULT";
  auto f1 = [](const std::string& root_key) {
    YAML::Node root;
    std::string what;
    bool ok = false;
    EXECUTION_TIME(ok = cnr::param::get(root_key, root, what);)
    if (!ok)
    {
      return false;
    }

    return ok;
  };

  EXPECT_TRUE(f1("/ns1/ns2/plan_hw/"));
}

// ====================================================================================================================
// === DeveloperTest, GetVector =======================================================================================
// ====================================================================================================================
TEST(CnrParam, GetVector)
{
  std::string what;
  std::vector<std::string> vv;
  bool ret = true;
  EXPECT_TRUE(ret = call("/n1/n3/v1", vv));

  std::vector<double> dd;
  EXPECT_FALSE(ret = call("/n1/n3/v1", dd));
  EXPECT_TRUE(ret = call("/n1/n3/v10", dd));

  Eigen::VectorXd ee;
  EXPECT_TRUE(ret = call("/n1/n3/v10", ee));
}

// ====================================================================================================================
// === DeveloperTest, GetMatrix =======================================================================================
// ====================================================================================================================
TEST(CnrParam, GetMatrix)
{
  std::string what;
  std::vector<std::vector<std::string>> vv;
  bool ret = true;
  EXPECT_TRUE(ret = cnr::param::get("/n1/n4/vv1", vv, what));

  std::vector<std::vector<double>> dd;
  EXPECT_TRUE(ret = cnr::param::get("/n1/n4/vv10", dd, what));

  Eigen::MatrixXd ee;
  EXPECT_TRUE(ret = cnr::param::get("/n1/n4/vv10", ee, what));

  EXPECT_FALSE(ret = cnr::param::get("/n1/n4/vv1", ee, what));
}


// ====================================================================================================================
// === DeveloperTest, GetComplexType ==================================================================================
// First we define a complex type
// then we inherit the get_map function to extract the complex type from the YAML node
// finally, we run the test
// ====================================================================================================================
namespace YAML
{
template <>
struct convert<ComplexType>
{
  static Node encode(const ComplexType& rhs)
  {
    Node node;
    node["name"] = rhs.name;
    node["value"] = rhs.value;
    return node;
  }

  static bool decode(const Node& node, ComplexType& rhs)
  {
    if (!node.IsMap() || !node["name"] || !node["value"])
    {
      return false;
    }
    rhs.name = node["name"].as<std::string>();
    rhs.value = node["value"].as<double>();
    return true;
  }
};
}  // namespace YAML

TEST(CnrParam, GetComplexType)
{
  std::string what;
  std::vector<ComplexType> vv;
  bool ret = true;
  EXPECT_TRUE(ret = cnr::param::get("/n1/n4/test_vector_complex_type", vv, what));
}

/**
 * @brief The main function of the program.
 *
 * This function initializes the parameter root directory based on the value of the environment variable "CNR_PARAM_ROOT_DIRECTORY".
 * If the environment variable is not set, it uses the default parameter root directory.
 * It then sets the environment variable "CNR_PARAM_ROOT_DIRECTORY" to the chosen value.
 * If setting the environment variable fails, it prints an error message.
 * Finally, it initializes the Google Test framework and runs all the tests.
 *
 * @param argc The number of command-line arguments.
 * @param argv An array of command-line arguments.
 * @return The exit code of the program.
 */
int main(int argc, char** argv)
{
  const char* env_p = std::getenv("CNR_PARAM_ROOT_DIRECTORY");
  param_root_directory = (env_p) ? std::string(env_p) : default_param_root_directory;
  int rc = setenv("CNR_PARAM_ROOT_DIRECTORY", param_root_directory.c_str(), true);
  if (rc != 0)
  {
    std::cerr << "Errono" << rc << ": " << strerror(rc) << std::endl;
  }

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

#else
int main(int, char**)
{
  std::cerr << "This test is for the MAPPED_FILE_MODULE, but the MAPPED_FILE_MODULE is not defined.\n" << std::endl;
  return 0;
}
#endif