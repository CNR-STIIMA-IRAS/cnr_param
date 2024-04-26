/**
 * @file test_yaml_server.cpp
 * @brief This file contains the unit tests for the YAML server functionality.
 */
#include <cstdlib>
#include <ostream>
#include <iostream>
#include <string>
#include <iostream>

#include <boost/interprocess/detail/os_file_functions.hpp>

#include <cnr_param/yaml_mapped_file/param.h>

#include <cnr_param/yaml_mapped_file/args_parser.h>
#include <cnr_param/yaml_mapped_file/yaml_manager.h>

#include <gtest/gtest.h>

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

// ====================================================================================================================
// === GLOBAL VARIABLES ===============================================================================================
// ====================================================================================================================
std::string param_root_directory;
const std::string default_param_root_directory = boost::interprocess::ipcdetail::get_temporary_path();

// ====================================================================================================================
// === HelloTest, BasicAssertions =====================================================================================
// ====================================================================================================================
TEST(HelloTest, BasicAssertions)
{
  // Expect two strings not to be equal.
  EXPECT_STRNE("hello", "world");
  // Expect equality.
  EXPECT_EQ(7 * 6, 42);
}

// ====================================================================================================================
// === ServerTest, ServerUsage ========================================================================================
// ====================================================================================================================
TEST(ServerTest, ServerUsage)
{
  const std::string default_shmem_name = "param_server_default_shmem";
  ArgParser* args = nullptr;
  YAMLParser* yaml_parser = nullptr;
  YAMLStreamer* yaml_streamer = nullptr;

  // Parsing of program inputs
  const int argc = 3;
  const char* const argv[] = { "test", 
    "--ns-and-path-to-file", "/,"       TEST_DIR "/config/mqtt_config.yaml",
    "--ns-and-path-to-file", "/,"       TEST_DIR "/config/mqtt_config.yaml",
    "--ns-and-path-to-file", "/ns1,"    TEST_DIR "/config/drape_cell_hw.yaml",
    "--ns-and-path-to-file", "/ns2,"    TEST_DIR "/config/drape_cell_hw.yaml",
    "--ns-and-path-to-file", "/ns1/ns2," TEST_DIR "/config/drape_cell_hw.yaml",
    "--ns-and-path-to-file", "/,"       TEST_DIR "/config/par.yaml",
};

  EXPECT_TRUE(does_not_throw([&] { args = new ArgParser(argc, argv, default_shmem_name); }));

  EXPECT_TRUE(args);

  // Parsing of the file contents, and storing in YAML::Node root

  EXPECT_TRUE(does_not_throw([&] { yaml_parser = new YAMLParser(args->getNamespacesMap()); }));

  EXPECT_TRUE(yaml_parser);

  // Streaming of all the files in mapping_files: shared memes mapped on files
  // The tree is build under the 'param_root_directory'

  EXPECT_TRUE(does_not_throw([&] { yaml_streamer = new YAMLStreamer(yaml_parser->root(), param_root_directory); }));

  EXPECT_TRUE(yaml_parser);

  EXPECT_NO_FATAL_FAILURE(delete yaml_parser);
  EXPECT_NO_FATAL_FAILURE(delete yaml_streamer);
  EXPECT_NO_FATAL_FAILURE(delete args);
}

// ====================================================================================================================
// === ClientTest, ClientUsage ========================================================================================
// ====================================================================================================================
TEST(ClientTest, ClientUsage)
{
  std::string what;
  std::string value;
  auto f1 = [&](const std::string& path) {
    bool ok = false;
    EXECUTION_TIME(ok = cnr::param::yaml::get(path, value, what);)
    if (!ok)
    {
      std::cout << "OK: " << ok << "| VALUE: " << value << "| WHAT: " << what << std::endl;
    }
    return ok;
  };

  EXPECT_TRUE(f1("/ns1/ns2/plan_hw/feedback_joint_state_topic"));
  value = value + "_CIAO";
  EXPECT_TRUE(cnr::param::yaml::set("/ns1/ns2/plan_hw/feedback_joint_state_topic", value, what));
  EXPECT_TRUE(f1("/ns1/ns2/plan_hw/feedback_joint_state_topic"));

  EXPECT_TRUE(cnr::param::yaml::set("/ns1/ns3/plan_hw_NEW_NOT_IN_FILE/feedback_joint_state_topic", value, what));
  EXPECT_TRUE(f1("/ns1/ns3/plan_hw_NEW_NOT_IN_FILE/feedback_joint_state_topic"));

  int val = 9, before, after;
  EXPECT_TRUE(cnr::param::yaml::set("/a", val, what));
  EXPECT_TRUE(cnr::param::yaml::get("/a", before, what));
  EXPECT_TRUE(cnr::param::yaml::set("/a", "0x0009", what));
  EXPECT_TRUE(cnr::param::yaml::get("/a", after, what));
  EXPECT_TRUE(before - after == 0);
  std::cout << "Value: before: " << before << " after: " << after << " Diff: " << before - after << std::endl;

  EXPECT_FALSE(cnr::param::yaml::get("a", before, what));
  std::cout << "What: " << what << std::endl;
}

// ====================================================================================================================
// === ClientErrorTest, ClientNonExistentParam ========================================================================
// ====================================================================================================================
TEST(ClientErrorTest, ClientNonExistentParam)
{
  std::string defval = "DEFAULT";
  auto f1 = [](const std::string& path) {
    std::string topic;
    std::string what;
    bool ok = false;
    EXECUTION_TIME(ok = cnr::param::yaml::get(path, topic, what);)
    if (!ok)
    {
      std::cout << "GOT: " << ok << " | VALUE:" << topic << " | WHAT: " << what << std::endl;
    }
    return ok;
  };

  auto f2 = [](const std::string& path, const std::string& defval) {
    std::string topic;
    std::string what;
    bool ok = cnr::param::yaml::get(path, topic, what, defval);
    std::cout << what << std::endl;
    return ok;
  };
  EXPECT_FALSE(f1("/ns1/ns2/plan_hw/feedback_joint_state_topic__NOT_EXIST"));
  EXPECT_TRUE(f2("/ns1/ns2/plan_hw/feedback_joint_state_topic__NOT_EXIST", defval));
}

// ====================================================================================================================
// === DeveloperTest, DeveloperFunctions ==============================================================================
// ====================================================================================================================
TEST(DeveloperTest, DeveloperFunctions)
{
  std::string defval = "DEFAULT";
  auto f1 = [](const std::string& root_key, const std::string& leaf_key) {
    YAML::Node root;
    std::string what;
    bool ok = false;
    EXECUTION_TIME(ok = cnr::param::yaml::get(root_key, root, what);)
    if (!ok)
    {
      std::cout << what << std::endl;
      return false;
    }

    YAML::Node leaf;
    EXECUTION_TIME(ok = cnr::param::yaml::get_leaf(root, leaf_key, leaf, what);)
    if (!ok)
    {
      std::cout << what << std::endl;
    }
    return ok;
  };

  EXPECT_TRUE(f1("/ns1/ns2/plan_hw/", "feedback_joint_state_topic"));
}

// ====================================================================================================================
// === DeveloperTest, GetVector =======================================================================================
// ====================================================================================================================
TEST(DeveloperTest, GetVector)
{
  std::string what;
  std::vector<std::string> vv;
  bool ret = true;
  EXPECT_TRUE(ret = cnr::param::yaml::get("/n1/n3/v1", vv, what));
  if (ret)
  {
    std::cout << "[";
    for (const auto& v : vv)
    {
      std::cout << v << ", ";
    }
    std::cout << "]" << std::endl;
  }
  else
  {
    std::cout << "Failed: " << what << std::endl;
  }

  std::vector<double> dd;
  EXPECT_FALSE(ret = cnr::param::yaml::get("/n1/n3/v1", dd, what));
  EXPECT_TRUE(ret = cnr::param::yaml::get("/n1/n3/v10", dd, what));
  std::cout << "[";
  for (const auto& v : dd)
  {
    std::cout << v << ", ";
  }
  std::cout << "]" << std::endl;

  Eigen::VectorXd ee;
  EXPECT_TRUE(ret = cnr::param::yaml::get("/n1/n3/v10", ee, what));
  if (ret)
  {
    std::cout << ee.transpose() << std::endl;
  }
  else
  {
    std::cout << "Failed: " << what << std::endl;
  }
}

// ====================================================================================================================
// === DeveloperTest, GetMatrix =======================================================================================
// ====================================================================================================================
TEST(DeveloperTest, GetMatrix)
{
  std::string what;
  std::vector<std::vector<std::string>> vv;
  bool ret = true;
  EXPECT_TRUE(ret = cnr::param::yaml::get("/n1/n4/vv1", vv, what));
  std::cout << "[\n";
  for (const auto& v : vv)
  {
    std::cout << " [ ";
    for (const auto& s : v)
    {
      std::cout << s << ", ";
    }
    std::cout << "] " << std::endl;
  }
  std::cout << "]" << std::endl;

  std::vector<std::vector<double>> dd;
  EXPECT_TRUE(ret = cnr::param::yaml::get("/n1/n4/vv10", dd, what));
  std::cout << "[\n";
  for (const auto& v : dd)
  {
    std::cout << " [ ";
    for (const auto& s : v)
    {
      std::cout << s << ", ";
    }
    std::cout << "] " << std::endl;
  }
  std::cout << "]" << std::endl;

  Eigen::MatrixXd ee;
  EXPECT_TRUE(ret = cnr::param::yaml::get("/n1/n4/vv10", ee, what));
  std::cout << ee << std::endl;

  EXPECT_FALSE(ret = cnr::param::yaml::get("/n1/n4/vv1", ee, what));
}


// ====================================================================================================================
// === DeveloperTest, GetComplexType ==================================================================================
// First we define a complex type
// then we inherit the get_map function to extract the complex type from the YAML node
// finally, we run the test
// ====================================================================================================================
struct ComplexType
{
  std::string name;
  double value;
};

namespace cnr
{
namespace param
{
namespace yaml
{
template <>
bool get_map(const YAML::Node& node, ComplexType& ret, std::stringstream& what)
{
  try
  {
    if (node["name"] && node["value"])
    {
      ret.name = node["name"].as<std::string>();
      ret.value = node["value"].as<double>();
      return true;
    }
  }
  catch (YAML::Exception& e)
  {
    what << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": "
         << "YAML Exception, Error in the extraction of an object of type '"
         << boost::typeindex::type_id_with_cvr<decltype(ret)>().pretty_name() << "'" << std::endl
         << "Node: " << std::endl
         << node << std::endl
         << "What: " << std::endl
         << e.what() << std::endl;
  }
  catch (std::exception& e)
  {
    what << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": "
         << "Exception, Error in the extraction of an object of type '"
         << boost::typeindex::type_id_with_cvr<decltype(ret)>().pretty_name() << "'" << std::endl
         << "Node: " << std::endl
         << node << std::endl
         << "What: " << std::endl
         << e.what() << std::endl;
  }
  return false;
}

}  // namespace yaml
}  // namespace param
}  // namespace cnr

TEST(DeveloperTest, GetComplexType)
{
  std::string what;
  std::vector<ComplexType> vv;
  bool ret = true;
  EXPECT_TRUE(ret = cnr::param::yaml::get("/n1/n4/test_vector_complex_type", vv, what));
  if (ret)
  {
    std::cout << "[\n";
    for (const auto& v : vv)
    {
      std::cout << " [ ";
      std::cout << v.name << "," << v.value;
      std::cout << "] " << std::endl;
    }
    std::cout << "]" << std::endl;
  }
  else
  {
    std::cerr << what << std::endl;
  }
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
