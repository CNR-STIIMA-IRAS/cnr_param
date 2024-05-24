/**
 * @file test_yaml_server.cpp
 * @brief This file contains the unit tests for the YAML server functionality.
 */
#include <cstdlib>
#include <ostream>
#include <iostream>
#include <string>
#include <iostream>

#if MAPPED_FILE_MODULE

#include <boost/interprocess/detail/os_file_functions.hpp>

#include <cnr_param/cnr_param.h>
#include <cnr_param/core/string.h>
#include <cnr_param/core/eigen.h>

#include <cnr_param/mapped_file/args_parser.h>
#include <cnr_param/mapped_file/yaml_parser.h>
#include <cnr_param/mapped_file/yaml_manager.h>

std::string param_root_directory;
const std::string default_param_root_directory = boost::interprocess::ipcdetail::get_temporary_path();

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
namespace core
{
template <>
bool get_map(const YAML::Node& node, ComplexType& ret, std::string& what)
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
    std::stringstream err;
    err << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": "
        << "YAML Exception, Error in the extraction of an object of type '"
        << boost::typeindex::type_id_with_cvr<decltype(ret)>().pretty_name() << "'" << std::endl
        << "Node: " << std::endl
        << node << std::endl
        << "What: " << std::endl
        << e.what() << std::endl;
    what = err.str();
  }
  catch (std::exception& e)
  {
    std::stringstream err;
    err << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": "
        << "Exception, Error in the extraction of an object of type '"
        << boost::typeindex::type_id_with_cvr<decltype(ret)>().pretty_name() << "'" << std::endl
        << "Node: " << std::endl
        << node << std::endl
        << "What: " << std::endl
        << e.what() << std::endl;
    what = err.str();
  }
  return false;
}

}  // namespace core
}  // namespace param
}  // namespace cnr

namespace std
{

std::string to_string(const ComplexType& n)
{
  return "Name: " + n.name + ", Value: " + std::to_string(n.value);
}

template<>
std::string to_string(const std::vector<ComplexType>& vv)
{
  std::string ret = "[ ";
  for (auto const & v : vv) ret += std::to_string(v) + " ";
  ret += "]";
  return ret;
}

}  // namespace std

template <typename T, typename O = T>
bool call(const std::string& key, T& value, O* o = nullptr)
{
  std::string what;
  if (o)
  {
    if (!cnr::param::get(key, value, what, *o))
    {
      std::cerr << "Key: " << key << ", What: " << what << std::endl;
      return false;
    }
  }
  else
  {
    if (!cnr::param::mapped_file::get(key, value, what))
    {
      std::cerr << "Key: " << key << ", What: " << what << std::endl;
      return false;
    }
  }

  std::cout << "Key: " << key << ", Value: " << std::to_string(value) << std::endl;
  return true;
}

template <typename T>
bool send(const std::string& key, const T& value)
{
  std::string what;
  if (!cnr::param::mapped_file::set(key, value, what))
  {
    std::cerr << "Key: " << key << ", What: " << what << std::endl;
    return false;
  }

  return true;
}

/**
 * @brief The main function of the program.
 *
 * This function initializes the parameter root directory based on the value of the environment variable
 * "CNR_PARAM_ROOT_DIRECTORY". If the environment variable is not set, it uses the default parameter root directory. It
 * then sets the environment variable "CNR_PARAM_ROOT_DIRECTORY" to the chosen value. If setting the environment
 * variable fails, it prints an error message. Finally, it initializes the Google Test framework and runs all the tests.
 *
 * @param argc The number of command-line arguments.
 * @param argv An array of command-line arguments.
 * @return The exit code of the program.
 */
int main(int, char**)
{
  const char* env_p = std::getenv("CNR_PARAM_ROOT_DIRECTORY");
  param_root_directory = (env_p) ? std::string(env_p) : default_param_root_directory;
  int rc = setenv("CNR_PARAM_ROOT_DIRECTORY", param_root_directory.c_str(), true);
  if (rc != 0)
  {
    std::cerr << "Errono" << rc << ": " << strerror(rc) << std::endl;
  }

  //////////////////////////////////////////////////////////////////////////
  // FILL THE PARAM SERVER
  {
    const std::string default_shmem_name = "param_server_default_shmem";

    // Parsing of program inputs
    const int argc = 13;
    const char* const argv[] = {
      "test",
      "--ns-and-path-to-file",
      "/," TEST_DIR "/config/mqtt_config.yaml",
      "--ns-and-path-to-file",
      "/," TEST_DIR "/config/mqtt_config.yaml",
      "--ns-and-path-to-file",
      "/ns1," TEST_DIR "/config/drape_cell_hw.yaml",
      "--ns-and-path-to-file",
      "/ns2," TEST_DIR "/config/drape_cell_hw.yaml",
      "--ns-and-path-to-file",
      "/ns1/ns2," TEST_DIR "/config/drape_cell_hw.yaml",
      "--ns-and-path-to-file",
      "/," TEST_DIR "/config/par.yaml",
    };

    cnr::param::mapped_file::ArgParser args(argc, argv, default_shmem_name);
    cnr::param::mapped_file::YAMLParser yaml_parser(args.getNamespacesMap());
    cnr::param::mapped_file::YAMLStreamer yaml_streamer(yaml_parser.root(), param_root_directory);
  }
  //////////////////////////////////////////////////////////////////////////

  // BASIC USAGE OF THE CLIENT LIBRARY
  std::string what;
  std::string value;
  call("/ns1/ns2/plan_hw/feedback_joint_state_topic", value);
  value = value + "_CIAO";

  send("/ns1/ns2/plan_hw/feedback_joint_state_topic", value);
  call("/ns1/ns2/plan_hw/feedback_joint_state_topic", value);

  send("/ns1/ns3/plan_hw_NEW_NOT_IN_FILE/feedback_joint_state_topic", value);
  call("/ns1/ns3/plan_hw_NEW_NOT_IN_FILE/feedback_joint_state_topic", value);

  int val = 9, before, after;
  send("/a", val);
  call("/a", before);
  send("/a", "0x0009");
  call("/a", after);
  call("a", before);

  std::string defval = "DEFAULT";
  call("/ns1/ns2/plan_hw/feedback_joint_state_topic__NOT_EXIST", value);
  call("/ns1/ns2/plan_hw/feedback_joint_state_topic__NOT_EXIST", value, &defval);

  YAML::Node root;
  call("/ns1/ns2/plan_hw/", root);

  std::vector<std::string> vv;
  call("/n1/n3/v1", vv);

  std::vector<double> dd;
  call("/n1/n3/v1", dd);
  call("/n1/n3/v10", dd);

  Eigen::VectorXd ee;
  call("/n1/n3/v10", ee);

  std::vector<std::vector<std::string>> vvv;
  call("/n1/n4/vv1", vvv);

  std::vector<std::vector<double>> ddd;
  call("/n1/n4/vv10", ddd);

  Eigen::MatrixXd eee;
  call("/n1/n4/vv10", eee);

  std::vector<ComplexType> ct;
  call("/n1/n4/test_vector_complex_type", ct);

  return 0;
}

#else
int main(int, char**)
{
  std::cerr << "This test is the yaml server test, availalbe for the MAPPED_FILE_MODULE, but the MAPPED_FILE_MODULE is "
               "not defined.\n"
            << std::endl;
  return 0;
}
#endif