#include <cstdlib>
#include <utility>
#include <iostream>
#include <string>
#include <iostream>

#include <boost/interprocess/detail/os_file_functions.hpp>

#include <cnr_param/cnr_param.hpp>

#include <cnr_param_server/args_parser.h>
#include <cnr_param_server/yaml_manager.h>

#include <gtest/gtest.h>

#if !defined(TEST_DIR)
#error "The TES_DIR macro MUST be defined to run the program"
#endif

std::map<std::string, std::map<std::string, std::vector<double> > > statistics;

#define EXECUTION_TIME( ... )\
{\
  struct timespec start, end;\
  clock_gettime(CLOCK_MONOTONIC, &start);\
  __VA_ARGS__;\
  clock_gettime(CLOCK_MONOTONIC, &end);\
  double time_taken;\
  time_taken = double(end.tv_sec - start.tv_sec) * 1e9;\
  time_taken = double(time_taken + (end.tv_nsec - start.tv_nsec)) * 1e-9;\
  std::cout << "Elapsed time [us]: " << time_taken * 1e6 << std::endl;\
}

std::string param_root_directory;
const std::string default_param_root_directory = boost::interprocess::ipcdetail::get_temporary_path();

// Demonstrate some basic assertions.
TEST(HelloTest, BasicAssertions) 
{
  // Expect two strings not to be equal.
  EXPECT_STRNE("hello", "world");
  // Expect equality.
  EXPECT_EQ(7 * 6, 42);
}

TEST(ServerTest, ServerUsage)
{
   const std::string default_shmem_name = "param_server_default_shmem";
   
   // Parsing of program inputs
   const int argc = 3;
   std::string fn = std::string(TEST_DIR) + "/example.config";
   std::cout << fn << std::endl;
   const char* const argv[] = {"test", "--config", fn.c_str()};
   ArgParser args(argc, argv, default_shmem_name);

   // Parsing of the file contents, and storing in YAML::Node root
   YAMLParser yaml_parser(args.getNamespacesMap());

   // Streaming of all the files in mapping_files: shared memes mapped on files
   // The tree is build under the 'param_root_directory'
   YAMLStreamer yaml_streamer(yaml_parser.root(), param_root_directory); 
}

TEST(ClientTest, ClientUsage)
{
  std::string defval = "DEFAULT";
  auto f1 = [](const std::string& path)
  {
    std::string topic;
    std::string what;
    bool ok = false;
    EXECUTION_TIME(
      ok = cnr::param::get(path, topic, what);
    )
    std::cout<< "GOT: " << ok << std::endl;
    std::cout<< "WHAT: " << what << std::endl;
    return ok;
  };


  auto f2 = [](const std::string& path,  const std::string& defval)
  {
    std::string topic;
    std::string what;
    bool ok = cnr::param::get(path, topic, what, defval);
    std::cout << what << std::endl;
    return ok;
  };
  EXPECT_TRUE(f1("/ns1/ns2/plan_hw/feedback_joint_state_topic"));
  EXPECT_FALSE(f1("/ns1/ns2/plan_hw/feedback_joint_state_topic__NOT_EXIST"));
  EXPECT_TRUE(f2("/ns1/ns2/plan_hw/feedback_joint_state_topic__NOT_EXIST", defval));

}

TEST(DeveloperTest, DeveloperFunctions)
{
  std::string defval = "DEFAULT";
  auto f1 = [](const std::string& root_key, const std::string& leaf_key)
  {
    cnr::param::node_t root;
    std::string what;
    bool ok = false;
    EXECUTION_TIME(
      ok = cnr::param::get(root_key, root, what);
    )
    if(!ok)
    {
      std::cout << what << std::endl;
      return false;
    }
    
    
    cnr::param::node_t leaf;
    EXECUTION_TIME(
      ok = cnr::param::get_leaf(root, leaf_key, leaf, what);
    )
    std::cout << what << std::endl;
    return ok;
  };

  EXPECT_TRUE(f1("/ns1/ns2/plan_hw/", "feedback_joint_state_topic"));
}

int main(int argc, char **argv) {

  const char* env_p = std::getenv("CNR_PARAM_ROOT_DIRECTORY");
  param_root_directory = (env_p) ? std::string(env_p) : default_param_root_directory;
  int rc = setenv("CNR_PARAM_ROOT_DIRECTORY", param_root_directory.c_str(), true);
  if(rc!=0)
  {
    std::cerr << "Errono" << rc << ": " << strerror(rc) << std::endl;
  }

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}