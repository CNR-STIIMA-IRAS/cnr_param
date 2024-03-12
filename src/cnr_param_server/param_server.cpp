
#include <cstdlib>
#include <utility>
#include <iostream>
#include <string>
#include <iostream>

#include <boost/filesystem.hpp>

#include <cnr_param_server/utils/args_parser.h>
#include <cnr_param_server/utils/yaml_manager.h>

#if defined(_WIN32)
#include <windows.h>
#endif

int main(int argc, char* argv[])
{
  const std::string default_shmem_name = "param_server_default_shmem";
  std::string default_param_root_directory;

  #if defined(_WIN32)
  // Windows
  char tempPath[MAX_PATH];
  GetTempPath(MAX_PATH, tempPath);
  default_param_root_directory = std::string(tempPath) + "cnr_param";
  #else
  // Assume Linux/Unix/Mac
  default_param_root_directory = "/tmp/cnr_param";
  #endif

  std::string param_root_directory;
  const char* env_p = std::getenv("CNR_PARAM_ROOT_DIRECTORY");

  if(env_p)
  {
    param_root_directory = std::string(env_p);
  }
  else
  {
    // Check if default_param_root_directory exists; if not, create it
    boost::filesystem::path dir(default_param_root_directory);
    if (!boost::filesystem::exists(dir))
    {
      if (!boost::filesystem::create_directories(dir)) {
        std::cerr << "Failed to create directory: " << default_param_root_directory << std::endl;
        return 1; // or handle error appropriately
      }
    }
    param_root_directory = default_param_root_directory;
  }

  // Set environment variable differently for Windows
  #if defined(_WIN32)
  int rc = _putenv_s("CNR_PARAM_ROOT_DIRECTORY", param_root_directory.c_str());
  #else
  int rc = setenv("CNR_PARAM_ROOT_DIRECTORY", param_root_directory.c_str(), 1);
  #endif

  if(rc!=0)
  {
    std::cerr << "Errono" << rc << ": " << strerror(rc) << std::endl;
  }

  // Parsing of program inputs
  ArgParser args(argc, argv, default_shmem_name);

  // Parsing of the file contents, and storing in YAML::Node root
  YAMLParser yaml_parser(args.getNamespacesMap());

  // Streaming of all the files in mapping_files: shared memes mapped on files
  // The tree is build under the 'param_root_directory'
  YAMLStreamer yaml_streamer(yaml_parser.root(), param_root_directory);

  // Done!
  return 0;
}
