
#include <cstdlib>
#include <iostream>
#include <string>
#include <iostream>

#include <boost/filesystem.hpp>

#include <cnr_param/mapped_file/yaml_parser.h>
#include <cnr_param/mapped_file/args_parser.h>
#include <cnr_param/mapped_file/yaml_manager.h>

#if defined(_WIN32)
#include <windows.h>
#endif

int main(int argc, char* argv[])
{
  const std::string default_shmem_name = "param_server_default_shmem";

  const char* env_p = std::getenv("CNR_PARAM_ROOT_DIRECTORY");
  std::string param_root_directory;
  if(env_p)
  {
    param_root_directory = std::string(env_p);
  }
  else
  {
    //Use a default directory
#if defined(_WIN32)
    // Windows
    char tempPath[MAX_PATH];
    GetTempPath(MAX_PATH, tempPath);
    param_root_directory = std::string(tempPath) + "cnr_param";
#else
    // Linux/Unix/Mac
    param_root_directory = "/tmp/cnr_param/";
#endif
  }

  // Check if param_root_directory exists; if not, create it
  boost::filesystem::path dir(param_root_directory);
  if (!boost::filesystem::exists(dir))
  {
    if (!boost::filesystem::create_directories(dir))
    {
      std::cerr << "Failed to create directory: " << param_root_directory << std::endl;
      return 1;
    }
  }

  // Set environment variable
#if defined(_WIN32)
  // Windows
  int rc = _putenv_s("CNR_PARAM_ROOT_DIRECTORY", param_root_directory.c_str());
#else
  // Linux/Unix/Mac
  int rc = setenv("CNR_PARAM_ROOT_DIRECTORY", param_root_directory.c_str(), 1);
#endif

  if(rc!=0)
  {
    std::cerr << "Errono" << rc << ": " << strerror(rc) << std::endl;
  }

  // Parsing of program inputs
  cnr::param::mapped_file::ArgParser args(argc, argv, default_shmem_name);

  // Parsing of the file contents, and storing in YAML::Node root
  cnr::param::mapped_file::YAMLParser yaml_parser(args.getNamespacesMap());

  // Streaming of all the files in mapping_files: shared memes mapped on files
  // The tree is build under the 'param_root_directory'
  cnr::param::mapped_file::YAMLStreamer yaml_streamer(yaml_parser.root(), param_root_directory);

  // Done!
  return 0;
}
