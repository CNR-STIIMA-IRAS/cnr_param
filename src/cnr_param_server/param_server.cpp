
#include <cstdlib>
#include <utility>
#include <iostream>
#include <string>
#include <iostream>

#include <boost/interprocess/detail/os_file_functions.hpp>

#include <cnr_param_server/utils/args_parser.h>
#include <cnr_param_server/utils/yaml_manager.h>


int main(int argc, char* argv[])
{

   const std::string default_shmem_name = "param_server_default_shmem";
   const std::string default_param_root_directory = boost::interprocess::ipcdetail::get_temporary_path();
   
   std::string param_root_directory;
   const char* env_p = std::getenv("CNR_PARAM_ROOT_DIRECTORY");
   param_root_directory = (env_p) ? std::string(env_p) : default_param_root_directory;

   int rc = setenv("CNR_PARAM_ROOT_DIRECTORY", param_root_directory.c_str(), true);
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
