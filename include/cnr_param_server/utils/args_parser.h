
#ifndef SRC_CNR_PARAM_INCLUDE_CNR_PARAM_SERVER_UTILS_ARGS_PARSER
#define SRC_CNR_PARAM_INCLUDE_CNR_PARAM_SERVER_UTILS_ARGS_PARSER

#include <iostream>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/program_options/errors.hpp>
#include <boost/program_options.hpp>

namespace po = boost::program_options;

namespace std
{
  istream& operator>>(istream& in, std::pair<std::string,int>& out);
  ostream& operator<(std::pair<std::string,int>& in, ostream& out);
  istream& operator>>(istream& in, std::pair<std::string,std::string>& out);
}

class ArgParser
{
private:
  po::options_description generic_;
  po::options_description shmem_options_;
  po::options_description pars_options_;
  po::options_description cmdline_options_;
  int argc_;
  std::string program_name_;

  bool reset_all_ns_;
  
  std::map<std::string, bool> reset_ns_map_;
  std::pair<bool, size_t> size_all_shmem_;
  std::map<std::string, size_t> size_shmem_map_;
  std::map<std::string, std::vector<boost::filesystem::path> > ns_fn_map_;

  const std::string default_shmem_id_;
  
public:
  ArgParser(int argc, const char* const argv[], const std::string& default_shmem_id = "param_server_default_shmem");

  void printUsage(const boost::program_options::options_description desc, bool verbose = true);

  const bool& getResetAll() const;
  const std::map<std::string, bool>& getResetMap() const;
  const std::pair<bool,size_t>& getSizeAll() const;
  const std::map<std::string, size_t>& getSizeMap() const;
  std::map<std::string, std::vector<std::string> > getNamespacesMap() const;
};

#endif  /* SRC_CNR_PARAM_INCLUDE_CNR_PARAM_SERVER_UTILS_ARGS_PARSER */
