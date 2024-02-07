#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <boost/program_options/errors.hpp>
#include <boost/program_options.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/regex.hpp>

#include <cnr_param/utils/string.h>
#include <cnr_param/utils/filesystem.h>

#include <cnr_param_server/utils/args_parser.h>

namespace std
{
istream& operator>>(istream& in, std::pair<std::string,int>& out)
{
  std::string s;
  in >> s;
  std::string delim = ", ";
  auto ss = cnr::param::utils::tokenize(s, delim);
  if(ss.size()!=2 || std::stoi(ss[1]) < 1024 )
  {
      out.first="error";
      out.second=-1;
  }
  out.first = ss.front();
  out.second = std::stoi(ss.back());
  return in;
}

ostream& operator<(std::pair<std::string,int>& in, ostream& out)
{
  out << "(" << in.first <<", " << in.second << ")";
  return out;
}

istream& operator>>(istream& in, std::pair<std::string,std::string>& out)
{
  out.first.clear();
  out.second.clear();

  std::string s;
  in >> s;
  std::string delim = "'<>()[],;| {}";

  auto ss = cnr::param::utils::tokenize(s, delim);
  if(ss.size()==2)
  {
    out.first = ss.front();
    out.second = ss.back();
  }
  return in;
}
}

//===============================================================================================
namespace po = boost::program_options;


ArgParser::ArgParser(int argc, const char* const argv[], const std::string& default_shmem_id) 
: generic_("Generic options", 160), shmem_options_("Namespaces (shared memories)", 160), 
    pars_options_("Path to Parameters", 160), cmdline_options_("Full List of the Options"), 
      argc_(argc), program_name_(argv[0]), reset_all_ns_(false), size_all_shmem_(false, 1024), 
      default_shmem_id_(default_shmem_id)
{

  try
  {
    generic_.add_options()
      ("version,v", "print version std::string")
      ("help,h", "produce help message")
      ("config-file,c", 
        po::value< std::string>()->value_name("filepath absolute or relative"),
        "config file name. It stores all the inline commands");

    shmem_options_.add_options()
      ("reset-all-ns,a", 
            "All the namespaces are cleared. The shared memories are removed.\n")
      ("reset-ns,r", 
          po::value< std::vector<std::string> >()->composing()->value_name("str"), 
          "Clear only the namespace listed. If the namespace does not exists, none error is raised.\n")
      ("size-of-all-ns,s",
          po::value<int>()->value_name("size (greater than 1024bytes)")->notifier([](const int v) {
            if(v < 1024) 
            { 
              throw po::validation_error(po::validation_error::invalid_option_value, "size-of-all-ns", std::to_string(v));
            } }),
          "Set the size for each shared memory corresponding to a namespace. Expressed in bytes.\n"\
          "IMPORTANT:\n"\
          "1) The size MUST be greater than 1024bytes\n"\
          "2) All the parameters will be erased if set.\n")
      ("size-of-ns,z",
          po::value< std::pair<std::string,int> >()->value_name("'str,size (greater than 1024bytes)'")->notifier([](std::pair<std::string,int> v) {
            if(v.second == -1 || v.second < 1024) 
            { 
              throw po::validation_error(po::validation_error::invalid_option_value, "size-of-ns");
            } }),
          "The first field is the namespace. At each 'ns' corresponds a different shared memory.\n"\
          "The second one is the size in bytes.\n"\
          "IMPORTANT:\n"\
          "1) no BLANK after comma\n"\
          "2) The size MUST be greater than 1024bytes\n"\
          "3) Indicate one size for each 'ns', also if idfferent files are mapped under the same 'ns'\n"\
          "4) The parameters under under the indicated shared memory will be erased, if set.\n");

    // Hidden options, will be allowed both on command line and
    // in config file, but will not be shown to the user.
    pars_options_.add_options()
      ("path-to-file,p",
          po::value<std::vector< std::string> >()->composing()->value_name("str"),
            "Path to yaml file to load [relative or absolute]. "\
            "Since the namespace is not indicated, the nodes are stored under the default shared memory (acting as the root ns).\n")
        ("ns-and-path-to-file,n",
          po::value< std::vector<std::pair<std::string,std::string> > >()->composing()->value_name("str,str"),
            "The first field is the namespace. At each namespace 'ns' corresponds a different shared memory.\n"\
            "The second field is the path to yaml file to load [relative or absolute].\n "\
            "The parameters belonging to the yaml file to load will be stored under the indicated 'ns'.\n"
            "IMPORTANT:\n"\
            "1) The str,str MUST be separated by a comma.\n"\
            "2) no BLANK after comma\n"
            "3) different files can be mapped under the same 'ns'\n");

    //============================================================
    cmdline_options_.add(shmem_options_).add(pars_options_).add(generic_);
    //============================================================

    po::variables_map vm;
    auto parsed = po::parse_command_line(argc_, argv, cmdline_options_);
    po::store(parsed, vm);
    po::notify(vm);
    //============================================================
    // do we have a config file?
    
    if (vm.count("help")) 
    {
      printUsage(cmdline_options_);
      exit(0);
    }

    if (vm.count("config-file"))
    {
      auto fn = vm["config-file"].as<std::string>();
      std::string what;
      boost::filesystem::path fp;
      if(!cnr::param::utils::filepath(fn, fp, what))
      {
        std::cerr << what << std::endl;
        throw po::validation_error(po::validation_error::invalid_option_value, "config-file");
      }

      std::ifstream fConfig{ fn.c_str() };
      if (fConfig)
      {
        boost::iostreams::filtering_istream prep_config;
        prep_config.push(boost::iostreams::regex_filter{ boost::regex{" "}, "="});
        prep_config.push(fConfig);

        po::store(po::parse_config_file( prep_config, cmdline_options_), vm);
        po::notify(vm);
      }
      else
      {
         std::cerr << "Could not open the configuration file - ignoring.\n";
      }
    }

    if(vm.count("reset-all-ns"))
    {
      reset_all_ns_ = true;
    }    

    if(vm.count("reset-ns"))
    {
      reset_ns_map_[ vm["reset-all-ns"].as<std::string>() ] = true;
    }

    if(vm.count("size-of-all-ns"))
    {
      size_all_shmem_.first = true;
      size_all_shmem_.second = vm["size-of-all-ns"].as<int>();
    }

    if(vm.count("size-of-ns"))
    {
      auto sz = vm["size-of-all-ns"].as<std::pair<std::string,int> >();
      size_shmem_map_[ sz.first ] = sz.second;
    }

    if(vm.count("path-to-file"))
    {
      auto pp = vm["path-to-file"].as<std::vector<std::string>>();
      for(const auto & p : pp)
      {
        std::string what; 
        boost::filesystem::path fp;
        bool ok = cnr::param::utils::filepath(p, fp, what);
        if(!ok)
        {
          std::cerr << what << std::endl;
          throw po::validation_error(po::validation_error::invalid_option_value, "path-to-file", p);
        }
        ns_fn_map_[ default_shmem_id_ ].push_back(fp);
      }
    } 
      
    if(vm.count("ns-and-path-to-file"))
    {
      auto pp = vm["ns-and-path-to-file"].as< std::vector< std::pair<std::string,std::string> > >();
      for(const auto & p : pp)
      {
        if(p.first.empty() || p.second.empty() )
        { 
          throw po::validation_error(po::validation_error::invalid_option_value, "ns-and-path-to-file");
        }
        std::string what; 
        boost::filesystem::path fp;
        bool ok = cnr::param::utils::filepath(p.second, fp, what);
        if(!ok) 
        { 
          std::cerr << what << std::endl;
          throw po::validation_error(po::validation_error::invalid_option_value,"ns-and-path-to-file" );
        }
        ns_fn_map_[ p.first ].push_back(fp);
      }
    }

    return;
  }
  catch(po::error_with_no_option_name& e)
  {
    std::cerr << "Error with no option name: " << e.what() <<"\n" << e.get_option_name() << "\n";
  }
  catch(po::error_with_option_name& e)
  {
    std::cerr << "Error : " << e.what() <<"\n";
    std::cerr << "Option: " << e.get_option_name() << "\n";
    std::string on = e.get_option_name();
    while(on.front()=='-')
    {
      on.erase(on.begin());
    }
    auto it = cmdline_options_.find_nothrow(on,true);
    if(it)
    {
      std::cerr << "Usage : " << it->format_name() << " " << it->format_parameter() << std::endl;
    }
  }
  catch(po::error& e)
  {
    std::cerr << "Generic Boost Error: " << e.what() << std::endl;
  }
  catch (const std::exception &e)
  {
    std::cerr << "Generic std error: " << e.what() << std::endl;
  }
  exit(-1);
}

void ArgParser::printUsage(const boost::program_options::options_description desc, bool verbose)
{
  std::cout << "Usage: " << program_name_ ;
  for(auto const & opt : desc.options() )
  {
    std::cout << " [" << opt->format_name() << " " << opt->format_parameter() << "]";
  }
  std::cout << std::endl;

  if(verbose)
    std::cout << desc << std::endl;
}

const bool& ArgParser::getResetAll() const
{
  return reset_all_ns_;
}

const std::map<std::string, bool>& ArgParser::getResetMap() const
{
  return reset_ns_map_;
}

const std::pair<bool,size_t>& ArgParser::getSizeAll() const
{
  return size_all_shmem_;
}

const std::map<std::string, size_t>& ArgParser::getSizeMap() const
{
  return size_shmem_map_;
}

std::map<std::string, std::vector<std::string> > ArgParser::getNamespacesMap() const
{
  std::map<std::string, std::vector<std::string>> ret;
  for(const auto & ns_fn : ns_fn_map_)
  {
    for(const auto & fn : ns_fn.second )
    {
      ret[ns_fn.first].push_back(fn.c_str());
    }
  }
  return ret;
}
