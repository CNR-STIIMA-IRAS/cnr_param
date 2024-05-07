#include <cstdlib>
#include <iostream>
#include <string>

#include <boost/filesystem.hpp>
#include <boost/program_options/errors.hpp>
#include <boost/program_options.hpp>

#include <cnr_param/core/filesystem.h>
#include <cnr_param/ros2/yaml_formatter.h>
#include <yaml-cpp/node/type.h>

namespace po = boost::program_options;

class ArgParser
{
private:
  po::options_description generic_;
  po::options_description conversion_options_;
  po::options_description cmdline_options_;
  int argc_;
  std::string program_name_;

  const std::string default_shmem_id_;

public:
  ArgParser(int argc, const char* const argv[])
    : generic_("Generic options", 160), conversion_options_("Namespaces (shared memories)", 160), 
        cmdline_options_("Full List of the Options"), argc_(argc), program_name_(argv[0])
    {
      try
      {
        generic_.add_options()("version,v", "print version std::string")("help,h", "produce help message");

        conversion_options_.add_options()
            ("to-ros2-format,t", po::value<std::vector<std::string>>()->composing()->value_name("str"), "Convert the yaml files to the ROS2 format. The path is relative to the executable, or absolute if it starts with '/'")
            ("from-ros2-format,f", po::value<std::vector<std::string>>()->composing()->value_name("str"),"Convert the yaml files from the ROS2 format.  The path is relative to the executable, or absolute if it starts with '/'")
            ("output-filenames,o", po::value<std::vector<std::string>>()->composing()->value_name("str"),"The name of the output file. If not specified, it is the input concat to '_autogererated.yaml'");

        //============================================================
        cmdline_options_.add(conversion_options_).add(generic_);
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

        std::vector<boost::filesystem::path> ofp;
        if (vm.count("output-filenames"))
        {
            auto ofns = vm["to-ros2-format"].as<std::vector<std::string>>();
            auto pwd = boost::filesystem::current_path();
            for(auto & of : ofns)
            {
                boost::filesystem::path fp = of;
                boost::filesystem::path _fp = boost::filesystem::canonical(fp);
                if(!boost::filesystem::exists(_fp.parent_path()))
                {
                    std::cerr << "The output file " << _fp.string() << " should be created a the directory " << _fp.parent_path() << " that does not exist." << std::endl;
                    throw po::validation_error(po::validation_error::invalid_option_value, "output-filenames", of);
                }
                ofp.push_back(_fp);
            }
        }

        for(const auto & opt : {"to-ros2-format", "from-ros2-format"} ) 
        {
            if (vm.count(opt))
            {
                auto pp = vm[opt].as<std::vector<std::string>>();
                for (std::size_t i=0; i< pp.size(); i++)
                {
                    std::cout << "Processing '" << opt << "' option on file " << pp.at(i) << std::endl;
                    const auto & p = pp.at(i);
                    std::string what;
                    boost::filesystem::path fp;
                    bool ok = cnr::param::core::filepath(p, fp, what);
                    if (!ok)
                    {
                        std::cerr << what << std::endl;
                        throw po::validation_error(po::validation_error::invalid_option_value, "path-to-file", p);
                    }
                    YAML::Node node = YAML::LoadFile(fp.string());
                    YAML::Node new_node;
                    if(opt==std::string("to-ros2-format"))
                    {
                        std::cout << "Adding the /** and ros__parameters prefixes" << std::endl;
                        new_node["/**"]["ros__parameters"] = cnr::param::ros2::ros2_yaml_encoder(node);
                    }
                    else 
                    {
                        std::cout << "Removing the /** and ros__parameters prefixes" << std::endl;
                        auto it = node.begin();
                        if((node.Type()!=YAML::NodeType::Map))
                        {
                            std::cerr << "Error in parsing the file. The root node is not the name of a node: \n" << node << std::endl;
                            throw po::validation_error(po::validation_error::invalid_option_value, opt, p);
                        }
                        YAML::Node _node = it->second;
                        if(!_node["ros__parameters"])
                        {
                            std::cerr << "Error in parsing the file. The node name and 'ros__parameters' clause are mssing: \n" << _node << std::endl;
                            throw po::validation_error(po::validation_error::invalid_option_value, opt, p);
                        }
                        new_node = cnr::param::ros2::ros2_yaml_decoder(_node["ros__parameters"]);
                    }
                    std::cout << "fn: " << fp.string() << std::endl;
                    std::string str = YAML::Dump(new_node);
                    if(i<ofp.size())
                    {

                        std::cout << "saving the converted file to '" << ofp.at(i).string() <<"'" << std::endl;
                        std::ofstream of(ofp.at(i).string());
                        of << str;
                        of.close();
                    }
                    else
                    {
                        std::string fn = fp.string();
                        std::size_t lastindex = fn.find_last_of("."); 
                        std::string rawname = fn.substr(0, lastindex);
                        std::cout << "saving the converted file to '" << rawname + "_autogenerated.yaml" <<"'" << std::endl;
                        std::ofstream of(rawname + "_autogenerated.yaml");
                        of << str;
                        of.close();
                    }
                }
            }
        }
        return;
      }
      catch (po::error_with_no_option_name& e)
      {
        std::cerr << "Error with no option name: " << e.what() << "\n" << e.get_option_name() << "\n";
      }
      catch (po::error_with_option_name& e)
      {
        std::cerr << "Error : " << e.what() << "\n";
        std::cerr << "Option: " << e.get_option_name() << "\n";
        std::string on = e.get_option_name();
        while (on.front() == '-')
        {
          on.erase(on.begin());
        }
        auto it = cmdline_options_.find_nothrow(on, true);
        if (it)
        {
          std::cerr << "Usage : " << it->format_name() << " " << it->format_parameter() << std::endl;
        }
      }
      catch (po::error& e)
      {
        std::cerr << "Generic Boost Error: " << e.what() << std::endl;
      }
      catch (const std::exception& e)
      {
        std::cerr << "Generic std error: " << e.what() << std::endl;
      }
      exit(-1);
    }

    void printUsage(const boost::program_options::options_description desc, bool verbose = true)
    {
      std::cout << "Usage: " << program_name_;
      for (auto const& opt : desc.options())
      {
        std::cout << " [" << opt->format_name() << " " << opt->format_parameter() << "]";
      }
      std::cout << std::endl;

      if (verbose)
        std::cout << desc << std::endl;
    }
};

int main(int argc, char* argv[])
{
  // Parsing of program inputs
  ArgParser args(argc, argv);

  // Done!
  return 0;
}
