#include <boost/config.hpp> /* keep it first to prevent nasty warns in MSVC */
#include <algorithm>

#include <string>
#include <istream>
#include <fstream>
#include <yaml-cpp/yaml.h>

#define BOOST_DATE_TIME_NO_LIB

#include <boost/interprocess/file_mapping.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/containers/string.hpp>
#include <boost/interprocess/managed_mapped_file.hpp>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>

#include <cnr_param/utils/string.hpp>
#include <cnr_param/utils/filesystem.hpp>
#include <cnr_param_server/yaml_manager.h>


#include <cnr_param/utils/yaml.hpp>
#include <cnr_param/utils/interprocess.hpp>





// ====================================================================================================
// ====================================================================================================
// ====================================================================================================
// ====================================================================================================
// ====================================================================================================
// ====================================================================================================

YAMLParser::YAMLParser(const std::map<std::string, std::vector<std::string> >& nodes_map)
{
  root_ = YAML::Node(YAML::NodeType::Map);
  std::cout << nodes_map.size() << std::endl;
  for(const auto & node_pair : nodes_map)
  {
    auto& ns_string = node_pair.first;

    std::vector<std::string> ns = cnr::param::utils::tokenize(ns_string,"/");
    
    auto& files = node_pair.second;
    for(auto& file : files)
    {
      // Each yaml file may be composed by different document, separated by 
      // the directives '---' and '...'
      // See https://camel.readthedocs.io/en/latest/yamlref.html
      auto nodes = YAML::LoadAllFromFile(file);
      
      for(const auto & node : nodes) 
      {
        YAML::Node new_node = cnr::param::utils::init_tree(ns, node);
        root_=cnr::param::utils::merge_nodes(root_, new_node);
      }
    }
  }
}

const YAML::Node& YAMLParser::root() const
{
  return root_;
}

//======================================================================
YAMLStreamer::YAMLStreamer(const YAML::Node& root, const std::string& path_to_shared_files)
  : root_(root)
{
  std::string what;
  fs::path absolute_root_path; 
  if(!cnr::param::utils::dirpath(path_to_shared_files, absolute_root_path, what))
  {
    std::string err = std::string(__PRETTY_FUNCTION__) + ":" + std::to_string(__LINE__) + ": " + what;
    throw std::runtime_error(err.c_str());
  }

  if(!streamLeaf(absolute_root_path.string()))
  {
    throw std::runtime_error("Error in creating the shared file mapping");
  }
  if(!YAMLStreamer::streamNodes(absolute_root_path.string()))
  {
    throw std::runtime_error("Error in creating the shared file mapping");
  }
}

bool YAMLStreamer::streamLeaf(const std::string& absolute_root_path_string)
{
  fs::path absolute_root_path(absolute_root_path_string); 

  std::map<std::string, std::vector<std::string> > tree = cnr::param::utils::toLeafMap(root_); //mapped file names;

  for(const auto & leaf : tree)
  {
    fs::create_directories(absolute_root_path / leaf.first);

    for(const auto & fn : leaf.second )
    {
      fs::path rp = fs::path(leaf.first) / fn;
      fs::path ap = fs::absolute(absolute_root_path / rp);

      auto l = __LINE__;
      try
      {
        l = __LINE__;
        auto regiorn = cnr::param::utils::createFileMapping(ap.string());
        if(!regiorn)
        {
          throw std::runtime_error("The file mapping cannot be created!");
        }

        l = __LINE__;
        
        auto keys = cnr::param::utils::tokenize(rp.string(), "/");
        auto node = cnr::param::utils::get_leaf(keys, root_);
        std::string str = YAML::Dump(node);
        str +="\n";

        std::memcpy(regiorn->get_address(), str.c_str(), str.size() );
        #if defined(NDEBUG)
          cnr::param::utils::printMemoryContent(ap.string(), regiorn->get_address(), false);
        #endif
        
      }
      catch(std::exception& e)
      {
        std::cerr << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": Aboslute path: " << ap << std::endl;
        std::cerr << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": last executed line " << l << ":" << e.what() << std::endl;
        return false;
      }
    }
  }
  return true;
}


bool YAMLStreamer::streamNodes(const std::string& absolute_root_path_string)
{
  fs::path absolute_root_path(absolute_root_path_string); 

  std::vector<std::pair<std::string,YAML::Node>> tree = cnr::param::utils::toNodeList(root_); //mapped file names;

  for(const auto & node : tree)
  {
    auto keys = cnr::param::utils::tokenize(node.first, "/");
    
    fs::path rp = fs::path(node.first).string() + ".yaml";
    fs::path ap = fs::absolute(absolute_root_path / rp);
    auto l = __LINE__;
    try
    {
      l = __LINE__;
      auto region = cnr::param::utils::createFileMapping(ap.string());
      if(!region)
      {
        throw std::runtime_error("The file mapping cannot be created!");
      }

      l = __LINE__;
      YAML::Node _node;
      _node[keys.back()] = node.second;

      l = __LINE__;
      std::string str = YAML::Dump(_node);
      str +="\n";
      
      l = __LINE__;
      std::memcpy(region->get_address(), str.c_str(), str.size() );
      
      l = __LINE__;
      #if defined(NDEBUG)
        cnr::param::utils::printMemoryContent(ap.string(), region->get_address(), false);
      #endif
    }
    catch(std::exception& e)
    {
      std::cerr << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": last executed line " << l << ":" << e.what() << std::endl;
      return false;
    }
  }
  
  return true;
}


