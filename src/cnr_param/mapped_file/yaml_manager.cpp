//#include <boost/config.hpp> /* keep it first to prevent nasty warns in MSVC */

#include <string>

#include <yaml-cpp/yaml.h>

#include <boost/interprocess/file_mapping.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/containers/string.hpp>
#include <boost/interprocess/managed_mapped_file.hpp>

#include <cnr_yaml/string.h>
#include <cnr_yaml/node_utils.h>

#include <cnr_param/core/string.h>
#include <cnr_param/core/filesystem.h>

#include <cnr_param/mapped_file/interprocess.h>
#include <cnr_param/mapped_file/yaml_manager.h>


/**
 * @brief
 *
 * @param root
 * @return std::map<std::string, std::vector<std::string> >
 */
std::map<std::string, std::vector<std::string>> toLeafMap(const YAML::Node& root)
{
  std::map<std::string, std::vector<std::string>> tree;  // mapped file names;
  std::vector<std::string> fns;                          // mapped file names;
  fns.reserve(1000000);
  cnr::yaml::get_keys_tree("", root, fns);

  std::sort(fns.begin(), fns.end(), [](std::string a, std::string b) { return a < b; });
  fns.erase(std::unique(fns.begin(), fns.end()), fns.end());
  for (const auto& fn : fns)
  {
    std::vector<std::string> pp = cnr::param::core::tokenize(fn, "/");
    boost::filesystem::path path;
    for (size_t i = 0; i < pp.size() - 1; i++)
    {
      path = path / pp.at(i);
    }
    tree[path.string()].push_back(pp.back());
  }
  return tree;
}

namespace cnr
{
namespace param
{
namespace mapped_file
{

YAMLStreamer::YAMLStreamer(const YAML::Node& root, const std::string& path_to_shared_files)
  : root_(root)
{
  std::string what;
  boost::filesystem::path absolute_root_path; 
  if(!cnr::param::core::dirpath(path_to_shared_files, absolute_root_path, what))
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
  boost::filesystem::path absolute_root_path(absolute_root_path_string); 

  std::map<std::string, std::vector<std::string> > tree = toLeafMap(root_); //mapped file names;

  for(const auto & leaf : tree)
  {
    boost::filesystem::create_directories(absolute_root_path / leaf.first);

    for(const auto & fn : leaf.second )
    {
      boost::filesystem::path rp = boost::filesystem::path(leaf.first) / fn;
      boost::filesystem::path ap = boost::filesystem::absolute(absolute_root_path / rp);

      auto l = __LINE__;
      try
      {
        l = __LINE__;
        
        auto keys = cnr::param::core::tokenize(rp.string(), "/");
        auto node = cnr::yaml::get_leaf(keys, root_);
        if(node.begin()->second.Type() == YAML::NodeType::Null)
        {
          std::cerr << "WARNING: the node '" << node << "' has a NULL Value. The streaming to the mapped file value is skipped" << std::endl;
          continue;
        }
        std::string str = YAML::Dump(node);

        l = __LINE__;
        auto regiorn = cnr::param::mapped_file::createFileMapping(ap.string(),2*str.size());
        if(!regiorn)
        {
          throw std::runtime_error("The file mapping cannot be created!");
        }
        str +="\n";

        if( str.size() > regiorn->get_size())
        {
          std::cerr << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": BUG --- TO BE FIXED!!!! " << std::endl;
          std::cerr << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": Aboslute path: " << ap << std::endl;
          std::cerr << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": Available Memory Size : " << regiorn->get_size() << " / Required Memory Size:" << str.size() << std::endl;
          return false;
        }
        std::memcpy(regiorn->get_address(), str.c_str(), str.size() );
        // #if defined(NDEBUG)
        //   cnr::param::mapped_file::printMemoryContent(ap.string(), regiorn->get_address(), false);
        // #endif
        
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
  boost::filesystem::path absolute_root_path(absolute_root_path_string); 

  std::vector<std::pair<std::string,YAML::Node>> tree = cnr::yaml::toNodeList(root_); //mapped file names;

  for(const auto & node : tree)
  {
    auto keys = cnr::param::core::tokenize(node.first, "/");
    
    boost::filesystem::path rp = boost::filesystem::path(node.first).string() + ".yaml";
    boost::filesystem::path ap = boost::filesystem::absolute(absolute_root_path / rp);
    auto l = __LINE__;
    try
    {
      l = __LINE__;
      YAML::Node _node;
      if(node.second.Type() == YAML::NodeType::Null)
      {
        std::cerr << "WARNING: the node '" << node.second << "' has a NULL Value. The streaming of the node to the mapped yaml file is skipped." << std::endl;
        continue;
      }
      _node[keys.back()] = node.second;

      l = __LINE__;
      std::string str = YAML::Dump(_node);
      str +="\n";
      
      l = __LINE__;
      auto region = cnr::param::mapped_file::createFileMapping(ap.string(),2*str.size());
      if(!region)
      {
        throw std::runtime_error("The file mapping cannot be created!");
      }

      l = __LINE__;
      std::memcpy(region->get_address(), str.c_str(), str.size() );
      
      l = __LINE__;
      // #if defined(NDEBUG)
      //   cnr::param::mapped_file::printMemoryContent(ap.string(), region->get_address(), false);
      // #endif
    }
    catch(std::exception& e)
    {
      std::cerr << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": last executed line " << l << ":" << e.what() << std::endl;
      return false;
    }
  }
  
  return true;
}

}  // namespace mapped_file
}  // namespace param
}  // namespace cnr