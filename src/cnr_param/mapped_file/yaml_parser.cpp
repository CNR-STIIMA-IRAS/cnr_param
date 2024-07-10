#include <string>
#include <map>
#include <vector>

#include <cnr_yaml/node_utils.h>

#include <cnr_param/mapped_file/yaml_parser.h>
#include <cnr_param/core/string.h>

namespace cnr
{
namespace param
{
namespace mapped_file
{

YAMLParser::YAMLParser(const std::map<std::string, std::vector<std::string> >& nodes_map)
{
  root_ = YAML::Node(YAML::NodeType::Map);
  for(const auto & node_pair : nodes_map)
  {
    auto& ns_string = node_pair.first;

    std::vector<std::string> ns = cnr::param::core::tokenize(ns_string,"/");
    
    auto& files = node_pair.second;
    for(auto& file : files)
    {
      // Each yaml file may be composed by different document, separated by 
      // the directives '---' and '...'
      // See https://camel.readthedocs.io/en/latest/yamlref.html
      auto nodes = YAML::LoadAllFromFile(file);
      
      for(const auto & node : nodes) 
      {
        YAML::Node new_node = cnr::yaml::init_tree(ns, node);
        root_=cnr::yaml::merge_nodes(root_, new_node);
      }
    }
  }
}

const YAML::Node& YAMLParser::root() const
{
  return root_;
}

}  // namespace mapped_file
}  // namespace param
}  // namespace cnr