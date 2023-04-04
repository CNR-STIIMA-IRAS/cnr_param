#ifndef SRC_CNR_PARAM_INCLUDE_CNR_PARAM_SERVER_YAML_MANAGER
#define SRC_CNR_PARAM_INCLUDE_CNR_PARAM_SERVER_YAML_MANAGER

#include <iostream>
#include <vector>
#include <string>
#include <yaml-cpp/yaml.h>

#include <cnr_param/visibility_control.h>

#include <string>

class YAMLParser
{
public:
  YAMLParser() = delete;
  virtual ~YAMLParser() = default;
  YAMLParser(const std::map<std::string, std::vector<std::string> >& nodes_map);

  const YAML::Node& root() const;
private:
  YAML::Node root_;
};

class YAMLStreamer
{
public:
  YAMLStreamer() = delete;
  virtual ~YAMLStreamer() = default;
  YAMLStreamer(const YAML::Node& root,const std::string& path_to_files);

private:
  //std::map< std::string, boost::interprocess::managed_mapped_file > shd_file_;
  YAML::Node root_;
  bool streamLeaf(const std::string& absolute_root_path);
  bool streamNodes(const std::string& absolute_root_path);
};

#endif  /* SRC_CNR_PARAM_INCLUDE_CNR_PARAM_SERVER_YAML_MANAGER */
