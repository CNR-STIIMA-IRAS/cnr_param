#ifndef CNR_PARAM__INCLUDE__CNR_PARAM__MAPPED_FILE__YAML_PARSER_H
#define CNR_PARAM__INCLUDE__CNR_PARAM__MAPPED_FILE__YAML_PARSER_H

#include <vector>
#include <string>
#include <yaml-cpp/yaml.h>

#include <cnr_param/core/visibility_control.h>

#include <string>


namespace cnr
{
namespace param
{
namespace mapped_file
{

/**
 * @brief The YAMLParser class is responsible for parsing YAML files and providing access to the parsed data.
 */
class YAMLParser
{
public:
  YAMLParser() = delete;
  virtual ~YAMLParser() = default;

  /**
   * @brief Constructs a YAMLParser object with the given nodes map.
   * @param nodes_map A map containing node names and their corresponding paths.
   */
  YAMLParser(const std::map<std::string, std::vector<std::string>>& nodes_map);

  /**
   * @brief Returns the root node of the parsed YAML file.
   * @return The root node of the parsed YAML file.
   */
  const YAML::Node& root() const;

private:
  YAML::Node root_; /**< The root node of the parsed YAML file. */
};

}  // namespace mapped_file
}  // namespace param
}  // namespace cnr

#endif  // CNR_PARAM__INCLUDE__CNR_PARAM__MAPPED_FILE__YAML_PARSER_H