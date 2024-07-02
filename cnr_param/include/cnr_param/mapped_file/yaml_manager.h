#ifndef CNR_PARAM__INCLUDE__CNR_PARAM__MAPPED_FILE__YAML_MANAGER_H
#define CNR_PARAM__INCLUDE__CNR_PARAM__MAPPED_FILE__YAML_MANAGER_H

#include <string>
#include <yaml-cpp/yaml.h>

namespace cnr
{
namespace param
{
namespace mapped_file
{

/**
 * @brief The YAMLStreamer class is responsible for streaming YAML data to files.
 */
class YAMLStreamer
{
public:
  YAMLStreamer() = delete;
  virtual ~YAMLStreamer() = default;

  /**
   * @brief Constructs a YAMLStreamer object with the given root node and path to files.
   * @param root The root node of the YAML data to be streamed.
   * @param path_to_files The path to the directory where the YAML files will be streamed.
   */
  YAMLStreamer(const YAML::Node& root, const std::string& path_to_files);

private:
  YAML::Node root_; /**< The root node of the YAML data to be streamed. */
  
  /**
   * @brief Streams the leaf nodes of the YAML data to files.
   * @param absolute_root_path The absolute root path where the YAML files will be streamed.
   * @return True if the leaf nodes were successfully streamed, false otherwise.
   */
  bool streamLeaf(const std::string& absolute_root_path);

  /**
   * @brief Streams the non-leaf nodes of the YAML data to files.
   * @param absolute_root_path The absolute root path where the YAML files will be streamed.
   * @return True if the non-leaf nodes were successfully streamed, false otherwise.
   */
  bool streamNodes(const std::string& absolute_root_path);
};

}  // namespace mapped_file
}  // namespace param
}  // namespace cnr

#endif   // CNR_PARAM__INCLUDE__CNR_PARAM__MAPPED_FILE__YAML_MANAGER_H
