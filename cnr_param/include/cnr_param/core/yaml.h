#ifndef CNR_PARAM__INCLUDE__CNR_PARAM__CORE__YAML_H
#define CNR_PARAM__INCLUDE__CNR_PARAM__CORE__YAML_H

#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace cnr
{
namespace param
{
namespace core
{

/**
 * @brief
 *
 * @param default_node
 * @param override_node
 * @return const YAML::Node
 */
const YAML::Node merge_nodes(const YAML::Node& default_node, const YAML::Node& override_node);

/**
 * @brief
 *
 * @param seq
 * @param node
 * @return YAML::Node
 */
YAML::Node init_tree(const std::vector<std::string> seq, const YAML::Node& node);

/**
 * @brief
 *
 * @param key
 * @param node_begin
 * @param node_end
 * @return YAML::iterator
 */
YAML::iterator get_node(const std::string& key, YAML::iterator& node_begin, YAML::iterator& node_end);

/**
 * @brief Get the leaf object
 *
 * @param keys
 * @param node
 * @return YAML::Node
 */
YAML::Node get_leaf(const std::vector<std::string>& keys, const YAML::Node& node);

/**
 * @brief Get the leaf object
 *
 * @param node
 * @param key
 * @param leaf
 * @param what
 * @return true
 * @return false
 */
bool get_leaf(const YAML::Node& node, const std::string& key, YAML::Node& leaf, std::string& what);

/**
 * @brief Get the keys tree object
 *
 * @param ns
 * @param node
 * @param tree
 */
void get_keys_tree(const std::string& ns, const YAML::Node& node, std::vector<std::string>& tree);

/**
 * @brief Get the nodes tree object
 *
 * @param ns
 * @param node
 * @param tree
 */
void get_nodes_tree(const std::string& ns, const YAML::Node& node,
                    std::vector<std::pair<std::string, YAML::Node>>& tree);

/**
 * @brief
 *
 * @param root
 * @return std::map<std::string, std::vector<std::string> >
 */
std::map<std::string, std::vector<std::string>> toLeafMap(const YAML::Node& root);

/**
 * @brief
 *
 * @param root
 * @return std::vector<std::pair<std::string, YAML::Node>>
 */
std::vector<std::pair<std::string, YAML::Node>> toNodeList(const YAML::Node& root);

}  // namespace core
}  // namespace param
}  // namespace cnr

#endif // CNR_PARAM__INCLUDE__CNR_PARAM__CORE__YAML_H
