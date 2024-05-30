#include <cassert>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>

//#include <cnr_param/filesystem.h>
#include <cnr_param/core/string.h>
#include <cnr_param/core/yaml.h>

namespace cnr
{
namespace param
{
namespace core
{

const YAML::Node merge_nodes(const YAML::Node& default_node, const YAML::Node& override_node)
{
  if (!override_node.IsMap())
  {
    // If override_node is not a map, merge result is override_node, unless override_node is null
    return override_node.IsNull() ? default_node : override_node;
  }
  if (!default_node.IsMap())
  {
    // If default_node is not a map, merge result is override_node
    return override_node;
  }
  if (!default_node.size())
  {
    return YAML::Node(override_node);
  }
  // Create a new map 'new_node' with the same mappings as default_node, merged with override_node
  auto new_node = YAML::Node(YAML::NodeType::Map);
  for (auto node : default_node)
  {
    if (node.first.IsScalar())
    {
      const std::string& key = node.first.Scalar();
      new_node[key] = bool(override_node[key]) ? merge_nodes(node.second, override_node[key]) : node.second;
    }
    else
    {
      new_node[node.first] = node.second;
    }
  }
  // Add the mappings from 'override_node' not already in 'new_node'
  for (auto node : override_node)
  {
    if (node.first.IsScalar())
    {
      const std::string& key = node.first.Scalar();
      new_node[key] =
          bool(default_node[key]) ? merge_nodes(default_node[key], node.second) : new_node[key] = node.second;
    }
    else
    {
      new_node[node.first] = node.second;
    }
  }
  return YAML::Node(new_node);
}

YAML::Node init_tree(const std::vector<std::string> seq, const YAML::Node& node)
{
  if (seq.size() == 0)
  {
    return YAML::Node(node);
  }
  else
  {
    auto new_node = YAML::Node(YAML::NodeType::Map);
    new_node[seq.back()] = node;
    if (seq.size() == 1)
    {
      return new_node;
    }

    std::vector<std::string> new_seq = seq;
    ;
    new_seq.erase(new_seq.end() - 1);
    return init_tree(new_seq, new_node);
  }
}

YAML::iterator get_node(const std::string& key, YAML::iterator& node_begin, YAML::iterator& node_end)
{
  YAML::iterator it = node_begin;
  for (it = node_begin; it != node_end; ++it)
  {
    if (it->first.IsScalar())
    {
      const std::string& node_key = it->first.Scalar();
      if (node_key == key)
      {
        return it;
      }
    }
  }
  return it;
}

void get_keys_tree(const std::string& ns, const YAML::Node& node, std::vector<std::string>& tree)
{
  auto l = __LINE__;
  try
  {
    if (!node.IsDefined())
    {
      throw std::runtime_error("Node not defined!");
    }
    l = __LINE__;
    if (node.IsMap())
    {
      l = __LINE__;
      for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
      {
        std::string key = (ns.empty() ? "/" : ns) + "/" + it->first.as<std::string>() + "/";
        if (it->second.IsMap())
        {
          l = __LINE__;
          get_keys_tree(key, it->second, tree);
        }
        else
        {
          l = __LINE__;
          tree.push_back(key);
        }
      }
    }
    // else if(node.IsScalar() )
    // {
    //   std::string key = node.begin()->first.as<std::string>();
    //   auto value = node.begin()->second;
    //   tree.push_back( (ns.empty() ? "/" : ns) + "/" + key +"/");
    //   get_keys_tree( tree.back(), value, tree );
    // }
  }
  catch (YAML::InvalidNode& e)
  {
    std::cout << __LINE__ << "@" << l << ":" << e.what() << std::endl;
    std::cout << "Input Node: \n" << node << std::endl;
    std::cout << "Continue" << std::endl;
    assert(0);
  }
  catch (std::exception& e)
  {
    std::cout << __LINE__ << "@" << l << ":" << e.what() << std::endl;
    std::cout << "Input Node Type: " << node.Type() << std::endl;
    std::cout << "Input Node: \n" << node << std::endl;
    std::cout << "Continue" << std::endl;
    assert(0);
  }
  return;
}

void get_nodes_tree(const std::string& ns, const YAML::Node& node,
                    std::vector<std::pair<std::string, YAML::Node>>& tree)
{
  auto l = __LINE__;
  try
  {
    if (!node.IsDefined())
    {
      throw std::runtime_error("Node not defined!");
    }
    l = __LINE__;
    if (node.IsMap())
    {
      l = __LINE__;
      for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
      {
        std::string _ns = ns.empty() ? "/" : ns.back() == '/' ? ns.substr(0, ns.length() - 1) : ns;
        std::string _key = _ns + "/" + it->first.as<std::string>();
        tree.push_back(std::make_pair(_key, it->second));
        if (it->second.IsMap())
        {
          l = __LINE__;
          get_nodes_tree(_key, it->second, tree);
        }
      }
    }
  }
  catch (YAML::InvalidNode& e)
  {
    std::cout << __LINE__ << "@" << l << ":" << e.what() << std::endl;
    std::cout << "Input Node: \n" << node << std::endl;
    std::cout << "Continue" << std::endl;
    assert(0);
  }
  catch (std::exception& e)
  {
    std::cout << __LINE__ << "@" << l << ":" << e.what() << std::endl;
    std::cout << "Input Node Type: " << node.Type() << std::endl;
    std::cout << "Input Node: \n" << node << std::endl;
    std::cout << "Continue" << std::endl;
    assert(0);
  }
  return;
}

YAML::Node get_leaf(const std::vector<std::string>& keys, const YAML::Node& node)
{
  if (keys.size() == 0)
  {
    return node;
  }

  auto it = keys.begin();
  std::string key = keys.front();
  for (YAML::const_iterator yt = node.begin(); yt != node.end(); ++yt)
  {
    const std::string& node_key = yt->first.as<std::string>();
    if (node_key == key)
    {
      if (keys.size() > 1)
      {
        std::vector<std::string> remaining_keys(it + 1, keys.end());
        return get_leaf(remaining_keys, yt->second);
      }
      else
      {
        auto ret = YAML::Node(YAML::NodeType::Map);
        ret[yt->first.as<std::string>()] = yt->second;
        return ret;
      }
    }
  }
  std::cout << "Weird error " << key << " < ";
  for (YAML::const_iterator yt = node.begin(); yt != node.end(); ++yt)
  {
    const std::string& node_key = yt->first.as<std::string>();
    std::cout << node_key << " ";
  }
  std::cout << ">" << std::endl;
  return YAML::Node();
}

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
  cnr::param::core::get_keys_tree("", root, fns);

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

/**
 * @brief
 *
 * @param root
 * @return std::vector<std::pair<std::string, YAML::Node>>
 */
std::vector<std::pair<std::string, YAML::Node>> toNodeList(const YAML::Node& root)
{
  std::vector<std::pair<std::string, YAML::Node>> tree;  // mapped file names;
  cnr::param::core::get_nodes_tree("", root, tree);
  return tree;
}



/**
 * @brief 
 * 
 * @param node 
 * @param key 
 * @return true 
 * @return false 
 */
bool has(const YAML::Node& node, const std::string& key)
{
  return bool(node[key]);
}

/**
 * @brief 
 * 
 * @param node 
 * @return true 
 * @return false 
 */
bool is_scalar(const YAML::Node& node)
{
  return node.IsScalar();
}

/**
 * @brief 
 * 
 * @param node 
 * @return true 
 * @return false 
 */
bool is_sequence(const YAML::Node& node)
{
  return node.IsSequence();
}

/**
 * @brief 
 * 
 * @param node 
 * @return true 
 * @return false 
 */
bool is_map(const YAML::Node& node)
{
  return node.IsMap();
}

/**
 * @brief 
 * 
 * @param node 
 * @return size_t 
 */
size_t size(const YAML::Node& node)
{
  if(!is_sequence(node))
  {
    std::stringstream err;
    err << __PRETTY_FUNCTION__ << ":" << std::to_string(__LINE__)  << ": " 
      << "The node is not a sequence!";
    throw std::runtime_error(err.str().c_str());
  }
  return node.size();
}

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
bool get_leaf(const YAML::Node& node, const std::string& key, YAML::Node& leaf, std::string& what)
{
   if(node[key])
   {
    leaf = node[key];
    return true;
   }
   what = "The key is not in the node dictionary";
   return false;
}



}  // namespace core
}  // namespace param
}  // namespace cnr