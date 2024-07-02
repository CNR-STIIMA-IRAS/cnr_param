#ifndef CNR_PARAM__INCLUDE__CNR_PARAM__CORE__PARAM_DICTIONARY__H
#define CNR_PARAM__INCLUDE__CNR_PARAM__CORE__PARAM_DICTIONARY__H

#include <string>
#include <map>
#include <variant>
#include <Eigen/Core>
#include <yaml-cpp/node/node.h>
#include <yaml-cpp/node/type.h>
#include <yaml-cpp/yaml.h>

#include <cnr_yaml/type_traits.h>

namespace cnr
{
namespace param
{
namespace core
{

/**
 * @brief The class is a recursive map. This is useful to associate a parameter value to the chain of namespaces
 */
template <typename P>
class ParamDictionary
{
public:
  using base_type = P;
  using EmptyParam = bool;
  using NestedParams = std::map<std::string, ParamDictionary>;
  using AllowedParam = std::variant<EmptyParam, P, NestedParams>;
  // root :
  using ParamValue = std::pair<std::string, AllowedParam>;

  ParamDictionary() = delete;
  ParamDictionary(const std::string& key);
  ParamDictionary(const ParamDictionary& tree) = default;
  ~ParamDictionary() = default;

  const std::string& name() const;
  const AllowedParam& value() const;
  ParamDictionary& operator=(const P& value);
  ParamDictionary& operator=(const NestedParams& value);
  ParamDictionary& operator[](const std::string& key);

  const ParamDictionary& operator[](const std::string& key) const;

  bool initialized() const;
  std::string to_string(const std::string& prefix) const;
  bool is_scalar() const;
  bool is_sequence() const;
  bool is_map() const;

private:
  ParamValue param_;
};

template <typename P>
bool to_yaml(const ParamDictionary<P>& tree, YAML::Node& node, std::string& what);

/**
 * @brief
 *
 * @tparam Key is the name of the param, eventually nested_params. The namespace separator is '.' as in ROS 2 standard.
 * @tparam Value
 * @param tree
 * @param key
 * @return ParamDictionary*
 */
template <typename P>
const ParamDictionary<P>* find(const ParamDictionary<P>& tree, const std::string key,
                               const std::string& parameter_separator_string, std::string& what);

}  // namespace core
}  // namespace param
}  // namespace cnr

#include <cnr_param/core/impl/param_dictionary.hpp>

#endif  // CNR_PARAM__INCLUDE__CNR_PARAM__CORE__PARAM_DICTIONARY__H