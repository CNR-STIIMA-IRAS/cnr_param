#ifndef CNR_PARAM__INCLUDE__CNR_PARAM__CORE__PARAM_RETRIEVER__H
#define CNR_PARAM__INCLUDE__CNR_PARAM__CORE__PARAM_RETRIEVER__H

#include <string>
#include <vector>
#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

#include <cnr_yaml/type_traits.h>
#include <cnr_param/core/param_dictionary.h>

namespace cnr
{
namespace param
{
namespace core
{
/**
 * @brief
 *
 */
template <typename N, typename P>
class ParamRetriever
{
public:
  ParamRetriever() = delete;
  ParamRetriever(std::shared_ptr<N> nh, const std::string& parameter_separator_string);

  /**
   * @brief Get the parameter object
   *
   * @param key
   * @param param
   * @param what
   * @param updated
   * @return true
   * @return false
   */
  bool get_parameter(const std::string& resolved_node_name, const std::string& resolved_key, ParamDictionary<P>& param,
                     std::string& what, bool updated = false);

  bool set_parameter(const std::string& resolved_node_name, const std::string& resolved_key, const P& param, std::string& what);
  /**
   * @brief
   *
   * @param ns
   * @param resolved_node_name
   * @param resolved_ns
   * @param what
   * @return true
   * @return false
   */
  bool resolve_names(const std::string& ns, std::string& resolved_node_name, std::string& resolved_ns,
                     std::string& what);

  ParamDictionary<P>& node_params(const std::string& node_name);

  const ParamDictionary<P>& node_params(const std::string& node_name) const;

  bool list_parameters(const std::string& node_name, const std::vector<std::string>& keys,
                       std::vector<std::string>& parameter_names, std::string& what);

protected:
  std::shared_ptr<N> node_;
  std::vector<ParamDictionary<P>> node_params_;
  const std::string parameter_separator_string_;

  /**
   * @brief Get the parameters for a vector of keys that have already been checked
   *
   * @param node_name
   * @param keys
   * @param parameters
   * @param what
   * @return true
   * @return false
   */
  bool get_existent_parameters(const std::string& node_name, const std::vector<std::string>& keys,
                               std::vector<P>& parameters, std::string& what);

  /**
   * @brief
   *
   * @param dictionary
   * @param key
   * @param value
   */
  void insert_dict(ParamDictionary<P>& dictionary, const std::string& key, const P& value);

  /**
   * @brief
   *
   * @param node_name
   * @param resolved_key is the name of the param, eventually nested_params. The namespace separator is '.' as in ROS 2
   * standard.
   * @param dictionary
   * @param updated
   * @return true
   * @return false
   */
  bool retrieve_parameters(const std::string& resolved_node_name, const std::string& resolved_key, std::string& what,
                           bool updated = false);
};

template <typename N>
bool getNodeNames(const std::shared_ptr<N>& n, std::vector<std::string>& names, std::string& what);

template <typename N>
bool resolveParamName(const std::shared_ptr<N>& no, const std::string& ns, std::string& resolved_node_name, std::string& resolved_key,
                      std::string& what);

template <typename N>
bool resolveNodeName(const std::shared_ptr<N>& n, std::string& resolved_name, std::string& what);

template <typename N>
std::string lintParamKey(const std::shared_ptr<N>& n, const std::string& param_key);

}  // namespace core
}  // namespace param
}  // namespace cnr

#include <cnr_param/core/impl/param_retriever.hpp>

#endif  // CNR_PARAM__INCLUDE__CNR_PARAM__CORE__PARAM_RETRIEVER__H