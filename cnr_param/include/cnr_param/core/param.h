#ifndef CNR_PARAM__INCLUDE__CNR_PARAM__CORE__PARAM_H
#define CNR_PARAM__INCLUDE__CNR_PARAM__CORE__PARAM_H

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
 * @brief Get the scalar object
 *
 * @tparam T
 * @param node
 * @param ret
 * @param what
 * @return true
 * @return false
 */
template <typename T>
inline bool get_scalar(const YAML::Node& node, T& ret, std::string& what, const bool& implicit_cast_if_possible);

/**
 * @brief Get a sequence
 *
 * @tparam T
 * @param node
 * @param ret
 * @param what
 * @return true
 * @return false
 */
template <typename T>
inline bool get_sequence(const YAML::Node& node, T& ret, std::string& what, const bool& implicit_cast_if_possible);

/**
 * @brief Get a map object
 *
 * @tparam T
 * @param node
 * @param ret
 * @param what
 * @return true
 * @return false
 */
template <typename T>
inline bool get_map(const YAML::Node& node, T& ret, std::string& what, const bool& implicit_cast_if_possible);

/**
 * @brief
 *
 * @param node
 * @param key
 * @return true
 * @return false
 */
bool has(const YAML::Node& node, const std::string& key);

/**
 * @brief
 *
 * @param node
 * @return true
 * @return false
 */
bool is_sequence(const YAML::Node& node);

/**
 * @brief
 *
 * @param node
 * @return true
 * @return false
 */
bool is_map(const YAML::Node& node);

// =============================================================================================

/**
 * @brief
 *
 * @tparam T
 * @param node
 * @param key
 * @param value
 * @param format : by now only format for int is supported. Specify if it is 'dec' or 'hex'

 */
template <typename T>
void insert(YAML::Node& node, const std::string& key, const T& value, const std::string& format = "");


// template <typename T>
// T extract(const YAML::Node& node, const std::string& key = "", const std::string& error_heading_msgs = "");

}  // namespace core
}  // namespace param
}  // namespace cnr

#include <cnr_param/core/impl/param_scalar.hpp>
#include <cnr_param/core/impl/param_sequence.hpp>
#include <cnr_param/core/impl/param_map.hpp>
#include <cnr_param/core/impl/param_insert.hpp>
//#include <cnr_param/core/impl/param_extract.hpp>

#endif /* CNR_PARAM__INCLUDE__CNR_PARAM__CORE__PARAM_H */
