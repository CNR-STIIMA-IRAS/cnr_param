#ifndef CNR_PARAM__INCLUDE__CNR_PARAM__ROS2__PARAM_RETRIEVER__H
#define CNR_PARAM__INCLUDE__CNR_PARAM__ROS2__PARAM_RETRIEVER__H

#include <string>
#include <vector>
#include <map>
#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

#include <rclcpp/node.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_client.hpp>

#include <cnr_param/core/type_traits.h>

namespace cnr
{
namespace param
{
namespace ros2
{

static constexpr const char* PARAMETER_SEPARATOR_CHAR = ".";
static constexpr const char* PARAMETER_SEPARATOR_STRING = ".";

/**
 * @brief 
 * 
 * @tparam T 
 */
template <typename T>
struct ParamType
{
  static constexpr rclcpp::ParameterType value =
      cnr::param::is_double<T>::value             ? rclcpp::ParameterType::PARAMETER_DOUBLE :
      cnr::param::is_double_v<T>::value           ? rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY :
      cnr::param::is_integer<T>::value            ? rclcpp::ParameterType::PARAMETER_INTEGER :
      cnr::param::is_integer_v<T>::value          ? rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY :
      cnr::param::is_unsigned_integer<T>::value   ? rclcpp::ParameterType::PARAMETER_INTEGER :
      cnr::param::is_unsigned_integer_v<T>::value ? rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY :
      cnr::param::is_string<T>::value             ? rclcpp::ParameterType::PARAMETER_STRING :
      cnr::param::is_string_v<T>::value           ? rclcpp::ParameterType::PARAMETER_STRING_ARRAY :
      cnr::param::is_bool<T>::value               ? rclcpp::ParameterType::PARAMETER_BOOL :
      cnr::param::is_bool_v<T>::value             ? rclcpp::ParameterType::PARAMETER_BOOL_ARRAY :
      cnr::param::is_byte<T>::value               ? rclcpp::ParameterType::PARAMETER_INTEGER :
      cnr::param::is_byte_v<T>::value             ? rclcpp::ParameterType::PARAMETER_BOOL_ARRAY :
                                                    rclcpp::ParameterType::PARAMETER_NOT_SET;
};

/**
 * @brief 
 * 
 */
using AllowedParamType = std::variant<bool, int64_t, double, std::string, std::vector<uint8_t>, std::vector<bool>,
                                      std::vector<int64_t>, std::vector<double>, std::vector<std::string>, void*>;

AllowedParamType as_generic(const rclcpp::Parameter& param);

/**
 * @brief 
 * 
 * @tparam T 
 * @param rhs 
 * @return T 
 */
template <typename T>
T implicit_cast(const AllowedParamType& rhs);

/**
 * @brief The class is a recursive map. This is useful to associate a parameter value to the chain of namespaces
 */
class ParamDictionary
{
public:
  using EmptyParam = bool;
  using NestedParams = std::map<std::string, ParamDictionary>;
  using AllowedParam = std::variant<EmptyParam, rclcpp::Parameter, NestedParams>;
  // root :
  using ParamValue = std::pair<std::string, AllowedParam>;

  ParamDictionary() = delete;
  ParamDictionary(const std::string& key);
  ParamDictionary(const ParamDictionary& tree) = default;
  ~ParamDictionary() = default;

  const std::string& name() const;
  const AllowedParam& value() const;
  ParamDictionary& operator=(const rclcpp::Parameter& value);
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

bool to_yaml(const ParamDictionary& tree, YAML::Node& node, std::string& what);


/**
 * @brief
 *
 * @tparam Key is the name of the param, eventually nested_params. The namespace separator is '.' as in ROS 2 standard.
 * @tparam Value
 * @param tree
 * @param key
 * @return ParamDictionary*
 */
const ParamDictionary* find(const ParamDictionary& tree, const std::string key, std::string& what);

/**
 * @brief
 *
 */
class ParamRetriever
{
public:
  ParamRetriever() = delete;
  ParamRetriever(rclcpp::Node::SharedPtr& node);

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
  bool get_parameter(const std::string& resolved_node_name, const std::string& resolved_key, ParamDictionary& param,
                     std::string& what, bool updated = false);

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

  ParamDictionary& node_params(const std::string& node_name);
  const ParamDictionary& node_params(const std::string& node_name) const;

  /**
   * @brief
   *
   * @param node_name
   * @param keys
   * @param parameter_names
   * @param what
   * @return true
   * @return false
   */
  bool list_parameters(const std::string& node_name, const std::vector<std::string>& keys,
                       std::vector<std::string>& parameter_names, std::string& what);
private:
  rclcpp::Node::SharedPtr& node_;
  std::map<std::string, rclcpp::AsyncParametersClient::SharedPtr> parameters_client_;
  std::vector<ParamDictionary> node_params_;

  /**
   * @brief
   *
   * @param node_name
   * @return true
   * @return false
   */
  bool init_async_params_client(const std::string& node_name);


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
                               std::vector<rclcpp::Parameter>& parameters, std::string& what);

  /**
   * @brief
   *
   * @param dictionary
   * @param key
   * @param value
   */
  static void insert_dict(ParamDictionary& dictionary, const std::string& key, const rclcpp::Parameter& value);

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

}  // namespace ros2
}  // namespace param
}  // namespace cnr

namespace std
{

/**
 * @brief 
 * 
 * @param val 
 * @return std::string 
 */
std::string to_string(const rclcpp::Parameter& val);

/**
 * @brief 
 * 
 * @param val 
 * @return std::string 
 */
std::string to_string(const std::shared_ptr<rclcpp::Parameter>& val);

/**
 * @brief 
 * 
 * @param val 
 * @return std::string 
 */
std::string to_string(const cnr::param::ros2::ParamDictionary& val);

}  // namespace std

#include <cnr_param/ros2/impl/param_retriever.hpp>

#endif  // CNR_PARAM__INCLUDE__CNR_PARAM__ROS2__PARAM_RETRIEVER__H