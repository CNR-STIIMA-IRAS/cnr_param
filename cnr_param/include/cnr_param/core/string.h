#ifndef CNR_PARAM_INCLUDE_CNR_PARAM_UTILS_STRING
#define CNR_PARAM_INCLUDE_CNR_PARAM_UTILS_STRING

#include <vector>
#include <string>
#include <cnr_param/core/type_traits.h>
#include <cnr_param/core/yaml.h>
#include <cnr_param/core/eigen.h>
#include <yaml-cpp/node/node.h>

namespace std
{
inline std::string to_string(const std::string& v)
{
  return v;
}

inline std::string to_string(const YAML::NodeType::value& v)
{
  switch(v)
  {
    case YAML::NodeType::Null: return "Null";
    case YAML::NodeType::Scalar: return "Scalar";
    case YAML::NodeType::Sequence: return "Sequence";
    case YAML::NodeType::Map: return "Map";
    case YAML::NodeType::Undefined: return "Undefined";
    default: return "Unknown";
  }
}
}  // namespace std

namespace cnr
{
namespace param
{
namespace core
{

/**
 * @brief Tokenize a string, using a/multiple delimiters
 *
 * @param str
 * @param delim
 * @return std::vector<std::string>
 */
std::vector<std::string> tokenize(const std::string& str, const std::string& delim);

template <typename T, typename std::enable_if<
                          !std::is_same<T, std::string>::value && !cnr::param::is_vector<T>::value &&
                              !cnr::param::is_matrix_expression<T>::value && !std::is_same<T, YAML::Node>::value,
                          bool>::type = true>
inline void to_string(const T& v, std::string& ret)
{
  ret = std::to_string(v);
}

template <typename T, typename std::enable_if<std::is_same<T, std::string>::value, bool>::type = true>
inline void to_string(const T& v, std::string& ret)
{
  ret = v;
}

template <typename T, typename std::enable_if<std::is_same<T, YAML::Node>::value, bool>::type = true>
inline void to_string(const T& v, std::string& ret)
{
  std::stringstream ss;
  ss << v;
  ret = ss.str();
}


template <typename D, typename std::enable_if<cnr::param::is_matrix_expression<D>::value, bool>::type = true>
inline void to_string(const Eigen::MatrixBase<D>& m, std::string& ret)
{
  Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
  std::stringstream ss;
  ss << std::fixed << m.transpose().format(CleanFmt);
  ret = ss.str();
}

template <typename T, typename std::enable_if<cnr::param::is_vector<T>::value, bool>::type = true>
inline void to_string(const T& v, std::string& ret)
{
  ret = "[ ";
  for (auto const& vi : v)
  {
    std::string tmp;
    to_string(vi, tmp);
    ret += tmp + " ";
  }
  ret += "]";
}

}  // namespace core
}  // namespace param
}  // namespace cnr

namespace std
{
template <typename T>
inline std::string to_string(const T& v)
{
  std::string ret;
  cnr::param::core::to_string(v, ret);
  return ret;
}
}  // namespace std

#endif /* CNR_PARAM_INCLUDE_CNR_PARAM_UTILS_STRING */