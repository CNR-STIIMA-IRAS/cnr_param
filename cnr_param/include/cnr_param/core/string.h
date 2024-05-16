#ifndef CNR_PARAM_INCLUDE_CNR_PARAM_UTILS_STRING
#define CNR_PARAM_INCLUDE_CNR_PARAM_UTILS_STRING

#include <vector>
#include <string>
#include <cnr_param/core/type_traits.h>
#include <cnr_param/core/yaml.h>
#include <cnr_param/core/eigen.h>

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

}
}
}


namespace std
{

/**
 * @brief 
 * 
 * @tparam T 
 * @param vv 
 * @return std::string 
 */
template<typename T>
inline std::string to_string(const std::vector<T>& vv)
{
  std::string ret = "[ ";
  for (auto const & v : vv) ret += std::to_string(v) + " ";
  ret += "]";
  return ret;
}

/**
 * @brief 
 * 
 * @tparam  
 * @param vv 
 * @return std::string 
 */
template<>
inline std::string to_string(const std::vector<std::string>& vv)
{
  std::string ret = "[ ";
  for (auto const & v : vv) ret += v + " ";
  ret += "]";
  return ret;
}

/**
 * @brief 
 * 
 * @tparam T 
 * @param vv 
 * @return std::string 
 */
template<typename T>
inline std::string to_string(const std::vector<std::vector<T>>& vv)
{
  std::string ret = "[\n";
  for (auto const & v : vv) ret += to_string(v) + "\n";
  ret += "]";
  return ret;
}

inline std::string to_string(const std::string& s)
{
  return s;
}

inline std::string to_string(const YAML::Node& n)
{
  std::stringstream ss;
  ss << n;
  return ss.str();
}


template<typename Derived>
inline std::string to_string(const Eigen::MatrixBase<Derived>& n)
{
  return cnr::param::core::to_string(n);
}




}  // namespace std

#endif  /* CNR_PARAM_INCLUDE_CNR_PARAM_UTILS_STRING */