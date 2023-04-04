#ifndef CNR_PARAM_INCLUDE_CNR_PARAM_IMPL_STRING
#define CNR_PARAM_INCLUDE_CNR_PARAM_IMPL_STRING

#include <string>
#include <vector>

#include <cnr_param/cnr_param.hpp>

namespace utils
{

template<typename T>
inline std::string to_string(const std::vector<T>& vv)
{
  std::string ret = "[ ";
  for (auto const & v : vv) ret += std::to_string(v) + " ";
  ret += "]";
  return ret;
}

template<>
inline std::string to_string(const std::vector<std::string>& vv)
{
  std::string ret = "[ ";
  for (auto const & v : vv) ret += v + " ";
  ret += "]";
  return ret;
}

template<typename T>
inline std::string to_string(const std::vector<std::vector<T>>& vv)
{
  std::string ret = "[\n";
  for (auto const & v : vv) ret += to_string(v) + "\n";
  ret += "]";
  return ret;
}

}  // namespace utils

#endif  /* CNR_PARAM_INCLUDE_CNR_PARAM_IMPL_STRING */
