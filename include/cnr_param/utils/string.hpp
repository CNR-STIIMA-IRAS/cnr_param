#ifndef CNR_PARAM_INCLUDE_CNR_PARAM_UTILS_STRING
#define CNR_PARAM_INCLUDE_CNR_PARAM_UTILS_STRING

#include <vector>
#include <string>

namespace cnr 
{
namespace param
{
namespace utils
{

/**
 * @brief Tokenize a string, using a/multiple delimiters
 * 
 * @param str 
 * @param delim 
 * @return std::vector<std::string> 
 */
std::vector<std::string> tokenize(const std::string& str, const std::string& delim);

}  // namespace utils
}  // namespace param
}  // namespace cnr

#endif  /* CNR_PARAM_INCLUDE_CNR_PARAM_UTILS_STRING */
