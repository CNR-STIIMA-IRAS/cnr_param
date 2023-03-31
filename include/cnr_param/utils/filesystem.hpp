#ifndef CNR_PARAM_INCLUDE_CNR_PARAM_UTILS_FILESYSTEM
#define CNR_PARAM_INCLUDE_CNR_PARAM_UTILS_FILESYSTEM

#include <string>
#include <vector>

#if defined(__cpp_lib_filesystem)
    #include <filesystem>
    namespace fs = std::filesystem;
#else
#if defined(__cpp_lib_experimental_filesystem)
    #define _SILENCE_EXPERIMENTAL_FILESYSTEM_DEPRECATION_WARNING 1;
    #include <experimental/filesystem>
    namespace fs = std::experimental::filesystem;
#else
  #include <boost/filesystem.hpp>
  namespace fs = boost::filesystem;
#endif
#endif

namespace cnr 
{
namespace param
{
namespace utils
{
/**
 * @brief check if the file path is OK
 * 
 * @param fp 
 * @param what 
 * @return true 
 * @return false 
 */
bool checkfilepath(const fs::path& fp, std::string& what);

/**
 * @brief 
 * 
 * @param fn 
 * @param out 
 * @param what 
 * @return true 
 * @return false 
 */
bool filepath(const std::string& fn, fs::path& out, std::string& what);
/**
 * @brief 
 * 
 * @param fd 
 * @param out 
 * @param what 
 * @return true 
 * @return false 
 */
bool dirpath(const std::string& fd, fs::path& out, std::string& what);

}  // namespace utils
}  // namespace param
}  // namespace cnr 


#endif  /* CNR_PARAM_INCLUDE_CNR_PARAM_UTILS_FILESYSTEM */
