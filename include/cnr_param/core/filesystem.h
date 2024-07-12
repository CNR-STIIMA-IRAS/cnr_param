#ifndef CNR_PARAM_INCLUDE_CNR_PARAM_UTILS_FILESYSTEM
#define CNR_PARAM_INCLUDE_CNR_PARAM_UTILS_FILESYSTEM

#include <string>
#include <boost/filesystem.hpp>

namespace cnr
{
namespace param
{
namespace core
{

/**
 * @brief check if the file path is OK
 * 
 * @param fp 
 * @param what 
 * @return true 
 * @return false 
 */
bool checkfilepath(const boost::filesystem::path& fp, std::string& what);

/**
 * @brief 
 * 
 * @param fn 
 * @param out 
 * @param what 
 * @return true 
 * @return false 
 */
bool filepath(const std::string& fn, boost::filesystem::path& out, std::string& what);

/**
 * @brief 
 * 
 * @param fd 
 * @param out 
 * @param what 
 * @return true 
 * @return false 
 */
bool dirpath(const std::string& fd, boost::filesystem::path& out, std::string& what);

}  // namespace core
}  // namespace param
}  // namespace cnr 


#endif  /* CNR_PARAM_INCLUDE_CNR_PARAM_UTILS_FILESYSTEM */
