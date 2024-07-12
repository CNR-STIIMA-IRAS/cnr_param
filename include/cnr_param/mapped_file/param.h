#ifndef CNR_PARAM__INCLUDE__CNR_PARAM__MAPPED_FILE__PARAM__H
#define CNR_PARAM__INCLUDE__CNR_PARAM__MAPPED_FILE__PARAM__H

#include <string>
#include <Eigen/Core>
#include <boost/filesystem.hpp>
#include <yaml-cpp/exceptions.h>
#include <yaml-cpp/yaml.h>

#if !defined(UNUSED)
#define UNUSED(expr) do { (void)(expr); } while (0)
#endif

namespace cnr 
{
namespace param
{
namespace mapped_file
{

/**
 * @brief 
 * 
 * @param key 
 * @param what 
 * @return true 
 * @return false 
 */
bool has(const std::string& key, std::string& what);

/**
 * @brief 
 * 
 * @tparam T 
 * @param key 
 * @param ret 
 * @param what 
 * @param default_val 
 * @return true 
 * @return false 
 */
template<typename T>
bool get(const std::string& key, T& ret, std::string& what, const bool& implicit_cast_if_possible = true);

/**
 * @brief 
 * 
 * @tparam T 
 * @param key 
 * @param ret 
 * @param what 
 * @return true 
 * @return false 
 */
template<typename T>
bool set(const std::string& key, const T& ret, std::string& what);

// =============================================================================== //
//                                                                                 //
//  UTILITIES                                                                      //
//                                                                                 //
//                                                                                 //
// =============================================================================== //
bool absolutepath(const std::string& key, const bool check_if_exist, boost::filesystem::path& ap, std::string& what);
bool recover(const std::string& key, YAML::Node& node, std::string& what);
bool is_sequence(const std::string& key);


}  // namespace mapped_file
}  // namespace param
}  // namespace cnr

#include <cnr_param/mapped_file/impl/param.hpp>

#endif  /* CNR_PARAM__INCLUDE__CNR_PARAM__MAPPED_FILE__PARAM__H */

