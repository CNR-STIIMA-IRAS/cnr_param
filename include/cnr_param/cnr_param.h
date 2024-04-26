#ifndef SRC_CNR_PARAM_INCLUDE_CNR_PARAM_CNR_PARAM
#define SRC_CNR_PARAM_INCLUDE_CNR_PARAM_CNR_PARAM

#include <string>

namespace cnr
{
namespace param
{

enum class ModulesID : uint8_t
{
  ROS1 = 0b001, RO2=0b010, MAPPED_FILE=0b100
};

//======================================================================================================================
//=== USER FUNCTIONS ===================================================================================================/**
/**
 * @brief 
 * 
 * @param[in] key to find (full path)
 * @param[out] what: a message with the error (return false) or a warning (return true)
 * @return true if ok, or if default value has been superimposed, false otherwise.
 */
bool has(const std::string& key, std::string& what);

/**
 * @brief get the param, and return true if found and ok. Store the error(s) in 'what'. If a default value is present,
 * it superimposes the defaul values, and it return true, but it stores a warning in 'what'. Typical error is the
 * mismatch of types between the actual parameter and the required type
 *
 * @param[in] key to find (full path)
 * @param[out] ret the value of the element
 * @param[out] what: a message with the error (return false) or a warning (return true)
 * @param[in] default_val: it is superimposed if the value is not in rosparam value. A warning is stored in 'what'
 * @return true if ok, or if default value has been superimposed, false otherwise.
 */
template<typename T>
bool get(const std::string& key, T& ret, std::string& what, const T& default_val);

/**
 * @brief get the param, and return true if found and ok. Store the error(s) in 'what'. If a default value is present,
 * it superimposes the defaul values, and it return true, but it stores a warning in 'what'. Typical error is the
 * mismatch of types between the actual parameter and the required type
 *
 * @param[in] key to find (full path)
 * @param[out] ret the value of the element
 * @param[out] what: a message with the error (return false) or a warning (return true)
 * @return true if ok, or if default value has been superimposed, false otherwise.
 */
template<typename T>
bool get(const std::string& key, T& ret, std::string& what);

/**
* @brief set the param, and return true if found and ok. Store the error(s) in 'what'. Typical error is the
* mismatch of types between the actual parameter and the required type
*
* @param[in] key: full path
* @param[in] val: element to be stored
* @param[out] what: a message with the error (return false) or a warning (return true)
* @return true if ok, or if default value has been superimposed, false otherwise.
*/
template<typename T>
bool set(const std::string& key, const T& ret, std::string& what);


}  // namespace param
}  // namespace cnr

// #include <cnr_param/impl/cnr_param_impl.h>


#endif  /* SRC_CNR_PARAM_INCLUDE_CNR_PARAM_CNR_PARAM */
