#ifndef INCLUDE_CNR_PARAM_CNR_PARAM
#define INCLUDE_CNR_PARAM_CNR_PARAM

#include <vector>
#include <string>

#include <sstream>
#include <vector>
#include <string>
#include <boost/array.hpp>
#include <boost/type_index.hpp>
#include <bitset>

#include <Eigen/Core>

#include "cnr_param/visibility_control.h"



#if defined(ROS2_AVAILABLE)
  #error "NOT YET IMPLEMENTED"  
#elif defined(ROS1_AVAILABLE)
  #error "NOT YET IMPLEMENTED"  
#else
  #include <yaml-cpp/yaml.h>
  namespace cnr
  {
  namespace param
  {
    using node_t = YAML::Node;
  }
  }
#endif


namespace cnr
{
namespace param
{

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

/**
 * @brief 
 * 
 * @param node 
 * @param key 
 * @return true 
 * @return false 
 */
bool has(const cnr::param::node_t& node, const std::string& key);

/**
 * @brief 
 * 
 * @param node 
 * @return true 
 * @return false 
 */
bool is_map(const cnr::param::node_t& node);

/**
 * @brief 
 * 
 * @param key 
 * @return true 
 * @return false 
 */
bool is_sequence(const std::string& key);

/**
 * @brief 
 * 
 * @param node 
 * @return true 
 * @return false 
 */
bool is_sequence(const cnr::param::node_t& node);

/**
 * @brief 
 * 
 * @param node 
 * @return std::size_t 
 */
std::size_t size(const cnr::param::node_t& node);

/**
 * @brief Get the leaf object
 * 
 * @param node 
 * @param key 
 * @param leaf 
 * @param what 
 * @return true 
 * @return false 
 */
bool get_leaf(const node_t& node, const std::string& key, node_t& leaf, std::string& what);

/**
 * @brief 
 * 
 * @tparam T 
 * @param node 
 * @param i 
 * @param element 
 * @param what 
 * @return true 
 * @return false 
 */
template<typename T>
bool at(const cnr::param::node_t& node, std::size_t i, T& element, std::string& what);


/**
 * @brief 
 * 
 * @tparam T 
 * @param node 
 * @param key
 * @param element 
 * @param what 
 * @return true 
 * @return false 
 */
template<typename T>
bool at(const cnr::param::node_t& node, const std::string& key, T& element, std::string& what);
//======================================================================================================================


//======================================================================================================================
//== SERVICE FUNCTION
//======================================================================================================================
/**
 * @brief Get the templated object from the node. 
 * 
 * @tparam T 
 * @param node 
 * @return T 
 */
template<typename T>
T extract(const node_t& node, const std::string& key ="", const std::string& error_heading_msgs ="");
//======================================================================================================================

//======================================================================================================================
//== SCALAR FUNCTION
//======================================================================================================================
/**
 * @brief Get the scalar object: YOU SHOULD SPECILIZE THE TEMPLATE IF YOU WANT AN AUTOMATIC LOADING 
 * OF YOUR SPECIFIC OBJECT BEYOND THE ONES ALREADY IMPLEMENTED
 * 
 * @tparam T 
 * @param node 
 * @param ret 
 * @return true 
 * @return false 
 */
template<typename T>
bool get_scalar(const node_t& node, T& ret, std::stringstream& what); // TO REIMPLEMENT ITS OWN COMPLEX TYPE

template<>
bool get_scalar(const node_t& node, double& ret, std::stringstream& what);

template<>
bool get_scalar(const node_t& node, int& ret, std::stringstream& what);

template<>
bool get_scalar(const node_t& node, bool& ret, std::stringstream& what);

template<>
bool get_scalar(const node_t& node, std::string& ret, std::stringstream& what);

//======================================================================================================================
//== SEQUENCE FUNCTION
//======================================================================================================================
/**
 * @brief Get the sequence object:  YOU SHOULD SPECILIZE THE TEMPLATE IF YOU WANT AN AUTOMATIC LOADING 
 * OF YOUR SPECIFIC OBJECT BEYOND THE ONES ALREADY IMPLEMENTED
 * 
 * @tparam T 
 * @param node 
 * @param ret 
 * @return true 
 * @return false 
 */
template<typename T>
bool get_sequence(const node_t& node, T& ret, std::stringstream& what);

template<typename T>
bool get_sequence(const node_t& node, std::vector<T>& ret, std::stringstream& what);

template<typename T>
bool get_sequence(const node_t& node, std::vector< std::vector<T> >& ret, std::stringstream& what);

template<typename T, std::size_t  N>
bool get_sequence(const node_t& node, std::array<T,N>& ret, std::stringstream& what);

template<typename T, std::size_t N, std::size_t M>
bool get_sequence(const node_t& node, std::array<std::array<T,M>,N>& ret, std::stringstream& what);

/**
 * @brief 
 * 
 * @tparam Derived 
 * @param key 
 * @return Eigen::MatrixBase<Derived> 
 */
template<typename Derived>
bool get_sequence(const node_t& node, Eigen::MatrixBase<Derived>& ret, std::stringstream& what);

//======================================================================================================================
//== MAP FUNCTION
//======================================================================================================================
/**
 * @brief Get the map object:  YOU SHOULD SPECILIZE THE TEMPLATE IF YOU WANT AN AUTOMATIC LOADING 
 * OF YOUR SPECIFIC OBJECT BEYOND THE ONES ALREADY IMPLEMENTED
 * 
 * @tparam T 
 * @param node 
 * @param ret 
 * @return true 
 * @return false 
 */
template<typename T>
bool get_map(const node_t& node, T& ret, std::stringstream& what);

}  // namespace param
}  // namespace cnr

#include <cnr_param/impl/cnr_param_impl.hpp>

#endif  /* INCLUDE_CNR_PARAM_CNR_PARAM */
