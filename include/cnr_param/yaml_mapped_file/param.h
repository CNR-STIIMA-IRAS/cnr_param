/*
 *  Software License Agreement (New BSD License)
 *
 *  Copyright 2020 National Council of Research of Italy (CNR)
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef CNR_PARAM__INCLUDE__CNR_PARAM__YAML_MAPPED_FILE__PARAM__H
#define CNR_PARAM__INCLUDE__CNR_PARAM__YAML_MAPPED_FILE__PARAM__H

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
namespace yaml
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
bool get(const std::string& key, T& ret, std::string& what);

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
bool get(const std::string& key, T& ret, std::string& what, const T& default_val);




// ================================================================================== //
//                                                                                    //
//                                                                                    //
//                                                                                    //
//  NODE ACCESSORS                                                                    //
//                                                                                    //
// ================================================================================== //

/**
 * @brief 
 * 
 * @param node 
 * @param key 
 * @return true 
 * @return false 
 */
bool has(const YAML::Node& node, const std::string& key);


/**
 * @brief 
 * 
 * @param node 
 * @return true 
 * @return false 
 */
bool is_sequence(const YAML::Node& node);

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
bool is_map(const YAML::Node& node);

/**
 * @brief 
 * 
 * @param node 
 * @return size_t 
 */
size_t size(const YAML::Node& node);

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
bool get_leaf(const YAML::Node& node, const std::string& key, YAML::Node& leaf, std::string& what);

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
bool at(const YAML::Node& node, std::size_t i, T& element, std::string& what);

/**
 * @brief 
 * 
 * @tparam  
 * @param node 
 * @param i 
 * @param element 
 * @param what 
 * @return true 
 * @return false 
 */
template<>
bool at(const YAML::Node& node, std::size_t i, YAML::Node& element, std::string& what);

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
bool at(const YAML::Node& node, const std::string& key, T& element, std::string& what);

// =============================================================================================


template<typename T>
void insert(YAML::Node& node, const std::string& key, const T& value);

template< typename T >
std::string int_to_hex( T i );

void insert(YAML::Node& node, const std::string& key, const int& value, const std::string& format);

// =============================================================================================
// SCALAR
// =============================================================================================
template<typename T>
bool get_scalar(const YAML::Node& node, T& ret, std::stringstream& what);

// =============================================================================================
// SEQUENCE
// =============================================================================================
template<typename T>
bool get_sequence(const YAML::Node& node, T& ret, std::stringstream& what);

// =============================================================================================
// MAP
// =============================================================================================
template<typename T>
bool get_map(const YAML::Node& node, T& ret, std::stringstream& what);
// =============================================================================== //
//                                                                                 //
//  UTILITIES                                                                      //
//                                                                                 //
//                                                                                 //
// =============================================================================== //
bool absolutepath(const std::string& key, const bool check_if_exist, boost::filesystem::path& ap, std::string& what);



bool recover(const std::string& key, YAML::Node& node, std::string& what);
// =============================================================================== //
//                                                                                 //
//                                                                                 //
//                                                                                 //
//                                                                                 //
// =============================================================================== //


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
T extract(const YAML::Node& node, const std::string& key ="", const std::string& error_heading_msgs ="");


}
}
}

#include <cnr_param/yaml_mapped_file/impl/param.hpp>

#endif  /* CNR_PARAM__INCLUDE__CNR_PARAM__YAML_MAPPED_FILE__PARAM__H */

