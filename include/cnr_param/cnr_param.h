#ifndef CNR_PARAM__INCLUDE__CNR_PARAM__CNR_PARAM_H
#define CNR_PARAM__INCLUDE__CNR_PARAM__CNR_PARAM_H

#include <vector>
#include <string>

#include "config.h"

#if ROS1_MODULE==1
#include <cnr_param/ros1/param.h>
#endif

#if ROS2_MODULE==1
#include <cnr_param/ros2/param.h>
#endif

#if MAPPED_FILE_MODULE==1
#include <cnr_param/mapped_file/param.h>
#endif

namespace cnr
{
namespace param
{
enum class ModulesID : uint8_t
{
  ROS1 = 0b001, ROS2=0b010, MAPPED_FILE=0b100
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
bool has(const std::string& key, std::string& what, const std::vector<ModulesID>& module = {ModulesID::ROS2, ModulesID::MAPPED_FILE});

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
bool get(const std::string& key, T& value, std::string& what, const T& default_val, const std::vector<ModulesID>& module = {ModulesID::ROS2, ModulesID::MAPPED_FILE});

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
bool get(const std::string& key, T& value, std::string& what, const std::vector<ModulesID>& module = {ModulesID::ROS2, ModulesID::MAPPED_FILE});

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
bool set(const std::string& key, const T& value, std::string& what, const std::vector<ModulesID>& module = {ModulesID::ROS2, ModulesID::MAPPED_FILE});


}  // namespace param
}  // namespace cnr



// ========================= IMPL 

namespace cnr
{
namespace param
{



const bool& is_ros1_avaliable();
const bool& is_ros2_avaliable();
const bool& is_mapped_file_avaliable();


#if !defined(UNUSED)
#define UNUSED(expr) do { (void)(expr); } while (0)
#endif

inline bool has(const std::string& key, std::string& what, const std::vector<ModulesID>& module)
{
  UNUSED(key);
  UNUSED(what);
  bool ret = false;
  for(const auto & m : module)
  {
    switch(m)
    {
      case ModulesID::ROS1:
        #if ROS1_MODULE==1
        {
          ret = cnr::param::ros1::has(key, what);
        }
        #else
        {
          what = "ROS1 module not available";
          ret = false;
        }
        #endif
        break;
      case ModulesID::ROS2:
        #if ROS2_MODULE==1
        {
          ret = cnr::param::ros2::has(key, what);
        }
        #else
        {
          what = "ROS2 module not available";
          ret = false;
        }
        #endif
        break;
      case ModulesID::MAPPED_FILE:
        #if MAPPED_FILE_MODULE==1
        {
          ret = cnr::param::mapped_file::has(key, what);
        }
        #else
        {
          what = "MAPPED FILE module not available";
          ret = false;
        }
        #endif
        break;
    }
    if(ret)
    {
      break;
    }
  }
  return ret;
}

template<typename T>
inline bool get(const std::string& key, T& value, std::string& what, const T& default_val, const std::vector<ModulesID>& module)
{
  UNUSED(key);
  UNUSED(what);
  UNUSED(value);
  UNUSED(default_val);
  bool ret = false;
  for(const auto & m : module)
  {
    switch(m)
    {
      case ModulesID::ROS1:
        #if ROS1_MODULE==1
        {
          ret = cnr::param::ros1::get<T>(key, value, what, default_val);
        }
        #else
        {
          what = "ROS1 module not available";
          ret = false;
        }
        #endif
        break;
      case ModulesID::ROS2:
        #if ROS2_MODULE==1
        {
          ret = cnr::param::ros2::get<T>(key, value, what, default_val);
        }
        #else
        {
          what = "ROS2 module not available";
          ret = false;
        }
        #endif
        break;
      case ModulesID::MAPPED_FILE:
        #if MAPPED_FILE_MODULE==1
        {
          ret = cnr::param::mapped_file::get<T>(key, value, what, default_val);
        }
        #else
        {
          what = "MAPPED FILE MODULE not available";
          ret = false;
        }
        #endif
        break;
    }
    if(ret)
    {
      break;
    }
  }
  return ret;
}




template<typename T>
inline bool get(const std::string& key, T& value, std::string& what, const std::vector<ModulesID>& module)
{
  UNUSED(key);
  UNUSED(what);
  UNUSED(value);
  bool ret = false;
  for(const auto & m : module)
  {
    switch(m)
    {
      case ModulesID::ROS1:
        #if ROS1_MODULE==1
        {
          ret = cnr::param::ros1::get<T>(key, value, what);
        }
        #else
        {
          what = "ROS1 module not available";
          ret = false;
        }
        #endif
        break;
      case ModulesID::ROS2:
        #if ROS2_MODULE==1
        {
          ret = cnr::param::ros2::get<T>(key, value, what);
        }
        #else
        {
          what = "ROS2 module not available";
          ret = false;
        }
        #endif
        break;
      case ModulesID::MAPPED_FILE:
        #if MAPPED_FILE_MODULE==1
        {
          ret = cnr::param::mapped_file::get<T>(key, value, what);
        }
        #else
        {
          what = "MAPPED FILE MODULE not available";
          ret = false;
        }
        #endif
        break;
    }
    if(ret)
    {
      break;
    }
  }
  return ret;
}



template<typename T>
inline bool set(const std::string& key, const T& value, std::string& what, const std::vector<ModulesID>& module)
{
  UNUSED(key);
  UNUSED(what);
  UNUSED(value);
  bool ret = false;
  for(const auto & m : module)
  {
    switch(m)
    {
      case ModulesID::ROS1:
        #if ROS1_MODULE==1
        {
          ret = cnr::param::ros1::set<T>(key, value, what);
        }
        #else
        {
          what = "ROS1 module not available";
          ret = false;
        }
        #endif
        break;
      case ModulesID::ROS2:
        #if ROS2_MODULE==1
        {
          ret = cnr::param::ros2::set<T>(key, value, what);
        }
        #else
        {
          what = "ROS2 module not available";
          ret = false;
        }
        #endif
        break;
      case ModulesID::MAPPED_FILE:
        #if MAPPED_FILE_MODULE==1
        {
          ret = cnr::param::mapped_file::set<T>(key, value, what);
        }
        #else
        {
          what = "MAPPED FILE MODULE not available";
          ret = false;
        }
        #endif
        break;
    }
    if(ret)
    {
      break;
    }
  }
  return ret;
}


}
}



#endif  /* CNR_PARAM__INCLUDE__CNR_PARAM__CNR_PARAM_H */
