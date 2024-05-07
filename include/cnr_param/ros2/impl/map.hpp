#ifndef CNR_PARAM__INCLUDE__CNR_PARAM__ROS2__IMPL__MAP__HPP
#define CNR_PARAM__INCLUDE__CNR_PARAM__ROS2__IMPL__MAP__HPP

#include <yaml-cpp/yaml.h>
#include <string>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <cnr_param/core/param.h>

#include <cnr_param/ros2/yaml_formatter.h>
#include <cnr_param/ros2/param_retriever.h>

namespace cnr
{
namespace param
{
namespace ros2
{

/**
 * @brief Get the map object
 * 
 * @tparam T 
 * @param node 
 * @param ret 
 * @param what 
 * @return true 
 * @return false 
 */
template<typename T>
inline bool get_map(const ParamDictionary& param, T& ret, std::string& what)
{
  
    YAML::Node node;
    if(!cnr::param::ros2::to_yaml(param, node, what))
    {
      return false;
    }
    YAML::Node uncrustified_node = cnr::param::ros2::ros2_yaml_decoder(node);
    
    return cnr::param::core::get_map(uncrustified_node, ret, what);
}


}  // namespace ros2
}  // namespace param
}  // namespace cnr

#endif  // CNR_PARAM__INCLUDE__CNR_PARAM__ROS2__IMPL__MAP__HPP