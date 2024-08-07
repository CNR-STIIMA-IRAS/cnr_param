#ifndef CNR_PARAM_ROS2_YAML_FORMATTER_H
#define CNR_PARAM_ROS2_YAML_FORMATTER_H

#include <yaml-cpp/node/node.h>
#include <yaml-cpp/yaml.h>

namespace cnr
{
namespace param
{
namespace ros2 
{

bool ros2_yaml_encoder(const YAML::Node& node, YAML::Node& ret, std::string& what);
bool ros2_yaml_decoder(const YAML::Node& node, YAML::Node& ret, std::string& what);

}
}
}

#endif // CNR_PARAM_ROS2_YAML_FORMATTER_H