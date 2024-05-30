#ifndef CNR_PARAM__INCLUDE__CNR_PARAM__ROS__PARAM_RETRIEVER__H
#define CNR_PARAM__INCLUDE__CNR_PARAM__ROS__PARAM_RETRIEVER__H


#include <Eigen/Core>
#include "XmlRpcValue.h"
#include <yaml-cpp/yaml.h>

#include <ros/node_handle.h>
#include <xmlrpcpp/XmlRpc.h>

#include <cnr_param/core/type_traits.h>
#include <cnr_param/core/param_retriever.h>
#include <cnr_param/ros/param_dictionary.h>

namespace cnr
{
namespace param
{
namespace ros
{

using ParamRetriever = cnr::param::core::ParamRetriever<::ros::NodeHandle, XmlRpc::XmlRpcValue>;

}  // namespace ros
}  // namespace param
}  // namespace cnr


#include <cnr_param/ros/impl/param_retriever.hpp>

#endif  // CNR_PARAM__INCLUDE__CNR_PARAM__ROS__PARAM_RETRIEVER__H