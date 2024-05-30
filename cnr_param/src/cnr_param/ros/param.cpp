
#include <string>
#include <memory>
#include <ros/param.h>
#include <XmlRpcValue.h>
#include <ros/node_handle.h>
#include <cnr_param/ros/param.h>
#include <cnr_param/core/param_retriever.h>


namespace cnr 
{
namespace param
{
namespace ros
{

std::shared_ptr<::ros::NodeHandle>& background_node(std::shared_ptr<::ros::NodeHandle> node)
{
  static std::shared_ptr<::ros::NodeHandle> __node(node);
  if(node)
  {
    __node = node;
  }
  return __node;
}

std::shared_ptr<ParamRetriever>& param_retriever()
{
  static std::shared_ptr<ParamRetriever> __pr = nullptr;
  return __pr;
}

void CNR_PARAM_INIT_ROS_MODULE(std::shared_ptr<::ros::NodeHandle> node)
{
  background_node() = node;
  param_retriever().reset(new ParamRetriever(background_node(), "/") );
}

void CNR_PARAM_CLEANUP_ROS_MODULE()
{
  background_node() = nullptr;
  param_retriever().reset( );
}

bool has(const std::string& key, std::string& what)
{
  if(::ros::param::has(key))
  {
    what = "The ros param server has not any param named '" + key +"'";
    return false;
  }
  return true;
}

}  // namespace ros2
}  // namespace param
}  // namespace cnr