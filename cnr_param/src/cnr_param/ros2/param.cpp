#include <cnr_param/ros2/impl/param.hpp>
#include <cnr_param/ros2/param_retriever.h>

namespace cnr 
{
namespace param
{
namespace ros2
{

std::shared_ptr<rclcpp::Node>& background_node()
{
  static std::shared_ptr<rclcpp::Node> __node = nullptr;
  return __node;
}

std::shared_ptr<cnr::param::ros2::ParamRetriever>& param_retriever()
{
  static std::shared_ptr<cnr::param::ros2::ParamRetriever> __pr = nullptr;
  return __pr;
}


void CNR_PARAM_INIT_RO2_MODULE(std::shared_ptr<rclcpp::Node>& node)
{
  std::cout << "Init the global varible inside the library" << std::endl;
  background_node() = node;
  param_retriever().reset(new cnr::param::ros2::ParamRetriever (background_node()) );
}

void CNR_PARAM_CLEANUP_RO2_MODULE()
{
  background_node() = nullptr;
  param_retriever().reset( );
}

bool has(const std::string& key, std::string& what)
{
  std::string resolved_node_name, resolved_key;
  if(!param_retriever())
  {
    what = "The param retriever is null! Did you call the CNR_PARAM_INIT_RO2_MODULE()?";
    return false;
  }
  if(!param_retriever()->resolve_names(key, resolved_node_name, resolved_key, what))
  {
    return false;
  }

  ParamDictionary d(resolved_node_name);
  return param_retriever()->get_parameter(resolved_node_name, resolved_key, d, what);

}

}  // namespace ros2
}  // namespace param
}  // namespace cnr