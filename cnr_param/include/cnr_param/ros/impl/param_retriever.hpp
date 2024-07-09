#ifndef CNR_PARAM__INCLUDE__CNR_PARAM__ROS__IMPL__PARAM_RETRIEVER__HPP
#define CNR_PARAM__INCLUDE__CNR_PARAM__ROS__IMPL__PARAM_RETRIEVER__HPP

#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include <ros/node_handle.h>
#include <ros/master.h>
#include <ros/names.h>
#include <ros/param.h>
#include <xmlrpcpp/XmlRpcValue.h>


#include <cnr_param/core/param_retriever.h>
#include <cnr_param/ros/param_retriever.h>
#include <cnr_param/ros/param_dictionary.h>

namespace cnr
{
namespace param
{
namespace core
{
template <>
inline std::string lintParamKey(const std::shared_ptr<::ros::NodeHandle>&, const std::string& param_key)
{
  std::string ret = param_key;
  std::replace(ret.begin(), ret.end(), '.', '/');
  return ret;
}

template <>
inline bool getNodeNames(const std::shared_ptr<::ros::NodeHandle>&, std::vector<std::string>& names, std::string& what)
{
  if (!::ros::master::getNodes(names))
  {
    what = "Impossible to retrive the node names";
    return false;
  }
  return true;
}

template <>
inline bool resolveParamName(const std::shared_ptr<::ros::NodeHandle>& node, const std::string& ns,
                             std::string& node_name, std::string& resolved_key, std::string& what)
{
  try
  {
    std::string _nn = node->getNamespace();
    auto _ns = lintParamKey(node, ns);
    resolved_key = ::ros::names::resolve(_ns);
    if (resolved_key.find(_nn) == 0)
    {
      node_name = _nn;
      return true;
    }

    std::vector<std::string> _nnn;
    if (!getNodeNames<::ros::NodeHandle>(node, _nnn, what))
    {
      return false;
    }
    for (const auto& nn : _nnn)
    {
      if (resolved_key.find(nn) == 0)
      {
        node_name = nn;
        return true;
      }
    }
    node_name = g_absolute_param_resolution;
    resolved_key = _ns;
  }
  catch (::ros::InvalidNameException& e)
  {
    what = "Impossible to retrive the name '" + ns + "'. Error: " + std::string(e.what());
    return false;
  }
  catch (std::exception& e)
  {
    what = "Impossible to retrive the name '" + ns + "'. Error: " + std::string(e.what());
    return false;
  }
  catch (...)
  {
    what = "Impossible to retrive the name '" + ns + "'. Error: Unhandled expcetion";
    return false;
  }
  return true;
}

template <>
inline bool resolveNodeName<::ros::NodeHandle>(const std::shared_ptr<::ros::NodeHandle>& n, std::string& resolved_name,
                                               std::string& what)
{
  if (!n)
  {
    what = "The node handle is not valid";
    return false;
  }

  resolved_name = n->getNamespace();
  return true;
}

/**
 * @brief Get the parameters for a vector of keys that have already been checked
 *
 * @param node_name
 * @param keys
 * @param parameters
 * @param what
 * @return true
 * @return false
 */
template <>
inline bool ParamRetriever<::ros::NodeHandle, XmlRpc::XmlRpcValue>::get_existent_parameters(
    const std::string& node_name, const std::vector<std::string>& keys, std::vector<XmlRpc::XmlRpcValue>& parameters,
    std::string& what)
{
  what.clear();
  for (const auto& k : keys)
  {
    XmlRpc::XmlRpcValue p;
    if (!::ros::param::get("/" + node_name + "/" + k, p))
    {
      what += "Impossible to get the parameter '" + k + "'. ";
      continue;
    }
    parameters.push_back(p);
  }
  return keys.size() == parameters.size();
}

template <>
inline bool ParamRetriever<::ros::NodeHandle, XmlRpc::XmlRpcValue>::retrieve_parameters(
    const std::string& resolved_node_name, const std::string& resolved_key, std::string& what, bool updated)
{
  std::size_t ll = __LINE__;
  try
  {
    ll = __LINE__;
    ParamDictionary<XmlRpc::XmlRpcValue>& _node_dict = node_params(resolved_node_name);
    bool value_found = false;
    ll = __LINE__;
    if (_node_dict.initialized())
    {
      ll = __LINE__;
      value_found = (_node_dict.name() == resolved_key) ? true : bool(find(_node_dict, resolved_key, "/", what));
    }
    ll = __LINE__;
    if (updated || !value_found)
    {
      ll = __LINE__;
      try
      {
        what = "Error in retriving the parameter '" + resolved_key + "' from the rosparam server. ";
        what +=
            (resolved_node_name != g_absolute_param_resolution) ? "The node name is '" + resolved_node_name + "'" : "";
        XmlRpc::XmlRpcValue p;
        std::string _ns = (resolved_node_name == g_absolute_param_resolution) ? resolved_key :
                                                                                resolved_node_name + "/" + resolved_key;
        value_found = ::ros::param::get(_ns, p);
        if (value_found)
        {
          what = "";
          insert_dict(node_params(resolved_node_name), resolved_key, p);
          return true;
        }
        what += "Value not found";
        return false;
      }
      catch (::ros::InvalidNameException& e)
      {
        what += "Error message: " + std::string(e.what());
      }
      catch (std::exception& e)
      {
        what += "Error message: " + std::string(e.what());
      }
      catch (...)
      {
        what += "Unhandled Exception";
      }
    }
    return value_found;
  }
  catch (std::exception& e)
  {
    what = std::string(e.what()) + " at line " + std::to_string(ll) + " in " + __FILE__;
    return false;
  }
  catch (...)
  {
    what = "Unknown exception at line " + std::to_string(ll) + " in " + __FILE__;
    return false;
  }
}

}  // namespace core
}  // namespace param
}  // namespace cnr

#endif  // CNR_PARAM__INCLUDE__CNR_PARAM__ROS__IMPL__PARAM_RETRIEVER__HPP