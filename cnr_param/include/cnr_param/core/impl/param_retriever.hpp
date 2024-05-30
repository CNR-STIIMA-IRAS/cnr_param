#ifndef CNR_PARAM__INCLUDE__CNR_PARAM__CORE__IMPL__PARAM_RETRIEVER__HPP
#define CNR_PARAM__INCLUDE__CNR_PARAM__CORE__IMPL__PARAM_RETRIEVER__HPP

#include <functional>
#include <iostream>
#include <stdexcept>
#include <boost/algorithm/string.hpp>
#include <cnr_param/core/string.h>
#include <cnr_param/core/param_retriever.h>


namespace cnr
{
namespace param
{
namespace core
{

static const char* __absolute_param_resolution = "/absolute_root_no_node_name";
static std::string g_absolute_param_resolution =
    std::string(__absolute_param_resolution) + std::to_string(std::hash<std::string>{}(__absolute_param_resolution));

//========================================================
//  ParamRetriever
//
//
//========================================================
template <typename N, typename P>
inline ParamRetriever<N, P>::ParamRetriever(std::shared_ptr<N> node, const std::string& parameter_separator_string) : node_(node), parameter_separator_string_(parameter_separator_string)
{
}

template <typename N, typename P>
inline bool ParamRetriever<N, P>::resolve_names(const std::string& ns, std::string& resolved_node_name,
                                                std::string& resolved_ns, std::string& what)
{
  if (ns.empty())
  {
    what = "The key '" + ns + "' is invalid.";
    return false;
  }

  if (node_ == nullptr)
  {
    what = "Do you have called the CNR_PARAM_INIT_NODE macro?";
    return false;
  }

  std::vector<std::string> node_names;
  if (!getNodeNames(node_, node_names, what))
  {
    return false;
  }

  if(!resolveParamName(node_, ns, resolved_node_name,resolved_ns , what))
  {
    return false;
  }

  return true;
}

template <typename N, typename P>
inline ParamDictionary<P>& ParamRetriever<N, P>::node_params(const std::string& node_name)
{
  auto it = std::find_if(node_params_.begin(), node_params_.end(),
                         [&node_name](const ParamDictionary<P>& v) { return v.name() == node_name; });
  if (it == node_params_.end())
  {
    node_params_.push_back(ParamDictionary<P>(node_name));
    return node_params_.back();
  }
  return *it;
}

template <typename N, typename P>
inline const ParamDictionary<P>& ParamRetriever<N, P>::node_params(const std::string& node_name) const
{
  auto it = std::find_if(node_params_.begin(), node_params_.end(),
                         [&node_name](const ParamDictionary<P>& v) { return v.name() == node_name; });
  if (it == node_params_.end())
  {
    throw std::runtime_error("The node '" + node_name + "' is not in the list of the node params");
  }
  return *it;
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
template <typename N, typename P>
inline bool ParamRetriever<N, P>::get_existent_parameters(const std::string&, const std::vector<std::string>&, std::vector<P>& , std::string&)
{
  std::string err = __PRETTY_FUNCTION__ + std::string(":") + std::to_string(__LINE__) + ": " +
                    "To be specialized to your specific case";
  throw std::runtime_error(err.c_str());
  return false;
}

template <typename N, typename P>
inline void ParamRetriever<N, P>::insert_dict(ParamDictionary<P>& dictionary, const std::string& key, const P& value)
{
  std::vector<std::string> split;
  boost::split(split, key, boost::is_any_of(parameter_separator_string_), boost::token_compress_on);
  if (split.size() > 1)
  {
    std::string relative_key;
    for (size_t i = 1; i < split.size(); i++)
    {
      relative_key += split.at(i);
      if (i < split.size() - 1)
      {
        relative_key += std::string(parameter_separator_string_);
      }
    }

    insert_dict(dictionary[split.front()], relative_key, value);
  }
  else
  {
    dictionary[key] = value;
  }
}

/**
 * @brief
 *
 * @param node_name
 * @param resolved_key is the name of the param, eventually nested. The namespace separator is '.' as in ROS 2
 * standard.
 * @param dictionary
 * @param updated
 * @return true
 * @return false
 */
template <typename N, typename P>
inline bool ParamRetriever<N, P>::retrieve_parameters(const std::string&, const std::string&, std::string&, bool)
{
  std::string err = __PRETTY_FUNCTION__ + std::string(":") + std::to_string(__LINE__) + ": " +
                    "To be specialized to your specific case";
  throw std::runtime_error(err.c_str());
  return false;
}

template <typename N, typename P>
inline bool ParamRetriever<N, P>:: list_parameters(const std::string&, const std::vector<std::string>&,
                       std::vector<std::string>&, std::string&)
{
  std::string err = __PRETTY_FUNCTION__ + std::string(":") + std::to_string(__LINE__) + ": " +
                    "To be specialized to your specific case";
  throw std::runtime_error(err.c_str());
  return false;
}

template <typename N, typename P>
inline bool ParamRetriever<N, P>::get_parameter(const std::string& resolved_node_name, const std::string& resolved_key,
                                                ParamDictionary<P>& param, std::string& what, bool updated)
{
  std::size_t ll = __LINE__;
  try
  {
    ll = __LINE__;
    if (!retrieve_parameters(resolved_node_name, resolved_key, what, updated))
    {
      return false;
    }

    ll = __LINE__;
    auto it = find(node_params(resolved_node_name), resolved_key, parameter_separator_string_, what);
    if (!it)
    {
      what = "Extraction failed (resolved node name: '" + resolved_node_name + "', resolved key: '" + resolved_key +
            "'): " + what;
      return false;
    }
    ll = __LINE__;
    param = *it;
    return true;
  }
  catch(std::exception& e)
  {
    what = std::string(e.what()) + " at line " + std::to_string(ll) + " in " + __FILE__;
    return false;
  }
  catch(...)
  {
    what = "Unknown exception at line " + std::to_string(ll) + " in " + __FILE__;
    return false;
  }
}


template <typename N>
inline bool getNodeNames(const std::shared_ptr<N>&, std::vector<std::string>&, std::string&)
{
  std::string err = __PRETTY_FUNCTION__ + std::string(":") + std::to_string(__LINE__) + ": " +
                    "To be specialized to your specific case";
  throw std::runtime_error(err.c_str());
  return false;
}

template <typename N>
inline bool resolveParamName(const std::shared_ptr<N>&, const std::string&, std::string&, std::string&, std::string&)
{
  std::string err = __PRETTY_FUNCTION__ + std::string(":") + std::to_string(__LINE__) + ": " +
                    "To be specialized to your specific case";
  throw std::runtime_error(err.c_str());
  return false;
}

template <typename N>
bool resolveNodeName(const std::shared_ptr<N>&, std::string&, std::string&)
{
  std::string err = __PRETTY_FUNCTION__ + std::string(":") + std::to_string(__LINE__) + ": " +
                    "To be specialized to your specific case";
  throw std::runtime_error(err.c_str());
  return false;
}

template <typename N>
inline std::string lintParamKey(const std::shared_ptr<N>&, const std::string&)
{
  std::string err = __PRETTY_FUNCTION__ + std::string(":") + std::to_string(__LINE__) + ": " +
                    "To be specialized to your specific case";
  throw std::runtime_error(err.c_str());
  return std::string{};
}

}  // namespace core
}  // namespace param
}  // namespace cnr

#endif  // CNR_PARAM__INCLUDE__CNR_PARAM__CORE__IMPL__PARAM_RETRIEVER__HPP