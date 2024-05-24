#ifndef CNR_PARAM__INCLUDE__CNR_PARAM__CORE__IMPL__PARAM_RETRIEVER__HPP
#define CNR_PARAM__INCLUDE__CNR_PARAM__CORE__IMPL__PARAM_RETRIEVER__HPP

#include <functional>
#include <iostream>
#include <stdexcept>
#include "cnr_param/core/type_traits.h"
#include <cnr_param/core/param_retriever.h>

namespace cnr
{
namespace param
{
static const char* __absolute_param_resolution = "/absolute_root_no_node_name";
static std::string g_absolute_param_resolution =
    std::string(__absolute_param_resolution) + std::to_string(std::hash<std::string>{}(__absolute_param_resolution));

/**
 * @brief
 *
 * @tparam T
 */
template <typename T, typename F, typename... Ts>
inline bool get_from_variant(T& ret, const std::variant<F, Ts...>& rhs, const bool& implicit_cast_if_possible,
                             std::string& what)
{
  if (std::holds_alternative<F>(rhs))
  {
    if constexpr (is_variant_member<T, decltype(rhs)>::value)
    {
      return std::get<T>(rhs);
    }
    else
    {
      if (!implicit_cast_if_possible)
      {
        what = "The variant contains an object type '" +
               boost::typeindex::type_id_with_cvr<decltype(F())>().pretty_name() + "', while the requested type is '" +
               boost::typeindex::type_id_with_cvr<decltype(T())>().pretty_name() + "'";
        return false;
      }
      if constexpr (is_forward_implicit_conversion_allowed<c_type_to_param_type<F>::value, T>::value)
      {
        ret = static_cast<T>(std::get<F>(rhs));
        return true;
      }
      else
      {
        if constexpr (sizeof...(Ts) > 0)
        { 
          std::variant<Ts...> _rhs;
          std::visit(overloaded{
              [&_rhs](auto arg){_rhs = arg;}, 
              [](F) {}
          }, rhs);
          return get_from_variant<T,Ts...>(ret, _rhs, implicit_cast_if_possible, what);
        }
        else
        {
          what = "The variant contains an object type '" +
                 boost::typeindex::type_id_with_cvr<decltype(F())>().pretty_name() + "', while the requested type is '" +
                 boost::typeindex::type_id_with_cvr<decltype(T())>().pretty_name() + "'. No conversion is possible";
          return false;
        }
      }
    }
  }

  if constexpr (sizeof...(Ts) > 0)
  {
    std::variant<Ts...> _rhs;
    std::visit(overloaded{ [&_rhs](auto arg) {
                            _rhs = arg;
                          },
                           [](F) {
                           } },
               rhs);
    return get_from_variant(ret, _rhs, implicit_cast_if_possible, what);
  }
}

template <typename A, typename P>
inline A as_generic(const P&)
{
  std::string err = __PRETTY_FUNCTION__ + std::string(":") + std::to_string(__LINE__) + ": " +
                    "To be specialized to your specific case";
  throw std::runtime_error(err.c_str());
  return A();
}

template <typename P>
inline ParamDictionary<P>::ParamDictionary(const std::string& key) : param_({ key, ParamDictionary<P>::EmptyParam() })
{
}

template <typename P>
inline const std::string& ParamDictionary<P>::name() const
{
  return param_.first;
}

template <typename P>
inline const ParamDictionary<P>::AllowedParam& ParamDictionary<P>::value() const
{
  return param_.second;
}

template <typename P>
inline ParamDictionary<P>& ParamDictionary<P>::operator=(const P& value)
{
  param_.second = value;
  return *this;
}

template <typename P>
inline ParamDictionary<P>& ParamDictionary<P>::operator[](const std::string& key)
{
  const std::string& _n = param_.first;
  ParamDictionary<P>::AllowedParam& _p = param_.second;
  if (std::holds_alternative<P>(_p))
  {
    if (key != param_.first)
    {
      throw std::runtime_error((__PRETTY_FUNCTION__ + (":" + std::to_string(__LINE__) + ":") +
                                " The Dictionary store a param under the name '" + _n +
                                "', while the required value name is '" + key + "'")
                                   .c_str());
    }
    return *this;
  }
  else if (std::holds_alternative<typename ParamDictionary<P>::NestedParams>(_p))
  {
    auto& _m = std::get<typename ParamDictionary<P>::NestedParams>(_p);
    if ((_m.find(key) == _m.end()))
    {
      _m.insert({ key, ParamDictionary(key) });
    }
    return _m.at(key);
  }
  else if (std::holds_alternative<ParamDictionary<P>::EmptyParam>(_p))
  {
    if (key != param_.first)
    {
      _p = NestedParams({ { key, ParamDictionary(key) } });
    }
    return std::get<NestedParams>(_p).at(key);
  }
  throw std::runtime_error(
      (__PRETTY_FUNCTION__ + (":" + std::to_string(__LINE__) + ":") + " Weird Error! Debug ....").c_str());
}

template <typename P>
inline const ParamDictionary<P>& ParamDictionary<P>::operator[](const std::string& key) const
{
  const std::string& _n = param_.first;
  const ParamDictionary<P>::AllowedParam& _p = param_.second;
  if (std::holds_alternative<P>(_p))
  {
    if (key != param_.first)
    {
      throw std::runtime_error((__PRETTY_FUNCTION__ + (":" + std::to_string(__LINE__) + ":") +
                                " The Dictionary store a param under the name '" + _n +
                                "', while the required value name is '" + key + "'")
                                   .c_str());
    }
    return *this;
  }
  else if (std::holds_alternative<typename ParamDictionary<P>::NestedParams>(_p))
  {
    auto& _m = std::get<typename ParamDictionary<P>::NestedParams>(_p);
    if ((_m.find(key) == _m.end()))
    {
      throw std::runtime_error((__PRETTY_FUNCTION__ + (":" + std::to_string(__LINE__) + ":") +
                                " The Dictionary store a map of parameters under the name '" + _n +
                                "'. The required '" + key + "' is not in the map")
                                   .c_str());
    }
    return _m.at(key);
  }

  throw std::runtime_error(
      (__PRETTY_FUNCTION__ + (":" + std::to_string(__LINE__) + ":") + " The Dictionary is unitialized").c_str());
}

template <typename P>
inline bool ParamDictionary<P>::initialized() const
{
  return std::holds_alternative<P>(param_.second) ||
         std::holds_alternative<typename ParamDictionary<P>::NestedParams>(param_.second);
}

template <typename P>
inline bool ParamDictionary<P>::is_scalar() const
{
  std::string err = __PRETTY_FUNCTION__ + std::string(":") + std::to_string(__LINE__) + ": " +
                    "To be specialized to your specific case";
  throw std::runtime_error(err.c_str());
  return false;
}

template <typename P>
inline bool ParamDictionary<P>::is_sequence() const
{
  std::string err = __PRETTY_FUNCTION__ + std::string(":") + std::to_string(__LINE__) + ": " +
                    "To be specialized to your specific case";
  throw std::runtime_error(err.c_str());
  return false;
}

template <typename P>
inline bool ParamDictionary<P>::is_map() const
{
  return std::holds_alternative<typename ParamDictionary<P>::NestedParams>(param_.second);
}

//========================================================
//  ParamRetriever
//
//
//========================================================
template <typename N, typename P>
inline ParamRetriever<N, P>::ParamRetriever(std::shared_ptr<N> node) : node_(node)
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

  auto _ns = lintParamKey(node_, ns);

  if (_ns.front() == '/')
  {
    std::vector<std::string> names;
    if (!getNodeNames(node_, names, what))
    {
      return false;
    }

    for (const auto& n : names)
    {
      std::string _n;
      if (!resolveParamName(node_, n, _n, what))
      {
        return false;
      }
      if (_ns.rfind(_n, 0) == 0)
      {
        auto tmp = _ns;
        tmp.erase(0, n.size());
        resolved_ns = lintParamKey(node_, tmp);
        resolved_node_name = n;
        return true;
      }
    }
    // the param is a absolute path, but the node name is not in the list
    resolved_node_name = g_absolute_param_resolution;
    resolved_ns = _ns;
  }
  else  // local
  {
    if (resolveNodeName(node_, resolved_node_name, what))
    {
      return false;
    }
    resolved_ns = lintParamKey(node_, _ns);
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
    std::cout << "Add empy node to the list of the node params" << std::endl;
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

template <typename N, typename P>
inline bool ParamRetriever<N, P>::list_parameters(const std::string&, const std::vector<std::string>&,
                                                  std::vector<std::string>&, std::string&)
{
  std::string err = __PRETTY_FUNCTION__ + std::string(":") + std::to_string(__LINE__) + ": " +
                    "To be specialized to your specific case";
  throw std::runtime_error(err.c_str());
  return false;
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
inline bool ParamRetriever<N, P>::get_existent_parameters(const std::string& node_name,
                                                          const std::vector<std::string>& keys,
                                                          std::vector<P>& parameters, std::string& what)
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
  boost::split(split, key, boost::is_any_of(PARAMETER_SEPARATOR_STRING), boost::token_compress_on);
  if (split.size() > 1)
  {
    std::string relative_key;
    for (size_t i = 1; i < split.size(); i++)
    {
      relative_key += split.at(i);
      if (i < split.size() - 1)
      {
        relative_key += std::string(PARAMETER_SEPARATOR_CHAR);
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
inline bool ParamRetriever<N, P>::get_parameter(const std::string& resolved_node_name, const std::string& resolved_key,
                                                ParamDictionary<P>& param, std::string& what, bool updated)
{
  if (!retrieve_parameters(resolved_node_name, resolved_key, what, updated))
  {
    return false;
  }

  auto it = find(node_params(resolved_node_name), resolved_key, what);
  if (!it)
  {
    what = "Extraction failed (resolved node name: '" + resolved_node_name + "', resolved key: '" + resolved_key +
           "'): " + what;
    return false;
  }
  param = *it;
  return true;
}

template <typename P>
inline const ParamDictionary<P>* find(const ParamDictionary<P>& tree, const std::string key, std::string& what)
{
  if (std::holds_alternative<P>(tree.value()))
  {
    if (key != tree.name())
    {
      what = "The Dictionary store a single param under the param name '" + tree.name() +
             "', but you are searching for the key '" + key + "'";
      return nullptr;
    }
    return &tree;
  }
  else if (std::holds_alternative<typename ParamDictionary<P>::NestedParams>(tree.value()))
  {
    std::vector<std::string> split;
    boost::split(split, key, boost::is_any_of(PARAMETER_SEPARATOR_STRING), boost::token_compress_on);
    auto& value = std::get<typename ParamDictionary<P>::NestedParams>(tree.value());

    auto it = value.begin();
    auto ft = value.begin();
    auto et = value.end();
    for (size_t i = 0; i < split.size(); i++)
    {
      std::string _key = split.at(i);
      ft = std::find_if(it, et,
                        [&_key](const std::pair<std::string, ParamDictionary<P>>& v) { return v.first == _key; });
      if (ft == et)
      {
        what = "The Dictionary store a map of params param under the root param name '" + tree.name() +
               "'. You are searching for the key '" + key +
               "' that is not in the dictionary: " + it->second.to_string("/");
        return nullptr;
      }
      if (std::holds_alternative<typename ParamDictionary<P>::NestedParams>(ft->second.value()))
      {
        auto& value = std::get<typename ParamDictionary<P>::NestedParams>(ft->second.value());
        it = value.begin();
        et = value.end();
        continue;
      }

      if (std::holds_alternative<P>(ft->second.value()) && (i < split.size() - 1))
      {
        what = "The Dictionary store a map of params param under the root param name '" + tree.name() + "'. The key '" +
               _key + "' is  not in the dictionary: " + it->second.to_string("/");
        return nullptr;
      }
    }
    return &(ft->second);
  }

  return nullptr;
}

template <typename P>
inline bool _to_yaml(const ParamDictionary<P>& tree, YAML::Node& node, std::string& what)
{
  if (std::holds_alternative<ParamDictionary<P>::EmptyParam>(tree.value()))
  {
    what = "The param is empty";
    return false;
  }
  else if (std::holds_alternative<P>(tree.value()))
  {
    auto& par = std::get<P>(tree.value());
    std::string json_str = par.toXml();
    YAML::Node yaml_node;
    if (!to_yaml(par, yaml_node, what))
    {
      return false;
    }
    node[tree.name()] = yaml_node;
  }
  else
  {
    auto& nested = std::get<typename ParamDictionary<P>::NestedParams>(tree.value());
    for (const auto& p : nested)
    {
      if (!_to_yaml(p.second, node, what))
      {
        return false;
      }
    }
  }
  return true;
}

template <typename P>
inline bool to_yaml(const ParamDictionary<P>& tree, YAML::Node& node, std::string& what)
{
  YAML::Node yaml_node;
  if (!_to_yaml(tree, yaml_node, what))
  {
    return false;
  }

  node[tree.name()] = yaml_node;
  return true;
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
inline bool resolveParamName(const std::shared_ptr<N>&, const std::string&, std::string&, std::string&)
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
inline std::string lintParamKey(const std::shared_ptr<N>& n, const std::string&)
{
  std::string err = __PRETTY_FUNCTION__ + std::string(":") + std::to_string(__LINE__) + ": " +
                    "To be specialized to your specific case";
  throw std::runtime_error(err.c_str());
  return std::string{};
}

}  // namespace param
}  // namespace cnr

#endif  // CNR_PARAM__INCLUDE__CNR_PARAM__CORE__IMPL__PARAM_RETRIEVER__HPP