#ifndef CNR_PARAM__INCLUDE__CNR_PARAM__UTILS__IMPL__PARAM_SCALAR__HPP
#define CNR_PARAM__INCLUDE__CNR_PARAM__UTILS__IMPL__PARAM_SCALAR__HPP

#include <iostream>
#include <type_traits>
#include <boost/type_index.hpp>
#include <yaml-cpp/yaml.h>

#include <cnr_param/core/string.h>
#include <cnr_param/core/type_traits.h>
#include <cnr_param/core/yaml.h>

#if !defined(UNUSED)
#define UNUSED(expr)                                                                                                   \
  do                                                                                                                   \
  {                                                                                                                    \
    (void)(expr);                                                                                                      \
  } while (0)
#endif

namespace cnr
{
namespace param
{
namespace core
{

template <typename T>
inline typename std::enable_if<cnr::param::is_scalar<T>::value, bool>::type _get_scalar(const YAML::Node& node, T& ret,
                                                                                       std::string& what)
{
  YAML::Node config(node);
  if (!config.IsScalar())
  {
    std::stringstream _node;
    _node << node;
    what = "Tried to extract a '" + boost::typeindex::type_id_with_cvr<decltype(T())>().pretty_name() +
           "' but the node is not a scalar\n Input Node: " + _node.str();
    return false;
  }

  try
  {
    ret = node.as<T>();
    return true;
  }
  catch (YAML::Exception & e)                                                                                          
  {
    std::stringstream err;
    err << "YAML Exception, Error in the extraction of an object of type '"                                      
        << boost::typeindex::type_id_with_cvr<decltype(ret)>().pretty_name() << "'" << std::endl                 
        << "Node: " << std::endl                                                                                 
        << node << std::endl                                                                                     
        << "What: " << std::endl                                                                                 
        << e.what() << std::endl;                                                                                
    what = err.str();
  }                                                                                                                    
  catch (std::exception & e)                                                                                           
  {
    std::stringstream err;                                                                                             
    err << "Exception, Error in the extraction of an object of type '"                                           
        << boost::typeindex::type_id_with_cvr<decltype(ret)>().pretty_name() << "'" << std::endl                 
        << "Node: " << std::endl                                                                                 
        << node << std::endl                                                                                     
        << "What: " << std::endl                                                                                 
        << e.what() << std::endl;                                                                                
    what = err.str();
  };
  return false;
}

template <typename T>
inline typename std::enable_if<!cnr::param::is_scalar<T>::value, bool>::type _get_scalar(const YAML::Node& node, T& ret,
                                                                                        std::string& what)
{
  UNUSED(node);
  UNUSED(ret);
  std::stringstream _node;
  _node << node;
  what = "Tried to extract a '" + boost::typeindex::type_id_with_cvr<decltype(T())>().pretty_name() +
         "' but the node is not a scalar\n Input Node: " + _node.str();
  return false;
}

template <typename T>
inline bool get_scalar(const YAML::Node& node, T& ret, std::string& what)
{
  return _get_scalar(node,ret,what);
}

}  // namespace utils
}  // namespace param
}  // namespace cnr

#endif  // CNR_PARAM__INCLUDE__CNR_PARAM__UTILS__IMPL__PARAM_SCALAR__HPP