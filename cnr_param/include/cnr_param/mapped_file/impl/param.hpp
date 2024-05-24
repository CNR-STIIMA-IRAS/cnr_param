#ifndef CNR_PARAM__INCLUDE__CNR_PARAM__mapped_file__IMPL__PARAM__HPP
#define CNR_PARAM__INCLUDE__CNR_PARAM__mapped_file__IMPL__PARAM__HPP

#include <string>
#include <Eigen/Core>
#include <yaml-cpp/exceptions.h>
#include <boost/interprocess/file_mapping.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/containers/string.hpp>
#include <boost/interprocess/managed_mapped_file.hpp>
#include <boost/type_index.hpp>

#include <cnr_param/core/string.h>
#include <cnr_param/core/eigen.h>
#include <cnr_param/core/filesystem.h>
#include <cnr_param/core/yaml.h>
#include <cnr_param/core/param.h>

#include <cnr_param/mapped_file/interprocess.h>
#include <cnr_param/mapped_file/param.h>
#include <yaml-cpp/node/node.h>


#if !defined(UNUSED)
#define UNUSED(expr) do { (void)(expr); } while (0)
#endif

namespace cnr 
{
namespace param
{
namespace mapped_file
{

// =============================================================================== //
//                                                                                 //
//  UTILITIES                                                                      //
//                                                                                 //
//                                                                                 //
// =============================================================================== //
inline bool absolutepath(const std::string& key, const bool check_if_exist, boost::filesystem::path& ap, std::string& what)
{
  if((key.size()==0)||(key.front()!='/'))
  {
    what = "The key '"+key+"' is ill-formed. cnr_param support only aboslute path, i.e., the key must start with '/'";
    return false;
  }
  const char* env_p = std::getenv("CNR_PARAM_ROOT_DIRECTORY");
  if(!env_p)
  {
    what = (what.size() ? (what + "\n") : std::string("") ) +
      "The env variable CNR_PARAM_ROOT_DIRECTORY is not set!" ;
      return false;
  }
  std::string _key = key;
  while(_key.back()=='/')
  {
    _key.pop_back();
  }
  boost::filesystem::path p = boost::filesystem::path(std::string(env_p)) / (_key + ".yaml");

  if(check_if_exist)
  {
    std::string errmsg;
    if(!core::checkfilepath(p.string(), errmsg))
    {
      what = (what.size() ? (what + "\n") : std::string("") ) +
        "The param '" + key + "' is not in param server.\n what(): " + errmsg;
      return false;
    }
  }
  ap = boost::filesystem::absolute(p);
  return true;
}

inline bool has(const std::string& key, std::string& what)
{
  boost::filesystem::path ap; 
  if(!absolutepath(key, true, ap, what))
  {
    return false;
  }
  return true;
}

inline bool recover(const std::string& key, YAML::Node& node, std::string& what)
{
  boost::filesystem::path ap; 
  if(!absolutepath(key, true, ap, what))
  {
    return false;
  }

  //Create a file mapping
  boost::interprocess::file_mapping file(ap.string().c_str(), boost::interprocess::read_only);

  //Map the whole file with read-write permissions in this process
  boost::interprocess::mapped_region region(file, boost::interprocess::read_only);

  //Get the address of the mapped region
  void * addr       = region.get_address();
  std::size_t size  = region.get_size();
  UNUSED(size);
  
  //Check that memory was initialized to 1
  const char *mem = static_cast<char*>(addr);
  std::string strmem(mem);

  auto config = YAML::Load(strmem);
  if(config.size()==0)
  {
    what = "The namespace server is empty";
    return false;
  }
  auto tokens = core::tokenize(key, "/");
  if(tokens.size()==0){
    what = "The key'"+key+"' is ill-formed, none '/' is present. Only Aboslute path are supported in cnr_param";
    return false;
  }
  node = config[tokens.back()];
  
  return bool(config[tokens.back()]);
}
// =============================================================================== //
//                                                                                 //
//                                                                                 //
//                                                                                 //
//                                                                                 //
// =============================================================================== //


/**
 * @brief 
 * 
 * @tparam T 
 * @param key 
 * @param ret 
 * @param what 
 * @param default_val 
 * @return true 
 * @return false 
 */
template<typename T>
inline bool get(const std::string& key, T& ret, std::string& what, const bool& implicit_cast_if_possible)
{
  if (!has(key, what))
  {
    return false;
  }

  YAML::Node node;
  if (!recover(key, node, what))
  {
    return false;
  }

  what = "Failed in getting the Node struct from parameter '" + key + "':\n";
  try
  {
    if constexpr(std::is_same<T,YAML::Node>::value)
    {
      ret = node;
      return true;
    }
    else
    {
     
      if(node.IsScalar())
      {
        return cnr::param::core::get_scalar<T>(node, ret, what, implicit_cast_if_possible);
      }
      else if(node.IsSequence())
      {
        return cnr::param::core::get_sequence<T>(node, ret, what, implicit_cast_if_possible);
      } 
      else if(node.IsMap())
      {
        return cnr::param::core::get_map<T>(node, ret, what, implicit_cast_if_possible);
      }
      
      std::stringstream _node;
      _node << node;
      what += "Tried to extract a '" + boost::typeindex::type_id_with_cvr<decltype(T())>().pretty_name() 
                + "' but the node type is undefined\n Input Node: \n" + _node.str();
    
    }
  }
  catch (std::exception& e)
  {
    what += e.what();
    return false;
  }
  return false;
}

template<typename T>
bool set(const std::string& key, const T& ret, std::string& what)
{
  boost::filesystem::path ap; 
  if(!absolutepath(key, false, ap, what))
  {
    return false;
  }

  auto keys = core::tokenize(key, "/");

  YAML::Node _node;
  _node[keys.back()] = ret;

  std::string str = YAML::Dump(_node);
  str +="\n";
  
  std::size_t fsz = 2 * str.size();
  auto region = createFileMapping(ap.string(),fsz);
  if(!region)
  {
    what = "IMpossible to create the file mapping '" + ap.string() +"'";
    return false;
  }
  std::memcpy(region->get_address(), str.c_str(), str.size() );
  
  return true;
}



/**
 * @brief 
 * 
 * @param key 
 * @return true 
 * @return false 
 */
inline bool is_sequence(const std::string& key)
{
  YAML::Node node;
  std::string what;
  if (!recover(key, node, what))
  {
    return false;
  }
  return cnr::param::core::is_sequence(node);
}

}  // namespace mapped_file
}  // namespace param
}  // namespace cnr

#endif  /* CNR_PARAM__INCLUDE__CNR_PARAM__mapped_file__IMPL__PARAM__HPP */

