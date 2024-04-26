#ifndef CNR_PARAM__INCLUDE__CNR_PARAM__YAML_MAPPED_FILE__IMPL__PARAM__HPP
#define CNR_PARAM__INCLUDE__CNR_PARAM__YAML_MAPPED_FILE__IMPL__PARAM__HPP

#include <string>
#include <vector>
#include <iostream>
#include <iomanip>
#include <type_traits>
#include <Eigen/Core>
#include <yaml-cpp/exceptions.h>
#include <boost/interprocess/file_mapping.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/containers/string.hpp>
#include <boost/interprocess/managed_mapped_file.hpp>
#include <boost/type_index.hpp>

#include <cnr_param/utils/string.h>
#include <cnr_param/utils/eigen.h>
#include <cnr_param/utils/filesystem.h>
#include <cnr_param/yaml_mapped_file/interprocess.h>

#include <cnr_param/yaml_mapped_file/param.h>

#if !defined(UNUSED)
#define UNUSED(expr) do { (void)(expr); } while (0)
#endif

namespace cnr 
{
namespace param
{
namespace yaml
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
    if(!utils::checkfilepath(p.string(), errmsg))
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
  auto tokens = utils::tokenize(key, "/");
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


//======================================================================================================================
//== SERVICE FUNCTION
//======================================================================================================================
/**
 * @brief Get the templated object from the node. 
 * 
 * @tparam T 
 * @param node 
 * @return T 
 */
template<typename T>
inline T extract(const YAML::Node& node, const std::string& key, const std::string& error_heading_msgs)
{
  T ret;
  bool ok = false;
  std::stringstream what;
  YAML::Node leaf;
  if(key.length())
  {
    if(node[key])
    {
      leaf = node[key];
      ok = true;
    }
    else
    {
      what <<(error_heading_msgs.length() ? error_heading_msgs  : 
          (__PRETTY_FUNCTION__  + std::string(":") + std::to_string(__LINE__) + ": "))
            << "Tried to extract a '"
              << boost::typeindex::type_id_with_cvr<decltype(T())>().pretty_name() 
                << "' but the type node key '"<< key <<"' is not available" << std::endl
                  << "Node: " << std::endl
                    << node << std::endl;
    }
  }
  else
  {
    leaf = node;
    ok = true;
  }

  if(ok)
  {
    if(leaf.IsScalar())
    {
      ok = get_scalar(leaf, ret, what);
    }
    else if(leaf.IsSequence())
    {
      ok = get_sequence(leaf, ret, what);
    } 
    else if(leaf.IsMap())
    {
      ok = get_map<T>(leaf, ret, what);
    }
    else
    {
      what
        <<(error_heading_msgs.length() ? error_heading_msgs  : 
          (__PRETTY_FUNCTION__  + std::string(":") + std::to_string(__LINE__) + ": "))
            << "Tried to extract a '"
              << boost::typeindex::type_id_with_cvr<decltype(T())>().pretty_name() 
                << "' but the type node is undefined." << std::endl
                  << "Node: " << std::endl
                    << node << std::endl;
    }
  }

  if(!ok)
  {
    throw std::runtime_error(what.str().c_str());
  }
  return ret;
}

template<>
inline YAML::Node extract(const YAML::Node& node, const std::string& key, const std::string& error_heading_msgs)
{
  UNUSED(key);
  UNUSED(error_heading_msgs);
  YAML::Node ret(node);
  return ret;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


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
inline bool get(const std::string& key, T& ret, std::string& what)
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

  try
  {
    ret = extract<T>(node);
  }
  catch (std::exception& e)
  {
    what = "Failed in getting the Node struct from parameter '" + key + "':\n";
    what += e.what();
    return false;
  }
  return true;
}

template<typename T>
bool set(const std::string& key, const T& ret, std::string& what)
{
  boost::filesystem::path ap; 
  if(!absolutepath(key, false, ap, what))
  {
    return false;
  }

  auto keys = utils::tokenize(key, "/");

  YAML::Node _node;
  _node[keys.back()] = ret;

  std::string str = YAML::Dump(_node);
  str +="\n";
  
  std::size_t fsz = 2 * str.size();
  auto region = utils::createFileMapping(ap.string(),fsz);
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
 * @tparam T 
 * @param key 
 * @param ret 
 * @param what 
 * @param default_val 
 * @return true 
 * @return false 
 */
template<typename T>
inline bool get(const std::string& key, T& ret, std::string& what, const T& default_val)
{
  if (!has(key, what))
  {
    what = (what.size() ? (what + "\n") : std::string("") ) + "Try to superimpose default value...";
    if (!utils::resize(ret, default_val))
    {
      what += " Error!";
      return false;
    }
    what += " OK!";
    ret = default_val;
    return true;
  }

  YAML::Node node;
  if (!recover(key, node, what))
  {
    return false;
  }

  try
  {
    ret = extract<T>(node);
  }
  catch (std::exception& e)
  {
    what = "Failed in getting the Node struct from parameter '" + key + "':\n";
    what += e.what();
    return false;
  }
  return true;
}

/**
 * @brief 
 * 
 * @param node 
 * @param key 
 * @return true 
 * @return false 
 */
inline bool has(const YAML::Node& node, const std::string& key)
{
  return bool(node[key]);
}


/**
 * @brief 
 * 
 * @param node 
 * @return true 
 * @return false 
 */
inline bool is_sequence(const YAML::Node& node)
{
  return node.IsSequence();
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
  return is_sequence(node);
}

/**
 * @brief 
 * 
 * @param node 
 * @return true 
 * @return false 
 */
inline bool is_map(const YAML::Node& node)
{
  return node.IsMap();
}

/**
 * @brief 
 * 
 * @param node 
 * @return size_t 
 */
inline size_t size(const YAML::Node& node)
{
  if(!is_sequence(node))
  {
    std::stringstream err;
    err << __PRETTY_FUNCTION__ << ":" << std::to_string(__LINE__)  << ": " 
      << "The node is not a sequence!";
    throw std::runtime_error(err.str().c_str());
  }
  return node.size();
}

/**
 * @brief Get the leaf object
 * 
 * @param node 
 * @param key 
 * @param leaf 
 * @param what 
 * @return true 
 * @return false 
 */
inline bool get_leaf(const YAML::Node& node, const std::string& key, YAML::Node& leaf, std::string& what)
{
   if(node[key])
   {
    leaf = node[key];
    return true;
   }
   what = "The key is not in the node dictionary";
   return false;
}

/**
 * @brief 
 * 
 * @tparam T 
 * @param node 
 * @param i 
 * @param element 
 * @param what 
 * @return true 
 * @return false 
 */
template<typename T>
inline bool at(const YAML::Node& node, std::size_t i, T& element, std::string& what)
{
  if(!is_sequence(node))
  {
    what = "The node is not a sequence";
    return false;
  }
  if(i>= node.size())
  {
    what = "The index is out the boundaries";
    return false;
  }

  if(node[i].IsScalar())
  {
    return get_scalar<T>(node[i], element);
  }
  
  what = std::string(__PRETTY_FUNCTION__ ) + ":" + std::to_string( __LINE__ ) + ": "
             "Error in the extraction of the element #" + std::to_string(i) + ". Type of the sequence: "
              + boost::typeindex::type_id_with_cvr<decltype(T())>().pretty_name() 
                + "' but the element is not a scalar type.";
  return false;
}

/**
 * @brief 
 * 
 * @tparam  
 * @param node 
 * @param i 
 * @param element 
 * @param what 
 * @return true 
 * @return false 
 */
template<>
inline bool at(const YAML::Node& node, std::size_t i, YAML::Node& element, std::string& what)
{
  if(!is_sequence(node))
  {
    what = "It is not a sequence";
    return false;
  }
  if(i>= node.size())
  {
    what = "Index out of boundaries";
    return false;
  }

  element = node[i];

  return true;
}

/**
 * @brief 
 * 
 * @tparam T 
 * @param node 
 * @param key 
 * @param element 
 * @param what 
 * @return true 
 * @return false 
 */
template<typename T>
inline bool at(const YAML::Node& node, const std::string& key, T& element, std::string& what)
{
  if(!is_map(node))
  {
    what = "The node is not a map";
    return false;
  }
  if(!node[key])
  {
    what = "The node has not the required key='" + key + "'";
    return false;
  }
  

  if(node[key].IsScalar())
  {
    return get_scalar<T>(node[key], element);
  }
  
  what = std::string(__PRETTY_FUNCTION__ ) + ":" + std::to_string( __LINE__ ) + ": "
             "Error in the extraction of the element '" + key + "'. Type of the required type: "
              + boost::typeindex::type_id_with_cvr<decltype(T())>().pretty_name() 
                + "' but the element is not a scalar type.";
  return false;
}

template<>
inline bool at(const YAML::Node& node, const std::string& key, YAML::Node& element, std::string& what)
{
  if(!is_map(node))
  {
    what = "The node is not a map";
    return false;
  }
  if(!node[key])
  {
    what = "The node has not the required key='" + key + "'";
    return false;
  }

  element = node[key];

  return true;
}
// =============================================================================================


template<typename T>
inline void insert(YAML::Node& node, const std::string& key, const T& value)
{
  try
  {
    if(key.length())
    {
      node[key] = value;
    }
    else
    {
      node = value;
    }
  }
  catch(const std::exception& e)
  {
    throw std::runtime_error(e.what());
  }
}

template< typename T >
std::string int_to_hex( T i )
{
  std::stringstream stream;
  stream << "0x" 
         << std::setfill ('0') << std::setw(sizeof(T)*2) 
         << std::hex << i;
  return stream.str();
}

inline void insert(YAML::Node& node, const std::string& key, const int& value, const std::string& format)
{
  std::string val;
  if(format=="dec")
  {
    val = std::to_string(value);
  }
  else if(format=="hex")
  {
    val = int_to_hex(value);
  }
  try
  {
    if(key.length())
    {
      node[key] = val;
    }
    else
    {
      node = val;
    }
  }
  catch(const std::exception& e)
  {
    throw std::runtime_error(e.what());
  }
}

#define CATCH(X)\
  catch(YAML::Exception& e)\
  {\
      std::cerr << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": "\
        << "YAML Exception, Error in the extraction of an object of type '"\
          << boost::typeindex::type_id_with_cvr<decltype( X )>().pretty_name() \
            << "'" << std::endl\
              << "Node: " << std::endl\
                << node << std::endl\
                  << "What: " << std::endl\
                    << e.what() << std::endl;\
    }\
    catch (std::exception& e)\
    {\
      std::cerr << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": "\
        << "Exception, Error in the extraction of an object of type '"\
          << boost::typeindex::type_id_with_cvr<decltype( X )>().pretty_name() \
            << "'" << std::endl\
              << "Node: " << std::endl\
                << node << std::endl\
                  << "What: " << std::endl\
                    << e.what() << std::endl;\
    }


// =============================================================================================
// SCALAR
// =============================================================================================
template<typename T>
inline bool _get_scalar(const YAML::Node& node, T& ret, std::stringstream& what)
{
  YAML::Node config(node);
  if(!config.IsScalar())
  {
    what << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": "
      << "Tried to extract a '"
        << boost::typeindex::type_id_with_cvr<decltype(T())>().pretty_name() 
          << "' but the node is not a scalar." << std::endl
            << "Node: " << std::endl
              << node << std::endl;
    return false;
  }
  
  try
  {
    ret = node.as<T>();
    return true;
  }
  CATCH(ret);
  return false;
}

template<typename T>
inline bool get_scalar(const YAML::Node& node, T& ret, std::stringstream& what)
{
  UNUSED(node);
  UNUSED(ret);
  what << "The type ' " << boost::typeindex::type_id_with_cvr<decltype(T())>().pretty_name() 
          << "' is not supported. You must specilized your own 'get_scalar' template function";
  return false;
}

template<>
inline bool get_scalar(const YAML::Node& node, double& ret, std::stringstream& what)
{
  return _get_scalar<double>(node, ret, what);
}

template<>
inline bool get_scalar(const YAML::Node& node, int& ret, std::stringstream& what)
{
  return _get_scalar<int>(node, ret, what);
}

template<>
inline bool get_scalar(const YAML::Node& node, bool& ret, std::stringstream& what)
{
  return _get_scalar<bool>(node, ret, what);
}

template<>
inline bool get_scalar(const YAML::Node& node, std::string& ret, std::stringstream& what)
{
  return _get_scalar<std::string>(node, ret, what);
}
// =============================================================================================
// END SCALAR
// =============================================================================================


// =============================================================================================
// SEQUENCE
// =============================================================================================
template <typename C> struct is_vector : std::false_type {};
template <typename T,typename A> struct is_vector< std::vector<T,A> > : std::true_type {};

template<typename Derived>
struct is_matrix_expression : std::is_base_of<Eigen::MatrixBase<std::decay_t<Derived> >, std::decay_t<Derived> > {};

// ffwd declarations
template<typename Derived>
bool _get_sequence_eigen(const YAML::Node& node, Eigen::MatrixBase<Derived> const & ret, std::stringstream& what);

template<typename T, typename A>
bool _get_sequence(const YAML::Node& node, std::vector<T, A>& ret, std::stringstream& what);

template<typename T, typename A>
bool _get_sequence(const YAML::Node& node, std::vector<std::vector<T, A>>& ret, std::stringstream& what);

template<typename T, std::size_t  N>
bool _get_sequence(const YAML::Node& node, std::array<T,N>& ret, std::stringstream& what);

template<typename T, std::size_t N, std::size_t M>
bool _get_sequence(const YAML::Node& node, std::array<std::array<T,M>,N>& ret, std::stringstream& what);


template<typename T, typename A>
inline bool _get_sequence(const YAML::Node& node, std::vector<T, A>& ret, std::stringstream& what)
{
  YAML::Node config(node);
  if(!config.IsSequence())
  {
    what << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": " 
      << "Tried to extract a '"
        << boost::typeindex::type_id_with_cvr<decltype(T())>().pretty_name() 
          << "' but the node is not a sequence." << std::endl
            << "Node: " << std::endl
              << node << std::endl;
    return false;
  }
  
  try
  {
    bool ok = false;
    ret.clear();
    for(std::size_t i=0; i<node.size();i++)
    {
      T v = T();
      if(node[i].IsScalar())
      {
        ok = get_scalar<T>(node[i], v, what);
      }
      else if(node[i].IsSequence())
      {
        ok = get_sequence<T>(node[i], v, what);
      }
      else if(node[i].IsMap())
      {
        ok = get_map<T>(node[i], v, what);
      }

      if(!ok)
      {
        what  << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": "
                << "Error in the extraction of the element #" << i << ". Type of the sequence: "
                  << boost::typeindex::type_id_with_cvr<decltype(std::vector<T>())>().pretty_name() 
                    << "' but the node is not a sequence." << std::endl
                      << "Node: " << std::endl
                        << node << std::endl;
        break;
      }
      ret.push_back(v);
    }
    return ok;
  }
  CATCH(ret);
  
  return false;
}

template<typename T, typename A>
inline bool _get_sequence(const YAML::Node& node, std::vector<std::vector<T, A>>& ret, std::stringstream& what)
{
  bool ok = false;
  YAML::Node config(node);
  if(!config.IsSequence())
  {
    what << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": "
      << "Tried to extract a '"
        << boost::typeindex::type_id_with_cvr<decltype(T())>().pretty_name() 
          << "' but the node is not a sequence." << std::endl
            << "Node: " << std::endl
              << node << std::endl;
  }
  else
  {
    try
    {
      ret.clear();
      for(std::size_t i=0; i<node.size();i++)
      {
        if(!config[i].IsSequence())
        {
          what << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": "
            << "Tried to extract a '"
              << boost::typeindex::type_id_with_cvr<decltype(T())>().pretty_name() 
                << "' but the node is not a sequence." << std::endl
                  << "Node: " << std::endl
                    << config << std::endl;
        }
        std::vector<T> v;
        ok = get_sequence(config[i],v,what);
        if(!ok)
        {
          what << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": "
            << "Error in the extraction of the element #" << i << ". Type of the sequence: "
              << boost::typeindex::type_id_with_cvr<decltype(std::vector<T>())>().pretty_name() 
                << "' but the node is not a sequence." << std::endl
                  << "Node: " << std::endl
                    << node << std::endl;
          break;
        }
        ret.push_back(v);
      }
    }
    CATCH(ret);
  }
  return ok;
}

template<typename T, std::size_t  N>
bool _get_sequence(const YAML::Node& node, std::array<T,N>& ret, std::stringstream& what)
{
  bool ok = false;
  try
  {
    std::vector<T> tmp;
    ok = get_sequence(node,tmp,what);
    if(!ok || (tmp.size()==N))
    {
      what << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": "
        << "Tried to extract a '"
          << boost::typeindex::type_id_with_cvr<decltype(T())>().pretty_name() 
            << "' ?error in size?" << std::endl
              << "Node: " << std::endl
                << node << std::endl;
    }
    else
    {
      std::copy_n(tmp.begin(), N, ret.begin());
      ok = true;
    }
  }
  CATCH(ret);

  return ok;
}

template<typename T, std::size_t N, std::size_t M>
bool _get_sequence(const YAML::Node& node, std::array<std::array<T,M>,N>& ret, std::stringstream& what)
{
  bool ok = false;
  try
  {
    std::vector<std::vector<T>> tmp;
    ok = _get_sequence(node, tmp, what);
    if(!ok || (tmp.size()!=N) || (tmp.front().size()!=M))
    {
      what << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": "
        << "Tried to extract a '"
          << boost::typeindex::type_id_with_cvr<decltype(T())>().pretty_name() 
            << "' ?error in size?" << std::endl
              << "Node: " << std::endl
                << node << std::endl;
    }
    else
    {
      for(std::size_t i=0;i<N;i++)
      {
        std::copy_n(tmp.at(i).begin(), M, ret.at(i).begin());
      }
      ok = true;
    }
  }
  CATCH(ret);

  return ok;
}

template<typename Derived>
inline bool _get_sequence_eigen(const YAML::Node& node, Eigen::MatrixBase<Derived> const & ret, std::stringstream& what)
{
  bool ok = false;
  Eigen::MatrixBase<Derived>& _ret = const_cast< Eigen::MatrixBase<Derived>& >(ret);
  YAML::Node config(node);

  int expected_rows = Eigen::MatrixBase<Derived>::RowsAtCompileTime;
  int expected_cols = Eigen::MatrixBase<Derived>::ColsAtCompileTime;
  bool should_be_a_vector = (expected_rows == 1 || expected_cols == 1);

  try
  {
    if (should_be_a_vector)
    {
      std::vector<double> vv;
      ok = _get_sequence(config, vv, what);
      if(!ok)
      {
        what << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": "
          << "Tried to extract a vector from the "
                << "Node: " << std::endl
                  << node << std::endl;
        return false;
      }

      int dim = static_cast<int>(vv.size());
      if (!utils::resize(_ret, (expected_rows == 1 ? 1 : dim), (expected_rows == 1 ? dim : 1)))
      {
        what << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": "
            << "It was expected a Vector (" +
                                 std::to_string(expected_rows) + "x" + std::to_string(expected_cols) +
                                 ") while the param store a " + std::to_string(dim) + "-vector" << std::endl
                << "Node: " << std::endl
                  << node << std::endl;
        return false;
      }
      for (int i = 0; i < dim; i++)
      {
        _ret(i) = vv.at(static_cast<std::size_t>(i));
      }
    }
    else  // matrix expected
    {
      std::vector<std::vector<double>> vv;
      ok = _get_sequence(node, vv, what);
      if(!ok)
      {
        std::cerr << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": "
          << "Tried to extract a vector from the "
                << "Node: " << std::endl
                  << node << std::endl;
        return false;
      }

      int rows = static_cast<int>(vv.size());
      int cols = static_cast<int>(vv.front().size());
      if (!utils::resize(_ret, rows, cols))
      {
        what << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": "
            << "It was expected a Vector (" +
                                 std::to_string(expected_rows) + "x" + std::to_string(expected_cols) +
                                 ") while the param store a " + std::to_string(vv.size()) + "-vector" << std::endl
                << "Node: " << std::endl
                  << node << std::endl;
        return false;
      }
      for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
          _ret(i, j) = vv.at(static_cast<int>(i)).at(static_cast<int>(j));
    }
  }
  catch (std::exception& e)
  {
    what << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": "
        << "Node: " << std::endl
          << node << std::endl
            << "What: " << std::endl
              << e.what() << std::endl;
    ok = false;
  }
  catch (...)
  {
    what  << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": "
            << "Node: " << std::endl
              << node << std::endl
                << "What: " << std::endl
                  << "Wrong format" << std::endl;
    ok = false;
  }
  return true;
}


template<typename T>
inline bool _get_sequence(const YAML::Node& node, T& ret, std::stringstream& what)
{
  if constexpr(is_matrix_expression<T>::value)
  {
    return _get_sequence_eigen(node,ret,what);
  }
  UNUSED(node);
  UNUSED(ret);

  what << "The type ' " <<
      boost::typeindex::type_id_with_cvr<decltype(T())>().pretty_name() 
        << "' is not supported. You must specilized your own 'get_sequence' template function";
  return false;
}

template<typename T>
inline bool get_sequence(const YAML::Node& node, T& ret, std::stringstream& what)
{
  return _get_sequence(node, ret, what);
}

template<typename T>
inline bool get_map(const YAML::Node& node, T& ret, std::stringstream& what)
{
  UNUSED(ret);
  what<< "The type ' "
        << boost::typeindex::type_id_with_cvr<decltype(T())>().pretty_name() 
          << "' is not supported. You must specilized your own 'get_map' template function" << std::endl
            << "' Node: " << std::endl
              << node  << std::endl;
  return false;
}

}
}
}

#endif  /* CNR_PARAM__INCLUDE__CNR_PARAM__YAML_MAPPED_FILE__IMPL__PARAM__HPP */

