#ifndef CNR_PARAM__INCLUDE__CNR_PARAM__CORE__IMPL__PARAM__HPP
#define CNR_PARAM__INCLUDE__CNR_PARAM__CORE__IMPL__PARAM__HPP

#include <type_traits>
#include <yaml-cpp/node/type.h>
#include <yaml-cpp/node/convert.h>

#include <cnr_param/core/string.h>
#include <cnr_param/core/type_traits.h>
#include <cnr_param/core/param_dictionary.h>
#include <cnr_param/core/param.h>


#include <cnr_param/core/eigen.h>
#include <cnr_param/core/impl/param_sequence.hpp>
#include <cnr_param/core/impl/param_map.hpp>
#include <cnr_param/core/impl/param_insert.hpp>


namespace cnr
{
namespace param
{
namespace core
{

// =====================================================================================================================
//
// Get 
//
//
// =====================================================================================================================
template <typename To, typename From, std::enable_if<std::is_convertible<std::decay_t<To>,std::decay_t<From>>::value, int>::type* = nullptr>
inline void cast(To&& to, From&& from)
{
  to = from;
}

template <typename To, typename From, std::enable_if<
  !std::is_convertible<std::decay_t<To>,std::decay_t<From>>::value && 
  cnr::param::are_compatible_vectors<std::decay_t<To>,std::decay_t<From>
>::value, double>::type* = nullptr>
inline void cast(To&& to, From&& from)
{
  container_cast(to, from);
}

template <typename To, typename From, std::enable_if<
  !std::is_convertible<std::decay_t<To>,std::decay_t<From>>::value && 
  !cnr::param::are_compatible_vectors<std::decay_t<To>,std::decay_t<From>>::value>::type* = nullptr
>
inline void cast(To&& to, From&& from)
{
  to = from;
}


template <typename T, unsigned int  I, unsigned int N, std::enable_if<(I==N), bool>::type = true>
inline bool decode(const YAML::Node& node, T& ret, std::string& what, const bool& implicit_cast_if_possible)
{
  bool ok = false;
  switch (node.Type())
  {
    case YAML::NodeType::Scalar:
      ok = false;
      break;
    case YAML::NodeType::Sequence:
      ok = cnr::param::core::get_sequence(node, ret, what, implicit_cast_if_possible);
      break;
    case YAML::NodeType::Map:
      ok = cnr::param::core::get_map(node, ret, what, implicit_cast_if_possible);
      break;
    default:
      ok = false;
      break;
  }

  if (!ok)
  {
    what = "Error! Deconding the type '" + boost::typeindex::type_id_with_cvr<decltype(ret)>().pretty_name() +
           "' from the node was not possible, neither using the available alternatives. Input Node:\n" + std::to_string(node);
  }
  return ok;
}

template <typename T, unsigned int I, unsigned int N, std::enable_if<(I>=0 && I<N), int>::type = 1>
inline bool decode(const YAML::Node& node, T& ret, std::string& what, const bool& implicit_cast_if_possible)
{
  using variant = typename decoding_type_variant_holder<T>::variant;
  using type    = std::variant_alternative<I, variant>::type;
  type _ret;
  try
  {
    if (!YAML::convert<type>::decode(node, _ret))
    {
      return decode<T, I+1, N>(node, ret, what, implicit_cast_if_possible);
    }

    cast(std::move(ret), std::move(_ret));
    return true;
  }
  catch (const std::exception& e)
  {
    what += std::string(e.what());
  }
  catch (...)
  {
    what += "Unknown error in decoding a '" + boost::typeindex::type_id_with_cvr<decltype(ret)>().pretty_name() +
            "' from node\n" + std::to_string(node) + "";
  }
  return decode<T, I+1, N>(node, ret, what, implicit_cast_if_possible);
}

template <typename T>
inline bool get(const YAML::Node& node, T& ret, std::string& what, const bool& implicit_cast_if_possible)
{
  try
  {
    if (decode<T, 0, std::variant_size<typename decoding_type_variant_holder<T>::variant>::value>(node, ret, what, implicit_cast_if_possible))
    {
      return true;
    }
  }
  catch (const std::exception& e)
  {
    what = std::string(e.what());
  }
  catch (...)
  {
    what = "Unknown error in decoding a '" + boost::typeindex::type_id_with_cvr<decltype(ret)>().pretty_name() +
           "' from node\n" + std::to_string(node) + "";
  }

  what = "Error! Implicit Cast not used. Failed in decoding a '" +
         boost::typeindex::type_id_with_cvr<decltype(ret)>().pretty_name() + "' from node\n" + std::to_string(node) +
         "";
  return false;
}


// =====================================================================================================================
//
// Set 
//
//
// =====================================================================================================================
template <typename T, unsigned int  I, unsigned int  N, std::enable_if<(I==N), bool>::type* = nullptr>
inline bool encode(const T&, YAML::Node&, std::string& what)
{
  what = "Error! Encoding the type '" + boost::typeindex::type_id_with_cvr<T>().pretty_name() +
           "' from the node was not possible, neither using the available alternatives.";
  return false;
}

template <typename T, unsigned int  I, unsigned int  N, std::enable_if<(I>=0 && I<N), int>::type* = nullptr>
inline bool encode(const T& value, YAML::Node& ret, std::string& what)
{
  using const_type = std::decay_t<const T&>;
  using variant = typename encoding_type_variant_holder<const_type>::variant;
  using type = std::variant_alternative<I, variant>::type;
  type _value;

  try
  {
    what = "Input Type: " + boost::typeindex::type_id_with_cvr<const T&>().pretty_name()  + ", variant type: " + boost::typeindex::type_id_with_cvr<type>().pretty_name();
    cast(std::move(_value), std::move(value));
    ret = YAML::convert<type>::encode(_value);
    what += ", Return Object: " + std::to_string(ret);
    what += ", Node Type: " + std::to_string(ret.Type());
    return true; // encode<T, I+1, N>(value, ret, what);
  }
  catch (const std::exception& e)
  {
    what = std::string(e.what()) + ", " + what;
  }
  catch (...)
  {
    what = "Unknown error. " + what;
  }
  return encode<T, I+1, N>(value, ret, what);
}

template <typename T>
inline bool set(const T& value, YAML::Node& ret, std::string& what)
{
  std::string err = "Error! Implicit Cast not used. Failed in encoding a '" +
         boost::typeindex::type_id_with_cvr<T>().pretty_name() + "'";
  try
  {
    if(encode<T, 0, std::variant_size<typename encoding_type_variant_holder<T>::variant>::value >(value, ret, what))
    {
      return true;
    }
  }
  catch (const std::exception& e)
  {
    what = err+" What: " + std::string(e.what());
  }
  catch (...)
  {
    what = err + " Unknown error in encoding a '" + boost::typeindex::type_id_with_cvr<T>().pretty_name() + "'";
  }
  return false;
}


}  // namespace core
}  // namespace param
}  // namespace cnr

#endif  // CNR_PARAM__INCLUDE__CNR_PARAM__CORE__IMPL__PARAM__HPP