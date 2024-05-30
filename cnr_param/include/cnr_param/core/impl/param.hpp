#ifndef CNR_PARAM__INCLUDE__CNR_PARAM__CORE__IMPL__PARAM__HPP
#define CNR_PARAM__INCLUDE__CNR_PARAM__CORE__IMPL__PARAM__HPP

#include <cnr_param/core/string.h>
#include <cnr_param/core/type_traits.h>
#include <cnr_param/core/param_dictionary.h>
#include <cnr_param/core/param.h>
#include <yaml-cpp/node/type.h>

#include <cnr_param/core/eigen.h>
#include <cnr_param/core/impl/param_sequence.hpp>
#include <cnr_param/core/impl/param_map.hpp>
#include <cnr_param/core/impl/param_insert.hpp>
#include <type_traits>

namespace cnr
{
namespace param
{
namespace core
{
template <typename T, unsigned int  N, std::enable_if<(N==0), bool>::type = true>
inline bool decode(const YAML::Node& node, T& ret, std::string& what)
{
  bool ok = false;
  switch (node.Type())
  {
    case YAML::NodeType::Scalar:
      ok = false;
      break;
    case YAML::NodeType::Sequence:
      ok = cnr::param::core::get_sequence(node, ret, what, true);
      break;
    case YAML::NodeType::Map:
      ok = cnr::param::core::get_map(node, ret, what, true);
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

template <typename T, unsigned int N, std::enable_if<(N>0), int>::type = 1>
inline bool decode(const YAML::Node& node, T& ret, std::string& what)
{
  using variant = typename type_variant_holder<T>::variant;
  using type = std::variant_alternative<N - 1, variant>::type;
  type _ret;
  try
  {
    if (YAML::convert<type>::decode(node, _ret))
    {
      ret = _ret;
      return true;
    }
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
  return decode<T, N - 1>(node, ret, what);
}

template <typename T>
inline bool get(const YAML::Node& node, T& ret, std::string& what, const bool& implicit_cast_if_possible)
{
  try
  {
    if (YAML::convert<T>::decode(node, ret))
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

  if (implicit_cast_if_possible)
  {
    return decode<T, std::variant_size<typename type_variant_holder<T>::variant>::value - 1 >(node, ret, what);
  }

  what = "Error! Implicit Cast not used. Failed in decoding a '" +
         boost::typeindex::type_id_with_cvr<decltype(ret)>().pretty_name() + "' from node\n" + std::to_string(node) +
         "";
  return false;
}
}  // namespace core
}  // namespace param
}  // namespace cnr

#endif  // CNR_PARAM__INCLUDE__CNR_PARAM__CORE__IMPL__PARAM__HPP