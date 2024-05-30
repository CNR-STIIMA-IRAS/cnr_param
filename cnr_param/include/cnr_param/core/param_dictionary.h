#ifndef CNR_PARAM__INCLUDE__CNR_PARAM__CORE__PARAM_DICTIONARY__H
#define CNR_PARAM__INCLUDE__CNR_PARAM__CORE__PARAM_DICTIONARY__H

#include <string>
#include <vector>
#include <map>
#include <variant>
#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

#include <cnr_param/core/type_traits.h>

namespace cnr
{
namespace param
{
namespace core
{

/**
 * @brief 
 * 
 * @tparam T 
 */
template <typename c_type, typename param_type>
struct ParamType 
{
  static constexpr param_type value = param_type();
};

/**
 * @brief This abstract template is for the implicit cast of a parameter value to a std::variant<..> that should collect the types that undeline to the pameter type
 * 
 * @tparam P 
 * @tparam A 
 * @param param 
 * @return A 
 */
template<typename P, typename A>
A as_generic(const P& param);

/**
 * @brief For each type of parameter, the template if the given type T is equivalent ot not
 * 
 * @tparam ParamType 
 * @tparam T 
 */
template<int ParamType, typename T> struct is_forward_implicit_conversion_allowed : std::false_type {};

template<typename c_type> 
struct type_variant_holder
{
  using base = c_type;
  using variant = std::variant<c_type>;
};


template<typename ParamType, ParamType p, typename T> 
struct type_holder
{
  using c_type = T;
  using param_type = ParamType;
  constexpr static const ParamType value = p;
};

template<typename ParamType, ParamType p>
struct param_type_to_c_type
{
  using c_type = void*;
  using param_type = ParamType;
  constexpr static const param_type value = p;
};

#define SPECILIZE_PARAM_TYPE_TO_C_TYPE( TYPE_HOLDER_SPECIALIZATION )\
template<> struct param_type_to_c_type< TYPE_HOLDER_SPECIALIZATION::param_type, TYPE_HOLDER_SPECIALIZATION::value >\
{\
  using c_type = TYPE_HOLDER_SPECIALIZATION::c_type;\
  using param_type = TYPE_HOLDER_SPECIALIZATION::param_type;\
  constexpr static const param_type value = TYPE_HOLDER_SPECIALIZATION::value;\
}

template<typename T> 
struct c_type_to_param_type
{
  using c_type = T;
  using param_type = void*;
  constexpr static const param_type value = param_type();
};

#define SPECILIZE_C_TYPE_TO_PARAM_TYPE( TYPE_HOLDER_SPECIALIZATION )\
template<> struct c_type_to_param_type< TYPE_HOLDER_SPECIALIZATION::c_type >\
{\
  using c_type = TYPE_HOLDER_SPECIALIZATION::c_type;\
  using param_type = TYPE_HOLDER_SPECIALIZATION::param_type;\
  constexpr static const param_type value = TYPE_HOLDER_SPECIALIZATION::value;\
}


template<typename T, typename F, typename ...Ts>
inline bool get_from_variant(T& ret, const std::variant<F, Ts...>& rhs, const bool& implicit_cast_if_possible, std::string& what);


/**
 * @brief The class is a recursive map. This is useful to associate a parameter value to the chain of namespaces
 */
template<typename P>
class ParamDictionary
{
public:
  using base_type = P;
  using EmptyParam = bool;
  using NestedParams = std::map<std::string, ParamDictionary>;
  using AllowedParam = std::variant<EmptyParam, P, NestedParams>;
  // root :
  using ParamValue = std::pair<std::string, AllowedParam>;

  ParamDictionary() = delete;
  ParamDictionary(const std::string& key);
  ParamDictionary(const ParamDictionary& tree) = default;
  ~ParamDictionary() = default;

  const std::string& name() const;
  const AllowedParam& value() const;
  ParamDictionary& operator=(const P& value);
  ParamDictionary& operator[](const std::string& key);

  const ParamDictionary& operator[](const std::string& key) const;

  bool initialized() const;
  std::string to_string(const std::string& prefix) const;
  bool is_scalar() const;
  bool is_sequence() const;
  bool is_map() const;

private:
  ParamValue param_;
};

template<typename P>
bool to_yaml(const ParamDictionary<P>& tree, YAML::Node& node, std::string& what);

// template<typename P>
// bool to_yaml(const P& tree, YAML::Node& node, std::string& what);

/**
 * @brief
 *
 * @tparam Key is the name of the param, eventually nested_params. The namespace separator is '.' as in ROS 2 standard.
 * @tparam Value
 * @param tree
 * @param key
 * @return ParamDictionary*
 */
template<typename P>
const ParamDictionary<P>* find(const ParamDictionary<P>& tree, const std::string key, const std::string& parameter_separator_string, std::string& what);

}  // namespace core
}  // namespace param
}  // namespace cnr

#include <cnr_param/core/impl/param_dictionary.hpp>

#endif  // CNR_PARAM__INCLUDE__CNR_PARAM__CORE__PARAM_DICTIONARY__H