#ifndef CNR_PARAM__VISIBILITY_CONTROL_H_
#define CNR_PARAM__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CNR_PARAM_EXPORT __attribute__ ((dllexport))
    #define CNR_PARAM_IMPORT __attribute__ ((dllimport))
  #else
    #define CNR_PARAM_EXPORT __declspec(dllexport)
    #define CNR_PARAM_IMPORT __declspec(dllimport)
  #endif
  #ifdef CNR_PARAM_BUILDING_LIBRARY
    #define CNR_PARAM_PUBLIC CNR_PARAM_EXPORT
  #else
    #define CNR_PARAM_PUBLIC CNR_PARAM_IMPORT
  #endif
  #define CNR_PARAM_PUBLIC_TYPE CNR_PARAM_PUBLIC
  #define CNR_PARAM_LOCAL
#else
  #define CNR_PARAM_EXPORT __attribute__ ((visibility("default")))
  #define CNR_PARAM_IMPORT
  #if __GNUC__ >= 4
    #define CNR_PARAM_PUBLIC __attribute__ ((visibility("default")))
    #define CNR_PARAM_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define CNR_PARAM_PUBLIC
    #define CNR_PARAM_LOCAL
  #endif
  #define CNR_PARAM_PUBLIC_TYPE
#endif

#endif  // CNR_PARAM__VISIBILITY_CONTROL_H_
