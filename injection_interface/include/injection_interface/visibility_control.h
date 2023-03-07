#ifndef INJECTION_INTERFACE__VISIBILITY_CONTROL_H_
#define INJECTION_INTERFACE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define INJECTION_INTERFACE_EXPORT __attribute__ ((dllexport))
    #define INJECTION_INTERFACE_IMPORT __attribute__ ((dllimport))
  #else
    #define INJECTION_INTERFACE_EXPORT __declspec(dllexport)
    #define INJECTION_INTERFACE_IMPORT __declspec(dllimport)
  #endif
  #ifdef INJECTION_INTERFACE_BUILDING_LIBRARY
    #define INJECTION_INTERFACE_PUBLIC INJECTION_INTERFACE_EXPORT
  #else
    #define INJECTION_INTERFACE_PUBLIC INJECTION_INTERFACE_IMPORT
  #endif
  #define INJECTION_INTERFACE_PUBLIC_TYPE INJECTION_INTERFACE_PUBLIC
  #define INJECTION_INTERFACE_LOCAL
#else
  #define INJECTION_INTERFACE_EXPORT __attribute__ ((visibility("default")))
  #define INJECTION_INTERFACE_IMPORT
  #if __GNUC__ >= 4
    #define INJECTION_INTERFACE_PUBLIC __attribute__ ((visibility("default")))
    #define INJECTION_INTERFACE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define INJECTION_INTERFACE_PUBLIC
    #define INJECTION_INTERFACE_LOCAL
  #endif
  #define INJECTION_INTERFACE_PUBLIC_TYPE
#endif

#endif  // INJECTION_INTERFACE__VISIBILITY_CONTROL_H_
