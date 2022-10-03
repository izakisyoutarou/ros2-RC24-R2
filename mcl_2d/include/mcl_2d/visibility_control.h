#ifndef MCL_2D__VISIBILITY_CONTROL_H_
#define MCL_2D__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MCL_2D_EXPORT __attribute__ ((dllexport))
    #define MCL_2D_IMPORT __attribute__ ((dllimport))
  #else
    #define MCL_2D_EXPORT __declspec(dllexport)
    #define MCL_2D_IMPORT __declspec(dllimport)
  #endif
  #ifdef MCL_2D_BUILDING_LIBRARY
    #define MCL_2D_PUBLIC MCL_2D_EXPORT
  #else
    #define MCL_2D_PUBLIC MCL_2D_IMPORT
  #endif
  #define MCL_2D_PUBLIC_TYPE MCL_2D_PUBLIC
  #define MCL_2D_LOCAL
#else
  #define MCL_2D_EXPORT __attribute__ ((visibility("default")))
  #define MCL_2D_IMPORT
  #if __GNUC__ >= 4
    #define MCL_2D_PUBLIC __attribute__ ((visibility("default")))
    #define MCL_2D_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MCL_2D_PUBLIC
    #define MCL_2D_LOCAL
  #endif
  #define MCL_2D_PUBLIC_TYPE
#endif

#endif  // MCL_2D__VISIBILITY_CONTROL_H_
