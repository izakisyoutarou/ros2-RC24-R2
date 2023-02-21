#ifndef SPLINE_PID__VISIBILITY_CONTROL_H_
#define SPLINE_PID__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SPLINE_PID_EXPORT __attribute__ ((dllexport))
    #define SPLINE_PID_IMPORT __attribute__ ((dllimport))
  #else
    #define SPLINE_PID_EXPORT __declspec(dllexport)
    #define SPLINE_PID_IMPORT __declspec(dllimport)
  #endif
  #ifdef SPLINE_PID_BUILDING_LIBRARY
    #define SPLINE_PID_PUBLIC SPLINE_PID_EXPORT
  #else
    #define SPLINE_PID_PUBLIC SPLINE_PID_IMPORT
  #endif
  #define SPLINE_PID_PUBLIC_TYPE SPLINE_PID_PUBLIC
  #define SPLINE_PID_LOCAL
#else
  #define SPLINE_PID_EXPORT __attribute__ ((visibility("default")))
  #define SPLINE_PID_IMPORT
  #if __GNUC__ >= 4
    #define SPLINE_PID_PUBLIC __attribute__ ((visibility("default")))
    #define SPLINE_PID_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SPLINE_PID_PUBLIC
    #define SPLINE_PID_LOCAL
  #endif
  #define SPLINE_PID_PUBLIC_TYPE
#endif

#endif  // SPLINE_PID__VISIBILITY_CONTROL_H_
