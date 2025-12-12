#ifndef RR_IMU_ACTION__VISIBILITY_CONTROL_H_
#define RR_IMU_ACTION__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define RR_IMU_ACTION_EXPORT __attribute__ ((dllexport))
    #define RR_IMU_ACTION_IMPORT __attribute__ ((dllimport))
  #else
    #define RR_IMU_ACTION_EXPORT __declspec(dllexport)
    #define RR_IMU_ACTION_IMPORT __declspec(dllimport)
  #endif
  #ifdef RR_IMU_ACTION_BUILDING_LIBRARY
    #define RR_IMU_ACTION_PUBLIC RR_IMU_ACTION_EXPORT
  #else
    #define RR_IMU_ACTION_PUBLIC RR_IMU_ACTION_IMPORT
  #endif
  #define RR_IMU_ACTION_PUBLIC_TYPE RR_IMU_ACTION_PUBLIC
  #define RR_IMU_ACTION_LOCAL
#else
  #define RR_IMU_ACTION_EXPORT __attribute__ ((visibility("default")))
  #define RR_IMU_ACTION_IMPORT
  #if __GNUC__ >= 4
    #define RR_IMU_ACTION_PUBLIC __attribute__ ((visibility("default")))
    #define RR_IMU_ACTION_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define RR_IMU_ACTION_PUBLIC
    #define RR_IMU_ACTION_LOCAL
  #endif
  #define RR_IMU_ACTION_PUBLIC_TYPE
#endif

#endif  // RR_IMU_ACTION__VISIBILITY_CONTROL_H_
