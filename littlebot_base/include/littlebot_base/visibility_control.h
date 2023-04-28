#ifndef LITTLEBOT_BASE__VISIBILITY_CONTROL_H_
#define LITTLEBOT_BASE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define LITTLEBOT_BASE_EXPORT __attribute__((dllexport))
#define LITTLEBOT_BASE_IMPORT __attribute__((dllimport))
#else
#define LITTLEBOT_BASE_EXPORT __declspec(dllexport)
#define LITTLEBOT_BASE_IMPORT __declspec(dllimport)
#endif
#ifdef LITTLEBOT_BASE_BUILDING_DLL
#define LITTLEBOT_BASE_PUBLIC LITTLEBOT_BASE_EXPORT
#else
#define LITTLEBOT_BASE_PUBLIC LITTLEBOT_BASE_IMPORT
#endif
#define LITTLEBOT_BASE_PUBLIC_TYPE LITTLEBOT_BASE_PUBLIC
#define LITTLEBOT_BASE_LOCAL
#else
#define LITTLEBOT_BASE_EXPORT __attribute__((visibility("default")))
#define LITTLEBOT_BASE_IMPORT
#if __GNUC__ >= 4
#define LITTLEBOT_BASE_PUBLIC __attribute__((visibility("default")))
#define LITTLEBOT_BASE_LOCAL __attribute__((visibility("hidden")))
#else
#define LITTLEBOT_BASE_PUBLIC
#define LITTLEBOT_BASE_LOCAL
#endif
#define LITTLEBOT_BASE_PUBLIC_TYPE
#endif

#endif  // LITTLEBOT_BASE__VISIBILITY_CONTROL_H_