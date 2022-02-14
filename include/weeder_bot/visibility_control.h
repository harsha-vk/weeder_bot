#ifndef __WEEDER_BOT__VISIBILITY_CONTROL_H__
#define __WEEDER_BOT__VISIBILITY_CONTROL_H__

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define WEEDER_BOT_HARDWARE_EXPORT __attribute__((dllexport))
#define WEEDER_BOT_HARDWARE_IMPORT __attribute__((dllimport))
#else
#define WEEDER_BOT_HARDWARE_EXPORT __declspec(dllexport)
#define WEEDER_BOT_HARDWARE_IMPORT __declspec(dllimport)
#endif
#ifdef WEEDER_BOT_HARDWARE_BUILDING_DLL
#define WEEDER_BOT_HARDWARE_PUBLIC WEEDER_BOT_HARDWARE_EXPORT
#else
#define WEEDER_BOT_HARDWARE_PUBLIC WEEDER_BOT_HARDWARE_IMPORT
#endif
#define WEEDER_BOT_HARDWARE_PUBLIC_TYPE WEEDER_BOT_HARDWARE_PUBLIC
#define WEEDER_BOT_HARDWARE_LOCAL
#else
#define WEEDER_BOT_HARDWARE_EXPORT __attribute__((visibility("default")))
#define WEEDER_BOT_HARDWARE_IMPORT
#if __GNUC__ >= 4
#define WEEDER_BOT_HARDWARE_PUBLIC __attribute__((visibility("default")))
#define WEEDER_BOT_HARDWARE_LOCAL __attribute__((visibility("hidden")))
#else
#define WEEDER_BOT_HARDWARE_PUBLIC
#define WEEDER_BOT_HARDWARE_LOCAL
#endif
#define WEEDER_BOT_HARDWARE_PUBLIC_TYPE
#endif

#endif  // __WEEDER_BOT__VISIBILITY_CONTROL_H__