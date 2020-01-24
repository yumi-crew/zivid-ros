// Copyright (c) 2020 Norwegian University of Science and Technology
// Copyright (c) 2019, Zivid AS
// Use of this source code is governed by the BSD 3-Clause license, see LICENSE

#ifndef ZIVID_CAMERA__VISIBILITY_CONTROL_H_
#define ZIVID_CAMERA__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ZIVID_CAMERA_EXPORT __attribute__ ((dllexport))
    #define ZIVID_CAMERA_IMPORT __attribute__ ((dllimport))
  #else
    #define ZIVID_CAMERA_EXPORT __declspec(dllexport)
    #define ZIVID_CAMERA_IMPORT __declspec(dllimport)
  #endif
  #ifdef ZIVID_CAMERA_BUILDING_DLL
    #define ZIVID_CAMERA_PUBLIC ZIVID_CAMERA_EXPORT
  #else
    #define ZIVID_CAMERA_PUBLIC ZIVID_CAMERA_IMPORT
  #endif
  #define ZIVID_CAMERA_PUBLIC_TYPE ZIVID_CAMERA_PUBLIC
  #define ZIVID_CAMERA_LOCAL
#else
  #define ZIVID_CAMERA_EXPORT __attribute__ ((visibility("default")))
  #define ZIVID_CAMERA_IMPORT
  #if __GNUC__ >= 4
    #define ZIVID_CAMERA_PUBLIC __attribute__ ((visibility("default")))
    #define ZIVID_CAMERA_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ZIVID_CAMERA_PUBLIC
    #define ZIVID_CAMERA_LOCAL
  #endif
  #define ZIVID_CAMERA_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // ZIVID_CAMERA__VISIBILITY_CONTROL_H_