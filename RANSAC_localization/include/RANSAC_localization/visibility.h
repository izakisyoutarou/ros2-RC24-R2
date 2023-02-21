// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RANSAC_LOCALIZATION__VISIBILITY_H_
#define RANSAC_LOCALIZATION__VISIBILITY_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__

  #ifdef __GNUC__
    #define RANSAC_LOCALIZATION_EXPORT __attribute__ ((dllexport))
    #define RANSAC_LOCALIZATION_IMPORT __attribute__ ((dllimport))
  #else
    #define RANSAC_LOCALIZATION_EXPORT __declspec(dllexport)
    #define RANSAC_LOCALIZATION_IMPORT __declspec(dllimport)
  #endif

  #ifdef RANSAC_LOCALIZATION_DLL
    #define RANSAC_LOCALIZATION_PUBLIC RANSAC_LOCALIZATION_EXPORT
  #else
    #define RANSAC_LOCALIZATION_PUBLIC RANSAC_LOCALIZATION_IMPORT
  #endif

  #define RANSAC_LOCALIZATION_PUBLIC_TYPE RANSAC_LOCALIZATION_PUBLIC

  #define RANSAC_LOCALIZATION_LOCAL

#else

  #define RANSAC_LOCALIZATION_EXPORT __attribute__ ((visibility("default")))
  #define RANSAC_LOCALIZATION_IMPORT

  #if __GNUC__ >= 4
    #define RANSAC_LOCALIZATION_PUBLIC __attribute__ ((visibility("default")))
    #define RANSAC_LOCALIZATION_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define RANSAC_LOCALIZATION_PUBLIC
    #define RANSAC_LOCALIZATION_LOCAL
  #endif

  #define RANSAC_LOCALIZATION_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // RANSAC_LOCALIZATION__VISIBILITY_H_
