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

#ifndef ICP_BASE_SLAM__VISIBILITY_H_
#define ICP_BASE_SLAM__VISIBILITY_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__

  #ifdef __GNUC__
    #define ICP_BASE_SLAM_EXPORT __attribute__ ((dllexport))
    #define ICP_BASE_SLAM_IMPORT __attribute__ ((dllimport))
  #else
    #define ICP_BASE_SLAM_EXPORT __declspec(dllexport)
    #define ICP_BASE_SLAM_IMPORT __declspec(dllimport)
  #endif

  #ifdef ICP_BASE_SLAM_DLL
    #define ICP_BASE_SLAM_PUBLIC ICP_BASE_SLAM_EXPORT
  #else
    #define ICP_BASE_SLAM_PUBLIC ICP_BASE_SLAM_IMPORT
  #endif

  #define ICP_BASE_SLAM_PUBLIC_TYPE ICP_BASE_SLAM_PUBLIC

  #define ICP_BASE_SLAM_LOCAL

#else

  #define ICP_BASE_SLAM_EXPORT __attribute__ ((visibility("default")))
  #define ICP_BASE_SLAM_IMPORT

  #if __GNUC__ >= 4
    #define ICP_BASE_SLAM_PUBLIC __attribute__ ((visibility("default")))
    #define ICP_BASE_SLAM_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ICP_BASE_SLAM_PUBLIC
    #define ICP_BASE_SLAM_LOCAL
  #endif

  #define ICP_BASE_SLAM_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // ICP_BASE_SLAM__VISIBILITY_H_
