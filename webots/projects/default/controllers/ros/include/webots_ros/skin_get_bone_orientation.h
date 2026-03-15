/*
 * Copyright 1996-2022 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef WEBOTS_ROS_MESSAGE_SKIN_GET_BONE_ORIENTATION_H
#define WEBOTS_ROS_MESSAGE_SKIN_GET_BONE_ORIENTATION_H

#include "ros/service_traits.h"

#include "skin_get_bone_orientationRequest.h"
#include "skin_get_bone_orientationResponse.h"

namespace webots_ros
{

struct skin_get_bone_orientation
{

typedef skin_get_bone_orientationRequest Request;
typedef skin_get_bone_orientationResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

};
} // namespace webots_ros

namespace ros
{
namespace service_traits
{

template<>
struct MD5Sum< ::webots_ros::skin_get_bone_orientation > {
  static const char* value()
  {
    return "f5889bc85962c2ea0f40d74e77b00b8b";
  }

  static const char* value(const ::webots_ros::skin_get_bone_orientation&) { return value(); }
};

template<>
struct DataType< ::webots_ros::skin_get_bone_orientation > {
  static const char* value()
  {
    return "webots_ros/skin_get_bone_orientation";
  }

  static const char* value(const ::webots_ros::skin_get_bone_orientation&) { return value(); }
};

template<>
struct MD5Sum< ::webots_ros::skin_get_bone_orientationRequest>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::skin_get_bone_orientation >::value();
  }
  static const char* value(const ::webots_ros::skin_get_bone_orientationRequest&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::skin_get_bone_orientationRequest>
{
  static const char* value()
  {
    return DataType< ::webots_ros::skin_get_bone_orientation >::value();
  }
  static const char* value(const ::webots_ros::skin_get_bone_orientationRequest&)
  {
    return value();
  }
};

template<>
struct MD5Sum< ::webots_ros::skin_get_bone_orientationResponse>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::skin_get_bone_orientation >::value();
  }
  static const char* value(const ::webots_ros::skin_get_bone_orientationResponse&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::skin_get_bone_orientationResponse>
{
  static const char* value()
  {
    return DataType< ::webots_ros::skin_get_bone_orientation >::value();
  }
  static const char* value(const ::webots_ros::skin_get_bone_orientationResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_SKIN_GET_BONE_ORIENTATION_H
