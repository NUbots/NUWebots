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

#ifndef WEBOTS_ROS_MESSAGE_FIELD_GET_ROTATION_H
#define WEBOTS_ROS_MESSAGE_FIELD_GET_ROTATION_H

#include "ros/service_traits.h"

#include "field_get_rotationRequest.h"
#include "field_get_rotationResponse.h"

namespace webots_ros
{

struct field_get_rotation
{

typedef field_get_rotationRequest Request;
typedef field_get_rotationResponse Response;
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
struct MD5Sum< ::webots_ros::field_get_rotation > {
  static const char* value()
  {
    return "ad330bd22eabd1d11d7ec7469e804a11";
  }

  static const char* value(const ::webots_ros::field_get_rotation&) { return value(); }
};

template<>
struct DataType< ::webots_ros::field_get_rotation > {
  static const char* value()
  {
    return "webots_ros/field_get_rotation";
  }

  static const char* value(const ::webots_ros::field_get_rotation&) { return value(); }
};

template<>
struct MD5Sum< ::webots_ros::field_get_rotationRequest>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::field_get_rotation >::value();
  }
  static const char* value(const ::webots_ros::field_get_rotationRequest&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::field_get_rotationRequest>
{
  static const char* value()
  {
    return DataType< ::webots_ros::field_get_rotation >::value();
  }
  static const char* value(const ::webots_ros::field_get_rotationRequest&)
  {
    return value();
  }
};

template<>
struct MD5Sum< ::webots_ros::field_get_rotationResponse>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::field_get_rotation >::value();
  }
  static const char* value(const ::webots_ros::field_get_rotationResponse&)
  {
    return value();
  }
};

template<>
struct DataType< ::webots_ros::field_get_rotationResponse>
{
  static const char* value()
  {
    return DataType< ::webots_ros::field_get_rotation >::value();
  }
  static const char* value(const ::webots_ros::field_get_rotationResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_FIELD_GET_ROTATION_H
